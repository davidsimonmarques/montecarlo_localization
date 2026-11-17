#!/usr/bin/env python3
"""
Run automated localization experiments replaying only base topics from a recorded rosbag2.

Reutiliza um bag gravado anteriormente reproduzindo apenas os tópicos base de entrada
(/scan, /imu/data, /odom, /ground_truth, etc.), sem reproduzir saídas antigas dos
algoritmos (/mcl_pose, /tf, /map, /fastlio2/*), permitindo reexecutar todos os 4
algoritmos (MCL, AMCL, MH-MCL, MH-AMCL) em N experimentos com seeds distintos.
"""

import argparse
import csv
import datetime
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_BASE_TOPICS = (
    "/scan",
    "/imu/data",
    "/odom",
    "/ground_truth",
    "/odom_ground_truth",
    "/pose_ground_truth",
)


def is_valid_rosbag_dir(path: str) -> bool:
    p = Path(path)
    if not p.is_dir():
        return False
    return (p / "metadata.yaml").is_file() or any(p.glob("*.db3"))


def resolve_source_bag(source_bag_arg: str) -> str:
    """Resolve explicit bag path or pick the newest exp_* bag in ~/Documents/BagsRecordings_RP."""
    if source_bag_arg:
        expanded = os.path.abspath(os.path.expanduser(source_bag_arg))
        if is_valid_rosbag_dir(expanded):
            return expanded
        # If user passed a root directory containing exp_* bags, pick newest valid exp_* inside it
        root = Path(expanded)
        if root.is_dir():
            candidates = sorted(
                [d for d in root.iterdir() if d.is_dir() and d.name.startswith("exp_") and is_valid_rosbag_dir(str(d))],
                key=lambda d: d.stat().st_mtime,
                reverse=True,
            )
            if candidates:
                return str(candidates[0])
        raise FileNotFoundError(f"Bag de origem inválido ou não encontrado: {expanded}")

    default_root = Path(os.path.expanduser("~/Documents/BagsRecordings_RP"))
    if default_root.is_dir():
        candidates = sorted(
            [d for d in default_root.iterdir() if d.is_dir() and d.name.startswith("exp_") and is_valid_rosbag_dir(str(d))],
            key=lambda d: d.stat().st_mtime,
            reverse=True,
        )
        if candidates:
            return str(candidates[0])

    raise FileNotFoundError(
        "Nenhum bag de origem informado e nenhum exp_* válido encontrado em ~/Documents/BagsRecordings_RP."
    )


def inspect_bag_base_topics(bag_path: str, odom_source: str, requested_topics=None):
    """Inspect topics present in source bag and select only existing base topics."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    available = {t.name for t in reader.get_all_topics_and_types()}

    candidates = list(requested_topics) if requested_topics else list(DEFAULT_BASE_TOPICS)
    selected = [t for t in candidates if t in available]

    if "/scan" not in selected:
        raise RuntimeError(f"Tópico obrigatório '/scan' ausente no bag: {bag_path}")
    if odom_source == "fastlio2" and "/imu/data" not in selected:
        raise RuntimeError(f"Tópico obrigatório '/imu/data' (para fastlio2) ausente no bag: {bag_path}")
    if odom_source == "wheel" and "/odom" not in selected:
        raise RuntimeError(f"Tópico obrigatório '/odom' (para wheel) ausente no bag: {bag_path}")

    has_gt = any(t in selected for t in ("/ground_truth", "/pose_ground_truth"))
    if not has_gt:
        print("\033[93m[AVISO] Nenhum tópico de ground truth (/ground_truth ou /pose_ground_truth) encontrado no bag.\033[0m")

    return selected


class ReadinessProbeNode(Node):
    """Waits until all 4 localization nodes finish loading the map and publish initial pose."""

    def __init__(self):
        super().__init__("bag_replay_readiness_probe")
        self.ready_topics = {
            "/mcl_pose": False,
            "/amcl_pose": False,
            "/mhmcl_pose": False,
            "/mhamcl_pose": False,
        }
        for topic in self.ready_topics:
            self.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                lambda _msg, t=topic: self._mark_ready(t),
                10,
            )

    def _mark_ready(self, topic: str):
        self.ready_topics[topic] = True

    def all_ready(self) -> bool:
        return all(self.ready_topics.values())


class BagExperimentRunner:
    def __init__(self, args):
        self.source_bag = resolve_source_bag(args.source_bag)
        self.num_experiments = args.num_experiments
        self.start_idx = args.start_idx
        self.bags_dir = os.path.abspath(os.path.expanduser(args.bags_dir))
        self.odom_source = args.odom_source
        self.rate = args.rate
        self.use_rviz = args.use_rviz
        self.wait_before_play = args.wait_before_play
        self.post_play_wait = args.post_play_wait
        self.fixed_seed = args.seed
        self.params_file = os.path.abspath(os.path.expanduser(args.params_file)) if args.params_file else ""
        self.base_topics = inspect_bag_base_topics(self.source_bag, self.odom_source, args.base_topics)

        self.active_processes = []
        self._is_terminating = False
        self._user_interrupted = False
        os.makedirs(self.bags_dir, exist_ok=True)

        print("=" * 80)
        print(f"[INFO] Bag de origem: {self.source_bag}")
        print(f"[INFO] Tópicos base selecionados: {', '.join(self.base_topics)}")
        print(f"[INFO] Odometria: {self.odom_source} | Rate: {self.rate}x | Experimentos: {self.start_idx}..{self.num_experiments}")
        print(f"[INFO] Diretório de saída: {self.bags_dir}")
        print("=" * 80)

        signal.signal(signal.SIGINT, self.handle_sigint)
        signal.signal(signal.SIGTERM, self.handle_sigint)

    def handle_sigint(self, signum, frame):
        if self._is_terminating or self._user_interrupted:
            return
        self._user_interrupted = True
        print("\n\033[91m[INTERRUPÇÃO] Sinal recebido! Encerrando subprocessos com segurança...\033[0m")
        self.terminate_all_processes()
        sys.exit(1)

    def start_process(self, cmd, name, show_output=False):
        print(f"\033[96m[START] {name}: {' '.join(cmd)}\033[0m")
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=None if show_output else subprocess.DEVNULL,
            stderr=None if show_output else subprocess.STDOUT,
            start_new_session=True,
        )
        self.active_processes.append((proc, name))
        return proc

    def stop_process(self, proc, name, sig=signal.SIGINT, timeout=6.0):
        if proc.poll() is not None:
            return
        print(f"\033[93m[STOP] Encerrando {name} (PID: {proc.pid})...\033[0m")
        try:
            pgid = os.getpgid(proc.pid)
            if pgid != os.getpgrp():
                os.killpg(pgid, sig)
            else:
                proc.send_signal(sig)
        except (ProcessLookupError, PermissionError):
            return

        t0 = time.time()
        while proc.poll() is None and (time.time() - t0) < timeout:
            time.sleep(0.1)

        if proc.poll() is None:
            print(f"\033[91m[KILL] Forçando SIGKILL em {name}...\033[0m")
            try:
                pgid = os.getpgid(proc.pid)
                if pgid != os.getpgrp():
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    proc.kill()
                proc.wait(timeout=2.0)
            except Exception:
                pass

    def terminate_all_processes(self):
        if self._is_terminating:
            return
        self._is_terminating = True

        prev_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        prev_sigterm = signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            for proc, name in reversed(self.active_processes):
                self.stop_process(proc, name, sig=signal.SIGINT, timeout=6.0)
            self.active_processes.clear()
        finally:
            self._is_terminating = False
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)

    @staticmethod
    def percentile(sorted_values, percentile):
        if not sorted_values:
            return ""
        position = (len(sorted_values) - 1) * percentile
        lower = int(position)
        upper = min(lower + 1, len(sorted_values) - 1)
        fraction = position - lower
        return sorted_values[lower] + fraction * (sorted_values[upper] - sorted_values[lower])

    def save_computation_metrics(self, bag_output_path):
        metric_topics = {
            "/montecarlo_localization_node/filter_cycle_ms": "MCL",
            "/amcl_node/filter_cycle_ms": "AMCL",
            "/mhmcl_node/filter_cycle_ms": "MH-MCL",
            "/mhamcl_node/filter_cycle_ms": "MH-AMCL",
        }
        samples = {method: [] for method in metric_topics.values()}
        scan_stamps = []

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_output_path, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        available_metrics = {
            topic: method for topic, method in metric_topics.items() if topic in topic_types
        }

        scan_topic = "/scan"
        topics_to_read = list(available_metrics)
        if scan_topic in topic_types:
            topics_to_read.append(scan_topic)
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics_to_read))
        message_types = {topic: get_message(topic_types[topic]) for topic in topics_to_read}

        if topics_to_read:
            while reader.has_next():
                topic, raw_message, _ = reader.read_next()
                message = deserialize_message(raw_message, message_types[topic])
                if topic == scan_topic:
                    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                    if stamp > 0.0:
                        scan_stamps.append(stamp)
                else:
                    duration_ms = float(message.data)
                    if math.isfinite(duration_ms) and duration_ms >= 0.0:
                        samples[available_metrics[topic]].append(duration_ms)

        scan_stamps.sort()
        scan_duration = scan_stamps[-1] - scan_stamps[0] if len(scan_stamps) > 1 else 0.0
        scan_periods = sorted(
            (later - earlier) * 1000.0
            for earlier, later in zip(scan_stamps, scan_stamps[1:])
            if later > earlier
        )
        scan_period_ms = self.percentile(scan_periods, 0.5)
        experiment_name = os.path.basename(bag_output_path)
        rows = []

        for method, durations in samples.items():
            ordered = sorted(durations)
            count = len(ordered)
            if count:
                mean_ms = sum(ordered) / count
                p50_ms = self.percentile(ordered, 0.5)
                p95_ms = self.percentile(ordered, 0.95)
                p99_ms = self.percentile(ordered, 0.99)
                max_ms = ordered[-1]
                updates_per_second = count / scan_duration if scan_duration > 0 else ""
                deadline_miss_pct = (
                    100.0 * sum(value > scan_period_ms for value in ordered) / count
                    if scan_period_ms
                    else ""
                )
                p95_utilization_pct = 100.0 * p95_ms / scan_period_ms if scan_period_ms else ""
            else:
                mean_ms = p50_ms = p95_ms = p99_ms = max_ms = ""
                updates_per_second = deadline_miss_pct = p95_utilization_pct = ""

            rows.append({
                "Experimento": experiment_name,
                "Metodo": method,
                "Ciclos": count,
                "Tempo_Medio_ms": mean_ms,
                "Tempo_P50_ms": p50_ms,
                "Tempo_P95_ms": p95_ms,
                "Tempo_P99_ms": p99_ms,
                "Tempo_Max_ms": max_ms,
                "Periodo_Scan_P50_ms": scan_period_ms,
                "Utilizacao_P95_Periodo_Scan_pct": p95_utilization_pct,
                "Ciclos_Acima_Periodo_Scan_pct": deadline_miss_pct,
                "Ciclos_por_segundo": updates_per_second,
            })

        fieldnames = list(rows[0].keys())
        metrics_path = os.path.join(bag_output_path, "computation_metrics.csv")
        with open(metrics_path, "w", newline="", encoding="utf-8") as metrics_file:
            writer = csv.DictWriter(metrics_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        all_rows = []
        for directory, _, filenames in os.walk(self.bags_dir):
            if "computation_metrics.csv" in filenames:
                with open(os.path.join(directory, "computation_metrics.csv"), newline="", encoding="utf-8") as mf:
                    all_rows.extend(csv.DictReader(mf))
        summary_path = os.path.join(self.bags_dir, "computation_summary.csv")
        with open(summary_path, "w", newline="", encoding="utf-8") as summary_file:
            writer = csv.DictWriter(summary_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)

        print(f"[INFO] Métricas de ciclo salvas em: {metrics_path}")
        for row in rows:
            if row["Ciclos"]:
                print(
                    f"  {row['Metodo']}: n={row['Ciclos']}, "
                    f"P50={row['Tempo_P50_ms']:.3f} ms, "
                    f"P95={row['Tempo_P95_ms']:.3f} ms, "
                    f"misses={row['Ciclos_Acima_Periodo_Scan_pct']:.2f}%"
                )
            else:
                print(f"  {row['Metodo']}: sem amostras de tempo de ciclo")

    def wait_for_localizations_ready(self, timeout=20.0):
        probe = ReadinessProbeNode()
        executor = SingleThreadedExecutor()
        executor.add_node(probe)
        t0 = time.time()
        try:
            while rclpy.ok() and not self._user_interrupted and (time.time() - t0) < timeout:
                executor.spin_once(timeout_sec=0.1)
                if probe.all_ready():
                    print(f"\033[92m[OK] Todos os 4 nós carregaram o mapa ({time.time() - t0:.1f}s).\033[0m")
                    return True
        finally:
            executor.remove_node(probe)
            probe.destroy_node()

        missing = [k for k, v in probe.ready_topics.items() if not v]
        print(f"\033[93m[AVISO] Timeout aguardando inicialização de: {missing}. Continuando...\033[0m")
        return False

    def run_single_experiment(self, exp_idx: int):
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        bag_name = f"exp_{exp_idx:02d}_{timestamp}"
        bag_output_path = os.path.join(self.bags_dir, bag_name)
        random_seed = self.fixed_seed if self.fixed_seed >= 0 else secrets.randbits(32)

        print("\n" + "=" * 80)
        print(f"\033[1;32m>>> EXPERIMENTO {exp_idx}/{self.num_experiments} (REPLAY BASE) <<<\033[0m")
        print(f"  • Seed compartilhada: {random_seed}")
        print(f"  • Bag de saída: {bag_output_path}")
        print("=" * 80)

        # 1. Publicar TF estática base_link -> base_scan (sem reproduzir /tf antigo do bag)
        tf_cmd = [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "--x", "0.0", "--y", "0.0", "--z", "0.0692",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "base_scan",
        ]
        self.start_process(tf_cmd, "StaticTF (base_link->base_scan)")

        # 2. Iniciar os 4 algoritmos de localização (+ FAST-LIO2 se odom_source == fastlio2)
        launch_file = (
            "all_localizations_fastlio2_launch.py"
            if self.odom_source == "fastlio2"
            else "all_localizations_launch.py"
        )
        loc_cmd = [
            "ros2", "launch", "montecarlo_localization", launch_file,
            f"random_seed:={random_seed}",
        ]
        if self.odom_source == "fastlio2":
            loc_cmd.append(f"use_rviz:={'true' if self.use_rviz else 'false'}")
            if self.params_file:
                loc_cmd.append(f"params_file:={self.params_file}")
        loc_proc = self.start_process(loc_cmd, f"All_Localizations ({self.odom_source})")

        # Aguardar mapa e inicialização dos 4 filtros ANTES de gravar o novo bag
        self.wait_for_localizations_ready(timeout=self.wait_before_play)

        # 3. Iniciar gravação do novo Rosbag2
        record_cmd = ["ros2", "bag", "record", "-a", "-o", bag_output_path]
        self.start_process(record_cmd, f"Rosbag2_Record ({bag_name})")
        time.sleep(1.0)

        # 4. Reproduzir apenas os tópicos base do bag de origem
        play_cmd = [
            "ros2", "bag", "play", self.source_bag,
            "--disable-keyboard-controls",
            "--rate", str(self.rate),
            "--topics", *self.base_topics,
        ]
        play_proc = self.start_process(play_cmd, "Rosbag2_Play (base_topics)", show_output=True)

        success = False
        reason = "desconhecido"
        try:
            while rclpy.ok() and not self._user_interrupted:
                ret = play_proc.poll()
                if ret is not None:
                    success = (ret == 0)
                    reason = "bag_completed" if success else f"bag_play_exit_{ret}"
                    break
                if loc_proc.poll() is not None:
                    reason = f"localization_exited_{loc_proc.returncode}"
                    break
                time.sleep(0.2)

            if not self._user_interrupted and success and self.post_play_wait > 0.0:
                print(f"[INFO] Aguardando {self.post_play_wait:.1f}s após término do bag...")
                time.sleep(self.post_play_wait)
        finally:
            self.terminate_all_processes()

        if os.path.isdir(bag_output_path):
            self.save_computation_metrics(bag_output_path)

        time.sleep(2.0)
        print(f"\033[1;32m[FIM EXPERIMENTO {exp_idx}]\033[0m Status: {reason} | Bag: {bag_output_path}\n")
        return success, reason, bag_output_path, random_seed

    def run_all(self):
        rclpy.init()
        results = []
        try:
            for idx in range(self.start_idx, self.num_experiments + 1):
                if self._user_interrupted:
                    break
                results.append((idx, *self.run_single_experiment(idx)))
        finally:
            self.terminate_all_processes()
            if rclpy.ok():
                rclpy.shutdown()

        if results:
            print("\n" + "=" * 80)
            print("\033[1;34m=== RELATÓRIO FINAL (BAG REPLAY) ===\033[0m")
            print("=" * 80)
            for idx, ok, reason, bag_path, seed in results:
                status = "\033[92mSUCESSO\033[0m" if ok else "\033[91mFALHA\033[0m"
                print(f"Exp {idx:02d}: {status} ({reason}) | seed={seed} -> {bag_path}")
            print("=" * 80)
            print(f"Para gerar gráficos e RMSEs, execute:\n  python3 processar_experimentos.py {self.bags_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Executa N experimentos dos 4 algoritmos usando apenas os tópicos base de um rosbag2."
    )
    parser.add_argument(
        "--source-bag",
        type=str,
        default="",
        help="Caminho do rosbag2 de origem (se omitido, usa o exp_* mais recente em ~/Documents/BagsRecordings_RP)",
    )
    parser.add_argument("--num-experiments", type=int, default=10, help="Número total de experimentos")
    parser.add_argument("--start-idx", type=int, default=1, help="Índice inicial do experimento")
    parser.add_argument(
        "--bags-dir",
        type=str,
        default="~/Documents/BagsRecordings_BagReplay",
        help="Diretório de saída dos novos bags",
    )
    parser.add_argument(
        "--odom-source",
        choices=("wheel", "fastlio2"),
        default="fastlio2",
        help="Fonte de odometria dos filtros (padrão: fastlio2)",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="Taxa de reprodução do bag (padrão: 1.0)")
    parser.add_argument("--use-rviz", action="store_true", default=False, help="Abrir RViz (apenas fastlio2)")
    parser.add_argument(
        "--wait-before-play",
        type=float,
        default=20.0,
        help="Tempo máximo (s) aguardando inicialização dos 4 filtros antes do play",
    )
    parser.add_argument(
        "--post-play-wait",
        type=float,
        default=3.0,
        help="Tempo adicional (s) após o fim do bag antes de encerrar gravação",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=-1,
        help="Seed fixa (>=0) ou -1 para sortear nova seed compartilhada por experimento",
    )
    parser.add_argument(
        "--params-file",
        type=str,
        default="",
        help="YAML customizado (opcional, usado quando odom-source=fastlio2)",
    )
    parser.add_argument(
        "--base-topics",
        nargs="+",
        default=None,
        help="Lista explícita de tópicos base a reproduzir do bag",
    )

    args = parser.parse_args()
    runner = BagExperimentRunner(args)
    runner.run_all()


if __name__ == "__main__":
    main()
