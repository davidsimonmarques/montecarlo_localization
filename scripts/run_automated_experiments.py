#!/usr/bin/env python3
"""
Run automated localization experiments.

Orquestrador para execução automatizada de 50 experimentos de localização:
1. Gerencia somente os processos filhos iniciados por esta execução.
2. Inicia CoppeliaSim com o cenário Factory.ttt (headless com xvfb se disponível, ou GUI automatizada com -s0).
3. Aguarda os tópicos necessários à fonte de odometria selecionada e /scan via rclpy.
4. Inicia os 4 algoritmos de localização simultâneos (MCL, AMCL, MH-MCL, MH-AMCL) sem RViz.
5. Inicia a gravação do rosbag2 com todos os tópicos (-a) em diretório dedicado.
6. Aguarda 5 segundos.
7. Inicia o seguimento de caminho via `diffdrive_fbl_control husky_coppelia_path_launch.py`.
8. Garante a entrega do /path ao nó de controle via publicador contínuo com verificação de recebimento.
9. Monitora o progresso do robô até atingir o objetivo final (ou atingir o timeout de segurança).
10. Finaliza todos os processos de forma limpa (SIGINT isolado com blindagem contra recursão).
11. Repete para os 50 experimentos.
"""

import argparse
import csv
import datetime
import math
import os
import shutil
import signal
import secrets
import subprocess
import sys
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, PoseStamped, Point
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def load_path_from_file(file_path, frame_id="odom"):
    """Lê coordenadas (x, y, z) de um arquivo txt e retorna uma mensagem nav_msgs/Path."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Arquivo de caminho não encontrado: {file_path}")

    raw_points = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.replace(',', ' ').replace(';', ' ').split()
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2]) if len(parts) >= 3 else 0.0
                    raw_points.append((x, y, z))
                except ValueError:
                    pass

    if not raw_points:
        raise ValueError(f"Nenhum waypoint válido encontrado em: {file_path}")

    path_msg = Path()
    path_msg.header.frame_id = frame_id
    n_pts = len(raw_points)

    for i, pt in enumerate(raw_points):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = pt[2]

        if i + 1 < n_pts:
            dx = raw_points[i + 1][0] - pt[0]
            dy = raw_points[i + 1][1] - pt[1]
            yaw = math.atan2(dy, dx)
        elif i > 0:
            dx = pt[0] - raw_points[i - 1][0]
            dy = pt[1] - raw_points[i - 1][1]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0

        half = yaw * 0.5
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        path_msg.poses.append(pose)

    goal_x = raw_points[-1][0]
    goal_y = raw_points[-1][1]
    return path_msg, (goal_x, goal_y)


class ExperimentMonitorNode(Node):
    def __init__(self, path_msg: Path, goal_x: float, goal_y: float, goal_tolerance: float = 0.25, min_run_time: float = 30.0):
        super().__init__('experiment_monitor_node')
        self.path_msg = path_msg
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_tolerance = goal_tolerance
        self.min_run_time = min_run_time
        self.start_time = time.time()
        
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_dist_to_goal = 999.0
        self.goal_reached = False
        self.received_pose = False
        self.is_stopped = False
        self.stopped_count = 0
        self.pose_source_priority = 0
        # Publicador de Path contínuo a 2 Hz para garantir que o DiffDriveFBLControlNode receba o caminho
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.path_timer = self.create_timer(0.5, self.publish_path_cb)

        # Assina múltiplas fontes de pose em ordem de prioridade:
        # 1. /current_position (Point emitido diretamente pelo controlador FBL)
        self.cur_pos_sub = self.create_subscription(Point, '/current_position', self.cur_pos_cb, 10)
        # 2. /odom_ground_truth (pose ground truth no referencial do path)
        self.gt_odom_sub = self.create_subscription(Odometry, '/odom_ground_truth', self.gt_odom_cb, 10)
        # 3. /fastlio2/odom (odometria lidar-inercial de alta precisão)
        self.fastlio_sub = self.create_subscription(Odometry, '/fastlio2/odom', self.fastlio_cb, 10)
        # 4. /odom (odometria de rodas padrão)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

    def publish_path_cb(self):
        if self.path_msg is not None:
            now = self.get_clock().now().to_msg()
            self.path_msg.header.stamp = now
            for p in self.path_msg.poses:
                p.header.stamp = now
            self.path_pub.publish(self.path_msg)

    def cur_pos_cb(self, msg: Point):
        self.pose_source_priority = 3
        self.process_pose(msg.x, msg.y)

    def gt_odom_cb(self, msg: Odometry):
        if self.pose_source_priority <= 3:
            self.process_pose(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def fastlio_cb(self, msg: Odometry):
        if self.pose_source_priority < 2:
            self.pose_source_priority = 2
            self.process_pose(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def odom_cb(self, msg: Odometry):
        if self.pose_source_priority < 1:
            self.pose_source_priority = 1
            self.process_pose(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def process_pose(self, x: float, y: float):
        self.last_x = x
        self.last_y = y
        self.received_pose = True

        dx = self.last_x - self.goal_x
        dy = self.last_y - self.goal_y
        self.last_dist_to_goal = (dx * dx + dy * dy) ** 0.5

        elapsed = time.time() - self.start_time
        if elapsed >= self.min_run_time:
            if self.last_dist_to_goal <= self.goal_tolerance:
                self.goal_reached = True

    def cmd_vel_cb(self, msg: Twist):
        elapsed = time.time() - self.start_time
        if elapsed >= self.min_run_time:
            # Se o controlador já finalizou o path, cmd_vel vai para 0
            if abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01:
                self.stopped_count += 1
                # Parada detectada: 20 ciclos (~1.0s) perto da meta (<= 3x goal_tolerance = 0.75m)
                if self.stopped_count >= 20 and self.last_dist_to_goal <= (self.goal_tolerance * 3.0):
                    self.is_stopped = True
                # Ou 80 ciclos (~4.0s) de parada estável próximo ao fim da trajetória
                elif self.stopped_count >= 80 and elapsed > 120.0 and self.last_dist_to_goal <= 1.5:
                    self.is_stopped = True
            else:
                self.stopped_count = 0


class AutomatedExperimentRunner:
    def __init__(self, args):
        self.num_experiments = args.num_experiments
        self.start_idx = args.start_idx
        self.bags_dir = os.path.expanduser(args.bags_dir)
        self.coppelia_bin = os.path.expanduser(args.coppelia_bin)
        self.scene_path = os.path.expanduser(args.scene_path)
        self.path_file = os.path.expanduser(args.path_file)
        self.timeout = args.timeout
        self.wait_before_fbl = args.wait_before_fbl
        self.headless = args.headless
        self.odom_source = args.odom_source
        self.xvfb_available = shutil.which("xvfb-run") is not None
        
        self.active_processes = []
        self._is_terminating = False
        self._user_interrupted = False
        os.makedirs(self.bags_dir, exist_ok=True)

        # Ler pontos do arquivo de poses e definir o objetivo
        self.path_msg, (self.goal_x, self.goal_y) = load_path_from_file(self.path_file, frame_id="odom")
        n_pts = len(self.path_msg.poses) if self.path_msg else 0
        print(f"[INFO] Carregados {n_pts} waypoints de '{os.path.basename(self.path_file)}'. Meta final: ({self.goal_x:.3f}, {self.goal_y:.3f})")

        # Limpeza preventiva de processos fantasmas de execuções anteriores (sem matar a si mesmo)

        signal.signal(signal.SIGINT, self.handle_sigint)
        signal.signal(signal.SIGTERM, self.handle_sigint)

    def handle_sigint(self, signum, frame):
        if self._is_terminating or self._user_interrupted:
            return
        self._user_interrupted = True
        print("\n\033[91m[INTERRUPÇÃO] Sinal recebido! Encerrando todos os subprocessos com segurança...\033[0m")
        self.terminate_all_processes()
        sys.exit(1)

    def start_process(self, cmd, name, show_output=False):
        """Inicia um processo em seu próprio grupo de processos isolado."""
        print(f"\033[96m[START] {name}: {' '.join(cmd) if isinstance(cmd, list) else cmd}\033[0m")
        proc = subprocess.Popen(
            cmd,
            shell=isinstance(cmd, str),
            stdout=None if show_output else subprocess.DEVNULL,
            stderr=None if show_output else subprocess.STDOUT,
            start_new_session=True
        )
        self.active_processes.append((proc, name))
        return proc

    def stop_process(self, proc, name, sig=signal.SIGINT, timeout=5.0):
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

        # Mascara temporariamente os sinais para impedir reentrância / recursão infinita
        prev_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        prev_sigterm = signal.signal(signal.SIGTERM, signal.SIG_IGN)

        try:
            # Encerra processos na ordem inversa (fbl, bag, localizacao, coppelia)
            for proc, name in reversed(self.active_processes):
                self.stop_process(proc, name, sig=signal.SIGINT, timeout=5.0)
            self.active_processes.clear()

        finally:
            self._is_terminating = False
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)

    @staticmethod
    def percentile(sorted_values, percentile):
        if not sorted_values:
            return ''
        position = (len(sorted_values) - 1) * percentile
        lower = int(position)
        upper = min(lower + 1, len(sorted_values) - 1)
        fraction = position - lower
        return sorted_values[lower] + fraction * (sorted_values[upper] - sorted_values[lower])

    def save_computation_metrics(self, bag_output_path):
        metric_topics = {
            '/montecarlo_localization_node/filter_cycle_ms': 'MCL',
            '/amcl_node/filter_cycle_ms': 'AMCL',
            '/mhmcl_node/filter_cycle_ms': 'MH-MCL',
            '/mhamcl_node/filter_cycle_ms': 'MH-AMCL',
        }
        samples = {method: [] for method in metric_topics.values()}
        scan_stamps = []

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_output_path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            ),
        )
        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        available_metrics = {
            topic: method for topic, method in metric_topics.items() if topic in topic_types
        }
        missing_metrics = sorted(set(metric_topics) - set(available_metrics))
        if missing_metrics:
            print(
                '[ERRO] Tópicos de tempo de ciclo ausentes no bag: '
                + ', '.join(missing_metrics))

        scan_topic = '/scan'
        topics_to_read = list(available_metrics)
        if scan_topic in topic_types:
            topics_to_read.append(scan_topic)
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics_to_read))
        message_types = {
            topic: get_message(topic_types[topic]) for topic in topics_to_read
        }

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
            if later > earlier)
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
                updates_per_second = count / scan_duration if scan_duration > 0 else ''
                deadline_miss_pct = (
                    100.0 * sum(value > scan_period_ms for value in ordered) / count
                    if scan_period_ms else '')
                p95_utilization_pct = (
                    100.0 * p95_ms / scan_period_ms if scan_period_ms else '')
            else:
                mean_ms = p50_ms = p95_ms = p99_ms = max_ms = ''
                updates_per_second = deadline_miss_pct = p95_utilization_pct = ''

            rows.append({
                'Experimento': experiment_name,
                'Metodo': method,
                'Ciclos': count,
                'Tempo_Medio_ms': mean_ms,
                'Tempo_P50_ms': p50_ms,
                'Tempo_P95_ms': p95_ms,
                'Tempo_P99_ms': p99_ms,
                'Tempo_Max_ms': max_ms,
                'Periodo_Scan_P50_ms': scan_period_ms,
                'Utilizacao_P95_Periodo_Scan_pct': p95_utilization_pct,
                'Ciclos_Acima_Periodo_Scan_pct': deadline_miss_pct,
                'Ciclos_por_segundo': updates_per_second,
            })

        fieldnames = list(rows[0].keys())
        metrics_path = os.path.join(bag_output_path, 'computation_metrics.csv')
        with open(metrics_path, 'w', newline='', encoding='utf-8') as metrics_file:
            writer = csv.DictWriter(metrics_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        all_rows = []
        for directory, _, filenames in os.walk(self.bags_dir):
            if 'computation_metrics.csv' in filenames:
                with open(
                    os.path.join(directory, 'computation_metrics.csv'),
                    newline='', encoding='utf-8') as metrics_file:
                    all_rows.extend(csv.DictReader(metrics_file))
        summary_path = os.path.join(self.bags_dir, 'computation_summary.csv')
        with open(summary_path, 'w', newline='', encoding='utf-8') as summary_file:
            writer = csv.DictWriter(summary_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)

        print(f"[INFO] Métricas de ciclo salvas em: {metrics_path}")
        for row in rows:
            if row['Ciclos']:
                print(
                    f"  {row['Metodo']}: n={row['Ciclos']}, "
                    f"P50={row['Tempo_P50_ms']:.3f} ms, "
                    f"P95={row['Tempo_P95_ms']:.3f} ms, "
                    f"P99={row['Tempo_P99_ms']:.3f} ms, "
                    f"misses={row['Ciclos_Acima_Periodo_Scan_pct']:.2f}%")
            else:
                print(f"  {row['Metodo']}: sem amostras de tempo de ciclo")

    def wait_for_coppelia_topics(self, timeout=20.0):
        """Aguarda até que os tópicos de sensores estejam sendo publicados pelo CoppeliaSim."""
        required_source_topic = "/odom" if self.odom_source == "wheel" else "/imu/data"
        print(
            f"[INFO] Aguardando /scan e {required_source_topic} "
            f"para odometria {self.odom_source}...")
        received_odom = False
        received_scan = False
        received_imu = False

        probe_node = rclpy.create_node('coppelia_probe_node')

        def odom_cb(msg):
            nonlocal received_odom
            received_odom = True

        def scan_cb(msg):
            nonlocal received_scan
            received_scan = True

        def imu_cb(msg):
            nonlocal received_imu
            received_imu = True

        probe_node.create_subscription(Odometry, '/odom', odom_cb, 10)
        probe_node.create_subscription(LaserScan, '/scan', scan_cb, 10)
        probe_node.create_subscription(Imu, '/imu/data', imu_cb, 10)

        start = time.time()
        try:
            while time.time() - start < timeout:
                rclpy.spin_once(probe_node, timeout_sec=0.2)
                source_ready = received_odom if self.odom_source == "wheel" else received_imu
                if received_scan and source_ready:
                    break
        finally:
            probe_node.destroy_node()

        source_ready = received_odom if self.odom_source == "wheel" else received_imu
        if received_scan and source_ready:
            print(f"\033[92m[OK] CoppeliaSim pronto e publicando sensores ({time.time() - start:.1f}s)!\033[0m")
            return True
        else:
            print(
                "\033[93m[AVISO] Timeout aguardando tópicos do CoppeliaSim "
                f"(scan={received_scan}, imu={received_imu}, odom={received_odom}). "
                "Continuando...\033[0m")
            return False

    def wait_for_fastlio_odom(self, timeout=60.0):
        """Aguarda a saída da ponte FAST-LIO2 antes de iniciar o experimento."""
        received_odom = False
        probe_node = rclpy.create_node('fastlio_odom_probe_node')

        def odom_cb(_msg):
            nonlocal received_odom
            received_odom = True

        probe_node.create_subscription(Odometry, '/fastlio2/odom', odom_cb, 10)
        start = time.time()
        try:
            while time.time() - start < timeout and not received_odom:
                rclpy.spin_once(probe_node, timeout_sec=0.2)
        finally:
            probe_node.destroy_node()

        if received_odom:
            print(f"\033[92m[OK] FAST-LIO2 publicou /fastlio2/odom ({time.time() - start:.1f}s)!\033[0m")
            return True

        print(
            "\033[91m[ERRO] FAST-LIO2 não publicou /fastlio2/odom "
            f"em {timeout:.0f}s; verifique IMU, FAST-LIO2 e a bridge.\033[0m")
        return False

    def run_single_experiment(self, exp_idx):
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        bag_name = f"exp_{exp_idx:02d}_{timestamp}"
        bag_output_path = os.path.join(self.bags_dir, bag_name)

        print("\n" + "=" * 80)
        print(f"\033[1;32m>>> INICIANDO EXPERIMENTO {exp_idx}/{self.num_experiments} <<<\033[0m")
        random_seed = secrets.randbits(32)
        print(f"  • Seed compartilhada pelos 4 métodos: {random_seed}")
        print(f"  • Bag de destino: {bag_output_path}")
        print("=" * 80)

        # 1. Iniciar CoppeliaSim
        coppelia_cmd = []
        if self.headless and self.xvfb_available:
            print("[INFO] Modo headless ativado com xvfb-run.")
            coppelia_cmd = ["xvfb-run", "-a", self.coppelia_bin, "-s0", self.scene_path]
        else:
            if self.headless and not self.xvfb_available:
                print("[AVISO] xvfb-run não instalado. O CoppeliaSim rodará com GUI minimizada/direta.")
                print("        (Para modo 100% headless sem janela, instale: sudo apt install xvfb)")
            coppelia_cmd = [self.coppelia_bin, "-s0", self.scene_path]

        coppelia_proc = self.start_process(coppelia_cmd, "CoppeliaSim")
        self.wait_for_coppelia_topics(timeout=25.0)

        # 2. Iniciar os 4 algoritmos de localização sem RViz
        launch_file = (
            "all_localizations_fastlio2_launch.py"
            if self.odom_source == "fastlio2"
            else "all_localizations_launch.py"
        )
        loc_cmd = [
            "ros2", "launch", "montecarlo_localization", launch_file,
            f"random_seed:={random_seed}"
        ]
        loc_proc = self.start_process(loc_cmd, f"All_Localizations ({self.odom_source})")
        time.sleep(3.0)

        if self.odom_source == "fastlio2" and not self.wait_for_fastlio_odom():
            self.terminate_all_processes()
            return False, "fastlio2_odom_unavailable", bag_output_path, random_seed

        # 3. Iniciar gravação do Rosbag2 (todos os tópicos)
        bag_cmd = ["ros2", "bag", "record", "-a", "-o", bag_output_path]
        bag_proc = self.start_process(bag_cmd, f"Rosbag2 ({bag_name})")

        # 4. Iniciar nó monitor e publicador ativo de /path
        monitor_node = ExperimentMonitorNode(
            path_msg=self.path_msg,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            goal_tolerance=0.25,
            min_run_time=30.0
        )
        monitor_executor = SingleThreadedExecutor()
        monitor_executor.add_node(monitor_node)
        monitor_thread = threading.Thread(target=monitor_executor.spin, daemon=True)
        monitor_thread.start()

        # 5. Esperar os ~5 segundos conforme requisito
        print(f"[INFO] Aguardando {self.wait_before_fbl:.1f}s antes de iniciar controle FBL...")
        time.sleep(self.wait_before_fbl)

        # 6. Iniciar controlador de seguimento de trajetória (FBL)
        fbl_cmd = [
            "ros2", "launch", "diffdrive_fbl_control", "husky_coppelia_path_launch.py",
            f"path_file:={self.path_file}"
        ]
        fbl_proc = self.start_process(fbl_cmd, "DiffDrive_FBL_Control", show_output=True)

        # 7. Monitorar conclusão do experimento
        print("[INFO] Monitorando robô na trajetória...")
        start_time = time.time()
        completed = False
        reason = "desconhecido"

        try:
            try:
                while rclpy.ok() and not self._user_interrupted:
                    time.sleep(0.2)
                    elapsed = time.time() - start_time

                    # Log periódico a cada 15s
                    if int(elapsed) > 0 and int(elapsed) % 15 == 0 and abs(elapsed - int(elapsed)) < 0.25:
                        print(f"  [{int(elapsed)}s] Distância ao objetivo: {monitor_node.last_dist_to_goal:.2f} m | Pos: ({monitor_node.last_x:.2f}, {monitor_node.last_y:.2f})")

                    if monitor_node.goal_reached:
                        print(f"\n\033[92m[SUCESSO] Robô atingiu o objetivo final! (Dist: {monitor_node.last_dist_to_goal:.2f} m em {elapsed:.1f}s)\033[0m")
                        completed = True
                        reason = "goal_reached"
                        break

                    if monitor_node.is_stopped:
                        print(f"\n\033[92m[SUCESSO] Trajetória concluída e robô parado! (Dist: {monitor_node.last_dist_to_goal:.2f} m em {elapsed:.1f}s)\033[0m")
                        completed = True
                        reason = "path_completed_and_stopped"
                        break

                    if elapsed > self.timeout:
                        print(f"\n\033[93m[TIMEOUT] Experimento atingiu o tempo limite de {self.timeout}s.\033[0m")
                        completed = False
                        reason = "timeout"
                        break

            except KeyboardInterrupt:
                self._user_interrupted = True
                raise

            if not self._user_interrupted:
                # Buffer de 4s para gravar a convergência final
                print("[INFO] Gravando 4 segundos adicionais para registro da pose estática final...")
                time.sleep(4.0)

            # 8. Finalização ordenada dos processos
            print("[INFO] Finalizando processos do experimento...")
            self.stop_process(loc_proc, f"All_Localizations ({self.odom_source})")
            self.terminate_all_processes()
        finally:
            monitor_executor.shutdown()
            monitor_thread.join(timeout=2.0)
            monitor_executor.remove_node(monitor_node)
            monitor_node.destroy_node()

        self.save_computation_metrics(bag_output_path)

        # Cooldown para liberação de portas e memória
        time.sleep(3.0)

        print(f"\033[1;32m[FIM EXPERIMENTO {exp_idx}]\033[0m Status: {reason} | Bag salvo: {bag_output_path}\n")
        return completed, reason, bag_output_path, random_seed

    def run_all(self):
        rclpy.init()
        results = []
        try:
            for i in range(self.start_idx, self.num_experiments + 1):
                if self._user_interrupted:
                    break
                success, reason, bag_path, random_seed = self.run_single_experiment(i)
                results.append((i, success, reason, bag_path, random_seed))
        except KeyboardInterrupt:
            print("\n[INFO] Execução cancelada pelo usuário.")
        finally:
            self.terminate_all_processes()
            if rclpy.ok():
                rclpy.shutdown()

        # Relatório Final
        if results:
            print("\n" + "=" * 80)
            print("\033[1;34m=== RELATÓRIO FINAL DOS EXPERIMENTOS ===\033[0m")
            print("=" * 80)
            for idx, success, reason, bag_path, random_seed in results:
                status_str = "\033[92mSUCESSO\033[0m" if success else "\033[91mFALHA/TIMEOUT\033[0m"
                print(f"Exp {idx:02d}: {status_str} ({reason}) | seed={random_seed} -> {bag_path}")
            print("=" * 80)


def main():
    parser = argparse.ArgumentParser(description="Automação de 50 experimentos de localização no CoppeliaSim.")
    parser.add_argument("--num-experiments", type=int, default=55, help="Número total de experimentos (padrão: 50)")
    parser.add_argument("--start-idx", type=int, default=1, help="Índice inicial do experimento (padrão: 1)")
    parser.add_argument("--bags-dir", type=str, default="~/Documents/BagsRecordings_RP", help="Diretório onde salvar os bags")
    parser.add_argument("--coppelia-bin", type=str, default="~/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu22_04/coppeliaSim.sh", help="Executável do CoppeliaSim")
    parser.add_argument("--scene-path", type=str, default="/home/davidsimon/ros2_ws/src/montecarlo_localization/maps/simulation/Factory.ttt", help="Caminho do Factory.ttt")
    parser.add_argument("--path-file", type=str, default="~/Documents/BagsRecordings_PADE_RandomSeeds/poses_amostradas.txt", help="Arquivo com poses do caminho")
    parser.add_argument(
        "--odom-source",
        choices=("wheel", "fastlio2"),
        default="wheel",
        help="Fonte de odometria dos filtros de partículas (padrão: wheel)")
    parser.add_argument("--timeout", type=float, default=360.0, help="Timeout de segurança por experimento em segundos (padrão: 360s)")
    parser.add_argument("--wait-before-fbl", type=float, default=5.0, help="Tempo de espera em segundos antes de iniciar o controle FBL (padrão: 5.0s)")
    parser.add_argument("--headless", action="store_true", default=True, help="Executar headless se xvfb disponível (padrão: True)")
    parser.add_argument("--no-headless", action="store_false", dest="headless", help="Desativar modo headless e usar GUI diretamente")

    args = parser.parse_args()
    runner = AutomatedExperimentRunner(args)
    runner.run_all()


if __name__ == "__main__":
    main()
