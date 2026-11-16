#!/usr/bin/env python3
"""
Post-process automated localization experiments.

Script para pós-processamento automatizado de experimentos de localização (ROS 2 Bags):
1. Varre todas as pastas que iniciam com 'exp_' no diretório informado (ou atual).
2. Para cada experimento válido:
   - Extrai as poses absolutas (/ground_truth ou /pose_ground_truth) e dos métodos:
     MCL (/mcl_pose), AMCL (/amcl_pose), MH-MCL (/mhmcl_pose) e MH-AMCL (/mhamcl_pose).
   - Realiza interpolação temporal e calcula as métricas quantitativas de erro:
     RMSE_pos (métrica de posição) e RMSE_yaw (métrica de orientação em graus e radianos).
   - Gera na pasta do experimento:
     * trajetoria_2d.png: Plot 2D da trajetória de cada método e da pose absoluta.
     * rmse_posicao.png: Plot quantitativo do erro de posição ao longo do tempo.
     * rmse_orientacao.png: Plot quantitativo do erro de orientação (yaw) ao longo do tempo.
     * metricas_experimento.xlsx: Planilha Excel com 3 abas ('Datalogs_Poses', 'RMSE_pos', 'RMSE_yaw').
     * datalogs_poses.csv, rmse_pos.csv, rmse_yaw.csv: Arquivos CSV equivalentes.
3. Se o bag não tiver conteúdo válido ou faltar tópicos essenciais, emite mensagem de erro e pula.
4. Ao final, gera um relatório consolidado com médias globais de todos os experimentos.
"""

import argparse
import glob
import math
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ROS 2 imports
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print(f"\033[91m[ERRO CRÍTICO] Módulos do ROS 2 não encontrados: {e}\033[0m")
    print("Certifique-se de executar: source /opt/ros/<distro>/setup.bash")
    sys.exit(1)


# Paleta de cores consistente para os métodos
COLORS = {
    'gt': '#000000',      # Preto para Ground Truth
    'mcl': '#1f77b4',     # Azul para MCL
    'amcl': '#2ca02c',    # Verde para AMCL
    'mhmcl': '#d62728',   # Vermelho para MH-MCL
    'mhamcl': '#9467bd'   # Roxo para MH-AMCL
}

LABELS = {
    'gt': 'Ground Truth (Absoluto)',
    'mcl': 'MCL',
    'amcl': 'AMCL',
    'mhmcl': 'MH-MCL',
    'mhamcl': 'MH-AMCL'
}


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Calcula o ângulo yaw (rotação em torno de Z) a partir de um quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    """Normaliza o ângulo no intervalo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def read_experiment_bag(bag_dir: str):
    """
    Abre o rosbag em bag_dir e extrai séries temporais de poses.

    Retorna (dicionário de poses, mensagem de erro se houver).
    """
    if not os.path.isdir(bag_dir):
        return None, f"Diretório não encontrado: {bag_dir}"

    db3_files = list(Path(bag_dir).glob("*.db3"))
    meta_file = Path(bag_dir) / "metadata.yaml"
    if not db3_files and not meta_file.exists():
        return None, f"Nenhum banco SQLite (.db3) ou metadata.yaml encontrado em '{os.path.basename(bag_dir)}'"

    try:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader.open(storage_options, converter_options)
    except Exception as e:
        return None, f"Falha ao abrir rosbag: {e}"

    all_topics = reader.get_all_topics_and_types()
    topic_names = {t.name: t.type for t in all_topics}

    # Identificar tópico de ground truth
    gt_topic = None
    if "/ground_truth" in topic_names:
        gt_topic = "/ground_truth"
    elif "/pose_ground_truth" in topic_names:
        gt_topic = "/pose_ground_truth"

    if gt_topic is None:
        return None, "Tópico de pose absoluta (/ground_truth ou /pose_ground_truth) ausente no bag"

    methods_topics = {
        'mcl': "/mcl_pose",
        'amcl': "/amcl_pose",
        'mhmcl': "/mhmcl_pose",
        'mhamcl': "/mhamcl_pose"
    }

    # Verificar presença dos métodos
    available_methods = {}
    for key, top in methods_topics.items():
        if top in topic_names:
            available_methods[key] = top

    if not available_methods:
        return None, "Nenhum tópico de estimativa (/mcl_pose, /amcl_pose, /mhmcl_pose, /mhamcl_pose) presente no bag"

    # Preparar classes de mensagem
    type_map = {}
    type_map[gt_topic] = get_message(topic_names[gt_topic])
    for key, top in available_methods.items():
        type_map[top] = get_message(topic_names[top])

    all_filter_topics = [gt_topic] + list(available_methods.values())
    storage_filter = rosbag2_py.StorageFilter(topics=all_filter_topics)
    reader.set_filter(storage_filter)

    poses_data = {'gt': []}
    for key in available_methods.keys():
        poses_data[key] = []

    poses_header_data = {'gt': []}
    for key in available_methods.keys():
        poses_header_data[key] = []

    while reader.has_next():
        t_name, raw_data, bag_ts = reader.read_next()
        if t_name in type_map:
            msg = deserialize_message(raw_data, type_map[t_name])
            t_sec = bag_ts * 1e-9
            h_sec = 0.0
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                h_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
            elif hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
                p = msg.pose.position
                q = msg.pose.orientation
            else:
                continue

            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

            if t_name == gt_topic:
                poses_data['gt'].append((t_sec, p.x, p.y, yaw))
                if h_sec > 0.0:
                    poses_header_data['gt'].append((h_sec, p.x, p.y, yaw))
            else:
                for k, top in available_methods.items():
                    if t_name == top:
                        poses_data[k].append((t_sec, p.x, p.y, yaw))
                        if h_sec > 0.0:
                            poses_header_data[k].append((h_sec, p.x, p.y, yaw))
                        break

    # Usar header.stamp sincronizado com o sensor quando disponível e consistente no mesmo domínio de tempo
    if len(poses_header_data['gt']) >= len(poses_data['gt']) * 0.8 and len(poses_header_data['gt']) >= 5:
        gt_h0 = poses_header_data['gt'][0][0]
        gt_h1 = poses_header_data['gt'][-1][0]
        header_consistent = (gt_h1 > gt_h0)
        for k in available_methods.keys():
            if len(poses_header_data[k]) >= 5:
                m_h0 = poses_header_data[k][0][0]
                if abs(m_h0 - gt_h0) > 600.0:
                    header_consistent = False
                    break
            elif len(poses_data[k]) >= 5:
                header_consistent = False
                break
        if header_consistent:
            poses_data = poses_header_data

    if len(poses_data['gt']) < 5:
        return None, f"Poucas leituras de ground truth no bag ({len(poses_data['gt'])} mensagens gravadas)"

    valid_methods = 0
    for k in available_methods.keys():
        if len(poses_data[k]) >= 5:
            valid_methods += 1

    if valid_methods == 0:
        return None, "Todos os tópicos de estimativa possuem menos de 5 mensagens gravadas"

    return poses_data, None


def compute_metrics(poses_data):
    """
    Sincroniza os métodos com o Ground Truth e calcula erros de posição e yaw.

    Retorna métricas consolidadas, dados sincronizados e séries de erros.
    """
    gt_arr = np.array(poses_data['gt'])
    t_gt = gt_arr[:, 0]
    x_gt = gt_arr[:, 1]
    y_gt = gt_arr[:, 2]
    yaw_gt_unwrapped = np.unwrap(gt_arr[:, 3])

    t0 = t_gt[0]
    t_end = t_gt[-1]

    # Malha temporal comum (10 Hz)
    dt = 0.1
    t_common = np.arange(t0, t_end + dt, dt)
    t_rel = t_common - t0

    # Interpolar GT na malha comum
    x_gt_common = np.interp(t_common, t_gt, x_gt)
    y_gt_common = np.interp(t_common, t_gt, y_gt)
    yaw_gt_common = np.array([wrap_angle(y) for y in np.interp(t_common, t_gt, yaw_gt_unwrapped)])

    datalog = {
        'Timestamp_s': t_common,
        'Tempo_decorrido_s': t_rel,
        'GT_X_m': x_gt_common,
        'GT_Y_m': y_gt_common,
        'GT_Yaw_rad': yaw_gt_common,
        'GT_Yaw_deg': np.degrees(yaw_gt_common)
    }

    metrics_pos_summary = []
    metrics_yaw_summary = []
    error_series_pos = {'Tempo_s': t_rel}
    error_series_yaw = {'Tempo_s': t_rel}

    for method in ['mcl', 'amcl', 'mhmcl', 'mhamcl']:
        if method not in poses_data or len(poses_data[method]) < 2:
            continue

        m_arr = np.array(poses_data[method])
        t_m = m_arr[:, 0]
        x_m = m_arr[:, 1]
        y_m = m_arr[:, 2]
        yaw_m_unwrapped = np.unwrap(m_arr[:, 3])

        # Interpolação na malha temporal comum
        x_m_common = np.interp(t_common, t_m, x_m)
        y_m_common = np.interp(t_common, t_m, y_m)
        yaw_m_common = np.array([wrap_angle(y) for y in np.interp(t_common, t_m, yaw_m_unwrapped)])

        datalog[f'{method.upper()}_X_m'] = x_m_common
        datalog[f'{method.upper()}_Y_m'] = y_m_common
        datalog[f'{method.upper()}_Yaw_rad'] = yaw_m_common
        datalog[f'{method.upper()}_Yaw_deg'] = np.degrees(yaw_m_common)

        # Erro de posição
        e_pos = np.sqrt((x_m_common - x_gt_common) ** 2 + (y_m_common - y_gt_common) ** 2)
        rmse_pos = float(np.sqrt(np.mean(e_pos ** 2)))
        mean_pos = float(np.mean(e_pos))
        std_pos = float(np.std(e_pos))
        max_pos = float(np.max(e_pos))
        min_pos = float(np.min(e_pos))

        error_series_pos[f'Erro_{method.upper()}_m'] = e_pos

        metrics_pos_summary.append({
            'Metodo': LABELS[method],
            'RMSE_pos_m': rmse_pos,
            'Erro_Medio_m': mean_pos,
            'Desvio_Padrao_m': std_pos,
            'Erro_Max_m': max_pos,
            'Erro_Min_m': min_pos
        })

        # Erro de orientação (Yaw)
        e_yaw_rad = np.array([abs(wrap_angle(ym - ygt)) for ym, ygt in zip(yaw_m_common, yaw_gt_common)])
        e_yaw_deg = np.degrees(e_yaw_rad)

        rmse_yaw_rad = float(np.sqrt(np.mean(e_yaw_rad ** 2)))
        rmse_yaw_deg = float(np.sqrt(np.mean(e_yaw_deg ** 2)))
        mean_yaw_deg = float(np.mean(e_yaw_deg))
        std_yaw_deg = float(np.std(e_yaw_deg))
        max_yaw_deg = float(np.max(e_yaw_deg))
        min_yaw_deg = float(np.min(e_yaw_deg))

        error_series_yaw[f'Erro_{method.upper()}_deg'] = e_yaw_deg
        error_series_yaw[f'Erro_{method.upper()}_rad'] = e_yaw_rad

        metrics_yaw_summary.append({
            'Metodo': LABELS[method],
            'RMSE_yaw_deg': rmse_yaw_deg,
            'RMSE_yaw_rad': rmse_yaw_rad,
            'Erro_Medio_deg': mean_yaw_deg,
            'Desvio_Padrao_deg': std_yaw_deg,
            'Erro_Max_deg': max_yaw_deg,
            'Erro_Min_deg': min_yaw_deg
        })

    df_datalog = pd.DataFrame(datalog)
    df_pos_summary = pd.DataFrame(metrics_pos_summary)
    df_pos_series = pd.DataFrame(error_series_pos)
    df_yaw_summary = pd.DataFrame(metrics_yaw_summary)
    df_yaw_series = pd.DataFrame(error_series_yaw)

    return df_datalog, df_pos_summary, df_pos_series, df_yaw_summary, df_yaw_series


def generate_plots(exp_dir: str, exp_name: str, poses_data, df_pos_summary, df_pos_series, df_yaw_summary, df_yaw_series):
    """Gera e salva os 3 gráficos solicitados com alta qualidade visual."""
    # -------------------------------------------------------------
    # 1. Plot 2D das Trajetórias (X vs Y)
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)

    # Plot Ground Truth
    gt_arr = np.array(poses_data['gt'])
    ax.plot(gt_arr[:, 1], gt_arr[:, 2], label=LABELS['gt'], color=COLORS['gt'], linewidth=2.5, zorder=5)

    # Plot dos Métodos
    for method in ['mcl', 'amcl', 'mhmcl', 'mhamcl']:
        if method in poses_data and len(poses_data[method]) > 0:
            m_arr = np.array(poses_data[method])
            ax.plot(m_arr[:, 1], m_arr[:, 2], label=LABELS[method], color=COLORS[method], linewidth=1.5, alpha=0.85)

    # Marcadores de Início e Fim da trajetória real
    ax.scatter(gt_arr[0, 1], gt_arr[0, 2], color='lime', edgecolors='black', s=120, zorder=6, label='Início (Start)')
    ax.scatter(gt_arr[-1, 1], gt_arr[-1, 2], color='red', edgecolors='black', marker='s', s=120, zorder=6, label='Fim (End)')

    ax.set_title(f"Trajetória 2D no Plano XY - {exp_name}", fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel("Posição X (metros)", fontsize=12)
    ax.set_ylabel("Posição Y (metros)", fontsize=12)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='best', framealpha=0.9, fontsize=10)

    plot_2d_path = os.path.join(exp_dir, "trajetoria_2d.png")
    fig.tight_layout()
    fig.savefig(plot_2d_path)
    plt.close(fig)

    # -------------------------------------------------------------
    # 2. Plot Quantitativo de Erro de Posição (RMSE_pos) ao Longo do Tempo
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 6), dpi=300)

    t_series = df_pos_series['Tempo_s'].to_numpy()
    for method in ['mcl', 'amcl', 'mhmcl', 'mhamcl']:
        col_name = f'Erro_{method.upper()}_m'
        if col_name in df_pos_series.columns:
            m_row = df_pos_summary[df_pos_summary['Metodo'] == LABELS[method]]
            rmse_val = m_row['RMSE_pos_m'].values[0] if not m_row.empty else 0.0

            ax.plot(
                t_series,
                df_pos_series[col_name].to_numpy(),
                label=f"{LABELS[method]} (RMSE: {rmse_val:.3f} m)",
                color=COLORS[method],
                linewidth=1.6
            )
            ax.axhline(rmse_val, color=COLORS[method], linestyle=':', alpha=0.5, linewidth=1.2)

    ax.set_title(f"Métrica Quantitativa: Erro de Posição (RMSE_pos) - {exp_name}", fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel("Tempo Decorrido (segundos)", fontsize=12)
    ax.set_ylabel("Erro de Posição (metros)", fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_ylim(bottom=0.0)
    ax.legend(loc='upper right', framealpha=0.9, fontsize=10)

    plot_pos_path = os.path.join(exp_dir, "rmse_posicao.png")
    fig.tight_layout()
    fig.savefig(plot_pos_path)
    plt.close(fig)

    # -------------------------------------------------------------
    # 3. Plot Quantitativo de Erro de Orientação (RMSE_yaw) ao Longo do Tempo
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 6), dpi=300)

    t_series = df_yaw_series['Tempo_s'].to_numpy()
    for method in ['mcl', 'amcl', 'mhmcl', 'mhamcl']:
        col_name = f'Erro_{method.upper()}_deg'
        if col_name in df_yaw_series.columns:
            m_row = df_yaw_summary[df_yaw_summary['Metodo'] == LABELS[method]]
            rmse_deg = m_row['RMSE_yaw_deg'].values[0] if not m_row.empty else 0.0
            rmse_rad = m_row['RMSE_yaw_rad'].values[0] if not m_row.empty else 0.0

            ax.plot(
                t_series,
                df_yaw_series[col_name].to_numpy(),
                label=f"{LABELS[method]} (RMSE: {rmse_deg:.2f}° | {rmse_rad:.3f} rad)",
                color=COLORS[method],
                linewidth=1.6
            )
            ax.axhline(rmse_deg, color=COLORS[method], linestyle=':', alpha=0.5, linewidth=1.2)

    ax.set_title(f"Métrica Quantitativa: Erro de Orientação (RMSE_yaw) - {exp_name}", fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel("Tempo Decorrido (segundos)", fontsize=12)
    ax.set_ylabel("Erro de Orientação / Yaw (graus)", fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_ylim(bottom=0.0)
    ax.legend(loc='upper right', framealpha=0.9, fontsize=10)

    plot_yaw_path = os.path.join(exp_dir, "rmse_orientacao.png")
    fig.tight_layout()
    fig.savefig(plot_yaw_path)
    plt.close(fig)

    return plot_2d_path, plot_pos_path, plot_yaw_path


def save_spreadsheets(exp_dir: str, df_datalog, df_pos_summary, df_pos_series, df_yaw_summary, df_yaw_series):
    """
    Gera a planilha Excel padronizada (.xlsx).

    Cada aba contém exatamente uma tabela limpa:
    - 'Datalogs_Poses': Séries temporais de poses (GT e métodos a 10 Hz)
    - 'RMSE_pos': Tabela resumo quantitativa de métricas de posição por método
    - 'RMSE_yaw': Tabela resumo quantitativa de métricas de orientação por método
    - 'Serie_Erro_pos': Série temporal do erro instantâneo de posição ao longo do tempo
    - 'Serie_Erro_yaw': Série temporal do erro instantâneo de yaw ao longo do tempo

    Gera também arquivos CSV individuais limpos (1 cabeçalho na linha 1), ideais para leitura automatizada:
    - datalogs_poses.csv
    - rmse_pos.csv
    - rmse_yaw.csv
    - serie_erro_pos.csv
    - serie_erro_yaw.csv
    """
    excel_path = os.path.join(exp_dir, "metricas_experimento.xlsx")

    with pd.ExcelWriter(excel_path, engine='openpyxl') as writer:
        df_datalog.to_excel(writer, sheet_name='Datalogs_Poses', index=False)
        df_pos_summary.to_excel(writer, sheet_name='RMSE_pos', index=False)
        df_yaw_summary.to_excel(writer, sheet_name='RMSE_yaw', index=False)
        df_pos_series.to_excel(writer, sheet_name='Serie_Erro_pos', index=False)
        df_yaw_series.to_excel(writer, sheet_name='Serie_Erro_yaw', index=False)

    csv_datalog_path = os.path.join(exp_dir, "datalogs_poses.csv")
    csv_pos_path = os.path.join(exp_dir, "rmse_pos.csv")
    csv_yaw_path = os.path.join(exp_dir, "rmse_yaw.csv")
    csv_pos_ser_path = os.path.join(exp_dir, "serie_erro_pos.csv")
    csv_yaw_ser_path = os.path.join(exp_dir, "serie_erro_yaw.csv")

    df_datalog.to_csv(csv_datalog_path, index=False)
    df_pos_summary.to_csv(csv_pos_path, index=False)
    df_yaw_summary.to_csv(csv_yaw_path, index=False)
    df_pos_series.to_csv(csv_pos_ser_path, index=False)
    df_yaw_series.to_csv(csv_yaw_ser_path, index=False)

    return excel_path, [csv_datalog_path, csv_pos_path, csv_yaw_path, csv_pos_ser_path, csv_yaw_ser_path]


def process_all_experiments(base_dir: str):
    base_path = Path(base_dir).expanduser().resolve()
    print("=" * 80)
    print(f"\033[1;34m=== INICIANDO PROCESSAMENTO DOS EXPERIMENTOS ===\033[0m")
    print(f"Diretório base: {base_path}")
    print("=" * 80)

    if not base_path.exists():
        print(f"\033[91m[ERRO] O diretório '{base_path}' não existe.\033[0m")
        return

    # Buscar todas as pastas que começam com 'exp_'
    exp_folders = sorted([f for f in base_path.iterdir() if f.is_dir() and f.name.startswith("exp_")])

    if not exp_folders:
        print(f"\033[93m[AVISO] Nenhuma pasta iniciando com 'exp_' foi encontrada em '{base_path}'.\033[0m")
        return

    print(f"Encontradas {len(exp_folders)} pastas de experimentos para análise.\n")

    successful_runs = []
    failed_runs = []

    for idx, folder in enumerate(exp_folders, 1):
        folder_name = folder.name
        folder_str = str(folder)
        print(f"[{idx}/{len(exp_folders)}] Processando: \033[1;36m{folder_name}\033[0m ...")

        poses_data, error_msg = read_experiment_bag(folder_str)

        if error_msg is not None:
            print(f"  \033[91m[ERRO NO BAG] {error_msg}. Pulando pasta...\033[0m\n")
            failed_runs.append((folder_name, error_msg))
            continue

        try:
            # Cálculo de Métricas
            df_datalog, df_pos_sum, df_pos_ser, df_yaw_sum, df_yaw_ser = compute_metrics(poses_data)

            # Geração dos 3 Plots
            p_2d, p_pos, p_yaw = generate_plots(
                folder_str, folder_name, poses_data, df_pos_sum, df_pos_ser, df_yaw_sum, df_yaw_ser
            )

            # Gravação de Planilhas Excel (.xlsx) e CSVs
            excel_file, csv_files = save_spreadsheets(
                folder_str, df_datalog, df_pos_sum, df_pos_ser, df_yaw_sum, df_yaw_ser
            )

            # Exibir resultados resumidos no console
            print("  \033[92m[SUCESSO] Métricas calculadas e arquivos gerados:\033[0m")
            for _, row in df_pos_sum.iterrows():
                m_name = row['Metodo']
                r_pos = row['RMSE_pos_m']
                yaw_row = df_yaw_sum[df_yaw_sum['Metodo'] == m_name]
                r_yaw = yaw_row['RMSE_yaw_deg'].values[0] if not yaw_row.empty else 0.0
                print(f"    • {m_name:10s} -> RMSE_pos: {r_pos:6.3f} m | RMSE_yaw: {r_yaw:6.2f}°")

            print(f"  Arquivos salvos:")
            print(f"    - Plots:       {os.path.basename(p_2d)}, {os.path.basename(p_pos)}, {os.path.basename(p_yaw)}")
            print(f"    - Planilha:    {os.path.basename(excel_file)} (5 abas padronizadas)")
            print(f"    - CSVs:        datalogs_poses.csv, rmse_pos.csv, rmse_yaw.csv, serie_erro_pos.csv, serie_erro_yaw.csv\n")

            successful_runs.append({
                'experimento': folder_name,
                'df_pos': df_pos_sum,
                'df_yaw': df_yaw_sum
            })

        except Exception as e:
            print(f"  \033[91m[ERRO DE PROCESSAMENTO] Falha ao processar métricas de '{folder_name}': {e}\033[0m\n")
            failed_runs.append((folder_name, str(e)))

    # -------------------------------------------------------------
    # Relatório Final Consolidado
    # -------------------------------------------------------------
    print("=" * 80)
    print("\033[1;34m=== RELATÓRIO FINAL CONSOLIDADO ===\033[0m")
    print("=" * 80)
    print(f"Total de pastas analisadas: {len(exp_folders)}")
    print(f"Experimentos com sucesso:  \033[92m{len(successful_runs)}\033[0m")
    print(f"Experimentos com falha:    \033[91m{len(failed_runs)}\033[0m\n")

    if failed_runs:
        print("\033[91mPastas com erro / ignoradas:\033[0m")
        for fname, err in failed_runs:
            print(f"  • {fname}: {err}")
        print()

    if successful_runs:
        summary_rows = []
        for run in successful_runs:
            exp_id = run['experimento']
            for _, r in run['df_pos'].iterrows():
                m_label = r['Metodo']
                rmse_p = r['RMSE_pos_m']
                yaw_r = run['df_yaw'][run['df_yaw']['Metodo'] == m_label]
                rmse_y = yaw_r['RMSE_yaw_deg'].values[0] if not yaw_r.empty else 0.0
                summary_rows.append({
                    'Experimento': exp_id,
                    'Metodo': m_label,
                    'RMSE_pos_m': rmse_p,
                    'RMSE_yaw_deg': rmse_y
                })

        df_global = pd.DataFrame(summary_rows)

        print("\033[1;32mMÉDIA GLOBAL DOS EXPERIMENTOS (MÉTRICAS MÉDIAS):\033[0m")
        grouped = df_global.groupby('Metodo')[['RMSE_pos_m', 'RMSE_yaw_deg']].agg(['mean', 'std'])
        print(grouped.to_string())

        # Salvar resumo global no diretório base
        resumo_csv_path = os.path.join(base_path, "resumo_geral_experimentos.csv")
        resumo_xlsx_path = os.path.join(base_path, "resumo_geral_experimentos.xlsx")
        df_global.to_csv(resumo_csv_path, index=False)
        with pd.ExcelWriter(resumo_xlsx_path, engine='openpyxl') as writer:
            df_global.to_excel(writer, sheet_name='Todos_Experimentos', index=False)
            grouped.to_excel(writer, sheet_name='Medias_Globais')

        print(f"\nResumo consolidado global salvo em:")
        print(f"  • {resumo_csv_path}")
        print(f"  • {resumo_xlsx_path}")

    print("=" * 80)


def main():
    default_dir = "/home/davidsimon/Documents/BagsRecordings_PADE"
    if not os.path.exists(default_dir):
        default_dir = os.getcwd()

    parser = argparse.ArgumentParser(description="Pós-processamento de experimentos de localização nos bags ROS 2.")
    parser.add_argument("diretorio", nargs="?", default=default_dir,
                        help=f"Diretório raiz contendo as pastas exp_* (padrão: {default_dir})")

    args = parser.parse_args()
    process_all_experiments(args.diretorio)


if __name__ == "__main__":
    main()
