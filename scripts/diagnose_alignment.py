#!/usr/bin/env python3
# ==============================================================================
# DIAGNÓSTICO DE ALINHAMENTO DO MCL E DO SCAN
# ==============================================================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import os
import yaml
from PIL import Image

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('mcl_diagnose_alignment')
        
        # Suporta tanto o novo /ground_truth (Odometry) quanto o antigo /pose_ground_truth (PoseStamped)
        self.gt_odom_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_odom_cb, 10)
        self.gt_pose_sub = self.create_subscription(PoseStamped, '/pose_ground_truth', self.gt_pose_cb, 10)
        
        self.mcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/mcl_pose', self.mcl_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        self.gt_pose = None
        self.mcl_pose = None
        self.odom_pose = None
        self.latest_scan = None
        self.map_data = None

    def gt_odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.gt_pose = (p.x, p.y, yaw)

    def gt_pose_cb(self, msg):
        p = msg.pose.position
        yaw = quaternion_to_yaw(msg.pose.orientation)
        self.gt_pose = (p.x, p.y, yaw)

    def mcl_cb(self, msg):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.mcl_pose = (p.x, p.y, yaw)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_pose = (p.x, p.y, yaw)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def map_cb(self, msg):
        self.map_data = msg

def main():
    rclpy.init()
    node = DiagnosticNode()
    
    print("Aguardando mensagens dos tópicos (/ground_truth, /mcl_pose, /odom, /scan, /map)...")
    
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.gt_pose and node.latest_scan:
            break

    if not node.gt_pose or not node.latest_scan:
        print("[ERRO] Nao foi possivel receber dados da simulacao. Verifique se o CoppeliaSim esta rodando (Play).")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Fallback para carregar o mapa diretamente caso o map_server não esteja ativo
    if not node.map_data:
        map_candidates = [
            '/home/davidsimon/ros2_ws/src/montecarlo_localization/maps/map_factory.yaml',
            '/home/davidsimon/ros2_ws/src/montecarlo_localization/maps/map_gazebo_world.yaml'
        ]
        for map_yaml in map_candidates:
            if os.path.exists(map_yaml):
                try:
                    with open(map_yaml, 'r') as f:
                        cfg = yaml.safe_load(f)
                    pgm_path = os.path.join(os.path.dirname(map_yaml), cfg['image'])
                    if os.path.exists(pgm_path):
                        img = Image.open(pgm_path)
                        w, h = img.size
                        arr = np.flipud(np.array(img))
                        grid = np.zeros_like(arr, dtype=np.int8)
                        grid[arr < 128] = 100
                        
                        class DummyObj: pass
                        dummy_map = DummyObj()
                        dummy_map.info = DummyObj()
                        dummy_map.info.width = w
                        dummy_map.info.height = h
                        dummy_map.info.resolution = cfg['resolution']
                        dummy_map.info.origin = DummyObj()
                        dummy_map.info.origin.position = DummyObj()
                        dummy_map.info.origin.position.x = cfg['origin'][0]
                        dummy_map.info.origin.position.y = cfg['origin'][1]
                        dummy_map.data = grid.flatten().tolist()
                        node.map_data = dummy_map
                        break
                except Exception:
                    pass

    print("\n" + "="*65)
    print("           RELATÓRIO DE DIAGNÓSTICO DO ROBÔ E SENSORES")
    print("="*65)

    # 1. Poses
    gt_x, gt_y, gt_yaw = node.gt_pose
    print(f"1. POSE REAL NO MUNDO (Ground Truth):")
    print(f"   X = {gt_x:.3f} m, Y = {gt_y:.3f} m, Yaw = {math.degrees(gt_yaw):.2f}° ({gt_yaw:.3f} rad)")

    if node.mcl_pose:
        m_x, m_y, m_yaw = node.mcl_pose
        err_x = m_x - gt_x
        err_y = m_y - gt_y
        err_dist = math.sqrt(err_x**2 + err_y**2)
        yaw_err = math.degrees(math.atan2(math.sin(m_yaw - gt_yaw), math.cos(m_yaw - gt_yaw)))
        print(f"\n2. POSE ESTIMADA PELO MCL (/mcl_pose):")
        print(f"   X = {m_x:.3f} m, Y = {m_y:.3f} m, Yaw = {math.degrees(m_yaw):.2f}°")
        print(f"   -> Erro de Posição MCL: {err_dist:.3f} metros | Erro Angular: {yaw_err:.2f}°")
    else:
        print("\n2. POSE ESTIMADA PELO MCL: [Ainda não publicada ou MCL não iniciado]")

    if node.odom_pose:
        o_x, o_y, o_yaw = node.odom_pose
        print(f"\n3. ODOMETRIA ATUAL (/odom):")
        print(f"   X = {o_x:.3f} m, Y = {o_y:.3f} m, Yaw = {math.degrees(o_yaw):.2f}°")

    # 4. Análise do Scan
    scan = node.latest_scan
    n_pts = len(scan.ranges)
    span_deg = math.degrees(n_pts * scan.angle_increment)
    min_deg = math.degrees(scan.angle_min)
    max_deg = math.degrees(scan.angle_max)

    print(f"\n4. PARÂMETROS DO LASERSCAN (/scan):")
    print(f"   Frame ID        : '{scan.header.frame_id}'")
    print(f"   Total de Pontos : {n_pts}")
    print(f"   angle_min       : {scan.angle_min:.4f} rad ({min_deg:.1f}°)")
    print(f"   angle_max       : {scan.angle_max:.4f} rad ({max_deg:.1f}°)")
    print(f"   angle_increment : {scan.angle_increment:.6f} rad ({math.degrees(scan.angle_increment):.3f}°)")
    print(f"   Abertura Total  : {span_deg:.1f}°")

    # Contagem de leituras reais vs máximas
    hits = [r for r in scan.ranges if r < (scan.range_max - 0.05)]
    print(f"   Pontos com obstáculo detectado (< {scan.range_max - 0.05:.2f}m): {len(hits)} de {n_pts}")

    # 5. Leituras e Verificação de Espelhamento (Esquerda vs Direita)
    def get_scan_at_angle(target_rad):
        idx = int(round((target_rad - scan.angle_min) / scan.angle_increment))
        if 0 <= idx < len(scan.ranges):
            return scan.ranges[idx], idx
        return float('nan'), -1

    scan_frente, idx_frente = get_scan_at_angle(0.0)
    scan_esq45, idx_esq45 = get_scan_at_angle(math.radians(45))
    scan_dir45, idx_dir45 = get_scan_at_angle(math.radians(-45))
    scan_esq90, idx_esq90 = get_scan_at_angle(math.radians(90))
    scan_dir90, idx_dir90 = get_scan_at_angle(math.radians(-90))

    print(f"\n5. LEITURAS DE DISTÂNCIA DO SCAN:")
    print(f"   Direita  (-90° / idx {idx_dir90:4d}) : {scan_dir90:.3f} m")
    print(f"   Dir-Diag (-45° / idx {idx_dir45:4d}) : {scan_dir45:.3f} m")
    print(f"   Frente   (  0° / idx {idx_frente:4d}) : {scan_frente:.3f} m")
    print(f"   Esq-Diag (+45° / idx {idx_esq45:4d}) : {scan_esq45:.3f} m")
    print(f"   Esquerda (+90° / idx {idx_esq90:4d}) : {scan_esq90:.3f} m")

    # Raycast no mapa para comparar com Ground Truth
    def cast_ray(start_x, start_y, angle, max_range=scan.range_max):
        if not node.map_data:
            return float('nan')
        res = node.map_data.info.resolution
        ox = node.map_data.info.origin.position.x
        oy = node.map_data.info.origin.position.y
        w = node.map_data.info.width
        h = node.map_data.info.height
        data = node.map_data.data
        step = res * 0.5
        dist = 0.0
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        while dist < max_range:
            dist += step
            cx = start_x + dist * cos_a
            cy = start_y + dist * sin_a
            mx = int((cx - ox) / res)
            my = int((cy - oy) / res)
            if mx < 0 or mx >= w or my < 0 or my >= h:
                return dist
            val = data[my * w + mx]
            if val >= 50:
                return dist
        return max_range

    map_frente = cast_ray(gt_x, gt_y, gt_yaw)
    map_dir45 = cast_ray(gt_x, gt_y, gt_yaw - math.radians(45))
    map_esq45 = cast_ray(gt_x, gt_y, gt_yaw + math.radians(45))
    map_dir90 = cast_ray(gt_x, gt_y, gt_yaw - math.radians(90))
    map_esq90 = cast_ray(gt_x, gt_y, gt_yaw + math.radians(90))

    print("\n" + "="*65)
    print("                 ANÁLISE E DIAGNÓSTICO DO CASAMENTO")
    print("="*65)
    print(f"-> Posição real do robô no mapa: ({gt_x:.2f}, {gt_y:.2f}, yaw={math.degrees(gt_yaw):.1f}°)")

    if node.map_data:
        print(f"\n-> COMPARAÇÃO MAPA REAL vs MEDIÇÃO DO LASER (na pose Ground Truth):")
        print(f"   Direita (-90°)  -> Esperado Mapa: {map_dir90:.2f} m | Medido Laser: {scan_dir90:.2f} m")
        print(f"   Dir-Diag (-45°) -> Esperado Mapa: {map_dir45:.2f} m | Medido Laser: {scan_dir45:.2f} m")
        print(f"   Frente (0°)     -> Esperado Mapa: {map_frente:.2f} m | Medido Laser: {scan_frente:.2f} m")
        print(f"   Esq-Diag (+45°) -> Esperado Mapa: {map_esq45:.2f} m | Medido Laser: {scan_esq45:.2f} m")
        print(f"   Esquerda (+90°) -> Esperado Mapa: {map_esq90:.2f} m | Medido Laser: {scan_esq90:.2f} m")

        # Análise de espelhamento
        err_normal = abs(scan_esq45 - map_esq45) + abs(scan_dir45 - map_dir45)
        err_mirrored = abs(scan_esq45 - map_dir45) + abs(scan_dir45 - map_esq45)

        print("\n-> CONCLUSÃO DO DIAGNÓSTICO:")
        if err_mirrored < (err_normal - 0.5):
            print("   [ALERTA: SCAN ESPELHADO]")
            print("   O Laser está invertido (Esquerda é Direita e vice-versa).")
        elif abs(scan_frente - map_frente) < 0.3 and abs(scan_esq45 - map_esq45) < 0.3:
            print("   [SUCESSO: SCAN PERFEITAMENTE CASADO COM O MUNDO REAL]")
            print("   A física do sensor e os ângulos batem com o mapa.")
            if node.mcl_pose:
                err_x = node.mcl_pose[0] - gt_x
                err_y = node.mcl_pose[1] - gt_y
                if math.sqrt(err_x**2 + err_y**2) > 0.5:
                    print("   -> AVISO: O desalinhamento no RViz ocorre porque o MCL ainda está")
                    print(f"      com erro de estimativa ({math.sqrt(err_x**2 + err_y**2):.2f}m de erro).")
                    print("      Pilote o robô para convergir ou inicialize próximo à pose real.")
        else:
            print("   [DESALINHAMENTO DETECTADO]")
            print(f"   Diferença na Frente: {abs(scan_frente - map_frente):.2f} m")
            print(f"   Diferença na Esq45 : {abs(scan_esq45 - map_esq45):.2f} m")
            print(f"   Diferença na Dir45 : {abs(scan_dir45 - map_dir45):.2f} m")

    print("="*65 + "\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
