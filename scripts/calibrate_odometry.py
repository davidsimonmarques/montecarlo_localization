#!/usr/bin/env python3
# ==============================================================================
# CALIBRAÇÃO AUTOMÁTICA DOS ALPHAS DO MCL VIA ROS 2 (TESTE A E TESTE B)
# ==============================================================================
# Este script roda pelo terminal no ROS 2, comunicando diretamente com o robô
# no CoppeliaSim através dos tópicos /cmd_vel, /odom e /pose_ground_truth.
# 
# Vantagens:
#   - NÃO trava o CoppeliaSim (sem mensagem de "Abort execution").
#   - Não precisa criar nem editar scripts internos no CoppeliaSim.
#   - Usa a odometria real e o ground truth publicados pelo robô.
#
# Como rodar:
#   1. Inicie a simulação no CoppeliaSim (Play).
#   2. Em um terminal aberto, execute:
#      python3 /home/davidsimon/ros2_ws/src/montecarlo_localization/scripts/calibrate_odometry.py
# ==============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class OdomCalibrator(Node):
    def __init__(self):
        super().__init__('mcl_odom_calibrator')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.gt_sub = self.create_subscription(PoseStamped, '/pose_ground_truth', self.gt_cb, 10)

        self.latest_odom = None
        self.latest_gt = None
        
        self.get_logger().info("Aguardando conexao com os topicos /odom e /pose_ground_truth do CoppeliaSim...")

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_odom = (pos.x, pos.y, yaw)

    def gt_cb(self, msg):
        pos = msg.pose.position
        yaw = quaternion_to_yaw(msg.pose.orientation)
        self.latest_gt = (pos.x, pos.y, yaw)

    def wait_for_data(self):
        start = time.time()
        while self.latest_odom is None or self.latest_gt is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10.0:
                return False
        return True

    def stop_robot(self):
        cmd = Twist()
        for _ in range(5):
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)

    def drive_distance(self, distance, speed=0.4):
        """Executa o Teste A: anda a distancia desejada em linha reta."""
        duration = distance / speed
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0

        t_end = time.time() + duration
        while time.time() < t_end:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.stop_robot()
        time.sleep(1.5)  # Estabilizacao

    def rotate_angle(self, angle_rad, omega=0.5):
        """Executa o Teste B: gira o angulo desejado no proprio eixo."""
        duration = abs(angle_rad) / omega
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = omega if angle_rad > 0 else -omega

        t_end = time.time() + duration
        while time.time() < t_end:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.stop_robot()
        time.sleep(1.5)  # Estabilizacao

def main():
    rclpy.init()
    node = OdomCalibrator()

    if not node.wait_for_data():
        node.get_logger().error("ERRO: Nao foi possivel receber /odom ou /pose_ground_truth. A simulacao esta rodando no CoppeliaSim?")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Topicos recebidos com sucesso! Estabilizando fisica (2s)...")
    time.sleep(2.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    # ==========================================================================
    # TESTE A: Translação em Linha Reta (Estima alpha2 e alpha3)
    # ==========================================================================
    target_dist = 4.0
    print("\n" + "="*60)
    print(f" [TESTE A] Iniciando Linha Reta: {target_dist:.1f} metros...")
    print("="*60)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    start_odom = node.latest_odom
    start_gt = node.latest_gt

    node.drive_distance(target_dist, speed=0.4)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    end_odom = node.latest_odom
    end_gt = node.latest_gt

    # Distância e desvio angular no odom vs ground truth
    dx_odom_a = end_odom[0] - start_odom[0]
    dy_odom_a = end_odom[1] - start_odom[1]
    dist_odom_a = math.sqrt(dx_odom_a**2 + dy_odom_a**2)
    yaw_odom_a = normalize_angle(end_odom[2] - start_odom[2])

    dx_gt_a = end_gt[0] - start_gt[0]
    dy_gt_a = end_gt[1] - start_gt[1]
    dist_gt_a = math.sqrt(dx_gt_a**2 + dy_gt_a**2)
    yaw_gt_a = normalize_angle(end_gt[2] - start_gt[2])

    err_trans_a = abs(dist_odom_a - dist_gt_a)
    err_rot_a = abs(normalize_angle(yaw_odom_a - yaw_gt_a))

    print(f"  -> Distancia Real (Ground Truth) : {dist_gt_a:.4f} m")
    print(f"  -> Distancia Odometria           : {dist_odom_a:.4f} m")
    print(f"  -> Erro de Translacao            : {err_trans_a:.4f} m")
    print(f"  -> Desvio Angular ao Andar Reto  : {math.degrees(err_rot_a):.3f} deg ({err_rot_a:.4f} rad)")

    raw_alpha3 = (err_trans_a / max(0.1, dist_gt_a)) ** 2
    raw_alpha2 = (err_rot_a / max(0.1, dist_gt_a)) ** 2

    time.sleep(2.0)

    # ==========================================================================
    # TESTE B: Rotação no Próprio Eixo (Estima alpha1 e alpha4)
    # ==========================================================================
    target_rot = 2.0 * math.pi
    print("\n" + "="*60)
    print(f" [TESTE B] Iniciando Giro no Eixo: {math.degrees(target_rot):.0f} graus...")
    print("="*60)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    start_odom_b = node.latest_odom
    start_gt_b = node.latest_gt

    node.rotate_angle(target_rot, omega=0.5)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    end_odom_b = node.latest_odom
    end_gt_b = node.latest_gt

    # Rotação odom vs ground truth e desvio de translação
    yaw_odom_b = normalize_angle(end_odom_b[2] - start_odom_b[2])
    yaw_gt_b = normalize_angle(end_gt_b[2] - start_gt_b[2])
    
    # Se deu volta completa (~2*pi), o delta normalizado pode ser pequeno, corrigimos:
    rot_nominal = 2.0 * math.pi
    err_rot_b = abs(yaw_odom_b - yaw_gt_b)

    dx_gt_b = end_gt_b[0] - start_gt_b[0]
    dy_gt_b = end_gt_b[1] - start_gt_b[1]
    drift_trans_b = math.sqrt(dx_gt_b**2 + dy_gt_b**2)

    print(f"  -> Rotacao Real (Ground Truth)   : {math.degrees(yaw_gt_b):.2f} deg")
    print(f"  -> Rotacao Odometria             : {math.degrees(yaw_odom_b):.2f} deg")
    print(f"  -> Erro Angular de Rotacao       : {math.degrees(err_rot_b):.2f} deg ({err_rot_b:.4f} rad)")
    print(f"  -> Deslocamento Linear no Giro   : {drift_trans_b:.4f} m")

    raw_alpha1 = (err_rot_b / max(0.1, rot_nominal)) ** 2
    raw_alpha4 = (drift_trans_b / max(0.1, rot_nominal)) ** 2

    # Fator de segurança (1.5x) e piso mínimo (0.02)
    SAFETY = 1.5
    MIN_A = 0.02

    final_alpha1 = max(MIN_A, round(raw_alpha1 * SAFETY, 4))
    final_alpha2 = max(MIN_A, round(raw_alpha2 * SAFETY, 4))
    final_alpha3 = max(MIN_A, round(raw_alpha3 * SAFETY, 4))
    final_alpha4 = max(MIN_A, round(raw_alpha4 * SAFETY, 4))

    print("\n" + "="*60)
    print("       RESULTADO DA CALIBRAÇÃO DOS ALPHAS (MCL)")
    print("="*60)
    print("Valores Brutos Medidos:")
    print(f"  alpha1 (rotacao <- rotacao)    : {raw_alpha1:.6f}")
    print(f"  alpha2 (rotacao <- translacao) : {raw_alpha2:.6f}")
    print(f"  alpha3 (translacao <- transl.) : {raw_alpha3:.6f}")
    print(f"  alpha4 (translacao <- rotacao) : {raw_alpha4:.6f}")
    print("\nValores Finais Recomendados (com margem de seguranca para MCL):")
    print("-------------------------------------------------------------")
    print(f"    alpha1: {final_alpha1}")
    print(f"    alpha2: {final_alpha2}")
    print(f"    alpha3: {final_alpha3}")
    print(f"    alpha4: {final_alpha4}")
    print("-------------------------------------------------------------\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
