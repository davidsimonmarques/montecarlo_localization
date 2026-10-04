#python
# ==============================================================================
# SCRIPT DE CALIBRAÇÃO AUTOMÁTICA DOS ALPHAS DO MCL (COPPELIASIM)
# ==============================================================================
# Compatível diretamente com o Python interno do CoppeliaSim (Non-threaded / Threaded)
# Não depende de 'client' (ZeroMQ) nem de bibliotecas externas.
#
# Como usar:
# 1. No CoppeliaSim, clique com o botão direito no robô -> Add -> Associated child script -> Python
# 2. Apague TUDO o que estiver no script novo e cole este código completo.
# 3. Dê Play na simulação.
# 4. O robô andará 4m reto (Teste A) e dará 1 volta completa no próprio eixo (Teste B).
# 5. Ao final, os parâmetros alpha1, alpha2, alpha3, alpha4 serão impressos no console.
# ==============================================================================

import math

# Globais da simulação
sim = None
robotHandle = None
frontLeftMotor = None
frontRightMotor = None
rearLeftMotor = None
rearRightMotor = None

# Constantes do seu Robô (Husky)
WHEEL_RADIUS = 0.165       # Raio da roda (m)
DIST_WHEELS  = 0.571       # Bitola / distância entre rodas (m)

# Parâmetros dos Testes
TEST_A_DIST  = 4.0         # 4 metros em linha reta
TEST_A_SPEED = 0.4         # 0.4 m/s
TEST_B_ROT   = 2.0 * math.pi  # 360 graus de giro no eixo
TEST_B_OMEGA = 0.5         # 0.5 rad/s

# Estados da Máquina de Estados
STATE_INIT      = 0
STATE_RUN_A     = 1
STATE_EVAL_A    = 2
STATE_RUN_B     = 3
STATE_EVAL_B    = 4
STATE_FINISHED  = 5

currentState = STATE_INIT
stateStartTime = 0.0

# Variáveis de Medição
start_gt_pos = [0, 0, 0]
start_gt_yaw = 0.0
prev_gt_yaw  = 0.0
accum_gt_yaw = 0.0

start_fl = 0.0
start_fr = 0.0
start_rl = 0.0
start_rr = 0.0

# Resultados brutos
raw_alpha1 = 0.0
raw_alpha2 = 0.0
raw_alpha3 = 0.0
raw_alpha4 = 0.0

def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def find_joint(name, parent):
    candidates = [f"./{name}", f"../{name}", name, f":/{name}"]
    for c in candidates:
        try:
            h = sim.getObject(c)
            if h != -1:
                return h
        except Exception:
            pass
    try:
        joints = sim.getObjectsInTree(parent, sim.object_joint_type)
        for j in joints:
            if name in sim.getObjectAlias(j):
                return j
    except Exception:
        pass
    return -1

def set_robot_vel(v, w):
    global frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor, WHEEL_RADIUS, DIST_WHEELS
    left_spd  = (v - (DIST_WHEELS / 2.0) * w) / WHEEL_RADIUS
    right_spd = (v + (DIST_WHEELS / 2.0) * w) / WHEEL_RADIUS
    sim.setJointTargetVelocity(frontLeftMotor, left_spd)
    sim.setJointTargetVelocity(rearLeftMotor, left_spd)
    sim.setJointTargetVelocity(frontRightMotor, right_spd)
    sim.setJointTargetVelocity(rearRightMotor, right_spd)

def sysCall_init():
    global sim, robotHandle, frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor
    global currentState, stateStartTime

    sim = require('sim')

    # Identifica o robô e os motores
    robotHandle = sim.getObject('.')
    alias = sim.getObjectAlias(robotHandle)
    print(f"[CALIB_MCL] Iniciado no robo: {alias}")

    frontLeftMotor  = find_joint("front_left_wheel", robotHandle)
    frontRightMotor = find_joint("front_right_wheel", robotHandle)
    rearLeftMotor   = find_joint("rear_left_wheel", robotHandle)
    rearRightMotor  = find_joint("rear_right_wheel", robotHandle)

    if min(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor) == -1:
        print("[ERRO] Nao foi possivel encontrar as 4 rodas do robo! Verifique os nomes das juntas.")
        return

    set_robot_vel(0.0, 0.0)
    currentState = STATE_INIT
    stateStartTime = sim.getSimulationTime()
    sim.addStatusbarMessage("[CALIB_MCL] Aguardando 1.5s para estabilizar fisica...")

def sysCall_actuation():
    global currentState, stateStartTime, sim, robotHandle
    global frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor
    global start_gt_pos, start_gt_yaw, prev_gt_yaw, accum_gt_yaw
    global start_fl, start_fr, start_rl, start_rr
    global raw_alpha1, raw_alpha2, raw_alpha3, raw_alpha4

    if currentState == STATE_FINISHED:
        set_robot_vel(0.0, 0.0)
        return

    sim_time = sim.getSimulationTime()
    elapsed = sim_time - stateStartTime

    # --------------------------------------------------------------------------
    # ESTADO 0: Estabilização Inicial
    # --------------------------------------------------------------------------
    if currentState == STATE_INIT:
        set_robot_vel(0.0, 0.0)
        if elapsed >= 1.5:
            # Prepara Teste A
            start_gt_pos = sim.getObjectPosition(robotHandle, -1)
            start_gt_yaw = sim.getObjectOrientation(robotHandle, -1)[2]

            start_fl = sim.getJointPosition(frontLeftMotor)
            start_rl = sim.getJointPosition(rearLeftMotor)
            start_fr = sim.getJointPosition(frontRightMotor)
            start_rr = sim.getJointPosition(rearRightMotor)

            currentState = STATE_RUN_A
            stateStartTime = sim_time
            print(f"\n[TESTE A] Andando {TEST_A_DIST}m em linha reta a {TEST_A_SPEED}m/s...")
            sim.addStatusbarMessage(f"[CALIB_MCL] Teste A: Reta ({TEST_A_DIST}m)...")
            set_robot_vel(TEST_A_SPEED, 0.0)

    # --------------------------------------------------------------------------
    # ESTADO 1: Executando Reta (Teste A)
    # --------------------------------------------------------------------------
    elif currentState == STATE_RUN_A:
        duration_a = TEST_A_DIST / TEST_A_SPEED
        if elapsed < duration_a:
            set_robot_vel(TEST_A_SPEED, 0.0)
        else:
            set_robot_vel(0.0, 0.0)
            currentState = STATE_EVAL_A
            stateStartTime = sim_time

    # --------------------------------------------------------------------------
    # ESTADO 2: Avalia Teste A e Prepara Teste B
    # --------------------------------------------------------------------------
    elif currentState == STATE_EVAL_A:
        set_robot_vel(0.0, 0.0)
        if elapsed >= 1.5:  # Aguarda frear completamente
            end_gt_pos = sim.getObjectPosition(robotHandle, -1)
            end_gt_yaw = sim.getObjectOrientation(robotHandle, -1)[2]

            end_fl = sim.getJointPosition(frontLeftMotor)
            end_rl = sim.getJointPosition(rearLeftMotor)
            end_fr = sim.getJointPosition(frontRightMotor)
            end_rr = sim.getJointPosition(rearRightMotor)

            # Odometria teórica por rotação das rodas
            d_left  = ((end_fl - start_fl) + (end_rl - start_rl)) / 2.0
            d_right = ((end_fr - start_fr) + (end_rr - start_rr)) / 2.0
            dist_odom = ((d_left + d_right) / 2.0) * WHEEL_RADIUS
            yaw_odom  = ((d_right - d_left) * WHEEL_RADIUS) / DIST_WHEELS

            # Ground truth real
            dx_gt = end_gt_pos[0] - start_gt_pos[0]
            dy_gt = end_gt_pos[1] - start_gt_pos[1]
            dist_gt = math.sqrt(dx_gt**2 + dy_gt**2)
            yaw_gt  = normalize_angle(end_gt_yaw - start_gt_yaw)

            err_trans = abs(dist_odom - dist_gt)
            err_rot   = abs(normalize_angle(yaw_odom - yaw_gt))

            print("--- Resultado Teste A (Reta) ---")
            print(f"  Distancia Real (Ground Truth) : {dist_gt:.4f} m")
            print(f"  Distancia Odometria (Encoders): {dist_odom:.4f} m")
            print(f"  Erro de Translacao            : {err_trans:.4f} m")
            print(f"  Desvio Angular na Reta        : {math.degrees(err_rot):.3f} deg ({err_rot:.4f} rad)")

            raw_alpha3 = (err_trans / max(0.1, dist_gt)) ** 2
            raw_alpha2 = (err_rot / max(0.1, dist_gt)) ** 2

            # Prepara Teste B (Giro no eixo)
            start_gt_pos = sim.getObjectPosition(robotHandle, -1)
            prev_gt_yaw  = sim.getObjectOrientation(robotHandle, -1)[2]
            accum_gt_yaw = 0.0

            start_fl = sim.getJointPosition(frontLeftMotor)
            start_rl = sim.getJointPosition(rearLeftMotor)
            start_fr = sim.getJointPosition(frontRightMotor)
            start_rr = sim.getJointPosition(rearRightMotor)

            currentState = STATE_RUN_B
            stateStartTime = sim_time
            print(f"\n[TESTE B] Girando {math.degrees(TEST_B_ROT):.0f} deg a {TEST_B_OMEGA} rad/s...")
            sim.addStatusbarMessage(f"[CALIB_MCL] Teste B: Giro no Eixo...")
            set_robot_vel(0.0, TEST_B_OMEGA)

    # --------------------------------------------------------------------------
    # ESTADO 3: Executando Giro (Teste B)
    # --------------------------------------------------------------------------
    elif currentState == STATE_RUN_B:
        curr_yaw = sim.getObjectOrientation(robotHandle, -1)[2]
        accum_gt_yaw += normalize_angle(curr_yaw - prev_gt_yaw)
        prev_gt_yaw = curr_yaw

        duration_b = TEST_B_ROT / TEST_B_OMEGA
        if elapsed < duration_b:
            set_robot_vel(0.0, TEST_B_OMEGA)
        else:
            set_robot_vel(0.0, 0.0)
            currentState = STATE_EVAL_B
            stateStartTime = sim_time

    # --------------------------------------------------------------------------
    # ESTADO 4: Avalia Teste B e Imprime Parâmetros Finais
    # --------------------------------------------------------------------------
    elif currentState == STATE_EVAL_B:
        set_robot_vel(0.0, 0.0)
        if elapsed >= 1.5:
            end_gt_pos = sim.getObjectPosition(robotHandle, -1)

            end_fl = sim.getJointPosition(frontLeftMotor)
            end_rl = sim.getJointPosition(rearLeftMotor)
            end_fr = sim.getJointPosition(frontRightMotor)
            end_rr = sim.getJointPosition(rearRightMotor)

            d_left  = ((end_fl - start_fl) + (end_rl - start_rl)) / 2.0
            d_right = ((end_fr - start_fr) + (end_rr - start_rr)) / 2.0
            yaw_odom = ((d_right - d_left) * WHEEL_RADIUS) / DIST_WHEELS

            dx_b = end_gt_pos[0] - start_gt_pos[0]
            dy_b = end_gt_pos[1] - start_gt_pos[1]
            drift_b = math.sqrt(dx_b**2 + dy_b**2)
            yaw_gt = accum_gt_yaw

            err_rot_b = abs(yaw_odom - yaw_gt)

            print("--- Resultado Teste B (Giro) ---")
            print(f"  Rotacao Real (Ground Truth)   : {math.degrees(yaw_gt):.2f} deg ({yaw_gt:.4f} rad)")
            print(f"  Rotacao Odometria (Encoders)  : {math.degrees(yaw_odom):.2f} deg ({yaw_odom:.4f} rad)")
            print(f"  Erro Angular de Rotacao       : {math.degrees(err_rot_b):.2f} deg ({err_rot_b:.4f} rad)")
            print(f"  Deslocamento Linear Indesejado: {drift_b:.4f} m")

            raw_alpha1 = (err_rot_b / max(0.1, abs(yaw_gt))) ** 2
            raw_alpha4 = (drift_b / max(0.1, abs(yaw_gt))) ** 2

            # Fator de segurança (1.5x) e piso mínimo (0.02)
            SAFETY = 1.5
            MIN_A  = 0.02

            final_alpha1 = max(MIN_A, round(raw_alpha1 * SAFETY, 4))
            final_alpha2 = max(MIN_A, round(raw_alpha2 * SAFETY, 4))
            final_alpha3 = max(MIN_A, round(raw_alpha3 * SAFETY, 4))
            final_alpha4 = max(MIN_A, round(raw_alpha4 * SAFETY, 4))

            print("\n=============================================================")
            print("       RESULTADO DA CALIBRAÇÃO DOS ALPHAS (MCL)")
            print("=============================================================")
            print("Valores Brutos:")
            print(f"  alpha1 (rotacao <- rotacao)    : {raw_alpha1:.6f}")
            print(f"  alpha2 (rotacao <- translacao) : {raw_alpha2:.6f}")
            print(f"  alpha3 (translacao <- transl.) : {raw_alpha3:.6f}")
            print(f"  alpha4 (translacao <- rotacao) : {raw_alpha4:.6f}")
            print("\nValores Recomendados para o MCL (com margem de seguranca):")
            print("-------------------------------------------------------------")
            print(f"    alpha1: {final_alpha1}")
            print(f"    alpha2: {final_alpha2}")
            print(f"    alpha3: {final_alpha3}")
            print(f"    alpha4: {final_alpha4}")
            print("-------------------------------------------------------------\n")

            sim.addStatusbarMessage(
                f"[CONCLUIDO] a1={final_alpha1}, a2={final_alpha2}, a3={final_alpha3}, a4={final_alpha4}"
            )

            currentState = STATE_FINISHED

def sysCall_cleanup():
    global sim
    if sim and robotHandle:
        set_robot_vel(0.0, 0.0)
