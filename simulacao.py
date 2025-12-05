import pybullet as p
import pybullet_data
import time
import threading
import random
from queue import Queue
from simple_pid import PID
import paho.mqtt.client as mqtt
import json
import math
import ssl

JOINT_1_ID = 0
JOINT_2_ID = 1
JOINT_WRIST_ID = 2
JOINT_LEFT_FINGER_ID = 4
JOINT_RIGHT_FINGER_ID = 5

TIMESTEP = 1.0 / 240.0

TARGET_ANGLE_J1 = 0.0
TARGET_ANGLE_J2 = -0.5
TARGET_GRIPPER_OPEN = 0.03

L1 = 0.70
L2 = 0.60

PERTURBATION_ENABLED = False
PERTURBATION_FORCE_MAGNITUDE = 10.0
PERTURBATION_DURATION = int(240 * 0.5)
PERTURBATION_FREQUENCY = 0.005
perturbation_counter = 0

PAYLOAD_MASS = 0.0
MQTT_BROKER = "bdffc9a5bf6e4bf28591393206fc27e0.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USERNAME = "sentinela"
MQTT_PASSWORD = "Sentinela123"
MQTT_TOPIC_LOG = "arm/data"

mqtt_queue = Queue(maxsize=200)
stop_threads = threading.Event()
payload_id = None
total_energy_j1 = 0.0
total_energy_j2 = 0.0


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Conectado ao Broker HiveMQ Cloud com sucesso!")
    else:
        print(f"Falha ao conectar ao MQTT, código de retorno: {rc}")


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.tls_set(tls_version=ssl.PROTOCOL_TLS)
try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
except Exception as e:
    print(f"AVISO: Falha ao conectar ao MQTT: {e}")


def mqtt_thread():
    global total_energy_j1, total_energy_j2
    print("Thread MQTT iniciada")
    while not stop_threads.is_set():
        try:
            if not mqtt_queue.empty():
                data = mqtt_queue.get(timeout=0.1)
                payload = json.dumps(data)
                client.publish(MQTT_TOPIC_LOG, payload=payload, qos=1)
                mqtt_queue.task_done()
        except Exception as e:
            pass
        time.sleep(0.01)
    print("Thread MQTT encerrada")


mqtt_worker = threading.Thread(target=mqtt_thread, daemon=True)
mqtt_worker.start()

BASE_KP_J1, BASE_KI_J1, BASE_KD_J1 = 50.0, 0.2, 5.0
BASE_KP_J2, BASE_KI_J2, BASE_KD_J2 = 80.0, 0.5, 8.0
pid_j1 = PID(Kp=BASE_KP_J1, Ki=BASE_KI_J1, Kd=BASE_KD_J1, setpoint=TARGET_ANGLE_J1, sample_time=TIMESTEP)
pid_j2 = PID(Kp=BASE_KP_J2, Ki=BASE_KI_J2, Kd=BASE_KD_J2, setpoint=TARGET_ANGLE_J2, sample_time=TIMESTEP)
pid_left_finger = PID(Kp=20.0, Ki=0.1, Kd=2.0, setpoint=TARGET_GRIPPER_OPEN, sample_time=TIMESTEP)
pid_right_finger = PID(Kp=20.0, Ki=0.1, Kd=2.0, setpoint=-TARGET_GRIPPER_OPEN, sample_time=TIMESTEP)

pid_j1.output_limits = (-20, 20)
pid_j2.output_limits = (-15, 15)
pid_left_finger.output_limits = (-5, 5)
pid_right_finger.output_limits = (-5, 5)

def adjust_gains_for_mass(mass):
    global pid_j1, pid_j2
    reduction_factor = 1.0 / (1.0 + 0.15 * mass)
    pid_j1.Kp = BASE_KP_J1 * reduction_factor
    pid_j1.Kd = BASE_KD_J1 * reduction_factor
    pid_j2.Kp = BASE_KP_J2 * reduction_factor
    pid_j2.Kd = BASE_KD_J2 * reduction_factor
    print(f"    🔧 Ganhos ajustados (fator: {reduction_factor:.2f}): J1 Kp={pid_j1.Kp:.1f}, J2 Kp={pid_j2.Kp:.1f}")

def add_payload(mass):
    global payload_id, PAYLOAD_MASS

    if payload_id is not None:
        p.removeBody(payload_id)
        payload_id = None

    if mass > 0:
        gripper_state = p.getLinkState(robot_id, JOINT_LEFT_FINGER_ID)
        gripper_pos = gripper_state[0]

        payload_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=[0.8, 0.2, 0.2, 1]
        )
        payload_collision = p.createCollisionShape(
            shapeType=p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02]
        )

        payload_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=payload_collision,
            baseVisualShapeIndex=payload_visual,
            basePosition=[gripper_pos[0], gripper_pos[1], gripper_pos[2] + 0.15],
        )

        p.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=JOINT_LEFT_FINGER_ID,
            childBodyUniqueId=payload_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0.15],
            childFramePosition=[0, 0, 0],
        )

        PAYLOAD_MASS = mass
        adjust_gains_for_mass(mass)
        print(f"✓ Carga de {mass}kg adicionada ao efetuador (braço mais lento)")
    else:
        PAYLOAD_MASS = 0
        adjust_gains_for_mass(0.0)
        print("✓ Carga removida (braço em velocidade normal)")

def apply_dynamic_pulse(joint_id):
    global perturbation_counter
    
    if perturbation_counter > 0:
        force = random.choice([-PERTURBATION_FORCE_MAGNITUDE, PERTURBATION_FORCE_MAGNITUDE])
        p.applyExternalForce(
            objectUniqueId=robot_id,
            linkIndex=joint_id,
            forceObj=[force, 0, 0],
            posObj=[0, 0, 0],
            flags=p.LINK_FRAME,
        )
        perturbation_counter -= 1

def direct_kinematics(q1, q2):
    """
    Calcula a posição (x, y, z) do efetuador a partir dos ângulos das juntas (q1, q2).
    A cinemática direta é simplificada para a geometria do braço 2DOF no plano YZ (assumindo q1=0).
    A junta 1 (q1) rotaciona em torno de Z (eixo de rotação), afetando X e Y.
    A junta 2 (q2) rotaciona em torno de Y.
    """

    link_state = p.getLinkState(robot_id, JOINT_LEFT_FINGER_ID)
    x_pos, y_pos, z_pos = link_state[0]
    
    return x_pos, y_pos, z_pos

client_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(TIMESTEP)
plane_id = p.loadURDF("plane.urdf")

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.5]
)

start_pos = [0, 0, 0.05]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF(
    "braco_2dof.urdf", start_pos, start_orientation, useFixedBase=True
)
p.resetJointState(robot_id, JOINT_1_ID, 0.0)
p.resetJointState(robot_id, JOINT_2_ID, -0.1)

for joint_id in [JOINT_1_ID, JOINT_2_ID, JOINT_WRIST_ID, JOINT_LEFT_FINGER_ID, JOINT_RIGHT_FINGER_ID]:
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=joint_id,
        controlMode=p.VELOCITY_CONTROL,
        force=0,
    )

print("\n═══════════════════════════════════════════════════════")
print("             CONTROLES DO TECLADO")
print("═══════════════════════════════════════════════════════")
print("  MOVIMENTO (Ajuste Setpoint):")
print("    J/L - Mover Junta 1 | I/K - Mover Junta 2")
print("\n  PERTURBAÇÕES:")
print("     P - Ativar/Desativar Pulso de Forças Aleatórias")
print("     1, 2, 3 - Adicionar Cargas Diferentes | 0 - Remover Carga")
print("═══════════════════════════════════════════════════════\n")


current_step = 0
total_energy_j1 = 0.0
total_energy_j2 = 0.0

try:
    while True:
        current_step += 1
        
        if PERTURBATION_ENABLED:
            if perturbation_counter == 0 and random.random() < PERTURBATION_FREQUENCY:
                perturbation_counter = PERTURBATION_DURATION
                print(f"   📢 PULSO DE FORÇA ATIVADO ({PERTURBATION_DURATION} passos)")
            
            if perturbation_counter > 0:
                apply_dynamic_pulse(JOINT_1_ID)
                apply_dynamic_pulse(JOINT_2_ID)

        state_j1 = p.getJointState(robot_id, JOINT_1_ID)
        current_angle_j1 = state_j1[0]
        current_vel_j1 = state_j1[1]
        
        state_j2 = p.getJointState(robot_id, JOINT_2_ID)
        current_angle_j2 = state_j2[0]
        current_vel_j2 = state_j2[1]

        torque_j1 = pid_j1(current_angle_j1)
        torque_j2 = pid_j2(current_angle_j2)

        p.setJointMotorControl2(
            bodyUniqueId=robot_id, jointIndex=JOINT_1_ID, controlMode=p.TORQUE_CONTROL, force=torque_j1
        )
        p.setJointMotorControl2(
            bodyUniqueId=robot_id, jointIndex=JOINT_2_ID, controlMode=p.TORQUE_CONTROL, force=torque_j2
        )
        
        erro_j1 = pid_j1.setpoint - current_angle_j1
        erro_j2 = pid_j2.setpoint - current_angle_j2
        
        x_eff, y_eff, z_eff = direct_kinematics(current_angle_j1, current_angle_j2)
        energy_step_j1 = abs(torque_j1 * current_vel_j1 * TIMESTEP)
        energy_step_j2 = abs(torque_j2 * current_vel_j2 * TIMESTEP)
        total_energy_j1 += energy_step_j1
        total_energy_j2 += energy_step_j2

        if current_step % 24 == 0:
            log_data = {
                "carga_kg": PAYLOAD_MASS,
                "energia": round(float(total_energy_j1 + total_energy_j2), 2)
            }
            
            if not mqtt_queue.full():
                mqtt_queue.put(log_data)

        if current_step % 240 == 0:
            pert_status = "⚡ON " if PERTURBATION_ENABLED else "OFF"
            payload_info = f" | Carga: {PAYLOAD_MASS:.1f}kg" if PAYLOAD_MASS > 0 else ""
            print(
                f"[T={current_step * TIMESTEP:.1f}s] J1: {current_angle_j1:.2f} (E:{erro_j1:.3f} | Tq:{torque_j1:.1f}) | "
                f"J2: {current_angle_j2:.2f} (E:{erro_j2:.3f} | Tq:{torque_j2:.1f}) | "
                f"E_Total: {total_energy_j1+total_energy_j2:.2f} J | Pert: {pert_status}{payload_info}"
            )

        keys = p.getKeyboardEvents()
        if ord("j") in keys and keys[ord("j")] & p.KEY_IS_DOWN: pid_j1.setpoint += 0.01
        if ord("l") in keys and keys[ord("l")] & p.KEY_IS_DOWN: pid_j1.setpoint -= 0.01
        if ord("i") in keys and keys[ord("i")] & p.KEY_IS_DOWN: pid_j2.setpoint += 0.01
        if ord("k") in keys and keys[ord("k")] & p.KEY_IS_DOWN: pid_j2.setpoint -= 0.01

        if ord("p") in keys and keys[ord("p")] & p.KEY_WAS_RELEASED:
            PERTURBATION_ENABLED = not PERTURBATION_ENABLED
            status = "ATIVADAS" if PERTURBATION_ENABLED else "DESATIVADAS"
            print(f"\n⚡ Perturbações aleatórias {status}")

        if ord("0") in keys and keys[ord("0")] & p.KEY_WAS_RELEASED: add_payload(0.0)
        if ord("1") in keys and keys[ord("1")] & p.KEY_WAS_RELEASED: add_payload(0.5)
        if ord("2") in keys and keys[ord("2")] & p.KEY_WAS_RELEASED: add_payload(2.0)
        if ord("3") in keys and keys[ord("3")] & p.KEY_WAS_RELEASED: add_payload(5.0)

        p.stepSimulation()
        time.sleep(TIMESTEP)

except KeyboardInterrupt:
    print("\nSimulação interrompida pelo usuário.")

finally:
    print("\nEncerrando sistema...")
    stop_threads.set()
    mqtt_worker.join(timeout=2.0)
    client.loop_stop()
    client.disconnect()
    p.disconnect()

    print("Sistema encerrado com sucesso.")