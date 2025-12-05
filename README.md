# Documentação Técnica: Simulação e Controle de Braço Robótico 2DOF

## 1. O Problema (Requisitos)

O objetivo deste projeto é simular um **Braço Robótico Planar (2 Juntas/Graus de Liberdade)** operando em **malha fechada** (feedback constante). O sistema deve atender aos seguintes requisitos:

1. **Controle PID:** Controlar a posição das juntas individualmente.
2. **Cinemática Direta:** Calcular a posição (X, Y, Z) do efetuador baseada nos ângulos.
3. **Perturbações:** Reagir a forças externas ou pesos adicionais (cargas), mantendo a estabilidade.
4. **Telemetria IoT:** Enviar métricas de desempenho (Energia, Carga) para supervisão externa via MQTT.

---

## 2. A Solução Implementada

O código foi desenvolvido em **Python** utilizando a biblioteca **PyBullet** para física e visualização. A arquitetura opera em um *loop* de simulação contínuo que integra leitura de sensores, cálculo de controle e comunicação em rede.

### Estrutura Geral

* **Física:** O PyBullet gerencia gravidade, colisões e dinâmica a partir do arquivo `braco_2dof.urdf`.
* **Controle (Cérebro):** A biblioteca `simple_pid` calcula o torque necessário para minimizar o erro entre a posição atual e o *setpoint*.
* **Conectividade:** Uma *thread* paralela (`mqtt_thread`) gerencia o envio de dados para o broker HiveMQ sem bloquear a simulação física.

---

## 3. Mapeamento: Requisitos vs. Implementação

Abaixo detalhamos como cada requisito do problema foi resolvido no código.

### A) Controle em Malha Fechada (PID Individual)

**Requisito:** PID reativo em cada junta (Referência - Medido = Torque).

**Solução:** O código lê a posição atual via `p.getJointState`, o PID calcula a correção necessária e o PyBullet aplica o torque.

```python
# Configuração dos PIDs
pid_j1 = PID(Kp=BASE_KP_J1, Ki=BASE_KI_J1, Kd=BASE_KD_J1, setpoint=TARGET_ANGLE_J1...)

# Loop de Controle
state_j1 = p.getJointState(robot_id, JOINT_1_ID) # Leitura do sensor (Encoder virtual)
current_angle_j1 = state_j1[0]
torque_j1 = pid_j1(current_angle_j1)             # PID calcula o torque baseado no erro
p.setJointMotorControl2(..., force=torque_j1)    # Aplicação do torque no motor
```

### B) Cinemática Direta

**Requisito:** Calcular a posição do efetuador a partir dos ângulos das juntas.

**Solução:** A função `direct_kinematics` obtém a posição (x, y, z) da garra diretamente do PyBullet:

```python
def direct_kinematics(q1, q2):
    link_state = p.getLinkState(robot_id, JOINT_LEFT_FINGER_ID)
    x_pos, y_pos, z_pos = link_state[0]
    return x_pos, y_pos, z_pos
```

### C) Perturbações e Cargas

**Requisito:** O sistema deve reagir a forças externas e cargas adicionais.

**Solução:** 
- **Perturbações:** Forças aleatórias são aplicadas via `apply_dynamic_pulse()` quando ativadas (tecla `P`).
- **Cargas:** Objetos físicos são anexados à garra via `add_payload()`, e os ganhos PID são automaticamente ajustados.

```python
def adjust_gains_for_mass(mass):
    reduction_factor = 1.0 / (1.0 + 0.15 * mass)
    pid_j1.Kp = BASE_KP_J1 * reduction_factor
    # ... reduz ganhos para evitar oscilações com carga
```

### D) Telemetria MQTT (IoT)

**Requisito:** Enviar dados de desempenho para monitoramento remoto.

**Solução:** Uma thread dedicada envia JSON para o broker HiveMQ Cloud:

```python
log_data = {
    "carga_kg": PAYLOAD_MASS,
    "energia": round(float(total_energy_j1 + total_energy_j2), 2)
}
client.publish(MQTT_TOPIC_LOG, payload=json.dumps(log_data), qos=1)
```

---

## 4. Controles do Teclado

| Tecla | Ação |
|-------|------|
| `J/L` | Mover Junta 1 (esquerda/direita) |
| `I/K` | Mover Junta 2 (cima/baixo) |
| `P` | Ativar/Desativar perturbações |
| `1` | Adicionar carga de 0.5 kg |
| `2` | Adicionar carga de 2.0 kg |
| `3` | Adicionar carga de 5.0 kg |
| `0` | Remover carga |

---

## 5. Dependências

```bash
pip install pybullet simple-pid paho-mqtt
```

---

## 6. Como Executar

```bash
python simulacao.py
```

---

## 7. Arquitetura do Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                    LOOP PRINCIPAL (240 Hz)                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌─────────┐      ┌─────────┐      ┌─────────┐            │
│   │ Sensor  │──────│   PID   │──────│  Motor  │            │
│   │ (Ângulo)│      │(Torque) │      │(Aplica) │            │
│   └─────────┘      └─────────┘      └─────────┘            │
│        ▲                                  │                 │
│        └──────────── feedback ────────────┘                 │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│   ┌─────────────┐                    ┌──────────────┐       │
│   │ Perturbações│                    │  MQTT Thread │       │
│   │  (Forças)   │                    │  (Telemetria)│       │
│   └─────────────┘                    └──────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. Configuração MQTT

O sistema envia dados para o HiveMQ Cloud:

- **Broker:** `bdffc9a5bf6e4bf28591393206fc27e0.s1.eu.hivemq.cloud`
- **Porta:** `8883` (TLS)
- **Tópico:** `arm/data`

---

## Licença

Este projeto é de uso educacional.
