# Simulação de Braço Robótico 2DOF - Guia Resumido

## 📋 O Que o Código Faz

Simula um braço robótico de 2 juntas com:
- **Controle PID** para posicionamento preciso
- **Telemetria MQTT** enviando dados para Node-RED
- **Perturbações** para testar robustez
- **Ajuste de ganhos** quando adiciona carga
- **Controle via teclado**

---

## 🏗️ Componentes Principais

### 1. Controle PID
```python
# Equação: τ = Kp·erro + Ki·∫erro·dt + Kd·d(erro)/dt
pid_j1 = PID(Kp=50.0, Ki=0.2, Kd=5.0)
pid_j2 = PID(Kp=80.0, Ki=0.5, Kd=8.0)
```

- **Kp:** Resposta rápida (maior = mais agressivo)
- **Ki:** Elimina erro residual
- **Kd:** Suaviza movimentos

**J2 tem ganhos maiores porque precisa vencer a gravidade**

### 2. Sistema MQTT
- Thread dedicada envia dados a 10 Hz
- Fila de 200 mensagens (buffer)
- Não bloqueia o loop principal (240 Hz)

**Dados enviados:**
- Ângulos atuais vs referência
- Torques aplicados
- Energia consumida
- Posição cartesiana (x, y, z)

### 3. Ajuste Adaptativo de Ganhos
```python
reduction_factor = 1.0 / (1.0 + 0.15 * mass)
```

| Carga | Fator | Velocidade |
|-------|-------|------------|
| 0 kg  | 1.00  | Normal     |
| 2 kg  | 0.77  | 77%        |
| 5 kg  | 0.57  | 57%        |

**Por quê:** Cargas pesadas → maior inércia → precisa ganhos menores para evitar oscilações

### 4. Loop Principal (240 Hz)
```
1. Ler ângulos das juntas
2. Calcular torque PID
3. Aplicar torque
4. Calcular energia (P = τ·ω)
5. Enviar dados MQTT (a cada 0.1s)
6. Processar teclas
7. Avançar simulação
```

---

## ⌨️ Controles

| Tecla | Ação |
|-------|------|
| `J` / `L` | Gira base (Junta 1) |
| `I` / `K` | Move braço (Junta 2) |
| `P` | Ativa/desativa perturbações |
| `0` | Remove carga |
| `1` / `2` / `3` | Adiciona carga (0.5kg / 2kg / 5kg) |

---

## 📊 Métricas Monitoradas

| Métrica | Fórmula | Ideal |
|---------|---------|-------|
| **Erro** | `\|setpoint - atual\|` | < 0.01 rad |
| **Energia** | `Σ(\|τ·ω·Δt\|)` | Menor possível |
| **Torque** | PID output | < limites (±20 N⋅m) |

---

## 🚀 Principais Melhorias Possíveis

### 1. **Anti-Windup no PID**
```python
# Limitar acumulador integral para evitar saturação
self._integral = max(-10, min(10, self._integral))
```

### 2. **Telemetria JSON Válida**
```python
# Atual: str(data) ❌
# Correto:
payload = json.dumps(data)  # ✅
client.publish(topic, payload, qos=1)
```

### 3. **Planejamento de Trajetória**
```python
# Interpolar suavemente entre posições (evita movimentos bruscos)
trajectory = plan_quintic(start=0.0, end=1.57, duration=3.0)
```

### 4. **Detecção de Colisão**
```python
contacts = p.getContactPoints(bodyA=robot_id)
if normal_force > threshold:
    # Para robô imediatamente
```

### 5. **Logging CSV**
```python
logger = DataLogger('log.csv')
logger.log(data)  # Salva para análise posterior
```

---

## 📈 Análise Rápida

**Frequências:**
- Simulação: 240 Hz (4.17ms)
- Controle PID: 240 Hz
- Telemetria: 10 Hz (0.1s)

**Cálculo de Energia:**
```
Potência = Torque × Velocidade Angular
Energia = Σ(Potência × Δt)
```

**Por que J2 consome mais energia:**
- Precisa vencer gravidade constantemente
- Braço mais longo = maior inércia

---

## 🎯 Conclusão

**✅ Pontos Fortes:**
- Arquitetura modular
- MQTT multithread eficiente
- Ajuste adaptativo de ganhos
- Interface completa

**⚠️ Limitações:**
- Sem anti-windup (integral pode saturar)
- Telemetria não é JSON válido
- Movimentos bruscos (sem interpolação)
- Sem detecção de colisão

**📌 Prioridades:**
1. Implementar anti-windup
2. Corrigir formato JSON no MQTT
3. Adicionar logging CSV
4. Implementar detecção de colisão
