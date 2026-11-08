# minisumo_teleop · Práctica 3 (Filtro de Kalman)

Tele-operación del robot minisumo con un guante IMU, estimación de estados con
**EKF y UKF** y un tablero web para visualizar, comparar y llenar las tablas de la guía.

```
 ESP32 guante (MPU6050) ──── ESP-NOW (50 Hz, directo) ────► ESP32 carro (MPU6050, encoders, motores)
        │                                                        │
        └──────────── micro-ROS (WiFi) ─────── PC ───────────────┘
                                                ├── hand_estimator  (EKF + UKF de la mano)
                                                ├── robot_estimator (EKF + UKF del robot) ── RViz2
                                                ├── glove_teleop    (estrategias 1 y 2 → /cmd_vel)
                                                └── dashboard       (app web :8090)
```

## Enlaces guante → carro

| Enlace | Cómo mueve al carro | Estrategias | Armado |
|---|---|---|---|
| **ESP-NOW directo** (por defecto) | El guante envía su inclinación al carro a 50 Hz; el ESP32 del carro calcula v y ω, con protección del borde a bordo | 2 (inclinación) | Botón BOOT del guante (corto: armar/desarmar, 1 s: postura neutra) o botón de la app |
| **PC · ROS 2** | `glove_teleop` publica `/cmd_vel` | 1 y 2 | Botón de la app |

Solo uno conduce a la vez (la app desarma el otro). En el carro la prioridad es
goto > guante armado por ESP-NOW > `/cmd_vel`. Si el guante deja de llegar 250 ms
o se desarma, el carro se detiene. El PC sigue recibiendo todo por micro-ROS
para los filtros de Kalman y la visualización, en ambos modos.

**Requisito de ESP-NOW:** los dos ESP32 deben estar en el mismo canal WiFi:
ambos conectados al mismo router, o ambos sin router (canal fijo
`CONFIG_ESPNOW_FALLBACK_CHANNEL`, por defecto 1). Además deben tener el mismo
`CONFIG_ESPNOW_GROUP_ID` (menuconfig → "Enlace ESP-NOW guante -> carro"); cambiadlo
si otro grupo del laboratorio usa este mismo código.

## Uso

```bash
cd ~/ros2_ws && colcon build --packages-select minisumo_teleop && source install/setup.bash

# Sistema completo (agente micro-ROS en Docker incluido)
ros2 launch minisumo_teleop teleop.launch.py agent:=true
# Con RViz2
ros2 launch minisumo_teleop teleop.launch.py agent:=true rviz:=true
# Sin hardware: simula guante y carro con los mismos tópicos
ros2 launch minisumo_teleop teleop.launch.py sim:=true hand_motion:=tilt   # o plane
```

Abre **http://localhost:8090** (o la IP del PC desde otro equipo de la red).

> ⚠️ Usa `sim:=true` con un `ROS_DOMAIN_ID` distinto si el carro real está conectado:
> el simulador y la tele-operación publican `/cmd_vel`.

**Atajos:** `Espacio` o `Esc` = parada de emergencia.

## Antes de la primera prueba con el robot

1. Pon el robot en el centro del dojo mirando hacia +x y pulsa **Robot al centro**.
2. Con la mano en reposo pulsa **Postura neutra**.
3. Arma el sistema con estrategia 2 y velocidad baja (Ajustes → v máx 0.15 m/s).
   Si al inclinar la mano hacia adelante el robot retrocede, activa **Invertir pitch**
   (lo mismo con roll): depende de cómo quedó montado el MPU6050 en el guante.

## Requisitos de la guía → dónde están

| Guía | Implementación |
|---|---|
| Micro 1: captura de IMU de la mano, envío al PC, filtrado | Firmware `~/laboratorio_final_guante` (MPU6050 + filtro de Mahony + EMA; ESP-NOW al carro y micro-ROS al PC) |
| Micro 2: lazos de control, sensores del robot, filtrado | Firmware del carro (`~/laboratorio_final_senales`) (PID por rueda, MPU6050 + DLPF + complementario, encoders + EMA) |
| PC: estimación de posición/velocidad de la mano con Kalman | `hand_estimator` (EKF + UKF, modelo de Singer, ZUPT) |
| PC: visualización de la mano | Tablero → Teleop (guante 3D, plano de la mano) y Kalman |
| PC: coordinación mano → robot | `glove_teleop`: estrategia 1 (plano) y 2 (pitch/roll) |
| PC: estimación de la posición del robot con Kalman | `robot_estimator` (EKF + UKF, uniciclo con entradas, sesgo del giroscopio, ZARU) |
| PC: visualización del robot en el dojo | Tablero → dojo 3D (three.js) y RViz2 (`/robot/*/odom`, `/dojo/markers`, TF) |
| Señal cruda vs filtrada, ¿cambia el error? | Tablero → Señales (σΔ y % de reducción) y conmutadores *Cruda / Filtrada* |
| Pruebas offline con datos almacenados | Botón **Grabar** + vista Offline / `ros2 run minisumo_teleop offline_eval <sesión>` |
| Tablas: esperada / medida / estimada y errores | Tablero → Pruebas (por estrategia y condición; exporta CSV) |
| EKF vs UKF: precisión, tiempo de cómputo, memoria | Tablero → Kalman (tiempos en vivo + benchmark con verdad conocida) |

## Modelos

**Robot** — estado `[x, y, θ, v, ω, b_g]`, entradas `(v_cmd, ω_cmd)` de `/cmd_vel`.
Proceso: uniciclo (no lineal en θ) con respuesta de primer orden del PID.
Mediciones: `v` de encoders, `ω + b_g` del giroscopio crudo (o `ω` del filtrado),
`ω = 0` cuando las ruedas están detenidas sin comando (ZARU) y, si existe, pose
de un sistema de tracking externo en `/robot/tracking` (PoseStamped).

**Mano** — estado `[x, y, vx, vy, ax, ay, ψ]` en el plano horizontal del operador.
Proceso: aceleración correlacionada (Singer). Medición no lineal: aceleración
lineal compensada en inclinación y expresada en el marco de rumbo de la mano,
`z = Rz(ψ)ᵀ [ax, ay]`; rumbo de la fusión; ZUPT (pseudo-medición `v = 0`) con
la mano quieta. La posición se obtiene integrando y **deriva** (~2 cm por
movimiento); por eso la estrategia 1 usa la velocidad, no la posición.

## Tópicos

| Tópico | Tipo | Publica |
|---|---|---|
| `glove/imu/raw`, `glove/imu/filtered`, `glove/rpy` | Imu, Imu, Vector3 | ESP32 guante |
| `imu/raw`, `imu/filtered`, `wheel_*/raw\|filtered`, `odom/pose`, `odom/twist` | — | ESP32 carro |
| `hand/{ekf,ukf}/odom`, `hand/{ekf,ukf}/accel`, `hand/stats` | Odometry, Vector3Stamped, String(JSON) | hand_estimator |
| `robot/{ekf,ukf}/odom`, `robot/stats`, `dojo/markers`, TF `dojo→base_link` | Odometry, String(JSON), MarkerArray | robot_estimator |
| `cmd_vel`, `teleop/state` | Twist, String(JSON) | glove_teleop |
| `glove/armed` / `glove/arm` | Bool | ESP32 guante / dashboard |
| `teleop/espnow` (v, ω, paquetes/s, enlace, RSSI, borde) | Twist | ESP32 carro |
| `glove/zero`, `hand/reset`, `robot/reset`, `goto_cmd` (modo 2) | Empty / Twist | dashboard |

## Datos

Todo queda en `~/teleop_data/`: `sessions/<fecha>_<nombre>/*.csv` (una por fuente)
y `experiments.json` (tablas de la guía). El CSV de las tablas se descarga desde
la vista Pruebas.
