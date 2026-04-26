# PIB Hand Controller - Dokumentation

## Übersicht

Der **Hand Controller** ist ein ROS2 Action Server für intelligente Grip-Steuerung mit **Admittance Control** (Nachgiebigkeits-Regelung). Er läuft als vierter Node im `motors` Package und ermöglicht die Ausführung vordefinierter Griffe mit automatischer.

**Schlüssel-Features:**
- ✅ **Grip-basierte Steuerung**: 7 vordefinierte Griffe (OPEN, SPITZGRIFF, DREIPUNKTGRIFF, ...)
- ✅ **Admittance Control**: Weiche Nachgiebigkeit statt hartem Stopp bei Widerstand
- ✅ **50Hz Control Loop**: Präzise Trajektorien-Interpolation mit Compliance-Detection
- ✅ **Action Server**: Async Grip-Ausführung mit Progress-Feedback
- ✅ **FFNN-Ready**: Vorbereitet für ML-basierte Schwellenwert-Vorhersage -> Oder zukünfitg RL Trajectory / FFNN

---

## Modulstruktur

Der Hand Controller ist als modulares `pib_hand` Package organisiert für bessere Wartbarkeit und Testbarkeit:

```
motors/pib_hand/
├── __init__.py           # Public API exports
├── config.py             # Configuration loading & validation
├── admittance.py         # Core admittance control algorithms
├── hardware.py           # Tinkerforge motor access & unit conversion
├── controller.py         # ROS2 Action Server node
├── data_logger.py        # Optional: Trajectory logging for FFNN training
├── simulation/           # Standalone MuJoCo testbed
│   ├── hand_sim.py       # Simulation main entry point
│   └── pib_upper_body/   # URDF models
├── README.md             # This file
└── DATA_LOGGING.md       # Trajectory logging documentation
```

### Module Responsibilities

| Module | Verantwortung | ROS2? | Testbar standalone? |
|--------|---------------|-------|---------------------|
| `config.py` | YAML-Parsing, Grip-Definitionen, Validation | ❌ | ✅ |
| `admittance.py` | Trajektorien-Berechnung, Admittance-Logik | ❌ | ✅ |
| `hardware.py` | Motor-Zugriff, State-Management, Unit-Conversion | ❌ | ✅ (mit Mock) |
| `controller.py` | ROS2 Action Server, State Machine, Control Loop | ✅ | ❌ |
| `data_logger.py` | CSV-Logging für FFNN Training-Daten | ❌ | ✅ |
| `simulation/` | MuJoCo Testbed für Algorithmen | ❌ | ✅ |

**Design-Prinzipien:**
- **ROS-agnostic Core**: `config`, `admittance`, `hardware` haben keine ROS2-Dependencies
- **Clear Separation**: Control-Algorithmen (admittance) getrennt von Hardware-Access (hardware)
- **Testability**: Jedes Modul kann einzeln getestet werden
- **Reusability**: Simulation nutzt gleiche Algorithmen wie Production-Code

### Import-Beispiele

```python
# ROS2 Node starten
from motors.pib_hand.controller import HandController, main

# Algorithmen nutzen (z.B. für Tests)
from motors.pib_hand import (
    load_hand_config,
    calculate_next_reference,
    apply_admittance_logic,
)

# Simulation starten
from motors.pib_hand.simulation.hand_sim import main as sim_main
```

---

## Architektur

### ROS2 Nodes

```
motors Container:
├── motor_control      (Node 1: JointTrajectory Service → Motor Positions)
├── motor_current      (Node 2: Motor Current Publisher, 4Hz)
├── relay_control      (Node 3: Relay Management)
└── hand_controller    (Node 4: Grip Execution mit Admittance Control)
```

### Admittance Control Prinzip

Der `hand_controller` nutzt eine **Trennung von Reference und Command Position**:

```
Bei jedem 50Hz Tick:
1. q_ref schreitet Richtung Ziel (ideale Trajektorie)
2. Messe aktuellen Motorstrom
3. Berechne q_cmd = q_ref - K_a * (current - threshold)
   → Bei Überstrom: q_cmd weicht zurück (Finger wird "weich")
   → Kein Überstrom: q_cmd = q_ref (normale Bewegung)
4. Sende q_cmd an Motor
5. Wenn q_ref am Ziel: Achse fertig
```

**Vorteile gegenüber hartem Compliance-Stopp:**
- Finger geben bei Widerstand nach statt zu stoppen
- Automatische Fortsetzung wenn Widerstand nachlässt
- Flüssigere Reaktion durch 50Hz Loop
- Weniger Vibrationen

---

## Build & Deploy

### 1. Docker Build

```bash
cd /home/leona/pib-backend

# Build motors Container (baut datatypes + motors packages)
docker compose build ros-motors

```

### 2. Start Container

```bash
# Starten (automatisch alle 4 Nodes)
docker compose up -d ros-motors

# Logs prüfen
docker compose logs --tail=50 ros-motors

# Erwartete Ausgabe:
# [motor_control-1] ... Now Running MOTOR_CONTROL
# [motor_current-2] ... Now Running MOTOR CURRENT
# [relay_control-3] ... Now Running RELAY_CONTROL
# [hand_controller-4] ... Hand Controller started [namespace=hand, rate=50Hz, axes=6, grips=7]
```

### 3. Dev-Modus vs. Production

**Dev-Modus** (DEV, ohne Hardware):
- In `launch/launch.py` ist `dev=True` gesetzt
- Hand Controller loggt: `[WARN] Running in DEV mode - motor access may be limited`
- Keine echten Motorbewegungen, nur Logik-Test

**Production** (auf Roboter mit Tinkerforge):
- In `launch/launch.py` `dev=True` zu `dev=False` ändern
- Hand Controller greift direkt auf `pib_motors.motor` zu
- Echte Hardware-Bewegungen mit Current-Messung

### 4. Trajectory Logging (optional, für FFNN Training)

**Aktivierung:**
```python
# launch/launch.py
Node(
    package="motors",
    executable="hand_controller",
    parameters=[
        {"dev": True},
        {"enable_logging": True},  # Aktiviert CSV-Logging
        {"log_path": "/data/trajectories"}
    ],
)
```

**Output:**
- CSV-Dateien pro Grip: `trajectory_SPITZGRIFF_20260425_143022.csv`
- Format: `timestamp,axis_id,q_cmd,dq_cmd,direction,measured_current`
- Details: siehe [DATA_LOGGING.md](DATA_LOGGING.md)

---

## ROS2 Interface

> ![TODO](https://img.shields.io/badge/TODO-red?style=flat-square) **TODO:** Aktuell wird nur die rechte Hand angesteuert. Fallunterscheidung in den ROS-Messages notwendig und Anpassung im `Hand_Controller`.

### Action Server: `/hand/execute_grip`

**Type:** `datatypes/action/ExecuteGrip`

**Action Definition:**
```
# Goal - Request to execute a predefined grip
string grip_name

---

# Result - Final outcome of the grip execution
bool success
string message

---

# Feedback - Continuous updates during grip execution
string current_axis
float32 progress_percent
int32[] motor_currents


### Beispiel-Nutzung

**1. Action Liste anzeigen:**
```bash
docker exec -it multirepo-ros-motors-1 bash -c \
  "cd /app/ros2_ws && source install/setup.bash && ros2 action list"

# Ausgabe:
# /hand/execute_grip
```

**2. Grip ausführen (mit Feedback):**
```bash
docker exec -it multirepo-ros-motors-1 bash -c \
  "cd /app/ros2_ws && source install/setup.bash && \
   ros2 action send_goal /hand/execute_grip datatypes/action/ExecuteGrip \
   '{grip_name: \"SPITZGRIFF\"}' --feedback"
```
**Oder auch interkativ im Container:**
# 1. Einmal in den Container gehen

docker exec -it multirepo-ros-motors-1 bash

# 2. Im Container: ROS2 Environment laden
cd /app/ros2_ws
source install/setup.bash

# Jetzt:
ros2 action send_goal /hand/execute_grip datatypes/action/ExecuteGrip \
  '{grip_name: "SPITZGRIFF"}' --feedback

# Oder andere Griffe:
ros2 action send_goal /hand/execute_grip datatypes/action/ExecuteGrip \
  '{grip_name: "OPEN"}' --feedback



**Feedback während Ausführung:**
```
Feedback:
    current_axis: daumen
    progress_percent: 25.5
    motor_currents: [120, 135, 0, 0, 0, 450]
```


### Verfügbare Griffe (grip_name)

> ![TODO](https://img.shields.io/badge/TODO-red?style=flat-square) **TODO:** Aktuell sind hier nur Dummy-Werte enthalten. Wertebereich und Zuordnung der TinkerForge-Motoren (z.B. `-9000`..`9000`) müssen mit der Hardware abgeglichen werden.

Definiert in `hand_config.yaml`:

| Grip Name | Beschreibung | Finger-Positionen |
|-----------|--------------|-------------------|
| `OPEN` | Hand öffnen (alle Finger gestreckt) 
| `SPITZGRIFF` | Pinch Grip (Daumen + Zeigefinger) 
| `DREIPUNKTGRIFF` | Three-point Grip
| `SCHLUESSELGRIFF` | Key Grip | 
| `ZYLINDERGRIFF` | Cylinder/Power Grip (alle Finger) |  
| `HAKENGRIFF` | Hook Grip (ohne Daumen) |

| `SPHAERISCHER_GRIFF` | Spherical Grip | 


**Position Range:** `-9000` (gestreckt/offen) bis `9000` (geschlossen)

> ![TODO](https://img.shields.io/badge/TODO-red?style=flat-square) **TODO:** Positionsbereich mit der Hardware abgleichen.


---

## Konfiguration

### hand_config.yaml

**Pfad:** `ros_packages/motors/config/hand_config.yaml`

**Struktur:**

```yaml
axes:
  daumen:
    motor_name: "thumb_right_stretch"
    min_pos: -9000
    max_pos: 9000
    max_speed: 800        # Tinkerforge units/second
    max_current: 1500     # mA
  # ... weitere Achsen

grips:
  SPITZGRIFF:
    daumen: -2000
    zeigefinger: -2000
    mittelfinger: -9000
    # ... weitere Finger
```

- **Wichtige Parameter:**

> ![TODO](https://img.shields.io/badge/TODO-red?style=flat-square) **TODO:** Die angegebenen Parameter sind derzeit Platzhalter/Dummy-Werte. Alle Werte (`min_pos`, `max_pos`, `max_speed`, `max_current` etc.) müssen mit der realen Hardware validiert und dokumentiert werden.

- **motor_name**: Name in `pib_motors.motor.name_to_motors` Dictionary 
- **min_pos/max_pos**: Position Range (-9000 bis 9000 Tinkerforge units)
- **max_speed**: Maximale Geschwindigkeit für Trajektorien-Interpolation
- **max_current**: Schwellenwert für Admittance Control (70% davon wird genutzt)

### Admittance Parameter

**In `pib_hand/admittance.py`:**

```python
CONTROL_LOOP_RATE_HZ = 50   # Control loop frequency
ADMITTANCE_GAIN = 1.5       # K_a: Units yielded per mA excess current
```

**In `pib_hand/controller.py`:**

```python
COMPLIANCE_THRESHOLD = 0.7  # 70% of max_current triggers admittance
```

**Tuning-Tipps:**
- **ADMITTANCE_GAIN** erhöhen → Finger weicher, mehr Nachgiebigkeit
- **ADMITTANCE_GAIN** verringern → Finger härter, weniger Nachgiebigkeit
- **COMPLIANCE_THRESHOLD** erhöhen → Admittance startet später (höherer Strom erlaubt)
- **COMPLIANCE_THRESHOLD** verringern → Admittance startet früher (sensibler)

---

## Funktion der Handsteuerung

### Control Loop (50Hz)

Der `hand_controller` führt für jede Achse folgende Schritte aus:

**1. Initialisierung (beim Action Goal):**
```python
# controller.py: _execute_grip_callback()
# Für jede Achse wird ein AxisState erstellt:
from pib_hand.hardware import AxisState, get_motor_by_name

motor = get_motor_by_name(axis_config.motor_name)
current_pos = motor.get_position()

state = AxisState(
    config=axis_config,
    motor=motor,
    start_position=current_pos,
    target_position=target_pos,
    reference_position=current_pos,  # q_ref
    current_cmd_pos=current_pos,     # q_cmd
    is_finished=False
)
```

**2. Control Loop Tick (50Hz):**
```python
# controller.py: _control_loop_tick()
from pib_hand.admittance import (
    calculate_next_reference,
    apply_admittance_logic,
    is_target_reached
)

for axis_name, state in axis_states.items():
    # Schritt 1: Update Reference (ideale Trajektorie)
    state.reference_position = calculate_next_reference(
        state.reference_position,
        state.target_position,
        state.config.max_speed
    )
    
    # Schritt 2: Messe Strom
    current_measured = state.motor.get_current()
    
    # Schritt 3: Admittance Logic
    threshold = state.config.max_current * COMPLIANCE_THRESHOLD
    state.current_cmd_pos = apply_admittance_logic(
        state.reference_position,
        current_measured,
        threshold
    )
    
    # Schritt 4: Sende Position
    state.motor.set_position(int(state.current_cmd_pos))
    
    # Schritt 5: Check ob fertig
    if is_target_reached(state.reference_position, state.target_position):
        state.is_finished = True
```

**3. Completion:**
```python
if all(state.is_finished for state in axis_states.values()):
    grip_state = GripState.COMPLETED
    return Result(success=True, message="Grip completed")
```

## FFNN Integration (Future)

Der Code ist vorbereitet für ML-basierte Schwellenwert-Vorhersage:

**Aktuell (statisch) in `pib_hand/controller.py`:**
```python
threshold = state.config.max_current * COMPLIANCE_THRESHOLD  # Fixer Schwellenwert (70%)
```

**Zukünftig (FFNN) - Integration in `pib_hand/admittance.py` oder als separates Modul:** 

> ![TODO](https://img.shields.io/badge/TODO-red?style=flat-square) **TODO:** Für den Start als Regressionsproblem (FFNN) planen. Später prüfen, ob ein Digital Twin / RL-Ansatz sinnvoller ist. Details und Verantwortliche noch klären.

```python
# In pib_hand/admittance.py oder pib_hand/ml_model.py
def predict_current_threshold(q_ref: float, velocity: float, finger_id: str) -> float:
    """Predict expected current based on position and velocity using FFNN."""
    # FFNN inference hier
    expected_current = ffnn_model.predict([q_ref, velocity, finger_id])
    return expected_current

# In controller.py
threshold = predict_current_threshold(
    state.reference_position, 
    velocity, 
    axis_name
)
state.current_cmd_pos = apply_admittance_logic(
    state.reference_position,
    current_measured,
    threshold  # Dynamischer Schwellenwert statt statisch
)
```

**Vorteil der modularen Struktur:**
- FFNN-Integration betrifft primär `admittance.py`
- `controller.py` muss nur Threshold-Berechnung austauschen
- Algorithmen bleiben testbar ohne ML-Model


## Integration mit bestehenden Systemen

### Mit motor_control Node

`pib_hand/controller.py` greift **direkt** auf `pib_motors.motor` zu (via `hardware.get_motor_by_name()`), **nicht** über den `ApplyJointTrajectory` Service. Das ermöglicht:
- Präzise 50Hz Kontrolle ohne Service-Overhead
- Direkte Current-Messung für Admittance
- Keine Interferenz zwischen verschiedenen Motion-Controllern

### Mit motor_current Node

`motor_current` published weiterhin alle Motorströme auf `/motor_current` Topic (4Hz). Das ist **unabhängig** vom `hand_controller`, der seine eigenen Current-Messungen macht (50Hz via `motor.get_current()` in `controller._control_loop_tick()`).


### Mit cerebra/rosbridge

TODO: Das ist spekuliert.

Aus der Web-UI kann der Hand Controller über rosbridge angesprochen werden:

```javascript
// JavaScript Client (cerebra)
const client = new ROSLIB.ActionClient({
    ros: ros,
    serverName: '/hand/execute_grip',
    actionName: 'datatypes/action/ExecuteGrip'
});

const goal = new ROSLIB.Goal({
    actionClient: client,
    goalMessage: { grip_name: 'SPITZGRIFF' }
});

goal.on('feedback', (feedback) => {
    console.log('Progress:', feedback.progress_percent);
});

goal.on('result', (result) => {
    console.log('Success:', result.success);
});

goal.send();
```

---



## Troubleshooting

### Motor Currents sind -1

Das bedeutet: Motor nicht verbunden oder dev-Modus.

- **Dev-Modus**: Normal, `motor.get_current()` gibt `NO_CURRENT=-1` zurück
- **Production**: Check Tinkerforge Verbindung, Motor-Namen in PIB-API

### Finger bewegen sich nicht weiter nach Widerstand

Das ist das **gewünschte Verhalten** mit Admittance Control:
- `q_ref` läuft weiter zum Ziel
- `q_cmd` bleibt zurück wegen hohem Strom
- Wenn Widerstand nachlässt → `q_cmd` folgt `q_ref` wieder

Grip ist **completed** wenn `q_ref` am Ziel ist, auch wenn `q_cmd` durch Admittance weiter zurück ist.

---



## Zusammenfassung

**Was passiert bei einem Grip-Request:**

1. **Action Goal** wird gesendet mit `grip_name`
2. **controller.py** lädt Ziel-Positionen aus `config/hand_config.yaml` via `config.py`
3. **controller.py** initialisiert `AxisState` für alle 6 Achsen via `hardware.py`
4. **50Hz Loop** startet (`controller._control_loop_tick()`):
   - `admittance.calculate_next_reference()` → berechnet nächsten `q_ref` Schritt
   - `hardware` → misst Motor-Strom
   - `admittance.apply_admittance_logic()` → berechnet `q_cmd = q_ref - K_a * excess_current`
   - `hardware` → sendet `q_cmd` an Motor
   - `admittance.is_target_reached()` → prüft ob `q_ref` am Ziel
5. **Feedback** wird kontinuierlich gesendet (current_axis, progress, currents)
6. **Result** wird zurückgegeben wenn alle `q_ref` am Ziel

**Bei Widerstand:**
- Finger wird "weich" (q_cmd weicht zurück durch `apply_admittance_logic()`)
- Trajektorie läuft weiter (q_ref → Ziel via `calculate_next_reference()`)
- Automatische Fortsetzung wenn Widerstand nachlässt

**Module-Interaktion:**
```
config.py → lädt YAML
    ↓
hardware.py → initialisiert AxisStates mit Motoren
    ↓
controller.py → startet 50Hz Loop
    ↓
admittance.py → berechnet Trajektorie & Compliance
    ↓
hardware.py → sendet Befehle an Tinkerforge
```


---

**Autoren:** PIB Robotics Team  
**Letzte Aktualisierung:** 2026-04-22  
