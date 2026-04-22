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

## Hardware-Konfiguration (Bricklet UIDs)

### Bricklet-UID Zuordnung

Die Motor-zu-Bricklet-Zuordnung (Bricklet-Nummer + Pin) ist bereits in der PIB-API-Datenbank definiert. **Fehlend sind die UIDs der physischen Bricklets.**

**Schritte zur UID-Konfiguration:**

1. **UIDs ermitteln** :

Steht Physich drauf ???

2. **UIDs in PIB-API eintragen** (für jedes Bricklet):
```bash
# Beispiel: Bricklet 1 hat physische UID "XYZ"
curl -X PUT http://localhost:5000/bricklet/1 -H "Content-Type: application/json" -d '{"uid":"XYZ"}'
curl -X PUT http://localhost:5000/bricklet/2 -H "Content-Type: application/json" -d '{"uid":"ABC"}'
curl -X PUT http://localhost:5000/bricklet/3 -H "Content-Type: application/json" -d '{"uid":"DEF"}'
curl -X PUT http://localhost:5000/bricklet/4 -H "Content-Type: application/json" -d '{"uid":"GHI"}'
```

3. **Container neu starten**:
```bash
docker-compose restart ros-motors
```

### Motor → Bricklet Zuordnung

**Hand-Motoren (für Hand Controller relevant):**

| Motor Name | Bricklet-Nr. | Pin | Beschreibung |
|------------|--------------|-----|--------------|
| `thumb_right_opposition` | 1 | 0 | Daumen rechts Opposition |
| `thumb_right_stretch` | 1 | 1 | Daumen rechts Streckung |
| `index_right_stretch` | 1 | 2 | Zeigefinger rechts |
| `middle_right_stretch` | 1 | 3 | Mittelfinger rechts |
| `ring_right_stretch` | 1 | 4 | Ringfinger rechts |
| `pinky_right_stretch` | 1 | 5 | Kleinfinger rechts |
| `thumb_left_opposition` | 3 | 0 | Daumen links Opposition |
| `thumb_left_stretch` | 3 | 1 | Daumen links Streckung |
| `index_left_stretch` | 3 | 2 | Zeigefinger links |
| `middle_left_stretch` | 3 | 3 | Mittelfinger links |
| `ring_left_stretch` | 3 | 4 | Ringfinger links |
| `pinky_left_stretch` | 3 | 5 | Kleinfinger links |

**Weitere Motoren (Arme, Schultern, Kopf):**

| Motor Name | Bricklet-Nr. | Pin | Beschreibung |
|------------|--------------|-----|--------------|
| `wrist_right` | 1 | 6 | Handgelenk rechts |
| `lower_arm_right_rotation` | 1 | 7 | Unterarm rechts Rotation |
| `elbow_right` | 1 | 8 | Ellbogen rechts |
| `upper_arm_right_rotation` | 1 | 9 | Oberarm rechts Rotation |
| `shoulder_horizontal_right` | 2 | 0 | Schulter rechts horizontal |
| `shoulder_vertical_right` | 2 | 1 | Schulter rechts vertikal |
| `turn_head_motor` | 2 | 4 | Kopf drehen |
| `tilt_forward_motor` | 2 | 5 | Kopf neigen |
| `shoulder_horizontal_left` | 2 | 8 | Schulter links horizontal |
| `shoulder_vertical_left` | 2 | 9 | Schulter links vertikal |
| `wrist_left` | 3 | 6 | Handgelenk links |
| `lower_arm_left_rotation` | 3 | 7 | Unterarm links Rotation |
| `elbow_left` | 3 | 8 | Ellbogen links |
| `upper_arm_left_rotation` | 3 | 9 | Oberarm links Rotation |

**Bricklet 4:** Solid State Relay (Relais für Motorstromversorgung)

> **Hinweis:** Die Bricklet-Nummern sind logische IDs in der Software. Die UIDs sind physische IDs auf den Hardware-Bricklets (aufgedruckt oder per Enumerate ermittelbar). Du musst zuordnen: "Welches physische Bricklet (UID) soll welche logische Nummer haben?"

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
docker-compose build ros-motors

```

### 2. Start Container

```bash
# Starten (automatisch alle 4 Nodes)
docker-compose up -d ros-motors

# Logs prüfen
docker-compose logs --tail=50 ros-motors

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

**In `hand_core.py`:**

```python
CONTROL_LOOP_RATE_HZ = 50   # Control loop frequency
ADMITTANCE_GAIN = 1.5       # K_a: Units yielded per mA excess current
```

**In `hand_controller.py`:**

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
# Für jede Achse (daumen, zeigefinger, ...):
- Lese current_position vom Motor
- Setze reference_position = current_position (q_ref)
- Setze current_cmd_pos = current_position (q_cmd)
- Setze target_position aus Grip-Config
```

**2. Control Loop Tick (50Hz):**
```python
for axis in axes:
    # Schritt 1: Update Reference (ideale Trajektorie)
    q_ref = calculate_next_reference(q_ref, target, max_speed)
    
    # Schritt 2: Messe Strom
    current_measured = motor.get_current()
    
    # Schritt 3: Admittance Logic
    threshold = max_current * 0.7
    excess = max(0, current_measured - threshold)
    q_cmd = q_ref - (1.5 * excess)  # Weicht zurück bei Überstrom
    
    # Schritt 4: Sende Position
    motor.set_position(q_cmd)
    
    # Schritt 5: Check ob fertig
    if q_ref erreicht target:
        axis.is_finished = True
```

**3. Completion:**
```python
if all axes finished:
    return Result(success=True, message="Grip completed")
```

## FFNN Integration (Future)

Der Code ist vorbereitet für ML-basierte Schwellenwert-Vorhersage:

**Aktuell (statisch):**
```python
threshold = max_current * 0.7  # Fixer Schwellenwert
```

**Zukünftig (FFNN):** 

> ![TODO](https://img.shields.io/badge/TODO-red?style=flat-square) **TODO:** Für den Start als Regressionsproblem (FFNN) planen. Später prüfen, ob ein Digital Twin / RL-Ansatz sinnvoller ist. Details und Verantwortliche noch klären.

```python
# Predict expected current based on position and velocity
expected_current = ffnn_model.predict(q_ref, velocity, finger_id)
threshold = expected_current 

# Only currents above prediction trigger admittance
excess = max(0, current_measured - threshold)
```


## Integration mit bestehenden Systemen

### Mit motor_control Node

`hand_controller` greift **direkt** auf `pib_motors.motor` zu, **nicht** über den `ApplyJointTrajectory` Service. Das ermöglicht:
- Präzise 50Hz Kontrolle ohne Service-Overhead
- Direkte Current-Messung für Admittance
- Keine Interferenz zwischen verschiedenen Motion-Controllern

### Mit motor_current Node

`motor_current` published weiterhin alle Motorströme auf `/motor_current` Topic (4Hz). Das ist **unabhängig** vom `hand_controller`, der seine eigenen Current-Messungen macht (50Hz).


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
2. **hand_controller** liest Ziel-Positionen aus `hand_config.yaml`
3. **50Hz Loop** startet für alle 6 Achsen:
   - Berechne nächsten `q_ref` Schritt
   - Messe Motor-Strom
   - Wende Admittance an: `q_cmd = q_ref - K_a * excess_current`
   - Sende `q_cmd` an Hardware
4. **Feedback** wird kontinuierlich gesendet (current_axis, progress, currents)
5. **Result** wird zurückgegeben wenn alle `q_ref` am Ziel

**Bei Widerstand:**
- Finger wird "weich" (q_cmd weicht zurück)
- Trajektorie läuft weiter (q_ref → Ziel)
- Automatische Fortsetzung wenn Widerstand nachlässt


---

**Autoren:** PIB Robotics Team  
**Letzte Aktualisierung:** 2026-04-22  
