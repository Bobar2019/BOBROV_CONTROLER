# Protocole de Communication Série — BOB-CONTROL ↔ Raspberry Pi 5

**Auteur :** DERO DIDIER — [ddero2014@gmail.com](mailto:ddero2014@gmail.com)
**Cible :** Cockpit RPi 5 (BOB-ROV) — développement Python / pyserial
**Objet :** Spécification complète du lien binaire série entre le Raspberry Pi 5 et le firmware **BOB-CONTROL** (ESP32-S3). Ce document contient tout le nécessaire pour écrire le code côté RPi 5 sans jamais ouvrir le firmware : format des trames octet par octet, sémantique exacte des champs, comportements de sécurité, exemple Python prêt à l'emploi et vecteurs de test.

---

## 1. Vue d'ensemble

| Élément | Valeur |
| :--- | :--- |
| Topologie | Liaison point à point RPi 5 ↔ ESP32-S3 (full-duplex) |
| Débit | **921 600 bauds**, 8 bits, sans parité, 1 stop (8N1) |
| Trames | **Taille fixe** — descendante : **39 octets**, montante : **72 octets** |
| Intégrité | **CRC16-CCITT-FALSE** (polynôme `0x1021`, init `0xFFFF`) sur chaque trame |
| Trame descendante | 20–50 Hz recommandé (commande : mode, armement, 16 consignes PWM) |
| Trame montante | **100 Hz fixe** (télémétrie : IMU, pression, température, puissance, PWM) |
| Failsafe | Watchdog **500 ms** par défaut : perte de liaison ⇒ tous les actionneurs au neutre |
| Endianness | Tous les champs multi-octets en **little-endian**, **sauf le CRC** (big-endian) |

**Principe de fonctionnement côté RPi 5 :** le Pi envoie en continu des trames de commande (au moins toutes les 400 ms pour ne pas déclencher le watchdog) et lit en continu les trames de télémétrie qui arrivent à 100 Hz. Le firmware applique les consignes, ou bascule automatiquement en sécurité si le flux s'interrompt.

---

## 2. Liaison physique — deux interfaces au choix

Le firmware embarque **deux interfaces physiques** pour le protocole binaire. Le choix se fait dans l'onglet **Paramètres** de l'interface Web de l'ESP32 et est mémorisé en NVS (**redémarrage nécessaire** après changement).

| Mode | Côté ESP32-S3 | Côté RPi 5 | Particularités |
| :--- | :--- | :--- | :--- |
| **USB-CDC Natif** (défaut) | Port USB-C | `/dev/ttyACM0` | Le debug texte du firmware **partage le même port** : la télémétrie binaire est entrecoupée de messages `[SERIAL]`, `[SENSORS]`… Le parseur doit être tolérant (voir §7) ou filtrer sur les en-têtes. |
| **UART Matériel** | `Serial1` — RX : **GPIO 1**, TX : **GPIO 2** | `/dev/serial0` (ou `/dev/ttyAMA0`) | Lien dédié, **propre** (aucun texte parasite). Câblage croisé obligatoire : TX du Pi → GPIO 1, GPIO 2 → RX du Pi. Référence commune (GND) indispensable. |

> **Note RPi 5 (mode UART) :** activer l'UART matériel via `raspi-config` → *Interface Options* → *Serial Port* : **login shell = NON**, **matériel série = OUI**. Vérifier aussi que `/boot/firmware/config.txt` contient `enable_uart=1`.
>
> **Permissions :** ajouter l'utilisateur au groupe `dialout` (`sudo usermod -aG dialout $USER`, puis reconnexion) pour accéder au port sans `sudo`.

---

## 3. Cadre général des trames

### 3.1 En-têtes et sens de circulation

| Sens | En-tête (2 octets) | Type (offset 2) | Taille totale |
| :--- | :--- | :--- | :--- |
| RPi 5 → ESP32-S3 (descendante) | `0xAA` `0x55` | `0x01` | **39 octets** |
| ESP32-S3 → RPi 5 (montante) | `0x55` `0xAA` | `0x02` | **72 octets** |

Les en-têtes ont des motifs **opposés** (`AA 55` vs `55 AA`) : un simple `find` sur la séquence correcte suffit à se resynchroniser dans un flux bruité.

### 3.2 CRC16-CCITT-FALSE

Le CRC couvre les octets **de l'offset 2 jusqu'à l'avant-dernier octet de la trame** (le champ CRC lui-même est exclu) :

| Trame | Octets couverts par le CRC | CRC stocké aux offsets |
| :--- | :--- | :--- |
| Descendante (39 octets) | `[2..36]` | `[37]` = octet **fort**, `[38]` = octet **faible** |
| Montante (72 octets) | `[2..69]` | `[70]` = octet **fort**, `[71]` = octet **faible** |

Paramètres : polynôme `0x1021`, valeur initiale `0xFFFF`, ni réflexion d'entrée/sortie, ni XOR final.
**Valeur de contrôle** : `crc16_ccitt(b"123456789") == 0x29B1`.

> **⚠️ Le CRC est stocké en big-endian** (octet fort d'abord), contrairement à **tous** les autres champs multi-octets de la trame qui sont en little-endian. C'est la source d'erreur n°1 des implémentations tierces — voir l'exemple validé §7.

### 3.3 Résynchronisation

Une trame descendante invalide (CRC KO) est **silencieusement ignorée** par le firmware : la machine à états repart en attente d'en-tête. Côté RPi 5, appliquer la même stratégie : chercher `0x55 0xAA`, tenter de décoder 72 octets, en cas de CRC invalide **avancer d'un seul octet** et recommencer (jamais sauter 72 octets d'un bloc).

---

## 4. Trame descendante — RPi 5 → ESP32-S3 (39 octets)

Format : `[0xAA][0x55][TYPE][MODE][ARM][PWM_0..15][CRC_HI][CRC_LO]`

| Offset | Taille | Champ | Type | Valeurs | Description |
| ---: | ---: | :--- | :--- | :--- | :--- |
| 0 | 1 | `header1` | `uint8_t` | `0xAA` | En-tête 1 |
| 1 | 1 | `header2` | `uint8_t` | `0x55` | En-tête 2 |
| 2 | 1 | `type` | `uint8_t` | `0x01` | Type « commande » |
| 3 | 1 | `mode` | `uint8_t` | `0` `1` `2` | Mode de pilotage : **0** = Passif (manuel direct), **1** = Auto Roulis + Tangage, **2** = Auto Full (Roll+Pitch+Yaw+Profondeur). Toute valeur > 2 est ignorée (mode inchangé). |
| 4 | 1 | `arm_state` | `uint8_t` | `0` `1` `2` | **0** = Désarmé, **1** = Armé, **2** = E-Stop (arrêt d'urgence) |
| 5 | 32 | `pwm[0..15]` | `uint16_t` × 16 (LE) | 1000–2000 | Consignes en **µs**. `pwm[k]` occupe les offsets `5 + 2k` (octet faible d'abord) : `pwm[0]` @ 5–6 … `pwm[15]` @ 35–36. Neutre : **1500**. Le firmware borne toute valeur à 1000–2000 µs. |
| 37 | 1 | `crc16_hi` | `uint8_t` | — | CRC16 (voir §3.2) — octet fort |
| 38 | 1 | `crc16_lo` | `uint8_t` | — | CRC16 — octet faible |

### 4.1 Affectation des 16 canaux

| Canal | Actionneur | Plage utile | Neutre / repos |
| :--- | :--- | :--- | :--- |
| 0–7 | ESC propulseurs M1–M8 (M1–M4 horizontaux, M5–M8 verticaux) | 1000–2000 µs | 1500 µs |
| 8–9 | Servomoteurs Pan & Tilt caméra | 1000–2000 µs | 1500 µs |
| 10–11 | Pince / rotation d'outil | 1000–2000 µs | 1500 µs |
| 12–13 | Gradateurs MOSFET projecteurs LED | 1000 µs = 0 % → 2000 µs = 100 % | 1000 µs (0 %) |
| 14–15 | Canaux auxiliaires / extensions | 1000–2000 µs | 1500 µs |

### 4.2 Cadence et comportement d'émission

* **Envoyer en continu**, même quand rien ne change : c'est ce flux qui alimente le watchdog (§6.1) et le statut « maître » (§6.3).
* Cadence recommandée : **20 à 50 trames/s** (une trame toutes les 20–50 ms). À 100 Hz, aucune contre-indication technique, seulement de la bande passante inutile.
* En cas de coupure, le failsafe s'active après le **timeout watchdog (500 ms par défaut)** : prévoir un envoi toutes les 400 ms *au pire* si l'application est chargée.

---

## 5. Trame montante — ESP32-S3 → RPi 5 (72 octets)

Format : `[0x55][0xAA][TYPE][STATUS][QUAT][GYRO][PRESS][TEMP][POWER×6][PWM_ACT×16][CRC]`

Émise **en continu à 100 Hz** (toutes les 10 ms) dès que le firmware a démarré.

| Offset | Taille | Champ | Type | Facteur de conversion | Description |
| ---: | ---: | :--- | :--- | :--- | :--- |
| 0 | 1 | `header1` | `uint8_t` | — | `0x55` |
| 1 | 1 | `header2` | `uint8_t` | — | `0xAA` |
| 2 | 1 | `type` | `uint8_t` | — | `0x02` (télémétrie) |
| 3 | 1 | `status` | `uint8_t` | bits — voir §5.1 | État du système |
| 4 | 8 | `quat[4]` | `int16_t` × 4 (LE) | **÷ 10 000** | Quaternion W, X, Y, Z du repère **véhicule** (voir §5.3) |
| 12 | 6 | `gyro[3]` | `int16_t` × 3 (LE) | **÷ 100** → °/s | Vitesses angulaires X, Y, Z du repère véhicule |
| 18 | 4 | `pressure` | `int32_t` (LE) | **÷ 10** → mbar | Pression absolue (ex. 10132 = 1013,2 mbar) |
| 22 | 4 | `temperature` | `int32_t` (LE) | **÷ 100** → °C | Température de l'eau (ex. 2500 = 25,00 °C) |
| 26 | 12 | `power[6]` | `uint16_t` × 6 (LE) | **÷ 1000** → V / A | 3 wattmètres : `[V1,I1, V2,I2, V3,I3]` en mV / mA — voir §5.2 |
| 38 | 32 | `pwm_actual[16]` | `uint16_t` × 16 (LE) | µs (entiers) | Consignes PWM courantes. `pwm_actual[k]` occupe les offsets `38 + 2k`. |
| 70 | 2 | `crc16` | `uint16_t` (**BE**) | — | CRC16 sur `[2..69]` — octet fort puis faible |

### 5.1 Sémantique exacte des bits de `status`

| Bit | Masque | Signification réelle (comportement du firmware) |
| :--- | :--- | :--- |
| 0 | `0x01` | Réservé (toujours 0) |
| 1 | `0x02` | **Liaison établie** : positionné dès qu'**au moins une trame descendante valide a été reçue depuis le boot**. ⚠️ Ce n'est **pas** l'état d'armement courant — c'est un témoin de liaison (« le firmware a déjà entendu le RPi 5 »). |
| 2 | `0x04` | Autopilote actif — **réservé, jamais émis** par le firmware actuel (toujours 0) |
| 3 | `0x08` | **Watchdog série déclenché** : plus aucune trame descendante valide reçue depuis > timeout (défaut 500 ms). Redescend automatiquement dès qu'une trame valide arrive. |
| 4 | `0x10` | **Dry-run (Mode Témoin)** : sorties physiques PCA9685 neutralisées (les consignes sont mémorisées dans `pwm_actual` mais aucune impulsion n'est émise vers les actionneurs). |

### 5.2 Détail des 6 mesures de puissance

| Index `power` | Offset | Mesure | Source (bus I2C n°1) |
| :--- | :--- | :--- | :--- |
| `power[0]` / `power[1]` | 26 / 28 | Tension (mV) / Courant (mA) | **INA226 #1** (`0x41`) — alimentation **Raspberry Pi 5** |
| `power[2]` / `power[3]` | 30 / 32 | Tension (mV) / Courant (mA) | **INA226 #2** (`0x44`) — ligne **Aux / LEDs** |
| `power[4]` / `power[5]` | 34 / 36 | Tension (mV) / Courant (mA) | **INA226 #3** (`0x45`) — ligne **Moteurs / propulsion** |

Le courant est émis en valeur **absolue** (mA). Capteur absent ⇒ 0.

### 5.3 Repère IMU et conversions complémentaires

Le quaternion et le gyroscope sont exprimés dans le **repère véhicule** (NED : **X = avant**, **Y = droite**, **Z = bas**), montage du BNO085 déjà compensé par le firmware. Le quaternion est **unitaire** : `w² + x² + y² + z² = 1` (tolérance numérique).

Conversions utiles pour l'affichage (formules identiques à celles du firmware) :

\[
\begin{aligned}
\text{roll} &= \operatorname{atan2}\big(2(wx+yz),\; 1-2(x^2+y^2)\big) \\
\text{pitch} &= \operatorname{asin}\big(2(wy-zx)\big) \\
\text{cap} &= -\operatorname{atan2}\big(2(wz+xy),\; 1-2(y^2+z^2)\big)
\end{aligned}
\]

Angles en radians, cap positif dans le sens **horaire** (nord → est). Normaliser le cap dans \([-180°, +180°]\) ou \([0°, 360°]\) selon usage.

Profondeur (le firmware n'envoie que la pression absolue ; à convertir côté RPi 5) :

* **Profondeur eau douce (m)** : \(h = \Delta P_{mbar} \times 100 / (\rho \cdot g) \approx \Delta P_{mbar} \times 0{,}010194\) avec \(\Delta P = P - P_{surface}\), \(\rho = 1000\ \text{kg/m}^3\), \(g = 9{,}81\ \text{m/s}^2\).
* **Altitude barométrique (m, hors de l'eau)** : \(h = 44330 \times \left(1 - (P/P_0)^{0,190284}\right)\) avec \(P_0 = 1013{,}25\) mbar (à adapter à la pression du jour).
* La référence surface \(P_{surface}\) se capture **capteur hors de l'eau au démarrage** — c'est à l'application RPi 5 de mémoriser sa propre tare (les données brutes reçues restent absolues).

### 5.4 `pwm_actual` — consignes vs sorties physiques

`pwm_actual` reflète **les consignes courantes appliquées au contrôleur PWM** (après clamp 1000–2000 µs et, en mode AUTO, après corrections PID et mixage) — et non l'état électrique des sorties :

* Si `status & 0x10` (**dry-run**) : les valeurs affichées sont mémorisées mais **aucune sortie physique** n'est pilotée (le PCA9685 reste au neutre).
* Si `status & 0x08` (**watchdog**) : les sorties ont été forcées au neutre ; `pwm_actual` peut encore montrer les dernières consignes jusqu'à la trame suivante.

---

## 6. Comportements du firmware (règles de sécurité et de priorité)

### 6.1 Watchdog série — failsafe 500 ms

* Le firmware attend **au moins une trame descendante valide** dans une fenêtre de **500 ms** (défaut, réglable 100 ms–plusieurs s dans l'onglet Paramètres via NVS — clé `serial_timeout`).
* Dépassement ⇒ **tous les canaux PWM forcés au neutre** (1500 µs, 0 % pour les gradateurs) et bit `0x08` positionné dans la télémétrie.
* Dès qu'une trame valide arrive : le watchdog est réarmé, le bit `0x08` disparaît, et **les consignes de cette trame sont appliquées au cycle suivant (≤ 10 ms)**.
* Au démarrage du firmware : le watchdog **ne se déclenche pas** tant qu'aucune trame n'a jamais été reçue (les sorties restent de toute façon en dry-run/neutre par défaut).

### 6.2 États d'armement (champ `arm_state`)

| `arm_state` | Effet immédiat à la réception de CHAQUE trame |
| :--- | :--- |
| **0 — Désarmé** | Canaux **0–7 forcés au neutre** (1500 µs). Canaux **8–15 acceptés tels quels** (pan/tilt, pince, gradateurs utilisables désarmé). Reset des PID. |
| **1 — Armé** | **Les 16 canaux** sont appliqués (`pwm[0..15]`). Le **mode** est appliqué (passif / auto roulis-tangage / auto full). Le contrôleur de vol considère le RPi 5 comme maître. |
| **2 — E-Stop** | **Tous** les canaux au neutre (1500 µs / 0 %) + reset des PID. Priorité absolue, appliqué à chaque trame reçue. |

> **Point opérationnel :** seules les trames **ARMÉES** transmettent le `mode` et rafraîchissent la maîtrise du RPi 5 (§6.3). Pour reprendre la main après un long arrêt, il suffit de renvoyer des trames armées.

### 6.3 Priorité « maître » RPi 5 / interface Web ESP32

* Chaque trame **armée** reçue marque le RPi 5 comme **maître** du contrôleur de vol.
* Après **1000 ms sans trame armée**, le firmware repasse en mode manuel ESP32 (l'interface Web peut alors changer le mode ; journal série : `[FLIGHT] Maître → Manuel ESP32 (timeout RPi 5)`).
* Tant que le RPi 5 est maître, toute commande de mode émise par l'interface Web est **ignorée** (journal : `[FLIGHT] Commande Web ignorée — RPi 5 est maître`).
* Conséquence pratique : **envoyer des trames armées en continu** (c'est le cas nominal à 20–50 Hz) pour rester maître.

### 6.4 Mode Dry-Run (Sorties Matérielles)

Au boot, les sorties physiques sont **neutralisées** (dry-run actif, bit `0x10`). Les consignes reçues sont suivies dans `pwm_actual` mais le PCA9685 n'émet rien. L'activation des sorties réelles se fait **uniquement** depuis l'interface Web (bouton « Sorties Matérielles », WebSocket `{output_enable: true}` ou REST `POST /api/pwm/output-enable`). C'est un garde-fou de banc : le RPi 5 ne peut pas le désactiver à distance.

---

## 7. Exemple Python complet (validé hors ligne)

Code autonome, sans dépendance autre que **pyserial** (`pip install pyserial`). Toutes les fonctions de décodage ont été vérifiées contre les vecteurs de test du §8.

```python
#!/usr/bin/env python3
"""Cockpit minimal BOB-ROV — liaison binaire avec BOB-CONTROL (ESP32-S3)."""
import struct
import time
import serial

# ==================== Constantes du protocole ====================

PORT = '/dev/ttyACM0'      # USB-CDC natif  |  UART GPIO : '/dev/serial0'
BAUD = 921600
TX_HZ = 25.0               # cadence descendante recommandée : 20-50 Hz

MODE_PASSIF, MODE_AUTO_ROULIS, MODE_AUTO_FULL = 0, 1, 2
ARM_DISARMED, ARM_ARMED, ARM_ESTOP = 0, 1, 2

STATUS_ARMED, STATUS_AUTO, STATUS_WDG, STATUS_DRYRUN = 0x02, 0x04, 0x08, 0x10

# ==================== CRC16-CCITT-FALSE ====================

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT-FALSE : poly 0x1021, init 0xFFFF, pas d'inversion.
    Valeur de contrôle : crc16_ccitt(b'123456789') == 0x29B1."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

# ==================== Trame descendante (39 octets) ====================

def build_downlink_frame(mode: int, arm_state: int, pwm_us) -> bytes:
    """Construit la trame de commande RPi 5 -> ESP32-S3 (39 octets).
    pwm_us : séquence de 16 valeurs en microsecondes (1000-2000)."""
    payload = struct.pack('<BBB16H', 0x01, mode, arm_state, *pwm_us)
    crc = crc16_ccitt(payload)                 # CRC sur [2..36]
    return bytes((0xAA, 0x55)) + payload + struct.pack('>H', crc)   # CRC big-endian

# ==================== Trame montante (72 octets) ====================

def parse_uplink_frame(frame: bytes) -> dict:
    """Valide et décode une trame de télémétrie (ESP32-S3 -> RPi 5)."""
    if len(frame) != 72:
        raise ValueError(f'trame de {len(frame)} octets (attendu 72)')
    if frame[0] != 0x55 or frame[1] != 0xAA or frame[2] != 0x02:
        raise ValueError('en-tête invalide')
    if (frame[70] << 8 | frame[71]) != crc16_ccitt(frame[2:70]):    # CRC sur [2..69]
        raise ValueError('CRC16 invalide')
    (type_, status, w, x, y, z, gx, gy, gz, pressure, temperature, *rest) = \
        struct.unpack_from('<BB4h3h2i6H16H', frame, 2)
    return {
        'status': status,
        'armed': bool(status & STATUS_ARMED),      # liaison établie (≥ 1 trame reçue)
        'auto': bool(status & STATUS_AUTO),        # réservé (jamais émis)
        'watchdog': bool(status & STATUS_WDG),
        'dry_run': bool(status & STATUS_DRYRUN),
        'quat': (w / 10000.0, x / 10000.0, y / 10000.0, z / 10000.0),
        'gyro': (gx / 100.0, gy / 100.0, gz / 100.0),          # °/s
        'pressure_mbar': pressure / 10.0,                       # mbar
        'temperature_c': temperature / 100.0,                   # °C
        'power': list(rest[:6]),                                # mV / mA
        'pwm_us': list(rest[6:22]),                             # µs
    }

class UplinkReader:
    """Réassemble les trames 72 octets dans un flux d'octets bruité
    (resynchronisation automatique sur 0x55 0xAA, ignore le bruit et le debug)."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        """Ajoute des octets reçus ; retourne la liste des trames valides."""
        self.buf.extend(data)
        frames = []
        while True:
            idx = self.buf.find(b'\x55\xaa')
            if idx < 0:
                del self.buf[:-1]          # conserve au plus 1 octet (préfixe possible)
                break
            del self.buf[:idx]
            if len(self.buf) < 72:
                break
            frame = bytes(self.buf[:72])
            if frame[2] == 0x02 and (frame[70] << 8 | frame[71]) == crc16_ccitt(frame[2:70]):
                frames.append(frame)
                del self.buf[:72]
            else:
                del self.buf[:1]           # faux en-tête : avancer d'un octet
        return frames

# ==================== Boucle principale ====================

def main():
    port = serial.Serial(PORT, BAUD, timeout=0)   # lecture non bloquante
    reader = UplinkReader()
    pwm = [1500] * 16                              # toutes voies au neutre
    pwm[0] = 1600                                  # exemple : M1 en avant (µs)

    next_tx = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now >= next_tx:                     # --- émission descendante 25 Hz ---
                next_tx += 1.0 / TX_HZ
                port.write(build_downlink_frame(MODE_PASSIF, ARM_ARMED, pwm))

            for frame in reader.feed(port.read(port.in_waiting or 4096)):
                t = parse_uplink_frame(frame)      # --- décodage télémétrie ---
                print('quat=(%.3f, %.3f, %.3f, %.3f)  P=%.1f mbar  T=%.1f C  '
                      'pwm0=%d  wdg=%s  dry=%s'
                      % (*t['quat'], t['pressure_mbar'], t['temperature_c'],
                         t['pwm_us'][0], t['watchdog'], t['dry_run']))
            time.sleep(0.001)
    finally:
        port.close()

if __name__ == '__main__':
    main()
```

---

## 8. Vecteurs de test (à utiliser pour valider votre parseur)

### 8.1 Valeur de contrôle CRC

```python
assert crc16_ccitt(b'123456789') == 0x29B1
```

### 8.2 Trame descendante de référence

`MODE_PASSIF` + `ARM_ARMED` + toutes voies à 1500 µs :

```
AA 55 01 00 01 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 34 16
```

* 39 octets. `DC 05` = `0x05DC` = 1500 LE, répété 16 fois. CRC final `0x3416` (octet fort `0x34` puis `0x16`).
* Génération : `build_downlink_frame(0, 1, [1500]*16).hex() == 'aa55010001' + 'dc05'*16 + '3416'`

### 8.3 Trame montante de référence

Télémétrie à l'équilibre (quaternion identité, 1013,2 mbar, 25,00 °C), `status = 0x12` (liaison + dry-run), canal 0 à 1600 µs, les autres à 1500 :

```
55 AA 02 12 10 27 00 00 00 00 00 00 00 00 00 00 00 00 94 27 00 00
C4 09 00 00 88 13 20 03 E0 2E 2C 01 D0 39 68 5B 40 06 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 26 A7
```

Décodage attendu :

| Champ | Valeur décodée |
| :--- | :--- |
| `status` | `0x12` → liaison établie + dry-run (`watchdog` = 0) |
| `quat` | `(1.0, 0.0, 0.0, 0.0)` |
| `gyro` | `(0.0, 0.0, 0.0)` °/s |
| `pressure_mbar` | `1013.2` |
| `temperature_c` | `25.0` |
| `power` | `[5000, 800, 12000, 300, 14800, 23400]` → Pi à 5,0 V/0,8 A, Aux 12,0 V/0,3 A, Moteurs 14,8 V/23,4 A |
| `pwm_us` | `[1600, 1500, 1500, …, 1500]` (16 valeurs) |
| CRC | `0x26A7` (offset 70 = `0x26`, offset 71 = `0xA7`) |

### 8.4 Test de robustesse du réassemblage

Injecter du bruit texte, couper une trame en deux, insérer un faux en-tête `55 AA 02 00`, puis vérifier que `UplinkReader.feed()` restitue exactement **2 trames valides**. Le code du §7 (classe `UplinkReader`) est conçu pour ce cas : validé sur la séquence `bruit + moitié1`, puis `moitié2 + faux en-tête + trame complète + bruit`.

---

## 9. Dépannage côté RPi 5

| Symptôme | Cause probable / Remède |
| :--- | :--- |
| `PermissionError` à l'ouverture du port | Utilisateur hors groupe `dialout` : `sudo usermod -aG dialout $USER` puis reconnexion. |
| Aucune trame reçue | 1) Interface choisie côté ESP32 (onglet Paramètres : USB-CDC / UART GPIO) ≠ branchement. 2) Baud ≠ **921600**. 3) UART : TX/RX **croisés** et GND commun. 4) UART Pi non activé (`raspi-config`). |
| Trames reçues mais CRC invalide en permanence | Endianness du CRC inversé (le CRC est **big-endian**, cf. §3.2) ou trame décodée sur un mauvais alignement. |
| Texte `[SERIAL] …` entre les trames binaires | Normal en mode **USB-CDC** (debug partagé) : le `UplinkReader` du §7 filtre tout seul. Pour un lien 100 % propre, passer en **UART matériel**. |
| Trames de **71 octets** (au lieu de 72) | **Firmware antérieur au correctif** : reflasher le firmware (voir §10). Un parseur conforme au présent document les rejettera (CRC calculé sur 67 octets et dernier canal tronqué). |
| `wdg=True`/`dry=True` persistants dans la télémétrie | `wdg` : vérifier que vos trames partent bien toutes les < 500 ms (et CRC valide). `dry` : activer les sorties via l'interface Web (bouton « Sorties Matérielles »). |
| Propulseurs inertes malgré `arm_state = 1` | Dry-run actif (`dry=True`) ou watchdog déclenché. Vérifier `status` dans la télémétrie. |

---

## 10. Note de version — correctif critique de la trame montante (71 → 72 octets)

Un défaut historique affectait la trame montante : la constante de taille valait **71** alors que la structure `UplinkFrame_t` occupe **72** octets. Conséquences pour les firmwares antérieurs au correctif :

* trame émise sur le fil : **71 octets** (dernier octet jamais transmis) ;
* CRC calculé sur `[2..68]` et stocké aux offsets **69–70** au lieu de 70–71 ;
* l'octet de poids fort du canal `pwm_actual[15]` était **écrasé par le CRC**.

Le firmware actuel émet la trame **conforme à ce document (72 octets, CRC sur `[2..69]`)`. Si vous observez des trames de 71 octets sur le fil, le firmware en place est antérieur au correctif : **reflasher** (USB ou OTA depuis l'interface Web) avant d'intégrer le code côté RPi 5.

---

**Auteur :** DERO DIDIER — [ddero2014@gmail.com](mailto:ddero2014@gmail.com)
*Document garanti conforme au firmware BOB-CONTROL — toute modification du protocole (`include/protocol.h`) doit être répercutée ici.*
