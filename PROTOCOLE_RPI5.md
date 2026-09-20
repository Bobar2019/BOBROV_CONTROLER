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
| Trames | **Tailles fixes** — descendante : **40 octets** ; montantes : **73 octets** (télémétrie) et **27 octets** (configuration des noms, §5.6) |
| Intégrité | **CRC16-CCITT-FALSE** (polynôme `0x1021`, init `0xFFFF`) sur chaque trame |
| Trame descendante | 20–50 Hz recommandé (commande : masque d'assistances, armement, 16 consignes PWM, masque GPIO ON/OFF) |
| Trame montante | **100 Hz fixe** (télémétrie : IMU, pression, température, puissance, PWM, état des sorties ON/OFF) |
| Synchronisation | Requête descendante `0xFF` (§4.4) ⇒ l'ESP32 renvoie les **noms personnalisés** des sorties (rafale de 10 trames de **27 octets**, §5.6). Aussi poussée automatiquement à la première commande reçue après (re)connexion, et après chaque modification via l'interface Web |
| Sorties ON/OFF | 4 sorties numériques (GPIO 15/16/17/18 par défaut) pilotées par `gpio_cmd`, état réel dans `gpio_state` — forcées à OFF sur watchdog ou E-Stop |
| Failsafe | Watchdog **500 ms** par défaut : perte de liaison ⇒ tous les actionneurs à leur position de sécurité + 4 sorties ON/OFF à OFF |
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
| RPi 5 → ESP32-S3 (descendante) | `0xAA` `0x55` | `0x01` | **40 octets** |
| RPi 5 → ESP32-S3 (descendante) | `0xAA` `0x55` | `0xFF` | **40 octets** — requête de synchronisation, contenu ignoré (§4.4) |
| ESP32-S3 → RPi 5 (montante) | `0x55` `0xAA` | `0x02` | **73 octets** |
| ESP32-S3 → RPi 5 (montante) | `0x55` `0xAA` | `0x03` | **27 octets** — nom d'un canal PWM auxiliaire 10–15 (§5.6) |
| ESP32-S3 → RPi 5 (montante) | `0x55` `0xAA` | `0x04` | **27 octets** — nom d'une sortie tout-ou-rien 1–4 (§5.6) |

Les en-têtes ont des motifs **opposés** (`AA 55` vs `55 AA`) : un simple `find` sur la séquence correcte suffit à se resynchroniser dans un flux bruité.

### 3.2 CRC16-CCITT-FALSE

Le CRC couvre les octets **de l'offset 2 jusqu'à l'avant-dernier octet de la trame** (le champ CRC lui-même est exclu) :

| Trame | Octets couverts par le CRC | CRC stocké aux offsets |
| :--- | :--- | :--- |
| Descendante (40 octets) | `[2..37]` | `[38]` = octet **fort**, `[39]` = octet **faible** |
| Montante (73 octets) | `[2..70]` | `[71]` = octet **fort**, `[72]` = octet **faible** |
| Configuration des noms (27 octets) | `[2..24]` | `[25]` = octet **fort**, `[26]` = octet **faible** |

Paramètres : polynôme `0x1021`, valeur initiale `0xFFFF`, ni réflexion d'entrée/sortie, ni XOR final.
**Valeur de contrôle** : `crc16_ccitt(b"123456789") == 0x29B1`.

> **⚠️ Le CRC est stocké en big-endian** (octet fort d'abord), contrairement à **tous** les autres champs multi-octets de la trame qui sont en little-endian. C'est la source d'erreur n°1 des implémentations tierces — voir l'exemple validé §7.

### 3.3 Résynchronisation

Une trame descendante invalide (CRC KO) est **silencieusement ignorée** par le firmware : la machine à états repart en attente d'en-tête. Côté RPi 5, appliquer la même stratégie : chercher `0x55 0xAA`, tenter de décoder la trame, en cas de CRC invalide **avancer d'un seul octet** et recommencer (jamais sauter la trame d'un bloc).

**Depuis la v1.5.0, la trame montante n'a plus une taille unique** : la longueur attendue se déduit du **type à l'offset 2** — `0x02` ⇒ 73 octets, `0x03`/`0x04` ⇒ 27 octets, tout autre octet ⇒ faux en-tête, avancer d'un octet (voir la classe `UplinkReader` du §7).

---

## 4. Trame descendante — RPi 5 → ESP32-S3 (40 octets)

Format : `[0xAA][0x55][TYPE][MODE][ARM][PWM_0..15][GPIO_CMD][CRC_HI][CRC_LO]`

| Offset | Taille | Champ | Type | Valeurs | Description |
| ---: | ---: | :--- | :--- | :--- | :--- |
| 0 | 1 | `header1` | `uint8_t` | `0xAA` | En-tête 1 |
| 1 | 1 | `header2` | `uint8_t` | `0x55` | En-tête 2 |
| 2 | 1 | `type` | `uint8_t` | `0x01` | Type « commande » |
| 3 | 1 | `mode` | `uint8_t` | masque de bits | **Masque d'assistances superposables** (v1.4.0) — chaque bit active une boucle indépendante : bit 0 (`0x01`) = Auto Roulis/Tangage, bit 1 (`0x02`) = Tenue de Profondeur, bit 2 (`0x04`) = Auto Cap (Yaw), bit 3 (`0x08`) = Retour Surface (**prioritaire**). `0x00` = Passif (manuel direct). Exemple : `0x03` = Auto R/T + Tenue de Profondeur. Bits 4–7 ignorés. |
| 4 | 1 | `arm_state` | `uint8_t` | `0` `1` `2` | **0** = Désarmé, **1** = Armé, **2** = E-Stop (arrêt d'urgence) |
| 5 | 32 | `pwm[0..15]` | `uint16_t` × 16 (LE) | 1000–2000 | Consignes en **µs**. `pwm[k]` occupe les offsets `5 + 2k` (octet faible d'abord) : `pwm[0]` @ 5–6 … `pwm[15]` @ 35–36. Neutre : **1500**. Le firmware borne toute valeur à 1000–2000 µs. |
| 37 | 1 | `gpio_cmd` | `uint8_t` | masque 4 bits | Commandes des 4 sorties tout-ou-rien : **bit 0 = sortie 1** … **bit 3 = sortie 4** (1 = ON, 0 = OFF). Bits 4–7 ignorés. Les sorties sont pilotables **même désarmé** (auxiliaires) ; neutralisées à OFF par l'E-Stop et le watchdog. |
| 38 | 1 | `crc16_hi` | `uint8_t` | — | CRC16 (voir §3.2) — octet fort |
| 39 | 1 | `crc16_lo` | `uint8_t` | — | CRC16 — octet faible |

> **Composition des assistances (v1.4.0) :** le champ `mode` est un masque — les boucles PID se superposent librement (ex. `0x03` = Auto R/T + tenue de profondeur). À l'activation d'une tenue, la consigne est capturée sur place : **profondeur courante** pour le bit 1, **cap courant** pour le bit 2. Le **bit 3 (Retour Surface) est prioritaire** : il suspend l'Auto R/T et la tenue de profondeur et force les moteurs verticaux M5–M8 vers la surface (≈ 1700 µs) ; l'Auto Cap (bit 2) reste actif sur les horizontaux M1–M4.

### 4.1 Affectation des 16 canaux

| Canal | Actionneur | Plage utile | Neutre / repos |
| :--- | :--- | :--- | :--- |
| 0–7 | ESC propulseurs M1–M8 (M1–M4 horizontaux, M5–M8 verticaux) | 1000–2000 µs | 1500 µs |
| 8–9 | Servomoteurs Pan & Tilt caméra | 1000–2000 µs | 1500 µs |
| 10–11 | Pince / rotation d'outil | 1000–2000 µs | 1500 µs |
| 12–13 | Gradateurs MOSFET projecteurs LED | 1000 µs = 0 % → 2000 µs = 100 % | 1000 µs (0 %) |
| 14–15 | Canaux auxiliaires / extensions | 1000–2000 µs | 1500 µs |

> **Depuis le protocole v1.1.0**, les canaux 10 à 15 sont **typables individuellement** (Bidirectionnel / Unidirectionnel), configurables dans l'onglet Paramètres de l'interface Web et persistés en NVS — voir §4.3.

### 4.2 Cadence et comportement d'émission

* **Envoyer en continu**, même quand rien ne change : c'est ce flux qui alimente le watchdog (§6.1) et le statut « maître » (§6.3).
* Cadence recommandée : **20 à 50 trames/s** (une trame toutes les 20–50 ms). À 100 Hz, aucune contre-indication technique, seulement de la bande passante inutile.
* En cas de coupure, le failsafe s'active après le **timeout watchdog (500 ms par défaut)** : prévoir un envoi toutes les 400 ms *au pire* si l'application est chargée.

### 4.3 Typage des canaux 10–15 (Bidirectionnel / Unidirectionnel)

Chaque canal 10 à 15 est configurable **individuellement** depuis l'onglet Paramètres de l'interface Web (ressource NVS, appliqué à chaud — aucun redémarrage requis) :

| Type | Plage des consignes | Position de sécurité (failsafe / E-Stop / watchdog) |
| :--- | :--- | :--- |
| **Bidirectionnel** | 1000–2000 µs, neutre 1500 µs | **1500 µs** |
| **Unidirectionnel** (pleine échelle) | 1000 µs = 0 % → 2000 µs = 100 % | **1000 µs (0 %)** |

Configuration par défaut : CH10/11 (pinces/outils) et CH14/15 (auxiliaires) en **Bidirectionnel**, CH12/13 (gradateurs LED) en **Unidirectionnel**. Le firmware borne toujours les consignes à 1000–2000 µs quel que soit le type ; le type ne change que la **position de sécurité** appliquée par le failsafe (et l'affichage côté interface Web).

### 4.4 Requête de synchronisation — type `0xFF` (v1.5.0)

Trame descendante de **40 octets** au **format identique** à la commande standard (§4) — seul le champ `type` vaut `0xFF`. **Tout le contenu (offsets 3 à 37) est ignoré** : envoyer des zéros suffit (CRC calculé normalement sur `[2..37]`).

Effets à la réception côté firmware :

* l'ESP32 renvoie immédiatement les **noms personnalisés** des sorties (rafale de trames montantes `0x03`/`0x04`, §5.6) — **sans modifier** les consignes en cours (PWM, GPIO, armement) ;
* la trame **réarme le watchdog** (elle vaut preuve de liaison, comme une commande valide) et efface le bit `0x08` de la télémétrie.

**À envoyer par le cockpit :**

1. **au démarrage** (avant la première trame de commande) — le cockpit récupère les libellés des sorties avant d'afficher quoi que ce soit ;
2. après toute **reprise de liaison** — en pratique redondant : le firmware resynchronise déjà automatiquement à la première commande reçue après (re)connexion (§5.6.2), mais l'envoi explicite est inoffensif.

> Le firmware renvoie **toujours la configuration complète** (6 canaux PWM + 4 sorties TOR, 10 trames), que des noms soient personnalisés ou non : une trame à `name_len = 0` signifie « nom par défaut » — le RPi 5 applique alors son propre libellé.

---

## 5. Trame montante — ESP32-S3 → RPi 5 (73 octets)

Format : `[0x55][0xAA][TYPE][STATUS][QUAT][GYRO][PRESS][TEMP][POWER×6][PWM_ACT×16][GPIO_STATE][CRC]`

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
| 26 | 12 | `power[6]` | `uint16_t` × 6 (LE) | **÷ 1000** → V / A | Énergie propulsion (v1.1.0) : `[V_BAT, rés, signal_H, I_H, signal_V, I_V]` en mV / mA — voir §5.2 |
| 38 | 32 | `pwm_actual[16]` | `uint16_t` × 16 (LE) | µs (entiers) | Consignes PWM courantes. `pwm_actual[k]` occupe les offsets `38 + 2k`. |
| 70 | 1 | `gpio_state` | `uint8_t` | masque 4 bits — voir §5.5 | État **réel relu** des 4 sorties ON/OFF : bit 0 = sortie 1 … bit 3 = sortie 4 (1 = ON) |
| 71 | 2 | `crc16` | `uint16_t` (**BE**) | — | CRC16 sur `[2..70]` — octet fort puis faible |

### 5.1 Sémantique exacte des bits de `status`

| Bit | Masque | Signification réelle (comportement du firmware) |
| :--- | :--- | :--- |
| 0 | `0x01` | Réservé (toujours 0) |
| 1 | `0x02` | **Liaison établie** : positionné dès qu'**au moins une trame descendante valide a été reçue depuis le boot**. ⚠️ Ce n'est **pas** l'état d'armement courant — c'est un témoin de liaison (« le firmware a déjà entendu le RPi 5 »). |
| 2 | `0x04` | **Auto Roulis/Tangage réellement actif** (v1.4.0) : boucle PID roulis + tangage en cours d'exécution — 0 si la boucle est absente du masque ou **suspendue par le retour surface**. |
| 3 | `0x08` | **Watchdog série déclenché** : plus aucune trame descendante valide reçue depuis > timeout (défaut 500 ms). Redescend automatiquement dès qu'une trame valide arrive. |
| 4 | `0x10` | **Dry-run (Mode Témoin)** : sorties physiques PCA9685 neutralisées (les consignes sont mémorisées dans `pwm_actual` mais aucune impulsion n'est émise vers les actionneurs). |
| 5 | `0x20` | **Tenue de Profondeur réellement active** (v1.4.0) : boucle PID altitude en cours d'exécution — 0 si absente du masque ou **suspendue par le retour surface**. |
| 6 | `0x40` | **Auto Cap réellement actif** (v1.4.0) : boucle PID lacet en cours d'exécution (indépendante du retour surface). |
| 7 | `0x80` | **Retour Surface réellement actif** (v1.4.0) : verticaux M5–M8 forcés vers la surface, Roll/Pitch et tenue de profondeur suspendus. |

### 5.2 Détail des 6 mesures de puissance (v1.1.0 — INA3221)

**Sémantique v1.1.0 :** l'INA3221 (triple mesure, `0x40`) remplace le premier INA226 et supervise l'énergie de propulsion. **Le format binaire est inchangé** (mêmes 6 slots mV/mA, mêmes offsets) — seule la sémantique des valeurs change :

| Index `power` | Offset | Mesure | Source (bus I2C n°1) |
| :--- | ---: | :--- | :--- |
| `power[0]` | 26 | **Tension batterie propulsion V_BAT** (mV) | **INA3221** (`0x40`) canal 1 |
| `power[1]` | 28 | Réservé (**0**) | INA3221 canal 1 — pas de shunt sur ce canal |
| `power[2]` | 30 | Signal brut capteur horizontaux (mV) | INA3221 canal 2 — tension ACS770 lue sur IN- |
| `power[3]` | 32 | **Courant propulseurs horizontaux M1–M4** (mA) | **ACS770-100U** via INA3221 canal 2 (40 mV/A) |
| `power[4]` | 34 | Signal brut capteur verticaux (mV) | INA3221 canal 3 — tension ACS770 lue sur IN- |
| `power[5]` | 36 | **Courant propulseurs verticaux M5–M8** (mA) | **ACS770-100U** via INA3221 canal 3 (40 mV/A) |

* Le courant est émis en valeur **absolue** (mA) ; plage utile **±4,09 A** (au-delà, l'ACS770 sature — le firmware le traite comme un défaut de surintensité). Capteur absent ⇒ 0.
* Les champs `power[2]` / `power[4]` (signal brut mV) sont fournis pour diagnostic — le firmware applique déjà la conversion en Ampères dans `power[3]` / `power[5]`.
* Les deux **INA226 résiduels** (#2 Aux/LEDs `0x44`, #3 `0x45`) ne transitent **pas** dans la trame série : ils restent visibles uniquement dans la télémétrie WebSocket de l'interface Web embarquée (`power[3]` / `power[4]` du JSON).
* Ancienne sémantique (firmware ≤ v1.0.1) : `[V,I]` du Pi, Aux et Moteurs — un récepteur ignorant la version décodait 3 wattmètres.

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
* Si `status & 0x08` (**watchdog**) : les sorties ont été forcées à leur position de sécurité ; `pwm_actual` peut encore montrer les dernières consignes jusqu'à la trame suivante.

### 5.5 `gpio_state` — état réel des 4 sorties ON/OFF

`gpio_state` reflète l'état **physiquement relu** des 4 sorties tout-ou-rien (niveau des broches après écriture) — et non la dernière commande reçue. Si `gpio_cmd` (trame descendante) et `gpio_state` diffèrent durablement, la commande n'a pas pu être appliquée.

* **Watchdog déclenché** (`status & 0x08`) : les 4 sorties sont forcées **à OFF** ⇒ `gpio_state = 0x00`.
* **E-Stop** (`arm_state = 2`) : idem, `gpio_state = 0x00`.
* Les broches sont assignées par défaut à **GPIO 15/16/17/18** (reconfigurables dans l'onglet Paramètres de l'interface Web, appliquées au redémarrage) et initialisées **à OFF** au boot.
* Les sorties ON/OFF sont indépendantes du **dry-run** (§6.4) : le dry-run ne concerne que les sorties PWM/PCA9685.

### 5.6 Trames de configuration des noms — types `0x03` / `0x04` (v1.5.0)

**Source unique de vérité :** l'ESP32-S3 détient les **noms personnalisés** des sorties (saisis dans l'interface Web, persistés en NVS) et les transmet au RPi 5 — le cockpit n'a **aucune valeur codée en dur** : il affiche les libellés reçus, ou son propre libellé par défaut quand `name_len = 0`.

#### 5.6.1 Format — 27 octets par sortie

Format : `[0x55][0xAA][TYPE][ID][LEN][NAME ×20][CRC_HI][CRC_LO]`

| Offset | Taille | Champ | Type | Valeurs | Description |
| ---: | ---: | :--- | :--- | :--- | :--- |
| 0 | 1 | `header1` | `uint8_t` | `0x55` | En-tête 1 |
| 1 | 1 | `header2` | `uint8_t` | `0xAA` | En-tête 2 |
| 2 | 1 | `type` | `uint8_t` | `0x03` `0x04` | `0x03` = nom d'un canal PWM auxiliaire (10–15) ; `0x04` = nom d'une sortie tout-ou-rien |
| 3 | 1 | `id` | `uint8_t` | 10–15 / 1–4 | Numéro de **canal PWM** (`0x03`) ou de **sortie TOR** (`0x04`) — voir §5.6.3 |
| 4 | 1 | `name_len` | `uint8_t` | 0–20 | Longueur **utile** du nom en octets UTF-8. **0 = nom par défaut** (le RPi 5 applique son libellé par défaut) |
| 5 | 20 | `name` | `char[20]` | UTF-8 | Nom, complété par des `0x00` au-delà de `name_len` — lire exactement `name_len` octets |
| 25 | 1 | `crc16_hi` | `uint8_t` | — | CRC16 sur `[2..24]` — octet **fort** |
| 26 | 1 | `crc16_lo` | `uint8_t` | — | CRC16 — octet **faible** |

#### 5.6.2 Déclencheurs d'envoi (comportement du firmware)

Les **10 trames** (6 PWM + 4 TOR) sont émises en **rafale lissée** : **une trame par cycle de télémétrie (10 ms)**, intercalée entre deux trames `0x02` complètes — la cadence de télémétrie **100 Hz reste intacte**. La rafale complète s'étale sur ~100 ms.

| Déclencheur | Quand |
| :--- | :--- |
| **Requête `0xFF`** | À réception de la trame descendante `0xFF` (§4.4) — cas nominal au démarrage du cockpit |
| **Première commande après (re)connexion** | À la première trame `0x01` valide reçue après le boot de l'ESP32 **ou** après un déclenchement du watchdog — le cockpit reçoit les noms même s'il n'émet jamais de requête `0xFF` |
| **Modification dans l'interface Web** | À chaque sauvegarde des noms (`POST /api/names`) — mise à jour du cockpit en temps réel, sans redémarrage |

#### 5.6.3 Sémantique des `id`

| `type` | `id` | Sortie concernée |
| :--- | :--- | :--- |
| `0x03` | 10–15 | Canaux PWM auxiliaires 10 à 15 (pinces/outils, gradateurs, auxiliaires — voir §4.1) |
| `0x04` | 1–4 | Sorties tout-ou-rien 1 à 4 (**id = bit + 1**, cohérent avec `gpio_cmd`/`gpio_state` §4/§5.5) |

Les canaux **0 à 9** (propulseurs M1–M8, servos caméra) ne sont **pas** transmis : ce sont des affectations fixes du véhicule.

#### 5.6.4 Traitement recommandé côté RPi 5

* **Au démarrage** : envoyer `build_sync_request()` (§7), puis émettre les commandes normales ; attendre la rafale (~100 ms) avant d'afficher les libellés.
* **À chaud** : chaque trame `0x03`/`0x04` remplace le libellé de la sortie `id` — appliquer immédiatement, sans redémarrer.
* `name_len = 0` ⇒ appliquer votre libellé par défaut. Nota : le **pilotage** utilise toujours `id`/masques (`gpio_cmd`, index PWM) — jamais le nom `name`.

---

## 6. Comportements du firmware (règles de sécurité et de priorité)

### 6.1 Watchdog série — failsafe 500 ms

* Le firmware attend **au moins une trame descendante valide** dans une fenêtre de **500 ms** (défaut, réglable 100 ms–plusieurs s dans l'onglet Paramètres via NVS — clé `serial_timeout`).
* Dépassement ⇒ **tous les canaux PWM forcés à leur position de sécurité** (1500 µs pour les bidirectionnels, 1000 µs / 0 % pour les unidirectionnels) et **les 4 sorties ON/OFF forcées à OFF** (`gpio_state = 0x00`). Bit `0x08` positionné dans la télémétrie.
* Dès qu'une trame valide arrive : le watchdog est réarmé, le bit `0x08` disparaît, et **les consignes de cette trame sont appliquées au cycle suivant (≤ 10 ms)**.
* Au démarrage du firmware : le watchdog **ne se déclenche pas** tant qu'aucune trame n'a jamais été reçue (les sorties restent de toute façon en dry-run/neutre par défaut).

### 6.2 États d'armement (champ `arm_state`)

| `arm_state` | Effet immédiat à la réception de CHAQUE trame |
| :--- | :--- |
| **0 — Désarmé** | Canaux **0–7 forcés au neutre** (1500 µs). Canaux **8–15 acceptés tels quels** (pan/tilt, pince, gradateurs utilisables désarmé). Les **4 sorties ON/OFF restent pilotables** (`gpio_cmd` appliqué — auxiliaires utilisables désarmé). Reset des PID. |
| **1 — Armé** | **Les 16 canaux** sont appliqués (`pwm[0..15]`) ainsi que le **masque `gpio_cmd`**. Le **masque d'assistances** est appliqué (superposition Auto R/T, tenue de profondeur, Auto Cap, retour surface). Le contrôleur de vol considère le RPi 5 comme maître. |
| **2 — E-Stop** | **Tous** les canaux à leur position de sécurité (1500 µs / 0 %) + **les 4 sorties ON/OFF forcées à OFF** + reset des PID. Priorité absolue, appliqué à chaque trame reçue. |

> **Point opérationnel :** seules les trames **ARMÉES** transmettent le `mode` et rafraîchissent la maîtrise du RPi 5 (§6.3). Pour reprendre la main après un long arrêt, il suffit de renvoyer des trames armées.

### 6.3 Priorité « maître » RPi 5 / interface Web ESP32

* Chaque trame **armée** reçue marque le RPi 5 comme **maître** du contrôleur de vol.
* Après **1000 ms sans trame armée**, le firmware repasse en mode manuel ESP32 (l'interface Web peut alors changer le mode ; journal série : `[FLIGHT] Maître → Manuel ESP32 (timeout RPi 5)`).
* Tant que le RPi 5 est maître, toute commande de mode émise par l'interface Web est **ignorée** (journal : `[FLIGHT] Commande Web ignorée — RPi 5 est maître`).
* Conséquence pratique : **envoyer des trames armées en continu** (c'est le cas nominal à 20–50 Hz) pour rester maître.

### 6.4 Mode Dry-Run (Sorties Matérielles)

Au boot, les sorties physiques sont **neutralisées** (dry-run actif, bit `0x10`). Les consignes reçues sont suivies dans `pwm_actual` mais le PCA9685 n'émet rien. L'activation des sorties réelles se fait **uniquement** depuis l'interface Web (bouton « Sorties Matérielles », WebSocket `{output_enable: true}` ou REST `POST /api/pwm/output-enable`). C'est un garde-fou de banc : le RPi 5 ne peut pas le désactiver à distance.

> **Sorties ON/OFF :** le dry-run ne les concerne pas — les 4 GPIO sont des sorties numériques à part entière, initialisées à OFF au boot puis pilotables par `gpio_cmd` (et forcées à OFF par le watchdog et l'E-Stop, voir §5.5).

### 6.5 Redémarrage automatique du firmware (perte définitive de l'IMU)

Le firmware embarque un **watchdog IMU** dédié au BNO085 (capteur d'attitude, seul sur le bus I2C n°1). Si le flux de quaternions se fige en opération (plongée, longue inactivité), la récupération escalade automatiquement en 3 niveaux :

1. **Reprise douce SH-2** — après 1 s sans quaternion : ré-initialisation logicielle du capteur (réécriture des rapports SH-2) ;
2. **Reset matériel du bus I2C** — si le niveau 1 échoue ou si le contrôleur I2C reste bloqué (`ESP_ERR_INVALID_STATE`) : destruction puis recréation de l'instance `Adafruit_BNO08x`, 9 impulsions SCL + condition STOP (bit-bang), réinitialisation complète de `Wire` ;
3. **Redémarrage contrôlé** — si le capteur (qui **a déjà fonctionné depuis le boot**) ne répond toujours pas après **3 tentatives de niveau 2** ou **8 s sans aucun quaternion** : le firmware force **tous les canaux PWM à leur position de sécurité** (1500 µs, 0 % pour les unidirectionnels), émet un log explicite sur le moniteur série, puis déclenche `esp_restart()` — **redémarrage automatique de l'ESP32**.

**Ce que le RPi 5 doit savoir :**

* **Propulsion au neutre pendant toute la phase de récupération (niveaux 1 & 2)** : dès le 1er échec, les canaux 0-7 sont verrouillés à 1500 µs **à chaque cycle de contrôle** — une consigne armée réappliquée entre deux tentatives ne peut jamais faire repartir la propulsion avec une attitude inconnue. Les canaux 8-15 (servos, pince, gradateurs) restent pilotables.
* **Pendant le redémarrage (~2-3 s)** : la télémétrie est interrompue, **mais les propulseurs restent physiquement au neutre** — le PCA9685 est un générateur autonome qui conserve ses registres pendant le reboot de l'ESP32 (neutre maintenu sans aucune intervention). Les trames descendantes émises pendant cette fenêtre sont simplement perdues.
* **Après le redémarrage** : la télémétrie reprend seule. Le bit **liaison établie (`0x02`) repasse à 0 puis à 1** dès la première trame descendante valide reçue, et les sorties repartent en **dry-run (`0x10` = 1)** — comme à chaque boot, la réactivation des sorties réelles redevient une action manuelle dans l'interface Web.
* **Aucune action spécifique n'est requise côté RPi 5** : continuer à émettre normalement (la cadence 20-50 Hz ré-établit liaison et maîtrise automatiquement).

> **Garde-fou anti-boucle :** ce redémarrage ne se produit que si le capteur **a déjà fourni au moins un quaternion depuis le boot**. Un BNO085 jamais détecté ou jamais fonctionnel au démarrage n'est **jamais** redémarré en boucle : il reste en mode espacé (badge rouge, une tentative complète toutes les 30 s) et la propulsion demeure au neutre par défaut.

---

## 7. Exemple Python complet (validé hors ligne)

Code autonome, sans dépendance autre que **pyserial** (`pip install pyserial`). Toutes les fonctions de décodage ont été vérifiées contre les vecteurs de test du §8.

```python
#!/usr/bin/env python3
"""Cockpit minimal BOB-ROV — liaison binaire avec BOB-CONTROL (ESP32-S3).
Télémétrie 100 Hz + synchronisation des noms de sorties au démarrage."""
import struct
import time
import serial

# ==================== Constantes du protocole ====================

PORT = '/dev/ttyACM0'      # USB-CDC natif  |  UART GPIO : '/dev/serial0'
BAUD = 921600
TX_HZ = 25.0               # cadence descendante recommandée : 20-50 Hz

# Mode = masque d'assistances superposables (v1.4.0) — combiner avec |
MODE_PASSIF = 0x00
MODE_RT, MODE_DEPTH, MODE_CAP, MODE_SURFACE = 0x01, 0x02, 0x04, 0x08

ARM_DISARMED, ARM_ARMED, ARM_ESTOP = 0, 1, 2

STATUS_ARMED, STATUS_RT, STATUS_WDG, STATUS_DRYRUN = 0x02, 0x04, 0x08, 0x10
STATUS_DEPTH, STATUS_CAP, STATUS_SURFACE = 0x20, 0x40, 0x80

# Trames (v1.5.0) : type à l'offset 2 + taille fixe associée
UL_TELEMETRY, UL_CFG_PWM, UL_CFG_GPIO = 0x02, 0x03, 0x04
DL_SYNC = 0xFF             # requête de synchronisation (descendante, 40 octets)
FRAME_SIZES = {UL_TELEMETRY: 73, UL_CFG_PWM: 27, UL_CFG_GPIO: 27}

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

# ==================== Trame descendante (40 octets) ====================

def build_downlink_frame(mode: int, arm_state: int, pwm_us, gpio_cmd: int = 0) -> bytes:
    """Construit la trame de commande RPi 5 -> ESP32-S3 (40 octets).
    mode : masque d'assistances superposables (MODE_RT | MODE_DEPTH | ...).
    pwm_us : séquence de 16 valeurs en microsecondes (1000-2000).
    gpio_cmd : masque 4 bits des sorties ON/OFF (bit 0 = sortie 1, 1 = ON)."""
    payload = struct.pack('<BBB16HB', 0x01, mode, arm_state, *pwm_us, gpio_cmd)
    crc = crc16_ccitt(payload)                 # CRC sur [2..37]
    return bytes((0xAA, 0x55)) + payload + struct.pack('>H', crc)   # CRC big-endian

def build_sync_request() -> bytes:
    """Requête de synchronisation (40 octets, contenu ignoré par le firmware).
    À envoyer au démarrage du cockpit : l'ESP32 renvoie alors les noms
    personnalisés des sorties (trames montantes 0x03/0x04, voir §5.6)."""
    payload = struct.pack('<B35x', DL_SYNC)    # type 0xFF + 35 octets à zéro
    crc = crc16_ccitt(payload)                 # CRC sur [2..37]
    return bytes((0xAA, 0x55)) + payload + struct.pack('>H', crc)   # CRC big-endian

# ==================== Trame montante (73 octets) ====================

def parse_uplink_frame(frame: bytes) -> dict:
    """Valide et décode une trame de télémétrie (ESP32-S3 -> RPi 5)."""
    if len(frame) != 73:
        raise ValueError(f'trame de {len(frame)} octets (attendu 73)')
    if frame[0] != 0x55 or frame[1] != 0xAA or frame[2] != 0x02:
        raise ValueError('en-tête invalide')
    if (frame[71] << 8 | frame[72]) != crc16_ccitt(frame[2:71]):    # CRC sur [2..70]
        raise ValueError('CRC16 invalide')
    (type_, status, w, x, y, z, gx, gy, gz, pressure, temperature, *rest) = \
        struct.unpack_from('<BB4h3h2i6H16HB', frame, 2)
    return {
        'status': status,
        'armed': bool(status & STATUS_ARMED),      # liaison établie (≥ 1 trame reçue)
        'auto_rt': bool(status & STATUS_RT),       # Auto Roulis/Tangage réellement actif
        'depth_hold': bool(status & STATUS_DEPTH), # tenue de profondeur active
        'cap_hold': bool(status & STATUS_CAP),     # Auto Cap actif
        'surface': bool(status & STATUS_SURFACE),  # retour surface prioritaire actif
        'watchdog': bool(status & STATUS_WDG),
        'dry_run': bool(status & STATUS_DRYRUN),
        'quat': (w / 10000.0, x / 10000.0, y / 10000.0, z / 10000.0),
        'gyro': (gx / 100.0, gy / 100.0, gz / 100.0),          # °/s
        'pressure_mbar': pressure / 10.0,                       # mbar
        'temperature_c': temperature / 100.0,                   # °C
        'power': list(rest[:6]),                                # mV / mA
        'pwm_us': list(rest[6:22]),                             # µs
        'gpio_state': rest[22],                                 # masque réel des 4 sorties ON/OFF
    }

# ==================== Trame de configuration des noms (27 octets) ====================

def parse_config_frame(frame: bytes) -> dict:
    """Valide et décode une trame de configuration des noms (types 0x03/0x04)."""
    if len(frame) != 27:
        raise ValueError(f'trame de {len(frame)} octets (attendu 27)')
    if frame[0] != 0x55 or frame[1] != 0xAA or frame[2] not in (UL_CFG_PWM, UL_CFG_GPIO):
        raise ValueError('en-tête ou type invalide')
    if (frame[25] << 8 | frame[26]) != crc16_ccitt(frame[2:25]):   # CRC sur [2..24]
        raise ValueError('CRC16 invalide')
    name_len = frame[4]
    if name_len > 20:
        raise ValueError('name_len > 20')
    return {
        'kind': 'pwm' if frame[2] == UL_CFG_PWM else 'gpio',
        'id': frame[3],                                            # canal 10-15 | sortie 1-4
        'name': frame[5:5 + name_len].decode('utf-8', errors='replace'),
        'default_name': name_len == 0,                             # → appliquer votre libellé
    }

class UplinkReader:
    """Réassemble les trames montantes (73 ou 27 octets) dans un flux bruité :
    resynchronisation sur 0x55 0xAA, taille déduite du type (offset 2)."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        """Ajoute des octets reçus ; retourne la liste des trames valides (bytes)."""
        self.buf.extend(data)
        frames = []
        while True:
            idx = self.buf.find(b'\x55\xaa')
            if idx < 0:
                del self.buf[:-1]          # conserve au plus 1 octet (préfixe possible)
                break
            del self.buf[:idx]
            if len(self.buf) < 3:
                break
            size = FRAME_SIZES.get(self.buf[2])
            if size is None:               # type inconnu : faux en-tête probable
                del self.buf[:1]
                continue
            if len(self.buf) < size:
                break
            frame = bytes(self.buf[:size])
            if (frame[size - 2] << 8 | frame[size - 1]) == crc16_ccitt(frame[2:size - 2]):
                frames.append(frame)
                del self.buf[:size]
            else:
                del self.buf[:1]           # CRC invalide : avancer d'un octet
        return frames

# ==================== Boucle principale ====================

def main():
    port = serial.Serial(PORT, BAUD, timeout=0)   # lecture non bloquante
    reader = UplinkReader()
    names = {}                                     # libellés reçus de l'ESP32 (v1.5.0)
    pwm = [1500] * 16                              # toutes voies au neutre
    pwm[0] = 1600                                  # exemple : M1 en avant (µs)
    gpio = 0b0000                                  # sorties ON/OFF (bit 0 = sortie 1)

    port.write(build_sync_request())               # demande les noms (trames 0x03/0x04)

    next_tx = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now >= next_tx:                     # --- émission descendante 25 Hz ---
                next_tx += 1.0 / TX_HZ
                port.write(build_downlink_frame(MODE_PASSIF, ARM_ARMED, pwm, gpio))

            for frame in reader.feed(port.read(port.in_waiting or 4096)):
                if frame[2] == UL_TELEMETRY:       # --- télémétrie (73 octets) ---
                    t = parse_uplink_frame(frame)
                    print('quat=(%.3f, %.3f, %.3f, %.3f)  P=%.1f mbar  T=%.1f C  '
                          'pwm0=%d  gpio=0x%X  wdg=%s  dry=%s'
                          % (*t['quat'], t['pressure_mbar'], t['temperature_c'],
                             t['pwm_us'][0], t['gpio_state'], t['watchdog'], t['dry_run']))
                else:                              # --- config noms (27 octets) ---
                    c = parse_config_frame(frame)
                    names[(c['kind'], c['id'])] = c['name'] or f"<défaut {c['kind']}{c['id']}>"
                    print('config :', c['kind'], c['id'], '->', names[(c['kind'], c['id'])])
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

`MODE_PASSIF` + `ARM_ARMED` + toutes voies à 1500 µs + `gpio_cmd = 0x05` (sorties 1 et 3 à ON) :

```
AA 55 01 00 01 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 05 30 72
```

* 40 octets. `DC 05` = `0x05DC` = 1500 LE, répété 16 fois. `gpio_cmd = 0x05` à l'offset 37. CRC final `0x3072` (octet fort `0x30` puis `0x72`).
* Génération : `build_downlink_frame(0, 1, [1500]*16, 0x05).hex() == 'aa55010001' + 'dc05'*16 + '05' + '3072'``

### 8.3 Trame montante de référence

Télémétrie à l'équilibre (quaternion identité, 1013,2 mbar, 25,00 °C), `status = 0x12` (liaison + dry-run), canal 0 à 1600 µs, les autres à 1500, `gpio_state = 0x05` (sorties 1 et 3 à ON) :

```
55 AA 02 12 10 27 00 00 00 00 00 00 00 00 00 00
00 00 94 27 00 00 C4 09 00 00 88 13 20 03 E0 2E
2C 01 D0 39 68 5B 40 06 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05
DC 05 DC 05 DC 05 05 B3 01
```

Décodage attendu :

| Champ | Valeur décodée |
| :--- | :--- |
| `status` | `0x12` → liaison établie + dry-run (`watchdog` = 0) |
| `quat` | `(1.0, 0.0, 0.0, 0.0)` |
| `gyro` | `(0.0, 0.0, 0.0)` °/s |
| `pressure_mbar` | `1013.2` |
| `temperature_c` | `25.0` |
| `power` | `[5000, 800, 12000, 300, 14800, 23400]` → sémantique v1.1.0 (valeurs de test arbitraires) : V_BAT 5,0 V, réservé 800, signal H 12,0 V / courant H 0,3 A, signal V 14,8 V / courant V 23,4 A — voir §5.2 |
| `pwm_us` | `[1600, 1500, 1500, …, 1500]` (16 valeurs) |
| `gpio_state` | `0x05` → sorties 1 et 3 à ON (offset 70) |
| CRC | `0xB301` (offset 71 = `0xB3`, offset 72 = `0x01`) |

### 8.4 Test de robustesse du réassemblage

Injecter du bruit texte, couper une trame en deux, insérer un faux en-tête `55 AA 02 00`, puis vérifier que `UplinkReader.feed()` restitue exactement **2 trames valides**. Le code du §7 (classe `UplinkReader`) est conçu pour ce cas : validé sur la séquence `bruit + moitié1`, puis `moitié2 + faux en-tête + trame complète + bruit`.

Le même test avec une **trame de configuration intercalée** (27 octets) entre deux trames de télémétrie doit restituer **3 trames valides** (2× `0x02` + 1× `0x03`/`0x04`) — c'est le cas réel sur le fil dès qu'une synchronisation est en cours.

### 8.5 Trame de configuration PWM de référence — type `0x03`

Canal **11** nommé « Treuil avant » (`name_len = 0x0C` = 12 octets) :

```
55 AA 03 0B 0C 54 72 65 75 69 6C 20 61 76 61 6E
74 00 00 00 00 00 00 00 00 38 54
```

* 27 octets. `id = 0x0B` = canal 11 ; nom `54 72 65 75 69 6C 20 61 76 61 6E 74` = « Treuil avant », complété de `00` (offsets 17–24) ; CRC `0x3854` sur `[2..24]`.
* Vérification :

```python
assert parse_config_frame(bytes.fromhex(
    '55aa030b0c' + '54726575696c206176616e74' + '00' * 8 + '3854')) \
    == {'kind': 'pwm', 'id': 11, 'name': 'Treuil avant', 'default_name': False}
```

### 8.6 Trame de configuration TOR de référence — nom par défaut

Sortie **2** sans nom personnalisé (`name_len = 0` ⇒ le RPi 5 applique son libellé par défaut) :

```
55 AA 04 02 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 99 EB
```

* 27 octets. `id = 0x02` = sortie 2 ; `name_len = 0` ; CRC `0x99EB`.
* Vérification :

```python
assert parse_config_frame(bytes.fromhex('55aa0402' + '00' * 21 + '99eb')) \
    == {'kind': 'gpio', 'id': 2, 'name': '', 'default_name': True}
```

### 8.7 Requête de synchronisation de référence — type `0xFF`

Trame descendante minimale (contenu à zéro — ignoré par le firmware) :

```
AA 55 FF 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 CA 33
```

* 40 octets. `type = 0xFF` à l'offset 2 ; CRC `0xCA33` sur `[2..37]`.
* Vérification :

```python
assert build_sync_request().hex() == 'aa55ff' + '00' * 35 + 'ca33'
```

---

## 9. Dépannage côté RPi 5

| Symptôme | Cause probable / Remède |
| :--- | :--- |
| `PermissionError` à l'ouverture du port | Utilisateur hors groupe `dialout` : `sudo usermod -aG dialout $USER` puis reconnexion. |
| Aucune trame reçue | 1) Interface choisie côté ESP32 (onglet Paramètres : USB-CDC / UART GPIO) ≠ branchement. 2) Baud ≠ **921600**. 3) UART : TX/RX **croisés** et GND commun. 4) UART Pi non activé (`raspi-config`). |
| Trames reçues mais CRC invalide en permanence | Endianness du CRC inversé (le CRC est **big-endian**, cf. §3.2) ou trame décodée sur un mauvais alignement. |
| Texte `[SERIAL] …` entre les trames binaires | Normal en mode **USB-CDC** (debug partagé) : le `UplinkReader` du §7 filtre tout seul. Pour un lien 100 % propre, passer en **UART matériel**. |
| Trames de **39/72 octets** (au lieu de 40/73) | **Firmware antérieur à la v1.1.0** (sans sorties ON/OFF, voir §10.1) : reflasher le firmware (USB ou OTA). Un parseur conforme au présent document rejettera ces trames (CRC décalé d'un octet). |
| Trames de **71 octets** (au lieu de 73) | **Firmware antérieur au correctif historique de taille montante** (voir §10.2) : reflasher. |
| Interruption de télémétrie de ~2-4 s puis reprise avec `dry=True` | **Normal** : redémarrage automatique du watchdog IMU (BNO085 définitivement muet, §6.5). La propulsion est restée au neutre pendant toute l'opération ; réactiver les sorties depuis l'interface Web si nécessaire. |
| `wdg=True`/`dry=True` persistants dans la télémétrie | `wdg` : vérifier que vos trames partent bien toutes les < 500 ms (et CRC valide). `dry` : activer les sorties via l'interface Web (bouton « Sorties Matérielles »). |
| Propulseurs inertes malgré `arm_state = 1` | Dry-run actif (`dry=True`) ou watchdog déclenché. Vérifier `status` dans la télémétrie. |

---

## 10. Notes de version

### 10.1 v1.1.0 — ajout des sorties tout-ou-rien (39/72 → 40/73 octets)

La version 1.1.0 du protocole ajoute les 4 sorties ON/OFF :

* trame descendante : champ `gpio_cmd` à l'offset **37** ⇒ **40 octets**, CRC sur `[2..37]` stocké en `[38]`/`[39]` ;
* trame montante : champ `gpio_state` à l'offset **70** ⇒ **73 octets**, CRC sur `[2..70]` stocké en `[71]`/`[72]`.

Un firmware antérieur (v1.0.0) émet et attend des trames de **39/72 octets** : l'incompatibilité est **binaire et totale** — les deux extrémités (firmware ESP32 et code RPi 5) doivent être mises à jour ensemble. Le firmware actuel embarque des `static_assert` (vérifiés à la compilation) garantissant tailles et offsets.

### 10.2 Correctif historique — trame montante 71 → 72 octets

Un défaut historique affectait la trame montante : la constante de taille valait **71** alors que la structure `UplinkFrame_t` occupait **72** octets. Conséquences pour les firmwares antérieurs à ce correctif :

* trame émise sur le fil : **71 octets** (dernier octet jamais transmis) ;
* CRC calculé sur `[2..68]` et stocké aux offsets **69–70** (au lieu de 71–72 depuis la v1.1.0, voir §10.1) ;
* l'octet de poids fort du canal `pwm_actual[15]` était **écrasé par le CRC**.

Si vous observez des trames de **71 octets** sur le fil, le firmware en place est antérieur à ce correctif : **reflasher** (USB ou OTA depuis l'interface Web). Le firmware actuel émet la trame **conforme à ce document** (73 octets, CRC sur `[2..70]`).

### 10.3 v1.4.0 — mode combinatoire (masque de bits) et confirmations d'assistance

**Tailles et offsets inchangés** (40/73 octets) : seuls les champs `mode` (offset 3, descendante) et `status` (offset 3, montante) changent de sémantique.

* **Descendante** : `mode` n'est plus une valeur énumérée (0, 1, 2) mais un **masque de bits** — bit 0 (`0x01`) Auto R/T, bit 1 (`0x02`) Tenue de Profondeur, bit 2 (`0x04`) Auto Cap, bit 3 (`0x08`) Retour Surface (prioritaire). Les assistances se **superposent** (ex. `0x03` = Auto R/T + Tenue de Profondeur) ; bits 4–7 ignorés (masqués, sans erreur CRC).
* **Montante** : `status` confirme l'activation **réelle et combinée** des boucles au moment de l'émission — bit 2 (`0x04`) Auto R/T, bit 5 (`0x20`) Tenue de Profondeur, bit 6 (`0x40`) Auto Cap, bit 7 (`0x80`) Retour Surface.
* **Sémantique de capture** : l'activation de la tenue de profondeur capture la **profondeur courante** comme consigne (maintien sur place) ; l'activation de l'Auto Cap capture le **cap courant**. Ces consignes sont recapturées à chaque changement de masque.
* ⚠️ **Incompatibilité sémantique** : un code conçu pour l'ancien protocole qui envoie `mode = 2` (ex-Auto Full) n'active plus que la **tenue de profondeur**. L'équivalent de l'ancien Auto Full est désormais `0x07` (R/T + profondeur + cap).

### 10.4 v1.5.0 — synchronisation des noms (source unique de vérité)

**Tailles des trames principales inchangées** (40 descendante / 73 montante) : la v1.5.0 ajoute des types de trames nouveaux — un parseur existant continue de fonctionner (voir compatibilité ci-dessous).

* **Nouvelle trame descendante `0xFF`** — requête de synchronisation (40 octets, contenu ignoré) : le RPi 5 l'envoie au démarrage pour récupérer les noms personnalisés (§4.4). Elle réarme le watchdog et ne modifie aucune consigne.
* **Nouvelles trames montantes `0x03`** (noms des canaux PWM 10–15, `id` = canal) et **`0x04`** (noms des sorties TOR 1–4, `id` = sortie) — **27 octets chacune**, émises une fois par sortie en **rafale lissée** (une trame par cycle de 10 ms, intercalée entre deux télémétries complètes) sur trois déclencheurs : requête `0xFF`, première commande reçue après (re)connexion, ou modification des noms via l'interface Web (§5.6).
* **Compatibilité ascendante** : un ancien parseur qui filtre `type == 0x02` et la taille 73 ignore naturellement les trames `0x03`/`0x04` (la resynchronisation octet par octet retrouve la télémétrie suivante). Il est toutefois **recommandé** d'étendre le lecteur au dispatch par taille (§3.3 / §7) pour exploiter la synchronisation.
* **Compatibilité descendante** : un ancien firmware (avant v1.5.0) qui reçoit une requête `0xFF` la rejette comme type inconnu (CRC valide requis) — sans effet de bord. Le cockpit peut donc émettre la requête inconditionnellement au démarrage.

---

**Auteur :** DERO DIDIER — [ddero2014@gmail.com](mailto:ddero2014@gmail.com)
*Document garanti conforme au firmware BOB-CONTROL — toute modification du protocole (`include/protocol.h`) doit être répercutée ici.*
