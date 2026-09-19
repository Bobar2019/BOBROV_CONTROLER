# BOB-CONTROL - Contrôleur Temps Réel ROV

**Version :** 1.3.0  
**Plateforme :** ESP32-S3 (PlatformIO / Arduino Core / FreeRTOS)  
**Auteur :** Didier Dero  
**Projet associé :** BOB-ROV (Cockpit Raspberry Pi 5)  
**Date :** Septembre 2026  

---

## 🎯 Philosophie & Rôle Système

**BOB-CONTROL** est le micrologiciel temps réel dédié à la gestion basse couche du sous-marin **BOB-ROV**. Embarqué sur un **ESP32-S3**, il isole les fonctions critiques de pilotage matériel et d'acquisition de données du système d'exploitation hôte (Raspberry Pi 5).

Il remplit un quadruple rôle :
1. **Acquisition & Fusion Capteurs :** Lecture haute fréquence des deux bus I2C matériels (centrale inertielle BNO085 **seule** sur le bus n°1 ; triple mesure d'énergie INA3221, wattmètres INA226, pression/profondeur MS5837 et contrôleur PWM PCA9685 sur le bus n°2 — chaque module étant re-routable — bus **et** adresse — via le routage Plug & Play v1.3.0).
2. **Contrôleur de Vol PID :** Stabilisation automatique roulis/tangage/cap/profondeur avec mixage différentiel sur les 8 propulseurs.
3. **Génération PWM Matérielle :** Contrôle autonome de 16 canaux PWM indépendants via PCA9685 (8 propulseurs, servos pan/tilt, pince, gradateurs d'éclairage), avec serveur Web/WebSocket autonome et portail captif pour la configuration et le banc de test.
4. **Gestion de Sécurité Propulsion (v1.1.0) :** check des propulseurs au démarrage (après la connexion Wi-Fi, non bloquant), surveillance surintensité continue via ACS770/INA3221, isolation logicielle du moteur en défaut et remontée d'urgence automatique avec les moteurs verticaux restants.

---

## 🏗️ Architecture Matérielle & Adressage

### 1. Périphériques I2C — Deux Bus Matériels

L'ESP32-S3 possède deux contrôleurs I2C indépendants. La répartition ci-dessous est la **configuration câblée de référence** : l'IMU BNO085 est **seule** sur le bus n°1 (son protocole SH-2 ne tolère aucune cohabitation), tous les autres périphériques vivent sur le bus n°2. Chaque module est re-routable dynamiquement — bus **et** adresse I2C (routage Plug & Play v1.3.0, voir section suivante) :

| Composant | Rôle | Adresse I2C | Bus I2C | Description technique |
| :--- | :--- | :--- | :--- | :--- |
| **BNO085** | IMU 9-DOF | `0x4A` | **n°1 (GPIO 10/11)** | **Seul périphérique du bus n°1** (toute cohabitation perturbe le flux SHTP). Fusion inertielle embarquée (protocole SH-2 via Adafruit BNO08x) : Rotation Vector 9-DOF (cap magnétique) + Game Rotation Vector (secours) + gyroscope calibré @ 50 Hz. Monté tourné de 90° (X capteur vers la droite du ROV) : quaternion **et** gyroscope remappés à la source dans le repère véhicule via `BNO085_MOUNT_YAW_DEG` (−90°) |
| **INA3221** | Énergie Propulsion (v1.1.0) | `0x40` | **n°2 (GPIO 12/13)** | Triple mesure : **Ch1** tension batterie V_BAT (`IN1+`), **Ch2/3** courants des propulseurs via capteurs à effet Hall **ACS770-100U** (sortie `VOUT` 0,5–4,5 V câblée sur `IN-` et lue sur le registre de tension bus — `I (A) = (V_bus_mV − 500) / 40`, plage utile 0–100 A ; voir « Mesure d'Énergie »). Pilote maison (moyennage ×16, ≈35 Hz/canal, identifiant fabricant `0x5449` vérifié au boot) |
| **INA226 #2** | Puissance Aux/LEDs | `0x44` | n°2 (GPIO 12/13) | Wattmètre résiduel éventuel — mesure de ligne d'éclairage et accessoires 12V |
| **INA226 #3** | Puissance Moteurs | `0x45` | n°2 (GPIO 12/13) | Wattmètre résiduel éventuel — mesure de ligne de puissance batterie/propulsion principale |
| **MS5803-30BA / MS5837** | Pression & Profondeur | `0x76` | **n°2 (GPIO 12/13)** | Capteur piézorésistif haute résolution étanche, compensation 24-bit (coefficients C1-C6) |
| **PCA9685** | Contrôleur PWM 16 Ch | `0x41` (ex.) | **n°2 (GPIO 12/13)** | Générateur PWM 12 bits autonome (VCC 3.3V, V+ 5V/Servo). **Re-adressé par straps A0–A5** (0x41 ou 0x42) pour cohabiter avec l'INA3221 qui occupe `0x40` sur le même bus |

> **Note — pourquoi deux bus :** le MS5837-30BA branché sur le même bus que le BNO085 perturbait ses lectures SHTP (tempête de NACK, horizon figé) — et plus généralement, toute cohabitation sur le bus de l'IMU. La règle câblée retenue est donc : **l'IMU seule sur le bus n°1**, tous les autres périphériques sur le bus n°2. Bus n°1 `Wire` (GPIO 10/11, configurables en NVS) et bus n°2 `Wire1` (GPIO 12/13, configurables en NVS). Chaque bus possède ses propres résistances de pull-up (4,7 kΩ vers 3V3). Depuis la v1.3.0, la répartition « quel module vit sur quel bus » **et l'adresse I2C de chaque module** ne sont plus codées en dur : elles sont **routées dynamiquement** (Plug & Play, voir ci-dessous).

**Schéma de câblage des deux bus I2C :**

```
               ┌──────────────────────────┐
               │        ESP32-S3          │
               └──────┬────────────┬──────┘
                      │            │
   ┌──────────────────┴───────┐  ┌─┴────────────────────────────────────────┐
   │ Bus n°1 — « Wire »       │  │ Bus n°2 — « Wire1 »                      │
   │ SDA GPIO 10 / SCL GPIO 11│  │ SDA GPIO 12 / SCL GPIO 13                │
   │ pull-ups 4,7 kΩ → 3V3    │  │ pull-ups 4,7 kΩ → 3V3                    │
   │                          │  │                                          │
   │  └─ BNO085      @ 0x4A   │  │  ├─ INA3221      @ 0x40  — énergie       │
   │     (IMU — SEUL)         │  │  ├─ PCA9685      @ 0x41  — PWM 16 ch.    │
   │                          │  │  ├─ MS5837-30BA  @ 0x76  — profondeur    │
   │                          │  │  └─ INA226 #2/#3 @ 0x44 / 0x45 (résid.)  │
   └──────────────────────────┘  └──────────────────────────────────────────┘
```

### 2. Routage I2C Plug & Play (v1.3.0 — bus ET adresses)

La topologie ci-dessus est la **configuration câblée de référence** du ROV, portée par la carte de routage NVS (appliquée au boot). À la toute première mise sous tension (NVS vierge), le firmware démarre sur sa répartition par défaut **historique** (bus n°1 : BNO085 + INA3221 + 2× INA226 ; bus n°2 : MS5803 + PCA9685, GPIO 6/7) — enregistrer une fois le routage de référence depuis l'interface fige la configuration ci-dessus. Chacun des **six modules routables** (BNO085, INA3221, INA226 #2, INA226 #3, MS5803/MS5837, PCA9685) peut être réassigné — **bus n°1 ou n°2 et adresse I2C** — depuis l'interface Web (onglet Paramètres → carte Scanner → **« Routage I2C Plug & Play »**), au fil d'un flux Plug & Play complet :

1. **Câbler** le module avec ses straps matériels (INA226 A0/A1, BNO085 PS0/PS1, MS5837 CSB…).
2. **Scanner** les deux bus (bouton « Scanner les bus I2C ») — les adresses détectées alimentent immédiatement les sélecteurs d'adresses du routage, sans recharger la page.
3. **Choisir** bus et adresse dans la table — chaque sélecteur d'adresse ne propose que les adresses **réellement découvertes par le scan** sur le bus sélectionné ; l'adresse routée actuelle reste visible en « (actuelle) » si le module n'est pas détecté.
4. **Enregistrer** → le routage est écrit en NVS et appliqué au **redémarrage différé de 3 s** (application au boot uniquement, jamais à chaud — voir API REST ci-dessous).

Aucune adresse n'est codée en dur dans les chemins de code : les constantes de `config.h` ne servent que de **défauts NVS** au premier démarrage, et toutes les transactions du firmware passent par `g_i2cRouter.addrOf()` / `busOf()` — capteurs (sondes de boot, lectures, re-sonde périodique, récupération de bus) comme PCA9685 (alloué par `PWMController::begin()` sur son adresse routée).

* **Persistance :** namespace NVS « config », **douze clés** — six bus `route_bno085`, `route_ina3221`, `route_ina226_2`, `route_ina226_3`, `route_ms5803`, `route_pca9685` (valeur 1 = `Wire`, 2 = `Wire1`) et six adresses `addr_bno085`, `addr_ina3221`, `addr_ina226_2`, `addr_ina226_3`, `addr_ms5803`, `addr_pca9685` (0x01–0x7F) — effaçables avec les autres paramètres via « Réinitialiser les paramètres ». La carte est chargée par `SensorDriver::begin()` **avant toute sonde de présence** ; chaque composant reçoit ensuite le pointeur `TwoWire` de SON bus (`&Wire` ou `&Wire1`) et **son adresse routée**.
* **API REST :** `GET /api/i2c/routing` (carte courante : modules + adresses hex/décimales + bus, broches des deux bus, bus protégé PWM, état de conflit, **dernier scan I2C intégré** au format de `/api/i2c/scan`) ; `POST /api/i2c/routing` (les **DOUZE clés** requises — paires `module`/`module_addr` — bus 1 ou 2, adresse décimale (`74`) ou hexadécimale (`0x4A`) — form-urlencoded ou JSON). Écriture NVS **vérifiée par relecture**, puis **redémarrage différé de 3 s** : le routage ne s'applique **qu'au boot**, jamais à chaud — même discipline que les broches SDA/SCL.
* **Conflit d'adresse généralisé :** deux modules routés sur le **même bus à la même adresse** est refusé — pas seulement le cas historique INA3221 ↔ PCA9685 (tous deux `0x40`) : la sauvegarde est **refusée** (HTTP 400, les deux modules en conflit nommés dans l'erreur) côté firmware, et l'interface bloque le bouton (avertissement live sur toute paire `bus:adresse` dupliquée).
* **Règle de sûreté du bus PWM :** le bus hébergeant le PCA9685 (`pwmBus()`) n'est **jamais bit-bangé ni reseté en vol** : le watchdog I2C global et la récupération de bus du scanner ne réinitialisent que **l'autre bus**. Les capteurs routés sur le bus PWM restent réinitialisables (réécriture de leurs registres) mais leur bus n'est pas réparable à chaud — limitation documentée : en cas d'effondrement du bus PWM, seul un redémarrage le répare.
* **Verrou Check Propulseurs :** le verrou `_inaBusLock` suit le bus **routé** de l'INA3221 (sérialisation entre la tâche `Motors` et la tâche capteurs sur ce bus).

### 3. Liaison Série RPi 5 — Deux Modes Configurables

Le protocole binaire RPi 5 ↔ ESP32-S3 peut emprunter **deux interfaces physiques** au choix, sélectionnable via l'onglet Paramètres de l'interface Web et persisté en NVS :

| Mode | Interface | Broches / Port | Côté RPi 5 | Description |
| :--- | :--- | :--- | :--- | :--- |
| **USB-CDC Natif** | `Serial` (USB) | Port USB-C de l'ESP32-S3 | `/dev/ttyACM0` | Protocole binaire sur le port USB natif. Le debug moniteur partage le même port. |
| **UART Matériel** | `Serial1` (UART1) | RX: **GPIO 1**, TX: **GPIO 2** | UART GPIO (921 600 bauds) | Protocole binaire dédié sur UART matériel. USB-CDC reste exclusivement réservé au debug. |

> **Note :** Par défaut, le mode **USB-CDC** est actif. Le choix est mémorisé en NVS (clé `comm_iface`) et rechargé à chaque démarrage. Un **redémarrage** est nécessaire après changement via l'interface Web.

#### Broches UART (mode UART matériel uniquement)

| Signal | GPIO | Direction | Description |
| :--- | :--- | :--- | :--- |
| **UART1 RX** | GPIO 1 | ESP32 ← RPi 5 | Réception trame descendante |
| **UART1 TX** | GPIO 2 | ESP32 → RPi 5 | Émission trame montante |

### 4. Affectation des Canaux PWM (PCA9685)

* **Canaux 0 à 7 :** 8 × ESC Propulseurs ($1000\,\mu\text{s}$ à $2000\,\mu\text{s}$, neutre à $1500\,\mu\text{s}$)
* **Canaux 8 & 9 :** Servomoteurs Pan & Tilt Caméra ($50\text{ Hz}$)
* **Canaux 10 & 11 :** Outil / Pince & Rotation — **Bidirectionnel** par défaut (neutre $1500\,\mu\text{s}$)
* **Canaux 12 & 13 :** Gradateurs MOSFET pour projecteurs LED — **Unidirectionnel** par défaut ($1000\,\mu\text{s} = 0\,\% \rightarrow 2000\,\mu\text{s} = 100\,\%$)
* **Canaux 14 & 15 :** Canaux auxiliaires / Extensions — **Bidirectionnel** par défaut

> **Typage configurable (canaux 10 à 15) :** chaque canal 10–15 est typable individuellement en **Bidirectionnel** (neutre $1500\,\mu\text{s}$, plage $1000-2000\,\mu\text{s}$) ou **Unidirectionnel / pleine échelle** ($1000\,\mu\text{s} = 0\,\%$, $2000\,\mu\text{s} = 100\,\%$) depuis l'onglet Paramètres — persistance NVS (clé `pwm_types` : masque 6 bits, bit n ↔ canal 10+n, 1 = bidirectionnel ; défaut `0x33`), appliqué **à chaud** sans redémarrage. Le typage ne modifie pas la plage des consignes (toujours bornées à 1000–2000 µs) : il détermine la **position de sécurité** appliquée par le failsafe (1500 µs en bidirectionnel, 1000 µs en unidirectionnel) et le rendu dynamique des barregraphes du **PWM Monitor**.

### 5. Sorties Tout-ou-Rien — 4 × GPIO ON/OFF

Quatre sorties numériques auxiliaires pilotées directement par l'ESP32-S3 (relais, éclairage tout-ou-rien, accessoires) :

| Sortie | GPIO par défaut | Commande (bit de `gpio_cmd`) | État (bit de `gpio_state`) |
| :--- | :--- | :--- | :--- |
| 1 | GPIO 15 | bit 0 | bit 0 |
| 2 | GPIO 16 | bit 1 | bit 1 |
| 3 | GPIO 17 | bit 2 | bit 2 |
| 4 | GPIO 18 | bit 3 | bit 3 |

* **Assignation configurable :** les numéros de GPIO sont modifiables dans l'onglet Paramètres et persistés en NVS (clés `gpio_p1`..`gpio_p4`) ; ils ne sont appliqués qu'**au redémarrage** (comme les broches I2C). Validation : broches 0–48 hors GPIO 19/20 (USB), hors broches I2C actives (deux bus) et UART1, sans doublon.
* **Initialisation :** au démarrage, les 4 broches sont configurées en sortie à l'état **BAS (OFF)** ; elles restent pilotables **même désarmé** (sorties auxiliaires).
* **Sécurité :** le déclenchement du watchdog série (perte du flux RPi 5) et l'**E-Stop** forcent les 4 sorties à **OFF** — l'état réel est relu physiquement et remonté dans `gpio_state`.

---

## 📡 Protocole Série Binaire RPi 5 ↔ ESP32-S3

La communication s'effectue à **921 600 bauds** sur l'interface sélectionnée (USB-CDC natif ou UART matériel GPIO 1/2). Toutes les trames sont à taille fixe avec validation par somme de contrôle CRC16-CCITT.

### 1. Trame Descendante : RPi 5 → ESP32-S3 (40 octets @ 20-50 Hz)

```
[0xAA][0x55][TYPE][MODE][ARM_STATE][PWM_0 ... PWM_15][GPIO_CMD][CRC16_HIGH][CRC16_LOW]
```

* **Header :** `0xAA`, `0x55` (2 octets)
* **Type :** `0x01` (1 octet)
* **Mode :** `uint8_t` (`0` = Passif, `1` = AUTO Roulis et Tangage, `2` = Auto-Full)
* **Arm State :** `uint8_t` (`0` = Disarmed, `1` = Armed, `2` = Emergency Stop)
* **PWM Channels (0–15) :** `uint16_t[16]` (32 octets, offsets 5–36, consigne en $\mu\text{s}$)
* **GPIO Command :** `uint8_t` (offset 37, masque 4 bits des sorties ON/OFF — bit 0 = sortie 1 … bit 3 = sortie 4, `1` = ON, `0` = OFF ; bits 4–7 ignorés)
* **CRC16 :** `uint16_t` (2 octets aux offsets [38] fort / [39] faible, polynôme 0x1021, init 0xFFFF — **stocké gros-boutiste** : octet fort puis faible, calculé sur les octets `[2..37]`)

### 2. Trame Montante : ESP32-S3 → RPi 5 (73 octets @ 100 Hz)

```
[0x55][0xAA][TYPE][STATUS][QUAT_W,X,Y,Z][GYRO_X,Y,Z][PRESS,TEMP][INA_1,2,3][PWM_ACT_0...15][GPIO_STATE][CRC16]
```

* **Header :** `0x55`, `0xAA` (2 octets)
* **Type :** `0x02` (1 octet)
* **Status Flags :** `uint8_t` (Bit 0: réservé, Bit 1: Liaison établie — au moins une trame valide reçue depuis le boot, Bit 2: Autopilote actif — réservé, jamais émis, Bit 3: Watchdog déclenché, Bit 4: Sorties physiques neutralisées / Dry-Run)
* **Quaternions IMU :** `int16_t[4]` (8 octets, $W, X, Y, Z$ scalés $\times 10\,000$, repère véhicule — montage BNO085 remappé à la source)
* **Gyroscope :** `int16_t[3]` (6 octets, $\omega_x, \omega_y, \omega_z$ scalés $\times 100$, repère véhicule)
* **Pression & Température :** `int32_t[2]` (8 octets, $0.1\text{ mbar}$ et $0.01\,^\circ\text{C}$)
* **Télémétrie Puissance :** `uint16_t[6]` (12 octets — **format binaire inchangé, sémantique v1.1.0** : $[V_{BAT}\ mV,\ 0,\ V_{signal}\ mV,\ I_H\ mA,\ V_{signal}\ mV,\ I_V\ mA]$ issus de l'INA3221/ACS770 ; anciennement $V$ et $I$ des trois INA226 en $mV$/$mA$)
* **PWM Effectifs :** `uint16_t[16]` (32 octets, offsets 38–69, impulsions réelles envoyées aux actionneurs)
* **GPIO State :** `uint8_t` (offset 70, état **réel relu** des 4 sorties ON/OFF — bit 0 = sortie 1 … bit 3 = sortie 4, `1` = ON)
* **CRC16 :** `uint16_t` (2 octets aux offsets [71] fort / [72] faible, **gros-boutiste**, calculé sur les octets `[2..70]`)

### 3. Watchdog Failsafe

Un watchdog logiciel surveille la réception de trames descendantes. Si aucune trame valide n'est reçue pendant le **timeout configurable** (défaut : 500 ms), chaque canal PWM est automatiquement forcé à sa **position de sécurité** ($1500\,\mu\text{s}$ en bidirectionnel — défaut canaux 0–11, 14–15 ; $1000\,\mu\text{s}$ / $0\,\%$ en unidirectionnel — défaut canaux 12–13, voir « Typage configurable ») et les **4 sorties ON/OFF sont forcées à OFF**.

> **📄 Documentation détaillée pour le développement côté RPi 5 :** voir [PROTOCOLE_RPI5.md](PROTOCOLE_RPI5.md) — description octet par octet des deux trames et des masques de sorties ON/OFF (`gpio_cmd` / `gpio_state`), sémantique exacte des bits de statut, comportements watchdog / armement / maître / dry-run, exemple Python prêt à l'emploi et vecteurs de test.

#### Watchdog IMU — Récupération BNO085 en 3 niveaux

Indépendamment du watchdog série, un **watchdog dédié surveille le flux de quaternions du BNO085** (Rotation Vector ou Game Rotation Vector — le gyroscope ne compte pas). Constat opérationnel : en plongée ou après une longue période d'inactivité, la communication avec le capteur (seul sur le bus I2C n°1) peut se figer définitivement. Si aucun quaternion n'est reçu pendant **1 s** (`BNO085_STALL_TIMEOUT_MS`), la récupération escalade automatiquement :

| Niveau | Déclenchement | Action | Propulsion |
| :--- | :--- | :--- | :--- |
| **1 — Reprise douce SH-2** | 1er échec (stall 1 s sans quaternion) | Ré-initialisation logicielle du capteur (réécriture des rapports SH-2) | **Forcée au neutre** pendant la récupération |
| **2 — Reset matériel du bus I2C** | Échec du niveau 1, ou blocage persistant du contrôleur I2C (`ESP_ERR_INVALID_STATE`) | Destruction de l'instance `Adafruit_BNO08x`, `Wire.end()`, **9 impulsions SCL + condition STOP** (bit-bang), réinitialisation complète de `Wire` (400 kHz, timeout 50 ms), puis recréation de l'instance et réinit de tous les capteurs | **Forcée au neutre** |
| **3 — Ultime secours : redémarrage contrôlé** | **3 tentatives de niveau 2** restées sans effet (`BNO085_REBOOT_ATTEMPTS`) **ou 8 s** sans aucun quaternion (`BNO085_REBOOT_TIMEOUT_MS`) | 1) tous les canaux PWM à leur position de sécurité (neutre $1500\,\mu\text{s}$, ou $1000\,\mu\text{s}$ / $0\,\%$ pour les canaux unidirectionnels), 2) log explicite sur le moniteur série, 3) `esp_restart()` — redémarrage automatique de l'ESP32 | **Position de sécurité physiquement maintenue** pendant tout le reboot (voir ci-dessous) |

**Garantie de maintien au neutre pendant la récupération / le reboot :**

1. **Pendant les niveaux 1 et 2** — la tâche de contrôle (100 Hz) verrouille les canaux 0-7 (propulseurs) à $1500\,\mu\text{s}$ **à chaque cycle** tant que le flux de quaternions est mort (`isBNO085Stalled()`) : une consigne RPi 5 réappliquée entre deux tentatives ne peut jamais repartir avec une attitude inconnue. Les servos/auxiliaires (8-15) ne sont pas concernés.
2. **Au niveau 3** — la neutralisation est doublée d'une écriture physique inconditionnelle sur les 16 canaux (même en Dry-Run) : le **PCA9685 est un générateur autonome qui conserve ses registres pendant le redémarrage de l'ESP32** — les propulseurs restent au neutre tout le reboot ; au redémarrage, le firmware repart en mode Dry-Run et réapplique le neutre.

> **Garde-fou anti-boucle :** le niveau 3 ne se déclenche que si le capteur **a déjà fonctionné depuis le boot** (au moins un quaternion reçu). Un capteur jamais fonctionnel dès la mise sous tension n'est pas redémarré en boucle : il reste en **mode espacé** (polling SHTP coupé, badge rouge + message Web, une tentative complète toutes les 30 s). Ce comportement garantit la récupération automatique du cas opérationnel réel (capteur qui gèle en plongée) sans risquer une boucle de redémarrages sur un capteur défaillant au boot.

---

## 🌐 Interface Web Embarquée & Portail Captif

L'ESP32-S3 héberge une application Web autonome stockée dans la mémoire flash **LittleFS**. Le design suit la charte graphique **BOB CNC** (thème sombre, polices Michroma + Source Code Pro, accents cyan/magenta).

### Portail Captif Automatique

Le point d'accès Wi-Fi (`BOB-CONTROL_AP`, mot de passe : `bobrov2026`) inclut un serveur DNS captif qui redirige **toutes** les requêtes DNS vers `192.168.4.1`. Des handlers HTTP spécifiques interceptent les URLs de détection de portail captif pour chaque OS :

| OS | URL de détection interceptée |
| :--- | :--- |
| Android / ChromeOS | `/generate_204`, `/gen_204`, `/hotspot-detect.html` |
| iOS / macOS | `/library/test/success.html` |
| Windows | `/connecttest.txt`, `/redirect` |
| Firefox | `/canonical.html` |
| Kindle / Fire OS | `/kindle-wifi/wifistub.html` |

La page s'ouvre automatiquement dès la connexion au réseau Wi-Fi.

### Tableau de Bord — Navigation par Tuiles Réorganisables

L'interface principale est un **tableau de bord** composé de tuiles cliquables disposées en grille 2×N. Chaque tuile représente une fonction du système. L'utilisateur peut :

1. **Cliquer sur une tuile** → affiche la section correspondante en pleine page avec un bouton retour (↩).
2. **Cliquer « Réorganiser »** ou **appui long (500 ms) sur une tuile** → active le mode édition : toutes les tuiles se mettent à trembler (jiggle iOS ±1,5°, **désynchronisé par tuile** via data-tile-id), et la tuile sous le doigt se **soulève immédiatement** — elle suit le doigt sans attendre un nouvel appui.
3. **Glisser-déposer les tuiles** → réordonne en temps réel, la tuile survolée fait place dynamiquement (Pointer Events unifiés souris/tactile, **repli touchstart/touchmove/touchend** pour les navigateurs anciens ; jamais l’attribut `draggable` HTML5, inopérant sur iOS Safari). La position est **sauvegardée dans `localStorage` à chaque dépôt** (ordre propre à l’appareil).
4. **Cliquer « Terminer »** ou **taper en dehors des tuiles** → quitte le mode édition.

### Fonctions du Tableau de Bord (6 Sections)

| Tuile | Contenu |
| :--- | :--- |
| **Wi-Fi** | **Connexion actuelle** temps réel : SSID, adresse IP, passerelle (DHCP), masque, RSSI, MAC. Infos point d'accès local (SSID, IP, clients). Scan des réseaux environnants. Connexion STA non-bloquante (sans tuer le point d'accès). |
| **PWM Monitor** | 16 barregraphes temps réel (WebSocket). 8 jauges moteurs bidirectionnelles centrées à $1500\,\mu\text{s}$, servos, gradateurs, auxiliaires. **Rendu dynamique des canaux 10–15 selon leur typage** : Bidirectionnel → curseur centré sur le neutre ($1500\,\mu\text{s}$) ; Unidirectionnel → jauge pleine échelle $0 - 100\,\%$ avec dégradé de couleur progressif vert → orange → rouge. **Commutateur Sorties Matérielles** (Mode Témoin / Dry-Run) : badge d'avertissement ambre/vert. |
| **Télémétrie** | **Cadre ROV 3D** : modèle 3D du ROV (`bob_rov_3D.glb`, LittleFS `/3D/`) animé en temps réel par l'IMU (roulis/tangage/cap) — three.js r128 servi **localement** (`/vendor/three.min.js`, aucun CDN : le ROV diffuse son AP sans Internet) et parseur GLB minimal embarqué (couleurs plates, sans texture) ; orbite/zoom à la souris, double-clic = vue initiale, grille polaire et flèche du Nord en référence de cap, cases « Filaire » / « Vert » (wireframe, lignes teintées vert) ; réalignement du modèle à 90° (R3D_MODEL_YAW_DEG) car l’avant exporté d’Onshape ne suit pas la convention glTF (-Z). **Cap magnétique BNO085** : le Rotation Vector 9-DOF (gyro + accéléro + magnétomètre) est la source principale du cap — cap absolu, le Game Rotation Vector relatif restant en secours automatique si le flux s'interrompt ; état de la boussole affiché en continu (source + calibration 0–3, auto-calibration en déplaçant le ROV en « 8 »). **Bouton « Régler Nord »** : tare du cap appliquée à la source — télémétrie, HSI, modèle 3D et PID de cap partagent le même référentiel ; ajustage fin de l'écart résiduel en cap magnétique (une fois suffit), référence obligatoire après chaque boot en secours relatif ; à faire en mode PASSIF. Assiette Euler (Roll/Pitch/Yaw), profondeur, pression, température. **Widget INA3221** (v1.1.0 : tension batterie + courants horizontaux/verticaux en A) et 2 wattmètres INA226. **Statut Moteurs** (v1.1.0) : voyant vert/orange/rouge + détail temps réel (check en cours/échoué, moteurs isolés, remontée d'urgence active). **Sélecteur de Mode de Pilotage** : 3 boutons radio (PASSIF / AUTO ROULIS ET TANGAGE / AUTO FULL) avec badge Maître (RPi 5 vert / Manuel ESP32 orange). Instruments aviation : horizon artificiel et HSI (Heading Situation Indicator) avec lissage LERP. Diagnostic capteurs (LED BNO085, MS5803, INA3221, INA226 ×2, PCA9685). |
| **Banc de Test** | 16 curseurs de commande manuelle **adaptés au typage configuré** (µs 1000–2000 en bidirectionnel ; **0–100 %** avec conversion automatique en µs en unidirectionnel) + **4 sorties GPIO ON/OFF**, avec verrou de sécurité matériel. Chaque sortie affiche son **nom personnalisé** (Paramètres → NVS). Bouton « Tout en position de sécurité » appliquant **par canal** le neutre adapté (1500 µs / 0 %). |
| **Paramètres** | Sélection liaison série RPi 5 (USB-CDC / UART GPIO 1/2). **Configuration Bus I2C n°1 / n°2** : broches SDA/SCL avec sauvegarde NVS et redémarrage automatique du contrôleur après 3 s — même comportement pour les deux bus (broches appliquées au boot ; le bus n°2 est partagé avec les sorties PWM). **Scanner I2C** : scan des deux bus (0x01–0x7F, colonne Bus avec les GPIO réels de chaque bus) avec identification des périphériques — l'étiquette de chaque adresse est arbitrée par le routage bus + adresse des six modules (heuristiques pour les périphériques non routés). **Routage I2C Plug & Play (v1.3.0 — bus et adresses)** : table des six modules avec sélecteur de bus (I2C_0/I2C_1, GPIO réels dans les libellés) et **sélecteur d'adresse** peuplé par les adresses découvertes au dernier scan I2C (filtrées par bus ; l'adresse routée actuelle reste proposée en « (actuelle) »), avertissement live et blocage du bouton sur toute paire bus:adresse en conflit, bouton « Enregistrer le routage I2C » (diff bus ET adresse affiché dans la confirmation) → NVS + redémarrage automatique après 3 s, l'interface ne se rechargeant qu'une fois l'API sert la nouvelle carte (preuve du redémarrage effectif, comme pour les broches). **Typage des canaux PWM 10–15** : 6 sélecteurs Bidirectionnel / Unidirectionnel, appliqués à chaud et persistés en NVS. **Sorties tout-ou-rien** : assignation des 4 GPIO ON/OFF (GPIO 15/16/17/18 par défaut) avec validation (doublons et broches réservées refusés — USB, I2C actifs, UART1), persistance NVS et redémarrage automatique après 3 s ; badges d'état ON/OFF en temps réel et **boutons de test individuels** (forçage direct via `POST /api/gpio/test`, écrasé par le watchdog, l'E-Stop ou une trame RPi 5 maître). **Mise à jour OTA** : téléversement sans câble du micrologiciel (firmware.bin) ou des fichiers Web (littlefs.bin) avec progression, neutralisation de la propulsion et redémarrage automatique. PID (Roll, Pitch, Yaw, Profondeur) : Kp, Ki, Kd. Timeout watchdog série réglable. **Calibration Altimètre (QNH)** : interrupteur d'activation (persisté NVS) et bouton « Actualiser le QNH maintenant » avec spinner (Open-Meteo — Chamoson, VS) — ne corrige que l'altitude barométrique hors de l'eau (voir Notes Techniques). **Gestion Moteurs — Sécurité Propulsion (v1.1.0)** : 5 paramètres NVS (check au démarrage, détection surintensité & isolation, remontée d'urgence, seuil d'intensité max, seuil de tension batterie) + bouton de relance manuelle du check (voir section dédiée). |
| **Câblage** | Schéma visuel ESP32-S3 (`esp32s3.png`). Liaison RPi 5 dynamique selon le mode actif (USB-CDC ou UART GPIO 1/2) avec diagrammes ASCII et tables de correspondance. Tableaux complets des deux bus I2C : broches SDA/SCL (bus n°1 GPIO 10/11, bus n°2 GPIO 12/13), adresses et fonctions de chaque module (IMU, wattmètres, baromètre, PCA9685). **Depuis la v1.3.0, toute la page suit le routage Plug & Play actif** : titres, schémas ASCII, tableaux de brochage, **adresses I2C affichées** et badges « Bus I2C » de chaque module sont reconstruits à partir de la carte de routage réellement chargée (repli sur la topologie par défaut tant que l'API n'a pas répondu ; un bus entièrement vidé affiche un message à la place du schéma). |

### Connexion Wi-Fi Externe (Mode STA)

Le firmware fonctionne en mode **WIFI_AP_STA** (point d'accès + station simultanés). La connexion à un réseau Wi-Fi externe utilise `WiFi.begin()` **sans** `WiFi.disconnect()` pour ne pas interrompre le point d'accès. Un callback d'événements Wi-Fi (`ARDUINO_EVENT_WIFI_STA_GOT_IP`) suit l'état de connexion de manière asynchrone.

---

## 🔋 Énergie Propulsion & Sécurité Moteurs (v1.1.0)

L'**INA3221** (triple mesure, bus n°2, `0x40` — pilote maison, aucune bibliothèque) remplace le premier INA226 et supervise l'énergie de propulsion. Les courants sont fournis par deux capteurs à effet Hall **ACS770-100U** (sensibilité 40 mV/A, sortie analogique **0,5 V à 0 A → 4,5 V à 100 A**) dont la broche `VOUT` est câblée sur l'entrée **`IN-`** des canaux 2 et 3 : l'INA3221 y lit cette tension **absolue** sur son **registre de tension bus** (LSB 8 mV, plage 26 V — le registre shunt, plafonné à ±163,84 mV, saturait la mesure à ±4,09 A) et le firmware applique **`I (A) = (V_bus_mV − 500) / 40`**.

| Canal INA3221 | Mesure | Source | Plage utile |
| :--- | :--- | :--- | :--- |
| **1** | Tension batterie propulsion (V_BAT) | `IN1+` (registre bus, LSB 8 mV) | 0–26 V |
| **2** | Courant propulseurs horizontaux M1–M4 | `VOUT` ACS770-H → `IN2-` (registre bus, LSB 8 mV) | 0–100 A |
| **3** | Courant propulseurs verticaux M5–M8 | `VOUT` ACS770-V → `IN3-` (registre bus, LSB 8 mV) | 0–100 A |

Moyennage matériel ×16 (≈35 Hz/canal). Les deux INA226 résiduels (#2 Aux/LED `0x44`, #3 Propulsion `0x45`) restent inchangés.

### Mesure d'Énergie (Propulsion & Batterie) — Câblage INA3221 + ACS770-100U

**Schéma de câblage (tension batterie + courants propulsion) :**

```
   ┌──────────────────────────────────────────────────────────────────────────┐
   │   MESURE D'ÉNERGIE (Propulsion & Batterie) — INA3221 @ 0x40, bus I2C n°2 │
   └──────────────────────────────────────────────────────────────────────────┘

   CANAL 1 — Tension batterie propulsion
     BATTERIE 12 V ───── V_BAT (0–26 V) ────────►  IN1+   (canal 1)
     GND (référence) ───────────────────────────►  IN1-

   CANAL 2 — Courant propulseurs horizontaux (M1–M4)
                   ┌──────────────────┐
     ALIM +12 V ──►│ IP+              │
                   │    ACS770-100U   ├───► IP- ────►  ESC M1–M4
                   │   (courant       │
                   │    traversant)   │
                   │          VOUT ───┼── 0,5–4,5 V ────►  IN2-   (canal 2)
                   └──────────────────┘

   CANAL 3 — Courant propulseurs verticaux (M5–M8)
                   ┌──────────────────┐
     ALIM +12 V ──►│ IP+              │
                   │    ACS770-100U   ├───► IP- ────►  ESC M5–M8
                   │   (courant       │
                   │    traversant)   │
                   │          VOUT ───┼── 0,5–4,5 V ────►  IN3-   (canal 3)
                   └──────────────────┘
```

> * **Le signal analogique des ACS770 (0,5 V à 4,5 V) est lu sur les broches `IN-` des canaux 2 et 3** de l'INA3221 : sortie `VOUT` câblée directement, référencée à la masse commune par les broches opposées (`IN2+`/`IN3+`). Le firmware exploite le **registre de tension bus** (LSB 8 mV, plage 26 V — le registre shunt, plafonné à ±163,84 mV soit ±4,09 A, est inadapté à ce signal) et applique `I (A) = (V_bus_mV − 500) / 40`.
> * **Canal 1 :** `IN1+` est connectée à la tension globale de la batterie (V_BAT) — mesure directe de la tension de propulsion (0–26 V).
> * **Conducteur de puissance :** le courant réellement consommé par chaque banc de propulseurs (M1–M4 horizontaux, M5–M8 verticaux) traverse le capteur (`IP+` → `IP-`, en série sur la ligne d'alimentation des ESC) ; `VOUT` le restitue en tension à 40 mV/A.

**Télémétrie :** WebSocket `power[5]` — canaux 0–2 = INA3221 (`{v, i}` en mV/mA : V_BAT, courant H, courant V), canaux 3–4 = INA226 #2/#3 — plus objet `motors` (`health`, `status`, `check_done`, `check_passed`, `check_pending`, `fault`, `emergency`, `isolated_mask`). **La trame série montante RPi 5 (73 octets) garde son format binaire inchangé** (voir section Protocole).

### Paramètres (onglet Paramètres → « Gestion Moteurs », NVS namespace `motors`)

| Paramètre | Clé NVS | Défaut | Bornes |
| :--- | :--- | :--- | :--- |
| Check Propulseurs au démarrage | `check_enable` | activé | — |
| Détection surintensité & isolation | `detect_enable` | activé | — |
| Remontée d'urgence si moteur vertical isolé | `emergency_enable` | **désactivée** (sûreté) | modifiable seulement si la détection est active |
| Seuil d'intensité max — moteur bloqué | `cur_thresh` | 3,0 A | 0,5–80 A |
| Seuil de tension basse — batterie propulseurs | `vbat_low` | 14,0 V | 9–30 V |

Écriture NVS **vérifiée par relecture** (HTTP 500 en cas d'échec, convention du projet). API REST : `GET/POST /api/motors/config` (corps form-urlencoded — champs absents = valeurs courantes conservées ; le GET renvoie aussi l'état courant du gestionnaire et les bornes), `POST /api/motors/check` (relance manuelle, **409** si séquence en cours ou défaut actif).

### Check des propulseurs au démarrage (non bloquant)

Exécuté par la tâche `Motors` (Core 1, priorité 3 — sous le contrôle/PID, période 100 ms). L'armement du check automatique est **figé au boot** : activer l'option à chaud depuis l'interface ne déclenche **jamais** d'impulsions en navigation — seule la relance manuelle (bouton Paramètres) teste à chaud. La connexion Wi-Fi est attendue **sans blocage** (polling de l'état STA, jamais de boucle d'attente), plus 3 s de stabilisation (`MOTOR_CHECK_POST_WIFI_DELAY_MS`) :

1. **Voyant remis à UNKNOWN**, puis vérification de V_BAT (canal 1) ≥ seuil bas — sinon WARNING (orange) et abandon.
2. **Impulsion séquentielle M1 → M8** : 1600 µs (≈20 % de poussée) par moteur — fenêtre de mesure de 400 ms (stabilisation 150 ms + maximum de 3 lectures espacées de 100 ms, la moyenne ×16 interne de l'INA3221 lissant déjà), retour immédiat au neutre **avant** toute décision, pause de 250 ms entre moteurs ; les canaux déjà isolés ce boot sont ignorés.
3. Courant > seuil pendant l'impulsion → moteur déclaré bloqué et **isolé immédiatement** ; lecture impossible (INA3221 absent/bus occupé) → WARNING et interruption (aucune conclusion sans donnée).
4. **Verdict** : FAULT (rouge) si le moindre isolement subsiste, OK (vert) sinon. Un défaut surintensité détecté pendant l'attente Wi-Fi (ESC en court-circuit) annule le check (échec) — jamais d'état « en attente » éternel dans l'interface.

### Surveillance en navigation, identification et isolation

Si `detect_enable` est actif, la tâche `Motors` surveille les canaux 2/3 à 10 Hz. Le seuil doit être confirmé **3 lectures consécutives** (~300 ms — `MOTOR_OVERCURRENT_CONFIRM`) avant action : immunité aux transitoires d'inversion de poussée, tout en restant bien plus rapide qu'un échauffement destructif. Un canal entièrement isolé (4 moteurs) n'est plus surveillé ; une remontée d'urgence active n'interprète pas le courant vertical (il reflète la poussée de survie).

**Identification du moteur fautif :** isolements **provisoires** successifs, triés par écart |consigne − 1500 µs| décroissant (le moteur le plus sollicité est suspecté en premier). Après chaque isolement (stabilisation 150 ms), le courant est relu : celui dont l'isolement fait retomber le courant sous le seuil est le coupable — les innocents sont **relâchés** (isolement levé, moteur au neutre). Si aucun isolement ne suffit, le canal entier est isolé.

**Isolation logicielle :** tout canal isolé **refuse toute consigne** (RPi 5, PID, banc de test — filtre dans `PWMController::setPWMuS`/`setAllPWM`) et l'isolement écrit **physiquement** le neutre, même en Dry-Run. Seuls un redémarrage ou l'identification d'un innocent lèvent un isolement.

### Remontée d'urgence

Si `emergency_enable` est actif et qu'un moteur **vertical** (M5–M8) vient d'être isolé, le flag de survie s'arme : les moteurs verticaux restants reçoivent **1700 µs** (≈40 % de poussée vers la surface — assez pour remonter avec 3 verticaux sur 4) **en remplacement de toute consigne**. Le filtre est centralisé dans `PWMController` (appliqué à la fréquence d'écriture, 100 Hz) : aucun combat entre une réaffirmation 10 Hz et le PID. Le failsafe série (perte du flux RPi 5) **n'interrompt pas** une remontée — la consigne de survie traverse le neutre logique ; seules des décisions humaines la coupent : **E-Stop** RPi 5 (trame descendante) et **OTA** appellent `abortEmergencySurface()` avant de neutraliser.

Le voyant « Statut Moteurs » (onglet Télémétrie) ne **reverdit jamais seul** (gravité croissante uniquement) : un check manuel relancé le remet à UNKNOWN puis le verdict le repositionne — pratique après correction d'un WARNING batterie.

---

## ⚙️ Configuration PlatformIO

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
board_build.partitions = bob_control_8mb.csv
board_build.filesystem = littlefs
build_flags = 
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1

lib_deps = 
    mathieucarbou/ESPAsyncWebServer @ ^3.6.0
    mathieucarbou/AsyncTCP @ ^3.3.2
    bblanchon/ArduinoJson @ ^7.0.0
    adafruit/Adafruit PWM Servo Driver Library @ ^3.0.1
    adafruit/Adafruit BNO08x @ ^1.2.5
    robtillaart/INA226 @ ^0.6.0
```

> **Note :** Les bibliothèques `ESPAsyncWebServer` et `AsyncTCP` utilisent le fork maintenu par `mathieucarbou` (compatible ESP32 Arduino Core récent). L'ancien dépôt `me_no-dev` est obsolète et provoque des erreurs de linking.

> **Table de partitions personnalisée (`bob_control_8mb.csv`) :** dérivée de `default_8MB.csv` — partitions applicatives OTA réduites de 3,3 Mo à **2,5 Mo** chacune (firmware ~1,2 Mo) au profit de LittleFS, étendu de 1,5 Mo à **~2,9 Mo** pour héberger le modèle 3D du ROV (`data/3D/bob_rov_3D.glb`, servi sur `/3D/bob_rov_3D.glb` avec le MIME `model/gltf-binary` via une route explicite — l'extension `.glb` est inconnue de la table MIME du core Arduino) ainsi que three.js (`data/vendor/three.min.js`, r128 UMD licence MIT). **Toute modification de la table de partitions impose un reflash complet par USB** : `pio run -t erase && pio run -t upload && pio run -t uploadfs` — la table (offset 0x8000) ne peut pas être écrite par OTA.

---

## 📂 Structure du Répertoire

```
BOB-CONTROL/
├── README.md                  # Résumé du projet (GitHub)
├── BOBCONTROL.md              # Spécification et documentation (ce fichier)
├── PROTOCOLE_RPI5.md          # Protocole série RPi 5 : trames octet par octet + exemple Python
├── platformio.ini             # Configuration PlatformIO
├── data/                      # Fichiers LittleFS (interface Web)
│   ├── index.html             # Tableau de bord (tuiles + sections)
│   ├── style.css              # Thème sombre BOB CNC + drag & drop
│   └── app.js                 # WebSocket, navigation, drag & drop
├── include/
│   ├── config.h               # Broches, adresses I2C, UART, constantes réseau
│   ├── protocol.h             # Structures de trames packed + CRC16-CCITT + constantes MODE_*
│   ├── autopilot.h            # Structures PIDParams (Roll/Pitch/Yaw/Depth), AutopilotConfig, pidCompute()
│   ├── flight_controller.h    # Classe FlightController : modes PID, mixage moteur, priorité RPi5/Web
│   ├── motor_manager.h        # Classe MotorManager : check propulseurs, surveillance surintensité, NVS (v1.1.0)
│   ├── i2c_router.h           # Routeur I2C Plug & Play : carte module → bus + adresse, NVS, conflits (v1.3.0)
│   └── gpio_outputs.h         # 4 sorties tout-ou-rien ON/OFF : pins NVS, init OFF, état relu
└── src/
    ├── main.cpp               # Setup séquentiel + tâche vTaskControl (Core 1, 100 Hz) + corrections PID
    ├── web_server.cpp         # Serveur HTTP, DNS captif OS-aware, WebSocket, REST API
    ├── serial_comm.cpp        # USB-CDC / UART GPIO : abstraction Stream*, machine à états, watchdog failsafe
    ├── i2c_router.cpp         # Routeur I2C Plug & Play : carte NVS module→bus/adresse chargée au boot, conflits, save() vérifiée par relecture (v1.3.0)
    ├── sensor_driver.cpp      # Drivers I2C réels — chaque capteur sur SON bus routé (BNO085, INA3221, INA226 ×2, MS5803/MS5837)
    ├── flight_controller.cpp  # PID 4 axes (Roll/Pitch/Yaw/Depth) + mixage M5-M8 + gestion priorité maître
    ├── pwm_controller.cpp     # PCA9685 (bus I2C routé) : conversion µs→ticks, typage canaux 10-15, failsafe + mode Témoin (Dry-Run) + isolement moteurs/remontée d'urgence (v1.1.0)
    ├── motor_manager.cpp      # Check propulseurs au démarrage, surveillance surintensité, identification/isolation (v1.1.0)
    └── gpio_outputs.cpp       # 4 sorties ON/OFF : masque gpio_cmd, relecture pad, persistance NVS des broches
```

---

## 🔧 Commandes de Téléversement

```bash
# Téléverser le firmware
pio run -t upload

# Téléverser les fichiers web (LittleFS)
pio run -t uploadfs

# Moniteur série debug (USB-CDC)
pio device monitor -b 115200
```

---

## 📋 Notes Techniques

* **Mémoire :** RAM ~15,5 %, Flash ~51 % (marge confortable pour extensions).
* **Contrôleur de Vol (Flight Controller) :** pilotage combinatoire par **masque de bits superposables** (v1.4.0) avec PID 4 axes et mixage différentiel sur les 8 propulseurs — chaque bit du champ `mode` (trame descendante) active une boucle indépendante :
  - **PASSIF (`0x00`)** — Consignes PWM appliquées telles quelles (manuel direct).
  - **Auto R/T (`0x01`)** — Stabilisation PID du roulis et du tangage (cible 0°) sur les moteurs verticaux M5-M8.
  - **Tenue de Profondeur (`0x02`)** — PID altitude sur M5-M8 ; la profondeur courante est capturée comme consigne à l'activation.
  - **Auto Cap (`0x04`)** — PID lacet sur les horizontaux M1-M4 ; le cap courant est capturé comme consigne à l'activation.
  - **Retour Surface (`0x08`, PRIORITAIRE)** — verticaux M5-M8 forcés vers la surface (≈1700 µs) ; suspend R/T et tenue de profondeur, l'Auto Cap reste indépendant.
  Les assistances se **superposent** (ex. `0x03` = Auto R/T + Tenue de Profondeur ; `0x07` ≡ ancien AUTO FULL). Les bits `status` de la trame montante (2/5/6/7) confirment l'activation **réelle et combinée** des boucles au moment de l'émission. Le clamp 1000-2000 µs est appliqué après la somme totale de toutes les corrections pour éviter la saturation prématurée. En mode Web sans RPi 5, les propulseurs sont forcés à 1500 µs pour validation sur établi.
* **Gestion de Priorité Maître :** Le RPi 5 est maître absolu tant qu'une trame est reçue (< 1s). En l'absence de communication RPi 5, l'interface Web ESP32 prend le contrôle. Si le Web tente de changer le mode pendant que le RPi 5 est maître, la commande est ignorée et un message série est émis.
* **Mode Témoin / Dry-Run :** Au boot, les sorties physiques PCA9685 sont neutralisées par défaut (`physicalOutputsEnabled = false`). Les consignes PWM reçues (RPi 5 ou simulateur) sont mémorisées dans le buffer d'état et affichées dans les 16 barregraphes, mais le PCA9685 applique la position de sécurité de chaque canal : 1500 µs en bidirectionnel (défaut canaux 0–11, 14–15) et 1000 µs / 0 % en unidirectionnel (défaut canaux 12–13). Un commutateur dans l'onglet PWM Monitor permet d'activer/désactiver les sorties. Le Bit 4 de la trame montante (`STATUS_BIT_DRYRUN = 0x10`) signale cet état au RPi 5. Commande WebSocket `{output_enable: true/false}` ou REST `POST /api/pwm/output-enable`.
* **Typage des canaux PWM 10–15 :** chaque canal est configurable individuellement en **Bidirectionnel** (neutre 1500 µs, plage 1000–2000 µs) ou **Unidirectionnel / pleine échelle** (1000 µs = 0 % → 2000 µs = 100 %), depuis l'onglet Paramètres. Persistance NVS sous forme d'un masque 6 bits (clé `pwm_types`, bit n ↔ canal 10+n, 1 = bidirectionnel ; défaut `0x33` : CH10/11 et CH14/15 bidirectionnels, CH12/13 unidirectionnels), appliqué **à chaud** sans redémarrage. Le typage détermine la **position de sécurité** du failsafe (1500 µs en bidirectionnel, 1000 µs en unidirectionnel) et le rendu des barregraphes du PWM Monitor (curseur centré sur le neutre, ou jauge 0–100 % avec dégradé vert → orange → rouge). API REST : `GET/POST /api/pwm/config`.
* **Sorties Tout-ou-Rien (GPIO ON/OFF ×4) :** 4 sorties numériques auxiliaires (GPIO 15/16/17/18 par défaut, configurables dans l'onglet Paramètres et persistés en NVS — clés `gpio_p1`..`gpio_p4`, appliqués au redémarrage), initialisées en sortie à l'état BAS (OFF) au boot. Commandées par le masque `gpio_cmd` de la trame descendante (bit 0 = sortie 1), pilotables **même désarmé** ; forcées à OFF par le watchdog série et l'E-Stop. L'état réel (relu physiquement sur le pad) est remonté dans `gpio_state` (trame montante, 100 Hz) et affiché par badges dans l'interface. Boutons de test (établi) : `POST /api/gpio/test` (masque 4 bits) force directement l'état des sorties — la réponse renvoie l'état réel relu et le flag `rpi5_master`. API REST : `GET/POST /api/gpio/config`, `POST /api/gpio/test`.
* **Noms des sorties (×20) :** chacune des 20 sorties — 16 canaux PWM et 4 GPIO ON/OFF — peut recevoir un **nom personnalisé** (20 caractères max) depuis la carte « Noms des sorties » de l'onglet Paramètres. Persistés en NVS (namespace « config », clé `names`) sous forme d'un JSON unique de 20 chaînes (une entrée vide = nom par défaut, restitué côté interface), appliqués **à chaud** sans redémarrage : libellés du Banc de Test, curseurs et barregraphes du PWM Monitor, info-bulles du modèle 3D et carte GPIO. Noms nettoyés des caractères HTML/de contrôle en double (firmware + interface, rendu systématique en `textContent` — aucune injection possible), longueur bornée en octets sans couper l'UTF-8, écriture NVS **vérifiée par relecture** (HTTP 500 en cas d'échec). API REST : `GET/POST /api/names`.
* **Calibration altimétrique QNH (v1.0.1) :** option **non destructive** calibrant l'altimètre hors de l'eau sur le QNH réel d'Internet — carte « Calibration Altimètre (QNH) » de l'onglet Paramètres (interrupteur « Activer la calibration QNH automatique (Internet) », bouton « Actualiser le QNH maintenant » avec spinner pendant la requête, texte d'état « QNH actuel utilisé : XXXX.X mbar »). Le firmware interroge l'API Open-Meteo (`https://api.open-meteo.com/v1/forecast?latitude=46.20&longitude=7.23&current=pressure_msl` — Chamoson, VS) dans une **tâche détachée** `QnhFetch` (jamais dans la boucle de contrôle ni dans les handlers async) avec un **timeout court de 2000 ms** (connexion + réponse) ; au démarrage, la requête n'est lancée que si l'option est activée **et** la station Wi-Fi connectée à Internet, **10 s après le boot** (laisser le STA joindre le réseau). **Le QNH ne modifie que la formule d'altitude barométrique** (h = 44330·(1 − (P/QNH)^0.190284)) : la **profondeur immergée reste strictement fondée sur la tare de surface locale** (ΔP) et la boucle de contrôle n'est jamais impactée. **Fallback transparent** sur la valeur standard **1013.25 mbar** si l'option est désactivée, si Internet est absent, si la requête échoue ou si la réponse sort des bornes physiques acceptées (850–1100 mbar) — la désactivation réapplique immédiatement le standard ; une valeur valide est appliquée à chaud (`SensorDriver::setQnh()`, protégé par mutex) et **persistée en NVS** (clé `current_qnh`, réappliquée au boot suivant ; HTTPS sans vérification du certificat, l'ESP32 n'ayant pas d'horloge RTC fiable).

**Offset matériel du capteur 30 Bars (compensation logicielle) :** le MS5803-30BA présente un offset d'usine d'environ 25 mbar — insignifiant sous 300 m d'eau, mais ~200 m d'erreur d'altitude dans l'air. Chaque requête Open-Meteo (automatique au boot ou « Actualiser maintenant ») extrait en plus du QNH le champ racine `elevation` (altitude topographique réelle du lieu), calcule la pression atmosphérique théorique locale `P_theo = QNH·(1 − 0.0065·elev/288.15)^5.255`, lit la pression absolue brute instantanée `P_brute` du MS5803 et en déduit l'offset matériel `hardware_offset = P_brute − P_theo`. Garde-fous : calcul uniquement capteur **hors de l'eau** (immergé, la colonne d'eau fausserait tout), pression plausible et élévation fournie, offset borné à ±100 mbar (`MS5803_HW_OFFSET_MAX_MBAR` — au-delà, jugé aberrant : l'ancien offset reste). Une valeur valide est appliquée à chaud (`SensorDriver::setHwOffset()`, protégé par mutex) et **persistée en NVS** (clé `hw_offset`, réappliquée au boot). Dans la boucle de télémétrie, cet offset est compensé **uniquement dans le calcul de l'altitude** : `P_corrigée = P_brute − hardware_offset` puis `Altitude = 44330·(1 − (P_corrigée/QNH)^0.190284)` — **la tare de surface et le calcul de profondeur (ΔP) continuent d'utiliser exclusivement la pression absolue brute, strictement intouchés**. La carte des Paramètres affiche l'offset compensé lorsqu'il est non nul (« offset matériel : +XX.X mbar »).

> *Pourquoi doter un drone sous-marin d'un altimètre aéronautique de haute précision ? Par pur refus de laisser une erreur matérielle de 25 mbar dicter sa loi. Cette précision altimétrique, d'une utilité totalement discutable, est le fruit d'un over-engineering assumé : un ROV conçu en Suisse se doit d'être aussi précis dans l'air que sous l'eau, même si cela ne sert absolument à rien.*

API REST : `GET /api/qnh/config` (état de l'option, QNH appliqué, offset matériel, diagnostic), `POST /api/qnh/config` (`enable=true|false`), `POST /api/qnh/refresh` **alias `/api/calibrate_qnh`** (requête manuelle : QNH + calcul d'offset — 503 si pas de connexion Internet, « busy » si déjà en cours).
* **Gestion de sécurité moteurs (v1.1.0) :** voir la section dédiée « Énergie Propulsion & Sécurité Moteurs ». Tâche `Motors` dédiée (Core 1, priorité 3, 10 Hz) ; filtres d'isolation et de remontée d'urgence centralisés dans `PWMController` (aucune réaffirmation périodique : signal propre face au PID 100 Hz) ; l'E-Stop RPi 5 et l'OTA interrompent une remontée d'urgence (`abortEmergencySurface()`) avant de neutraliser la propulsion.
* **Centrale Inertielle BNO085 :** Intégrée via la bibliothèque **Adafruit BNO08x** (protocole SH-2 par paquets, et non par lectures de registres). Rapports activés à 50 Hz : **Rotation Vector 9-DOF** (gyro + accéléro + magnétomètre — source principale, cap magnétique absolu), **Game Rotation Vector** (relatif, secours automatique si le flux magnétique s'interrompt) et gyroscope calibré. **Montage tourné de 90°** autour de la verticale (X capteur → droite du ROV, Y → arrière) : quaternion **et** gyroscope sont remappés à la source dans le repère véhicule via `BNO085_MOUNT_YAW_DEG` (−90°), avant extraction des angles d'Euler embarquée — télémétrie série, horizon artificiel, HSI, modèle 3D et PID de cap partagent ainsi le même référentiel (lever l'avant → tangage +, penché à droite → roulis +). Auto-guérison du flux magnétique : re-activation du Rotation Vector renvoyée toutes les 5 s tant qu'aucun rapport n'arrive. **Supervision anti-gel :** le flux de quaternions est surveillé en continu par le watchdog 3 niveaux (ré-init SH-2 → reset bus I2C matériel → redémarrage contrôlé de l'ESP32 avec propulsion neutralisée) — un figement en plongée ou après une longue inactivité ne nécessite plus aucun redémarrage manuel (voir « Watchdog IMU »). Le capteur peut nécessiter un court délai après la mise sous tension ; un reboot relance l'initialisation si le BNO085 n'était pas prêt au premier démarrage. Il possède son propre bus dédié par défaut (bus n°1) — le MS5837 perturbant ses lectures SHTP lorsqu'ils partageaient le même bus, ce dernier a été déplacé sur le bus n°2 ; le routage Plug & Play permet de les déplacer (bus et adresse), mais cette cohabitation sur un même bus reste déconseillée pour la même raison.
* **Propriété des Bus I2C (v1.2.0 — selon routage) :** les DEUX bus sont configurés une seule fois au boot dans `SensorDriver::begin()`, avant la création des tâches, afin que leur fréquence/timeout ne changent jamais pendant une transaction. **Bus des capteurs purs** (celui qui n'héberge PAS le PCA9685 — bus n°1 par défaut) : propriété de la tâche capteurs (Core 1). Les demandes de scan issues du serveur Web (Core 0) sont mises en file (`_scanReq`) et exécutées par la tâche capteurs au prochain cycle — plus aucune collision possible entre le bit-bang SCL du scan et une transaction en cours (les changements de broches ne passent plus par la file : sauvegarde NVS vérifiée + redémarrage, voir `saveBusPins`). **Bus du PCA9685** (bus n°2 par défaut) : partagé entre la tâche capteurs (capteurs routés dessus, ex. lecture pression 2 Hz) et la tâche de contrôle (écritures PCA9685 100 Hz) ; chaque transaction est atomique et sérialisée par le verrou interne de l'instance `TwoWire` (un mutex par bus). La séquence de récupération de bus bloqué (9 impulsions SCL + condition STOP) avant chaque scan et le reset du watchdog ne touchent JAMAIS le bus du PCA9685 (sorties PWM 100 Hz en vol) — uniquement l'autre bus ; timeout réduit à 10 ms pendant le scan des deux bus.
* **Watchdog I2C global :** compteur d'échecs de transactions consécutifs (instrumenté dans les wrappers unifiés `_i2cWrite`/`_i2cWrite2B`/`_i2cRead16`/`_i2cDevicePresent`, chacun paramétré par le `TwoWire*` du périphérique routé ; les lectures réussies des wattmètres (INA3221 + INA226 sur leur bus routé) remettent le compteur à zéro en continu). Au-delà de 30 échecs (ou si tous les capteurs vus au boot sont perdus), reset du bus NON-PWM (celui qui n'héberge pas le PCA9685 — `Wire.end` + 9 impulsions SCL + `Wire.begin`) et réinitialisation complète de tous les capteurs — BNO085 compris, même s'il avait été raté au boot ; les capteurs routés sur le bus PWM sont réinitialisés (réécriture de leurs registres) mais leur bus n'est pas reseté à chaud (limitation documentée du routage). Cooldown de 5 s entre deux récupérations. Complété par la supervision BNO085 sur le flux de quaternions (stall 1000 ms sans quaternion (Rotation Vector ou Game Rotation Vector) — le gyro ne compte pas — → escalade en 3 niveaux : ré-init SHTP légère, puis reset bus I2C matériel, puis redémarrage contrôlé de l'ESP32 après 3 resets bus sans rétablissement ou 8 s sans quaternion ; propulsion verrouillée au neutre pendant toute la récupération — voir la section « Watchdog IMU » ; un capteur jamais fonctionnel depuis le boot reste en mode espacé : polling SHTP coupé, badge rouge + message Web, une tentative complète toutes les 30 s, sans redémarrage en boucle). Détection de perte du MS5803 : après 3 échecs de lecture consécutifs (~1,5 s), le capteur est déclaré perdu et n'est plus pollé ; il est re-sondé toutes les 10 s pour prendre en compte un rebranchage à chaud.
* **Persistance NVS :** Les identifiants Wi-Fi, paramètres PID (Roll, Pitch, Yaw, Depth), choix d'interface série, **broches I2C (SDA/SCL)**, **routage I2C Plug & Play** (v1.3.0 — clés bus `route_bno085`..`route_pca9685` et clés adresses `addr_bno085`..`addr_pca9685`), **typage des canaux PWM 10–15** (clé `pwm_types`), **broches des 4 sorties ON/OFF** (clés `gpio_p1`..`gpio_p4`), **noms personnalisés des 20 sorties** (namespace « config », clé `names`) et **calibration altimétrique QNH** (clés `qnh_auto_enable`, `current_qnh` et `hw_offset`) et **paramètres de sécurité moteurs** (v1.1.0 — namespace « motors » : clés `check_enable`, `detect_enable`, `emergency_enable`, `cur_thresh`, `vbat_low`) sont sauvegardés en NVS via la classe `Preferences`.
* **Mise à jour OTA (sans câble USB) :** Carte « Mise à jour OTA » de l'onglet Paramètres : téléversement du micrologiciel (`firmware.bin`, produit par `pio run`) vers la partition applicative inactive, ou des fichiers Web (`littlefs.bin`, produit par `pio run -t buildfs`) vers la partition LittleFS. Le schéma de partitions `bob_control_8mb.csv` contient deux partitions OTA (`ota_0`/`ota_1` de 2,5 Mo) : l'ancien firmware reste la partition de boot tant que la nouvelle n'est pas validée — une coupure en cours d'écriture ne brike jamais le contrôleur. Sécurités : remontée d'urgence interrompue (`abortEmergencySurface()`) puis propulsion neutralisée (Dry-Run + neutre PCA9685) AVANT la première écriture flash (les écritures suspendent brièvement les deux cœurs) ; LittleFS démonté avant réécriture (remonté en cas d'échec, sinon redémarrage) ; octet magie ESP `0xE9` vérifié au premier bloc (refuse l'inversion firmware / image FS — la corrompant silencieusement) ; redémarrage différé 3 s après validation complète ; l'interface ne se recharge que lorsque `GET /api/system/status` montre un uptime reparti de zéro (preuve du redémarrage effectif, comme pour les broches I2C). API REST : `POST /api/ota/firmware`, `POST /api/ota/filesystem` (multipart), `GET /api/system/status`.
* **Version Git dynamique (pied de page) :** le script `scripts/git_version.py` (exécuté avant chaque compilation via `extra_scripts = pre:` dans `platformio.ini`) récupère `git describe --tags --always --dirty` et génère l'en-tête `include/git_version.h` définissant la macro `GIT_VERSION` (fichier régénéré seulement si le contenu change — builds incrémentaux préservés ; ignoré par Git ; fallback `"dev"` hors dépôt). Pourquoi un en-tête plutôt que `-D GIT_VERSION="..."` : la couche SCons/PlatformIO altère les guillemets des `CPPDEFINES` contenant des tirets, livrant un corps de macro déquoté que le compilateur rejette. La version est exposée par `GET /api/system/status` (champ `version`) et affichée par le pied de page global de l'interface (« BOB-CONTROL vX | Auteur : Dero Didier | Licence MIT »), rafraîchi au chargement par `app.js` (libellé statique conservé si l'API est indisponible) — les deux images (firmware + LittleFS) doivent être flashées pour la voir remonter.
* **Diagnostic I2C :** Scanner complet des DEUX bus I2C (adresses 0x01–0x7F, colonne Bus dans les résultats) accessible depuis l'onglet Paramètres. Identification automatique des périphériques (PCA9685, INA3221, INA226 ×2, BNO085, MS5803/MS5837). Configuration dynamique des broches SDA/SCL des deux bus (`POST /api/i2c/config`, paramètre `bus=1|2`) : sauvegarde NVS vérifiée par relecture puis redémarrage différé de 3 s pour les DEUX bus (broches appliquées au boot ; le bus n°2 est partagé avec les sorties PWM — un `Wire1.end()` en vol couperait la propulsion ; le bus n°1 suit la même procédure pour une interface uniforme). L'interface ne se recharge qu'une fois l'API revenue avec les nouvelles broches (preuve du redémarrage effectif). Récupération automatique de bus bloqué avant chaque scan — sur le bus NON-PWM uniquement (v1.2.0). **Routage I2C Plug & Play (v1.3.0 — bus et adresses)** : table des six modules routables avec sélecteurs de bus (I2C_0/I2C_1, GPIO réels dans les libellés) et sélecteurs d'adresses (adresses détectées par le scan, filtrées par bus ; « (actuelle) » si non détectée), avertissement live + bouton bloqué sur toute paire bus:adresse en conflit, bouton « Enregistrer le routage I2C » → NVS vérifiée + redémarrage différé 3 s (application au boot uniquement, jamais à chaud — le bus du PCA9685 porte les sorties PWM), l'interface attendant la nouvelle carte de routage avant de se recharger (preuve du redémarrage effectif). Persistance NVS : les broches des deux bus (clés `i2c_sda`/`i2c_scl` et `i2c2_sda`/`i2c2_scl`) et le routage (clés `route_*` et `addr_*`) sont rechargées au boot. La page « Câblage & Schémas » suit les broches réellement configurées et le routage actif (schémas, brochage et tableaux reconstruits dynamiquement). API REST : `GET /api/i2c/scan`, `GET/POST /api/i2c/config`, `GET/POST /api/i2c/routing`.
* **Architecture FreeRTOS :** Tâches épinglées par cœur (Core 0 : Wi-Fi/WebServer/DNS, Core 1 : Série/Capteurs/PWM/Contrôle). Le bus I2C n°2 (Wire1) est partagé entre la tâche capteurs et la tâche de contrôle — transactions sérialisées par le verrou interne de Wire1 (voir « Propriété des Bus I2C »).
