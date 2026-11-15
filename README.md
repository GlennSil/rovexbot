# RovexBot

Robot mobile à conduite différentielle sous **ROS 2 Humble**, développé en simulation (Gazebo) puis porté sur un robot réel piloté par une Jetson Orin Nano et un Arduino Uno.

Le même code de contrôle tourne dans les deux cas : seule la couche matérielle change (plugin Gazebo en simulation, liaison série vers l'Arduino sur le robot réel).

## Crédits

Ce projet s'appuie sur le travail d'**Antonio Brandi** et de son robot **Bumperbot**, que j'ai suivi à travers ses cours *Self Driving and ROS 2 – Learn by Doing*.

- Dépôt Bumperbot : https://github.com/AntoBrandi/Bumper-Bot
- Cours Odometry & Control : https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control
- Cours Map & Localization : https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization

L'architecture des packages, les nodes de contrôle, de localisation et de cartographie, ainsi que la base de la hardware interface et du firmware Arduino proviennent de ce travail.

Ce que j'ai fait de mon côté :

- adaptation à mon propre robot : modèle URDF, maillages, dimensions (roues de 65 mm, entraxe de 160 mm) ;
- hardware interface `ros2_control` (`RovexbotInterface`) : corrections de robustesse (dimensionnement des vecteurs, lignes série incomplètes) et choix simulation / robot réel par argument `is_sim` ;
- firmware Arduino : lecture des encodeurs sécurisée vis-à-vis des interruptions, gestion explicite du sens de rotation ;
- driver MPU6050 porté sur la Jetson Orin Nano (bus I2C paramétrable, lecture en bloc, covariances) ;
- fichiers de lancement `rovexbot_bringup` pour la simulation et le robot réel.

Les mondes Gazebo `small_house` et `small_warehouse` viennent des modèles AWS RoboMaker.

## Matériel

| Élément | Rôle |
|---|---|
| Jetson Orin Nano | Ordinateur embarqué (ROS 2 Humble) |
| Arduino Uno R3 | Commande des moteurs, lecture des encodeurs, PID de vitesse |
| L298N | Pont en H pour les deux moteurs |
| 2 moteurs CC avec encodeurs en quadrature | Roues motrices (385 impulsions par tour de roue) |
| MPU6050 | Centrale inertielle (I2C) |

## Packages

| Package | Contenu |
|---|---|
| `rovexbot_description` | URDF/xacro, maillages, mondes Gazebo, lancement de la simulation |
| `rovexbot_controller` | Configuration `diff_drive_controller`, contrôleurs pédagogiques, téléopération (manette, `twist_mux`) |
| `rovexbot_firmware` | Hardware interface `ros2_control` (C++), codes Arduino, driver MPU6050 |
| `rovexbot_localization` | Filtre de Kalman, EKF (`robot_localization`), AMCL |
| `rovexbot_mapping` | Cartographie à poses connues, `slam_toolbox`, cartes |
| `rovexbot_utils` | Arrêt de sécurité |
| `rovexbot_bringup` | Lancement complet : simulation ou robot réel |

## Chaîne de commande du robot réel

```
clavier / manette
      │  Twist
      ▼
  twist_mux ──▶ twist_relay ──▶ diff_drive_controller
                                      │  vitesses des roues (rad/s)
                                      ▼
                         RovexbotInterface (ros2_control)
                                      │  port série, 115200 bauds
                                      ▼
                         Arduino : PID de vitesse ──▶ L298N ──▶ moteurs
                                      ▲
                                 encodeurs
```

### Protocole série

| Sens | Format | Exemple |
|---|---|---|
| Jetson → Arduino (consignes) | `r<signe><vitesse>,l<signe><vitesse>,` | `rp05.00,ln03.20,` |
| Arduino → Jetson (mesures, toutes les 100 ms) | `r<signe><vitesse>,l<signe><vitesse>,` | `rp4.73,ln2.10,` |

`r` / `l` : roue droite / gauche. `p` / `n` : sens positif / négatif. Vitesses en rad/s.

## Câblage

**Arduino Uno**

| Broche | Connexion |
|---|---|
| 9 | L298N `ENA` (PWM roue droite) |
| 12, 13 | L298N `IN1`, `IN2` (sens roue droite) |
| 11 | L298N `ENB` (PWM roue gauche) |
| 7, 8 | L298N `IN3`, `IN4` (sens roue gauche) |
| 3, 5 | Encodeur droit, phases A et B |
| 2, 4 | Encodeur gauche, phases A et B |

Les cavaliers `ENA` et `ENB` du L298N sont retirés, et les masses de l'Arduino et du L298N sont reliées.

**MPU6050 sur la Jetson Orin Nano** (connecteur 40 broches)

| MPU6050 | Jetson |
|---|---|
| VCC | Broche 1 (3,3 V) |
| GND | Broche 6 |
| SDA | Broche 3 |
| SCL | Broche 5 |

Ces broches correspondent au bus `/dev/i2c-7`.

## Installation

```bash
mkdir -p ~/rovexbot_ws && cd ~/rovexbot_ws
git clone https://github.com/GlennSil/rovexbot.git src
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Pour le robot réel, téléverser `rovexbot_firmware/firmware/robot_control/robot_control.ino` sur l'Arduino (bibliothèque **PID** de Brett Beauregard requise).

## Utilisation

**Simulation**

```bash
ros2 launch rovexbot_bringup simulated_robot.launch.py
```

**Robot réel** (sur la Jetson)

```bash
ros2 launch rovexbot_bringup real_robot.launch.py
```

Arguments : `port` (par défaut `/dev/ttyACM0`) et `i2c_bus` (par défaut `7`).

**Pilotage au clavier**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel -p speed:=0.1 -p turn:=0.5
```

## État d'avancement

- [x] Simulation Gazebo : contrôle, odométrie, localisation, SLAM
- [x] Robot réel : hardware interface, PID de vitesse, téléopération
- [x] IMU MPU6050 sur la Jetson
- [ ] Calibration de l'odométrie et de l'IMU sur le robot réel
- [ ] Lidar et SLAM sur le robot réel
- [ ] Navigation autonome (Nav2)
