# Robot-el-chino-de-los-mandados
Low-cost car-like robot (ESP32 + ROS 2) for under-canopy row following using LiDAR/IMU and offline leaf-image screening.
This repository contains the core software stack and selected experimental logs for the robot **“El Chino de los Mandados”**.  
It integrates:

- **ESP32** (low-level sensing + serial communication),
- **ROS 2** (sensor fusion + LiDAR processing + fuzzy wall/row following + image logging),
- **Data logs** (5 selected trials) for analysis and plotting,
- **A neural-network pipeline** for plantain leaf disease detection (*distributed via GitHub Releases*).

> **Important note (IMU usage):** the IMU (BNO055) is used as a **geometric auxiliary** to orient/correct the LiDAR measurement cone (yaw compensation). It is **not** used as part of a vehicle stabilization control loop.

---

##  What’s inside

### 1) `CSV_ROBOT/` — Selected experimental logs (5 trials)
The robot collects sensor and control data during field trials.  
This folder contains **5 selected trials** (the best-performing ones), plus scripts/data used to plot key signals such as:

- **Lateral distance error**: difference between measured distance and desired setpoint.
- **BNO055 measurements** and **IMU yaw error** (terrain irregularities cause yaw deviations; the robot includes correction behavior).
- **Valid LiDAR beams (red cone)**: used to estimate/count how many plantain plants were traversed.
- **Valid LiDAR beams (green cone)**: used to estimate lateral distance to the plants.
- **`lidarLeft`**: lateral distance measured by the LiDAR.
- **`omega1`**: steering actuator angle (front steering motor).
- **Steering reference**: desired steering actuator position.

> Goal: enable reproducible plotting and analysis of the robot behavior across representative trials.

---

### 2) `esp32/` — ESP32 firmware (PlatformIO / VS Code)
PlatformIO project for the ESP32 responsible for low-level tasks, including:

- Encoder measurement for the steering motor,
- IMU (BNO055) reading,
- Serial communication between ESP32 and the onboard computer (send/receive IMU + steering information).

Main entry point:
- `esp32/src/main.cpp` (or the equivalent `main.cpp` in the PlatformIO structure)

---

### 3) `hinf/` — ROS 2 main node(s) / workspace
ROS 2 codebase that implements the main autonomy pipeline.  
Key components include:

- **`esp32_serial_bridge`**  
  Direct communication with ESP32 (reads IMU + steering state, sends steering commands).

- **`lidarReadings`**  
  Computes/filters LiDAR distance readings while compensating for IMU yaw deviation (measurement cone correction).

- **`snapshot_saver`**  
  Saves camera snapshots during trials (image logging).

- **`wallFollower`**  
  Main **fuzzy controller**: receives sensor information and outputs the **desired steering angle** for the robot.

---

##  Leaf-disease model (Neural Network) — Distributed via Releases
The **neural network model/weights** are provided as a **GitHub Release asset** to keep the repository lightweight.

Inside the `patrones` pipeline you will find:
- The inference script (e.g., `predict_percent`) that processes images one by one and generates CSV outputs.
- Results folders per trial:
  - **Two CSV files per trial** (normalized and non-normalized),
  - Plots/figures summarizing detected disease patterns.

### How to get the model
1. Go to **Releases** in this repository.
2. Download the model asset (e.g., `.zip` / `.onnx` / `.h5` / `.pt`).
3. Place it in the path expected by your `patrones` inference scripts.

> If needed, create a small `MODEL_PATH` variable in the script or a `config.yaml` to point to the downloaded file.

---

##  Repository structure (high level)

```text
Robot-el-chino-de-los-mandados/
  CSV_ROBOT/     # 5 selected trials + sensor/control logs for plotting
  esp32/         # PlatformIO firmware (encoder + IMU + serial)
  hinf/          # ROS 2 code (bridge + LiDAR readings + snapshots + fuzzy control)
  (Releases)     # Neural-network model/weights (patrones)
