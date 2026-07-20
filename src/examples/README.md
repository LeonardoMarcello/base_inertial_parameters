# Base Inertial Parameter Identification - Examples

This directory contains Example of the identification Pipeline obtained with a collaborative robot (Franka Emika Panda) and a dexterous Anthropomorphic Platform (Allegro Hand)

---
### 1. Franka Emika Panda (7-DoF Collaborative Manipulator)

* **7DoF robotic manipulators (REAL)**
* **Target Directory:** `src/examples/franka_real`
* **Data Stream:** `/rosbag/franka_real_fourier_CT`
* **Description:** Estimate the full dynamic of the 7-DoF manipulator using data collected on the real hardware.

---

* **Payload Adaptation  (SIM)**
* **Target Directory:** `src/examples/franka_sim_softhand`.
* **Data Stream:** `/rosbag//rosbag/softhand_lean_fourier_CT`
* **Description:** Estimate the full dynamic of the 7-DoF manipulator using data collected on the real hardware.

---

* **Trajectory Comparison (SIM)**
* **Target Directory:** `src/examples/franka_sim_trajectories`
* **Data Stream:** `./rosbag/softhand_chirp_CT`, `./rosbag/softhand_lean_fourier_CT`, `./rosbag/softhand_sinusoidal_CT`
* **Description:** Comparison of estimates of the Base Inertial Parameters using three different exciting trajectories


---

### 2. Wonik Allegro Hand V5+ (Dexterous Anthropomorphic Platform)

* **Dexterous Robotic Hand Calibration  (REAL)**
* **Target Directory:** `src/examples/allegro_hand`
* **Data Stream:** `ahand_finger` via `allegro_hand_4traject_bag` and `ahand_thumb` via `allegro_hand_thumb_2traject_bag`.
* **Description:** Identification of the multi-finger allegro hand



---