# AI-Driven Adaptive Torque Allocation for Electric Vehicles on Rough Terrain 🚗⚡🌍

[![Project Demo](https://img.shields.io/badge/Watch%20Demo-YouTube-red?logo=youtube)](https://youtu.be/OZV3y7bALE8)
![Techgium Finalist](https://img.shields.io/badge/Techgium%20Finalist-blue)

---

## 🔥 Project Overview

This project, selected among the **finalists of the 8th edition of TECHgium (L&T Technology Services)** from over **40,000+ students**, proposes a novel **AI-based torque allocation system** for electric vehicles navigating **rough terrains**. The aim is to **maximize vehicle stability, avoid skidding, and adaptively balance torque** across all four wheels based on real-time sensor input.

We built a complete simulation using **Gazebo**, developed an **optimization algorithm (SLSQP)** to compute adaptive torque distribution, trained a **deep learning model**, and finally **deployed it in a Raspberry Pi-powered prototype**. The real-time control and monitoring were achieved via a custom-built **Flask web application**.

---

## 📁 Project Structure

```plaintext
Rubicon/
├── Evata/                          # EV Car Model
│   ├── Materials/
│   ├── Meshes/
│   ├── Thumbnails/
│   └── model.sdf
│
├── World_Assets/                  # Terrain Assets for Gazebo
│   ├── Materials/
│   ├── Meshes/
│   ├── Thumbnails/
│   └── model.sdf
│
├── try_world.sdf                  # Custom rough terrain Gazebo world

├── imu_reader5.py                 # IMU sensor reading and preprocessing script
├── training_data.csv              # Raw data from Gazebo simulation
├── ai_model.csv                   # Processed dataset after optimization and feature engineering
├── ai_model.ipynb                 # AI training notebook
├── ai_model.keras                 # Trained neural network model (exported)
├── index.html                     # Flask dashboard template
├── raspi3.py                      # Main hardware control script (Raspberry Pi + sensors + AI + Flask)
```
---

## 🚀 Key Highlights

### 🛠️ Phase 1: Simulation in Gazebo

- **Rough Terrain World**: Created a custom uneven terrain in Gazebo.
- **EV Model**: Integrated a 4-wheel-drive electric car model with sensor mounts.
- **Data Logging**: Captured `ax`, `ay`, `az`, `wx`, `wy`, `wz` from the IMU during runs.

### 📊 Phase 2: Data Processing & Optimization

- **Velocity Calculation**: Derived `vx`, `vy` using acceleration data.
- **Wheel Speed Estimation**: Applied rigid body kinematics.
- **Slip Angle**: Estimated slip angle for each wheel — a major factor in torque gaps.
- **Optimization Algorithm**: Used **SLSQP** to:
  - Ensure torque ≤ motor max torque.
  - Fulfill driver-demanded torque.
  - Balance left vs. right wheels.
- **Torque to Duty Cycle**: Converted optimized torque to RPM → Duty Cycle (for motor drivers).

### 🧠 Phase 3: AI Model Development

- Input Features: `ax`, `ay`, `wx`, `wy`, `wz`
- Target: `fl_duty`, `fr_duty`, `rl_duty`, `rr_duty`
- Model: **Neural Network** (4 hidden layers, ReLU + Dropout)
- Optimizer: **Adam**
- Output: `.keras` file + `scaler.pkl`

### 💻 Phase 4: Deployment on Raspberry Pi 4B

- Created a **virtual environment** for isolated dependency management.
- Installed packages like `keras`, `scikit-learn`, `flask`, `RPi.GPIO`, etc.
- Uploaded model and scaler to run real-time inference from IMU.

### 🔧 Phase 5: Hardware Prototype

- **Chassis**: 4-wheel drive (4 DC motors).
- **Motor Control**: Two L298N motor drivers for independent control.
- **Sensors**:
  - **IMU**: Real-time terrain sensing.
  - **Ultrasonic Sensor**: Obstacle avoidance (20cm threshold).
  - **Buzzer**: Alerts on obstacle detection.
- **AI Inference**: AI model predicts optimal duty cycles in real-time.

### 🌐 Phase 6: Flask-Based Web App (RC + Visualization)

- Control: Forward, Backward, Left, Right
- Live Graphs:
  - **Roll vs. Time** 📈
  - **Pitch vs. Time** 📉
  - **Duty Cycle Distribution**
- Toggle AI Mode ON/OFF to compare stability
- Emergency Stop Button
- Responsive and clean UI

---

## 🧠 Project Architecture
<img width="1429" alt="Screenshot 2025-05-20 at 1 50 10 AM" src="https://github.com/user-attachments/assets/5797bb44-8b52-4ca8-8745-6dff0588d579" />


---

## 📽️ Project at a Glance
<img width="1436" alt="Screenshot 2025-05-20 at 1 58 00 AM" src="https://github.com/user-attachments/assets/6cb0a2cb-36c2-4548-81eb-78c5c74d2d6f" />


---

## 🌟 Results

| Condition        | Stability (Pitch/Roll) | Torque Distribution |
|------------------|------------------------|---------------------|
| Without AI       | High Variance          | Random/Fixed        |
| With AI          | Smoother Performance   | Dynamic             |

- Significant improvement in **vehicle stability**
- Real-time torque balancing with AI
- Effective slip angle adaptation

---

## 🔮 Future Enhancements

- Integrate **360° LIDAR** for advanced obstacle detection.
- Implement **Reinforcement Learning** for adaptive control.
- Expand model to handle **multi-terrain transitions**.
- Deploy on **real EV platforms** for field testing.
- Add **cloud sync** and **mobile app control**.

---

## 📦 Requirements

### 🧪 Software Setup

Install Python dependencies using:

```bash
pip install -r requirements.txt
```

This includes:
- `tensorflow` / `keras` for AI model inference  
- `scikit-learn` for data preprocessing (StandardScaler)  
- `flask` for the web dashboard  
- `matplotlib`, `numpy`, etc.

---

### 🌍 Gazebo Harmonic (For Mac M1 Users)

If you are using **Mac M1**, follow these steps to run the simulation and collect IMU data:

1. **Start the Gazebo Server**:
   ```bash
   gz sim -v 4 try_world.sdf -s
   ```

2. **Launch the Gazebo GUI (in a new terminal)**:
   ```bash
   gz sim -v 4 -g
   ```

3. **To view IMU data from the running simulation**, use:
   ```bash
   gz topic -e -t /imu
   ```

Ensure you have the EV model and terrain world files placed correctly in your project directory (`try_world.sdf`, `Evata/`, and `World_Assets/`).

---

### 🔩 Hardware Prototyping Requirements

To replicate the **real-world testing on a prototype**, you'll need:

| Component          | Quantity | Purpose                                   |
|--------------------|----------|-------------------------------------------|
| Raspberry Pi 4B    | 1        | Main controller for AI inference + sensors|
| L298N Motor Driver | 2        | For independent control of 4 DC motors    |
| DC Motors          | 4        | To drive each wheel                       |
| MPU6050 Sensor     | 1        | For real-time pitch, roll, and acceleration|
| Ultrasonic Sensor  | 1        | For obstacle detection                    |
| Buzzer             | 1        | Alerts when obstacle is near              |
| Jumper Wires       | Many     | To connect components on breadboard       |
| Li-Po Battery Pack | 1        | Power supply for motors + Raspberry Pi    |
| Car Chassis        | 1        | Mounting platform for motors and sensors  |
| Breadboard         | 1        | Prototyping connections                   |
| Power Bank         | 1 (optional) | Power Raspberry Pi independently     |

> 📦 Total POC cost ≈ ₹18,800 (as per Techgium presentation)

---

## 🤝 Connect With Me

Feel free to reach out if you're curious about:

- 🔍 AI/ML for real-world robotics
- 🤖 AI in embedded hardware systems
- 📈 Data science or computer vision applications
- 💡 Startup ideas around intelligent mobility

📧 ujjwalrajbgis@gmail.com  
🔗 [LinkedIn](https://www.linkedin.com/in/beingujjwalraj/)

## 🧑‍💻 Authors

- **Ujjwal Raj** – System Design, AI Modeling, Hardware & Integration  
  > *Finalist at TECHgium 8th Edition | IIITDM Kancheepuram*

---
