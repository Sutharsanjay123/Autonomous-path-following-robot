# 🤖 Autonomous Path Following Robot

## Precision Lane Navigation with Raspberry Pi

This project showcases an intelligent, self-driving robot designed to autonomously follow road lanes using a Raspberry Pi, OpenCV, and PiCamera2. The system leverages image histogram-based lane detection and dynamically adjusts motor speeds through an **L298N motor controller**, ensuring precise path alignment in real time.

---

## 📸 Visual Demo

Add screenshots or demo footage in the `media/` directory to showcase:

| Lane Detection   |
| ---------------- |
![img2_after](https://github.com/user-attachments/assets/30f3228f-415c-4d3b-9bf6-5ed980f501bd)
---

## 🚀 Key Features

* 🎥 **Live video capture** using **PiCamera2**
* 🧠 **Robust lane detection** via histogram-based processing
* 🔁 **Perspective warping** for improved lane geometry understanding
* 📐 **Real-time angle calculation** and directional control
* ⚙️ **L298N Motor Driver**: GPIO-controlled differential drive
* 📝 Fully customizable behavior via `YAML` config
* 💾 Option to **save frames** for analysis/debugging
* 🚨 **Failsafe mechanism** to halt if lane detection fails
* 📈 Logs detailed runtime metrics to `lane_follower.log`


---

## 🛠 Hardware Requirements

* Raspberry Pi 4 or 3B+
* Official PiCamera2 (libcamera-compatible)
* L298N dual H-bridge motor controller
* 2x DC Motors
* Jumper wires and breadboard
* Dedicated external power supply for motors

---

## ⚙️ Software Installation

Ensure the Raspberry Pi is configured with `libcamera` support. Then install dependencies:

```bash
sudo apt update
sudo apt install python3-opencv python3-pip libyaml-dev -y
pip3 install RPi.GPIO picamera2 numpy pyyaml
```


---

## ▶️ Execution

To launch the robot:

```bash
python3 lane_follower.py
```

Once initialized, the robot begins tracking the lane, steering with real-time feedback using differential drive logic.

---

## 🧪 Troubleshooting Tips

* **Camera issues?** Run `libcamera-hello` to verify PiCamera2 is operational.
* **No motor response?** Recheck all GPIO pin connections and L298N wiring.
* **Unstable steering?** Tune `warp_points` and `lane_threshold` values.
* **No debug overlay?** Enable `debug_display: true` in the config.

---

## 🙏 Acknowledgements

* [OpenCV](https://opencv.org/) — Image processing
* [PiCamera2](https://github.com/raspberrypi/picamera2) — Camera integration
* [Tux Robotics](https://tuxrpi.com/) — Inspiration & resources
* Raspberry Pi & the open-source community
