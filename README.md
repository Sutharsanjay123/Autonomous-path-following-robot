# 🚗 Autonomous-path-following-robot

## Autonomous Lane Follower using Raspberry Pi

This project demonstrates an autonomous robot that follows lane markings using a Raspberry Pi, OpenCV, and Picamera2. It uses histogram-based lane detection and adjusts motor speeds accordingly via an **L298N motor controller** to stay within the lane.

---

## 🖼️ Screenshots (Optional)

You can place demo images or videos in a `media/` folder.

| Lane Detection      | Perspective Warp View | Steering Overlay    |
| ------------------- | --------------------- | ------------------- |
| *(Add images here)* | *(Add images here)*   | *(Add images here)* |

---

## 🔥 Features

* 🎥 Real-time video feed from **PiCamera2**
* 🧠 Histogram-based lane detection
* 🔁 Perspective transformation for top-down view
* 📐 Angle estimation and smooth motor control
* ⚙️ L298N motor driver to drive 2 DC motors via GPIO
* 📝 Configurable parameters via YAML file
* 💾 Optional frame saving for debugging
* 🚨 Failsafe stop if no lane detected
* 📈 Runtime logging to `lane_follower.log`

---

## 📁 Folder Structure

```
├── lane_follower.py              # Main controller script
├── lane_follower_config.yaml    # YAML configuration
├── lane_follower.log            # Logging file
├── /home/pi/lane_follower_images/  # (Optional) Saved debug frames
├── README.md
└── media/                        # (Optional) Screenshots, GIFs
```

---

## 🧰 Requirements

* Raspberry Pi 4/3B+
* PiCamera2 (libcamera compatible)
* L298N motor controller
* 2x DC motors
* Jumper wires, breadboard
* External power for motors

---

## 🔧 Installation

```bash
sudo apt update
sudo apt install python3-opencv python3-pip libyaml-dev -y
pip3 install RPi.GPIO picamera2 numpy pyyaml
```

Make sure your Raspberry Pi is configured for `libcamera` and `picamera2`.

---

## ⚙️ Configuration Example

**lane\_follower\_config.yaml**:

```yaml
wheelbase: 0.2
wheel_radius: 0.03
base_speed: 40
pwm_frequency: 1000
warp_points: [102, 80, 20, 214]
lane_threshold: [80, 255]
base_rpm: 60
max_angle: 30
autonomous_mode: true
debug_display: false
save_images: true
save_path: "/home/pi/lane_follower_images/"
max_run_time: 120
stop_on_no_lane: true
```

---

## ▶️ Run the Robot

```bash
python3 lane_follower.py
```

The robot will start detecting lanes and follow them by adjusting its left and right motor speeds dynamically.

---

## 🛠️ Troubleshooting

* **Camera not detected?** Run `libcamera-hello` to verify.
* **Motors not moving?** Double-check GPIO connections.
* **Too much lane jitter?** Adjust `warp_points`, histogram logic, or add image smoothing.
* **Debugging visuals not shown?** Set `debug_display: true` in YAML.

---

## 📜 License

This project is released under the **MIT License**.

---

## 🙌 Acknowledgements

* [OpenCV](https://opencv.org/)
* [PiCamera2](https://github.com/raspberrypi/picamera2)
* [Tux Robotics](https://tuxrpi.com/)
* Community tutorials on line following with Raspberry Pi
