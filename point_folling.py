import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import time
import os
import logging
import yaml
from threading import Thread
import signal
import sys

# -----------------------------------------------------------------------------
# PART 1: SYSTEM CONFIGURATION
# -----------------------------------------------------------------------------

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("/home/pi/lane_follower.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("LaneFollower")

class Config:
    """Configuration loader and holder"""
    DEFAULT_CONFIG = {
        # Robot physical properties
        "wheelbase": 0.2,      # Distance between wheels (m)
        "wheel_radius": 0.03,  # Wheel radius (m)
        "max_angle": 30,       # Maximum steering angle (degrees)
       
        # Motor control
        "left_motor_pin1": 17,   # GPIO pins
        "left_motor_pin2": 18,
        "left_motor_pwm": 27,
        "right_motor_pin1": 22,
        "right_motor_pin2": 23,
        "right_motor_pwm": 24,
        "pwm_frequency": 1000,   # PWM frequency (Hz)
        "base_speed": 40,        # Base speed as percentage of PWM duty cycle
       
        # Lane detection
        "image_width": 480,
        "image_height": 240,
        "curve_avg_factor": 10,  # Number of frames for curve averaging
        "warp_points": [102, 80, 20, 214],  # Initial warp points [x1, y1, x2, y2]
        "lane_threshold": [80, 255],  # Binary threshold values [min, max]
       
        # Control parameters
        "move_time_min": 0.1,    # Minimum time to move before recalculating (seconds)
        "move_time_max": 1.0,    # Maximum time to move before recalculating (seconds)
        "base_rpm": 60,          # Base RPM for straight movement
       
        # Operational parameters
        "autonomous_mode": True,   # Run in autonomous mode without user input
        "debug_display": False,    # Show visual debug info (not for headless Pi)
        "save_images": True,       # Save captured images for debugging
        "save_path": "/home/pi/lane_follower_images/",  # Path to save images
        "max_run_time": 120,       # Maximum run time in seconds (0 for unlimited)
        "stop_on_no_lane": True    # Stop if no lane is detected
    }
   
    @classmethod
    def load(cls, config_path="/home/pi/lane_follower_config.yaml"):
        """Load configuration from file or use defaults"""
        config = cls.DEFAULT_CONFIG.copy()
       
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    loaded_config = yaml.safe_load(file)
                    if loaded_config:
                        config.update(loaded_config)
                logger.info(f"Configuration loaded from {config_path}")
            else:
                logger.warning(f"Config file not found at {config_path}, using defaults")
                # Create default config file
                os.makedirs(os.path.dirname(config_path), exist_ok=True)
                with open(config_path, 'w') as file:
                    yaml.dump(config, file, default_flow_style=False)
                logger.info(f"Default configuration saved to {config_path}")
        except Exception as e:
            logger.error(f"Error loading configuration: {e}")
           
        return config

# Load configuration
config = Config.load()

# Ensure save directory exists
if config["save_images"]:
    os.makedirs(config["save_path"], exist_ok=True)

# -----------------------------------------------------------------------------
# PART 2: IMAGE PROCESSING (LANE DETECTION)
# -----------------------------------------------------------------------------

class LaneDetector:
    """Lane detection and processing class"""
    def __init__(self, config):
        self.config = config
        self.width = config["image_width"]
        self.height = config["image_height"]
        self.curve_list = []
        self.avg_val = config["curve_avg_factor"]
        self.warp_points = config["warp_points"]
        self.threshold_values = config["lane_threshold"]
       
    def thresholding(self, img):
        """Convert image to binary to isolate lane markings"""
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, self.threshold_values[0]])
        upper_white = np.array([179, 30, self.threshold_values[1]])
        mask = cv2.inRange(img_hsv, lower_white, upper_white)
        return mask
   
    def warp_img(self, img, points, w, h, inv=False):
        """Transform image perspective (bird's eye view)"""
        pts1 = np.float32([
            [points[0], h],  # Bottom left
            [points[1], points[2]],  # Top left
            [w-points[1], points[2]],  # Top right
            [w-points[0], h]  # Bottom right
        ])
        pts2 = np.float32([[0, h], [0, 0], [w, 0], [w, h]])
       
        if inv:
            matrix = cv2.getPerspectiveTransform(pts2, pts1)
        else:
            matrix = cv2.getPerspectiveTransform(pts1, pts2)
           
        warped = cv2.warpPerspective(img, matrix, (w, h))
        return warped
   
    def get_histogram(self, img, min_per=0.1, region=1, display=False):
        """Find lane position in the image using histogram analysis"""
        if region == 1:
            hist_values = np.sum(img, axis=0)
        else:
            hist_values = np.sum(img[img.shape[0]//region:, :], axis=0)
           
        # Calculate threshold for minimum percentage
        max_value = np.max(hist_values)
        min_value = min_per * max_value
       
        # Create indices array
        indices = np.arange(len(hist_values))
       
        # Filter values above threshold
        filtered_indices = indices[hist_values > min_value]
       
        # Find average index (center point)
        if len(filtered_indices) > 0:
            base_point = int(np.average(filtered_indices))
        else:
            base_point = int(img.shape[1] // 2)  # Default to center if no lane found
           
        if display and self.config["debug_display"]:
            img_hist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
            for i, intensity in enumerate(hist_values):
                if intensity > min_value:
                    cv2.line(img_hist, (i, img.shape[0]), (i, img.shape[0] - int(intensity // 255 // region)), (255, 0, 255), 1)
            cv2.circle(img_hist, (base_point, img.shape[0] - int(hist_values[base_point] // 255 // region)), 10, (0, 255, 0), cv2.FILLED)
            return base_point, img_hist
       
        return base_point, None
   
    def get_lane_curve(self, img):
        """Process image to calculate lane curvature and steering angle"""
        # Create copies for processing
        img_copy = img.copy()
        img_result = img.copy()
       
        # Step 1: Thresholding
        img_thres = self.thresholding(img)
       
        # Step 2: Perspective warp
        h, w, c = img.shape
        img_warp = self.warp_img(img_thres, self.warp_points, w, h)
       
        # Step 3: Histogram analysis
        mid_point, _ = self.get_histogram(img_warp, region=4, min_per=0.5)
        curve_avg_point, _ = self.get_histogram(img_warp, region=1, min_per=0.9)
        curve_raw = curve_avg_point - mid_point
       
        # Step 4: Curve smoothing (moving average)
        self.curve_list.append(curve_raw)
        if len(self.curve_list) > self.avg_val:
            self.curve_list.pop(0)
       
        # Calculate average curve value
        curve = int(sum(self.curve_list) / len(self.curve_list))
       
        # Step 5: Convert curve to angle
        # Normalize curve value between -1 and 1
        curve_norm = max(min(curve / 100, 1), -1)
       
        # Convert to steering angle in degrees
        angle = np.arctan(curve_norm) * (180 / np.pi)
       
        # Generate debug visualization if needed
        if self.config["debug_display"]:
            # Create inverse warp for visualization
            img_inv_warp = self.warp_img(img_warp, self.warp_points, w, h, inv=True)
            img_inv_warp = cv2.cvtColor(img_inv_warp, cv2.COLOR_GRAY2BGR)
            img_inv_warp[0:h//3, 0:w] = 0, 0, 0  # Blacken top portion
           
            # Create lane color overlay
            img_lane_color = np.zeros_like(img)
            img_lane_color[:] = 0, 255, 0  # Green color
            img_lane_color = cv2.bitwise_and(img_inv_warp, img_lane_color)
           
            # Combine original image with lane overlay
            img_result = cv2.addWeighted(img_result, 1, img_lane_color, 1, 0)
           
            # Add text and indicators
            mid_y = 150
            cv2.putText(img_result, f'Curve: {curve}', (w//2-100, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            cv2.putText(img_result, f'Angle: {angle:.2f} deg', (w//2-100, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
           
            # Draw steering lines
            cv2.line(img_result, (w//2, mid_y),
                    (w//2 + (curve * 3), mid_y), (255, 0, 255), 5)
            cv2.line(img_result, (w//2 + (curve * 3), mid_y - 25),
                    (w//2 + (curve * 3), mid_y + 25), (0, 255, 0), 5)
           
            # Show result
            cv2.imshow('Lane Detection', img_result)
            cv2.waitKey(1)
       
        # Detect if no lane is visible (curve list has high variance)
        no_lane_detected = False
        if len(self.curve_list) >= 3:
            curve_variance = np.var(self.curve_list)
            if curve_variance > 10000:  # Threshold for "no lane detected"
                no_lane_detected = True
                logger.warning("High variance in curve values - possible lane detection issue")
       
        return angle, curve_raw, img_result, no_lane_detected

# -----------------------------------------------------------------------------
# PART 3: MOTOR CONTROL
# -----------------------------------------------------------------------------

class MotorController:
    """Handles motor control for the robot"""
    def __init__(self, config):
        self.config = config
        self.wheelbase = config["wheelbase"]
        self.wheel_radius = config["wheel_radius"]
        self.max_angle = config["max_angle"]
        self.base_rpm = config["base_rpm"]
        self.setup_gpio()
       
        # Convert base RPM to velocity
        self.base_velocity = self.rpm_to_velocity(self.base_rpm)
       
    def setup_gpio(self):
        """Initialize GPIO pins for motor control"""
        # Set GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
       
        # Setup motor pins as output
        GPIO.setup(self.config["left_motor_pin1"], GPIO.OUT)
        GPIO.setup(self.config["left_motor_pin2"], GPIO.OUT)
        GPIO.setup(self.config["left_motor_pwm"], GPIO.OUT)
       
        GPIO.setup(self.config["right_motor_pin1"], GPIO.OUT)
        GPIO.setup(self.config["right_motor_pin2"], GPIO.OUT)
        GPIO.setup(self.config["right_motor_pwm"], GPIO.OUT)
       
        # Initialize PWM on PWM pins
        self.left_pwm = GPIO.PWM(self.config["left_motor_pwm"], self.config["pwm_frequency"])
        self.right_pwm = GPIO.PWM(self.config["right_motor_pwm"], self.config["pwm_frequency"])
       
        # Start PWM with 0% duty cycle
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        logger.info("GPIO setup completed")
   
    def cleanup(self):
        """Clean up GPIO pins before exiting"""
        self.stop()
        GPIO.cleanup()
        logger.info("GPIO cleanup completed")
   
    def rpm_to_velocity(self, rpm):
        """Convert wheel RPM to linear velocity (m/s)"""
        return (rpm * 2 * np.pi * self.wheel_radius) / 60
   
    def velocity_to_rpm(self, velocity):
        """Convert linear velocity to wheel RPM"""
        return (velocity * 60) / (2 * np.pi * self.wheel_radius)
   
    def calculate_wheel_rpms(self, steering_angle):
        """Calculate left and right wheel RPMs based on turn angle"""
        # Constrain steering angle to allowed range
        angle = max(min(steering_angle, self.max_angle), -self.max_angle)
       
        # Handle straight line case
        if abs(angle) < 0.1:
            return self.base_rpm, self.base_rpm
       
        # Calculate turning radius (add small epsilon to prevent division by zero)
        angle_rad = np.radians(angle)
        R = self.wheelbase / np.tan(angle_rad + 1e-6)
       
        # Calculate wheel velocities using differential drive kinematics
        # Positive angle means turning right, so left wheel goes faster
        Vl = self.base_velocity * (1 + (self.wheelbase / (2 * R)))
        Vr = self.base_velocity * (1 - (self.wheelbase / (2 * R)))
       
        # Convert velocities back to RPM
        rpm_l = self.velocity_to_rpm(Vl)
        rpm_r = self.velocity_to_rpm(Vr)
       
        return rpm_l, rpm_r
   
    def control_motors(self, left_rpm, right_rpm):
        """Control motors based on calculated RPMs"""
        # Calculate PWM duty cycles based on RPM ratios
        base_pwm = self.config["base_speed"]
       
        # Calculate ratios with respect to base RPM
        # If RPMs exceed base_rpm, we'll scale up the PWM proportionally
        left_ratio = left_rpm / self.base_rpm
        right_ratio = right_rpm / self.base_rpm
       
        # Calculate PWM values (constrained between 0-100%)
        left_duty = max(min(base_pwm * left_ratio, 100), 0)
        right_duty = max(min(base_pwm * right_ratio, 100), 0)
       
        # Set motor directions
        if left_rpm >= 0:
            GPIO.output(self.config["left_motor_pin1"], GPIO.HIGH)
            GPIO.output(self.config["left_motor_pin2"], GPIO.LOW)
        else:
            GPIO.output(self.config["left_motor_pin1"], GPIO.LOW)
            GPIO.output(self.config["left_motor_pin2"], GPIO.HIGH)
            left_duty = abs(left_duty)
       
        if right_rpm >= 0:
            GPIO.output(self.config["right_motor_pin1"], GPIO.HIGH)
            GPIO.output(self.config["right_motor_pin2"], GPIO.LOW)
        else:
            GPIO.output(self.config["right_motor_pin1"], GPIO.LOW)
            GPIO.output(self.config["right_motor_pin2"], GPIO.HIGH)
            right_duty = abs(right_duty)
       
        # Set PWM duty cycles
        self.left_pwm.ChangeDutyCycle(left_duty)
        self.right_pwm.ChangeDutyCycle(right_duty)
       
        logger.debug(f"Motor Control: Left PWM: {left_duty:.1f}%, Right PWM: {right_duty:.1f}%")
   
    def stop(self):
        """Stop both motors"""
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
        GPIO.output(self.config["left_motor_pin1"], GPIO.LOW)
        GPIO.output(self.config["left_motor_pin2"], GPIO.LOW)
        GPIO.output(self.config["right_motor_pin1"], GPIO.LOW)
        GPIO.output(self.config["right_motor_pin2"], GPIO.LOW)
        logger.info("Motors stopped")

# -----------------------------------------------------------------------------
# PART 4: CAMERA MODULE
# -----------------------------------------------------------------------------

class CameraModule:
    """Handles camera initialization and image capture"""
    def __init__(self, config):
        self.config = config
        self.width = config["image_width"]
        self.height = config["image_height"]
        self.frame_count = 0
        self.picam = None
       
    def initialize(self):
        """Initialize the Raspberry Pi camera"""
        try:
            self.picam = Picamera2()
            self.picam.configure(self.picam.create_preview_configuration(
                main={"format": 'RGB888', "size": (640, 480)}
            ))
            self.picam.start()
            # Allow camera to adjust exposure
            time.sleep(0.5)
            logger.info("Camera initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            return False
   
    def capture_image(self):
        """Capture an image from the Raspberry Pi camera"""
        try:
            # Capture image
            image = self.picam.capture_array()
           
            # Convert from RGB to BGR (OpenCV uses BGR)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
           
            # Resize to expected size
            image = cv2.resize(image, (self.width, self.height))
           
            # Save image if configured
            if self.config["save_images"]:
                self.frame_count += 1
                filename = f"{self.config['save_path']}/frame_{self.frame_count:04d}.jpg"
                cv2.imwrite(filename, image)
                logger.debug(f"Saved image to {filename}")
           
            return image
        except Exception as e:
            logger.error(f"Error capturing image: {e}")
            return None
   
    def shutdown(self):
        """Shutdown the camera"""
        if self.picam:
            self.picam.stop()
            logger.info("Camera shut down")

# -----------------------------------------------------------------------------
# PART 5: MAIN CONTROL SYSTEM
# -----------------------------------------------------------------------------

class LaneFollowerRobot:
    """Main class that ties everything together"""
    def __init__(self, config):
        self.config = config
        self.start_time = time.time()
        self.camera = CameraModule(config)
        self.lane_detector = LaneDetector(config)
        self.motor_controller = MotorController(config)
        self.running = False
       
    def initialize(self):
        """Initialize all components"""
        logger.info("Initializing Lane Follower Robot...")
       
        # Initialize camera
        if not self.camera.initialize():
            logger.error("Failed to initialize camera - exiting")
            return False
       
        # Register signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
       
        logger.info("Lane Follower Robot initialized successfully")
        return True
   
    def signal_handler(self, sig, frame):
        """Handle SIGINT and SIGTERM signals for clean shutdown"""
        logger.info(f"Received signal {sig} - shutting down")
        self.running = False
   
    def run(self):
        """Main execution loop"""
        if not self.initialize():
            return
       
        self.running = True
        consecutive_no_lane = 0
        max_no_lane_frames = 5  # Stop after 5 consecutive frames with no lane
       
        logger.info("Starting autonomous lane following")
       
        # Main control loop
        try:
            while self.running:
                # Check max run time if configured
                if self.config["max_run_time"] > 0:
                    elapsed_time = time.time() - self.start_time
                    if elapsed_time > self.config["max_run_time"]:
                        logger.info(f"Maximum run time reached ({self.config['max_run_time']} seconds)")
                        break
               
                # Capture image
                img = self.camera.capture_image()
                if img is None:
                    logger.warning("Failed to capture image, retrying...")
                    time.sleep(0.5)
                    continue
               
                # Process image to get steering angle
                steering_angle, curve_raw, result_img, no_lane = self.lane_detector.get_lane_curve(img)
               
                # Check if lane is detected
                if no_lane and self.config["stop_on_no_lane"]:
                    consecutive_no_lane += 1
                    if consecutive_no_lane >= max_no_lane_frames:
                        logger.warning("No lane detected for multiple frames - stopping")
                        self.motor_controller.stop()
                        # Wait briefly then try again
                        time.sleep(1)
                        consecutive_no_lane = 0
                        continue
                else:
                    consecutive_no_lane = 0  # Reset counter when lane is detected
               
                # Calculate wheel RPMs based on steering angle
                left_rpm, right_rpm = self.motor_controller.calculate_wheel_rpms(steering_angle)
               
                # Log current status
                logger.info(f"Steering Angle: {steering_angle:.1f}°, L-RPM: {left_rpm:.1f}, R-RPM: {right_rpm:.1f}")
               
                # Control motors
                self.motor_controller.control_motors(left_rpm, right_rpm)
               
                # Calculate move time - proportional to curve value but within limits
                curve_factor = abs(curve_raw) / 100  # Normalize curve (0 to ~1)
                move_time = self.config["move_time_min"] + curve_factor * (
                    self.config["move_time_max"] - self.config["move_time_min"]
                )
               
                # The sharper the curve, the shorter we move before recalculating
                move_time = max(self.config["move_time_min"],
                               min(move_time, self.config["move_time_max"]))
               
                # Wait briefly before recalculating
                time.sleep(move_time)
               
                # Process keyboard input if in debug mode
                if self.config["debug_display"] and cv2.waitKey(1) & 0xFF == ord('q'):
                    logger.info("User requested stop (pressed 'q')")
                    break
       
        except Exception as e:
            logger.error(f"Error in main control loop: {e}")
       
        finally:
            self.shutdown()
   
    def shutdown(self):
        """Clean shutdown of all components"""
        logger.info("Shutting down Lane Follower Robot...")
        self.running = False
       
        # Stop motors
        self.motor_controller.stop()
       
        # Release camera
        self.camera.shutdown()
       
        # Cleanup GPIO
        self.motor_controller.cleanup()
       
        if self.config["debug_display"]:
            cv2.destroyAllWindows()
       
        logger.info("Shutdown complete")

# -----------------------------------------------------------------------------
# PART 6: MAIN ENTRY POINT
# -----------------------------------------------------------------------------

def main():
    """Main entry point"""
    try:
        # Create and run the robot
        robot = LaneFollowerRobot(config)
        robot.run()
    except Exception as e:
        logger.error(f"Unhandled exception: {e}")
        # Try to clean up GPIO even if an exception occurred
        try:
            GPIO.cleanup()
        except:
            pass

if __name__ == "__main__":
    main()
