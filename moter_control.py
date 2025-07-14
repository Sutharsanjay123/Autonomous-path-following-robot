import RPi.GPIO as GPIO
from time import sleep
import logging

# Setup logging
logging.basicConfig(filename='motor_log.txt', level=logging.INFO, format='%(asctime)s - %(message)s')

# GPIO pin definitions
in1, in2, in3, in4 = 23, 24, 17, 27  # Motor direction pins
en1, en2 = 25, 22                    # PWM enable pins

# Default direction (1 = forward, 0 = backward)
dir1, dir2 = 1, 1

# Default speed (duty cycle in %)
base_speed = 50
turn_duration_base = 0.01  # seconds per degree (calibrate for your robot)

MAX_RPM = 300  # Motor's max RPM at 100% duty cycle

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([in1, in2, in3, in4, en1, en2], GPIO.OUT)
GPIO.output([in1, in2, in3, in4], GPIO.LOW)

# Setup PWM
p1, p2 = GPIO.PWM(en1, 1000), GPIO.PWM(en2, 1000)  # 1 kHz frequency
p1.start(base_speed)
p2.start(base_speed)

# Utility Functions
def calculate_rpm(duty_cycle):
    return (abs(duty_cycle) / 100.0) * MAX_RPM

def set_motor(in_a, in_b, direction):
    GPIO.output(in_a, GPIO.HIGH if direction == 1 else GPIO.LOW)
    GPIO.output(in_b, GPIO.LOW if direction == 1 else GPIO.HIGH)

def update_motor_direction():
    set_motor(in1, in2, dir1)
    set_motor(in3, in4, dir2)

def stop_motors():
    GPIO.output([in1, in2, in3, in4], GPIO.LOW)

# Turning function
def turn_vehicle(angle):
    global base_speed

    if angle < -180 or angle > 180:
        print("Invalid angle! Enter between -180° and 180°")
        return

    direction = "RIGHT" if angle > 0 else "LEFT"
    print(f"Turning vehicle {abs(angle)}° to the {direction}...")

    turn_duration = turn_duration_base * abs(angle)
    adjust = min(50, int(abs(angle) / 4))  # scale PWM difference

    if angle > 0:
        left_speed = base_speed + adjust
        right_speed = base_speed - adjust
    elif angle < 0:
        left_speed = base_speed - adjust
        right_speed = base_speed + adjust
    else:
        print("No turn needed for 0°")
        return

    if abs(angle) == 180:
        left_speed = base_speed
        right_speed = -base_speed

    # Determine direction for left motor
    if left_speed < 0:
        set_motor(in1, in2, 0)
    else:
        set_motor(in1, in2, 1)

    # Determine direction for right motor
    if right_speed < 0:
        set_motor(in3, in4, 0)
    else:
        set_motor(in3, in4, 1)

    left_pwm = max(0, min(100, abs(left_speed)))
    right_pwm = max(0, min(100, abs(right_speed)))

    p1.ChangeDutyCycle(left_pwm)
    p2.ChangeDutyCycle(right_pwm)

    rpm1 = calculate_rpm(left_speed)
    rpm2 = calculate_rpm(right_speed)

    print(f"Left: {left_pwm}% ({rpm1:.2f} RPM), Right: {right_pwm}% ({rpm2:.2f} RPM)")
    logging.info(f"Turn {angle}° -> L:{left_pwm}%, R:{right_pwm}%, L_RPM:{rpm1:.2f}, R_RPM:{rpm2:.2f}")

    sleep(turn_duration)

    # Reset motors
    update_motor_direction()
    p1.ChangeDutyCycle(base_speed)
    p2.ChangeDutyCycle(base_speed)

# CLI Instructions
print("\nMotor Controller Started (Default Speed: 50%)")
print("Commands:")
print("r - Run motors | s - Stop motors | f - Forward | b - Backward")
print("l - Low Speed | m - Medium Speed | h - High Speed")
print("t - Turn (Enter angle -180 to 180) | e - Exit\n")

try:
    while True:
        x = input("Enter command: ").strip().lower()

        if x == 'r':
            print("Running motors...")
            update_motor_direction()

        elif x == 's':
            print("Stopping motors...")
            stop_motors()

        elif x == 'f':
            print("Direction set to FORWARD")
            dir1, dir2 = 1, 1
            update_motor_direction()

        elif x == 'b':
            print("Direction set to BACKWARD")
            dir1, dir2 = 0, 0
            update_motor_direction()

        elif x == 'l':
            base_speed = 25
            p1.ChangeDutyCycle(base_speed)
            p2.ChangeDutyCycle(base_speed)
            print(f"Speed set to LOW: {calculate_rpm(base_speed):.2f} RPM")

        elif x == 'm':
            base_speed = 50
            p1.ChangeDutyCycle(base_speed)
            p2.ChangeDutyCycle(base_speed)
            print(f"Speed set to MEDIUM: {calculate_rpm(base_speed):.2f} RPM")

        elif x == 'h':
            base_speed = 75
            p1.ChangeDutyCycle(base_speed)
            p2.ChangeDutyCycle(base_speed)
            print(f"Speed set to HIGH: {calculate_rpm(base_speed):.2f} RPM")

        elif x == 't':
            try:
                angle = int(input("Enter turn angle (-180 to 180): "))
                turn_vehicle(angle)
            except ValueError:
                print("Invalid input! Please enter an integer.")

        elif x == 'e':
            print("Exiting... Cleaning up GPIO")
            break

        else:
            print("Invalid command! Try again.")

finally:
    stop_motors()
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    GPIO.cleanup()
    print("GPIO cleanup complete. Goodbye!")
