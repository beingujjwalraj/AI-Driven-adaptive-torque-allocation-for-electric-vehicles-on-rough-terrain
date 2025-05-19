from flask import Flask, render_template, request, jsonify
from tensorflow.keras.models import load_model
import RPi.GPIO as GPIO
import joblib
import numpy as np
from threading import Thread, Lock
import time
import smbus2
import math
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

app = Flask(__name__)

# Load AI model and scaler
model = load_model('wheel_duty_predictor.keras')
scaler = joblib.load('scaler_ai.pkl')

# MPU6050 Registers and Initialization
PWR_MGMT_1 = 0x6B
DEVICE_ADDRESS = 0x68
FSYNC = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
bus = smbus2.SMBus(1)

# GPIO Pins
TRIG = 16
ECHO = 18
BUZZER = 12

# Motor Pins Configuration
motor_pins = [
    {'ena': 25, 'in1': 24, 'in2': 23},  # Front Right
    {'ena': 22, 'in1': 17, 'in2': 27},  # Rear Left
    {'ena': 13, 'in1': 5, 'in2': 6},    # Rear Right
    {'ena': 21, 'in1': 19, 'in2': 26}   # Front Left
]

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize Motors
for motor in motor_pins:
    GPIO.setup(motor['in1'], GPIO.OUT)
    GPIO.setup(motor['in2'], GPIO.OUT)
    GPIO.setup(motor['ena'], GPIO.OUT)
    GPIO.output(motor['in1'], GPIO.LOW)
    GPIO.output(motor['in2'], GPIO.LOW)

# PWM Initialization
pwms = [GPIO.PWM(motor['ena'], 1000) for motor in motor_pins]
for pwm in pwms:
    pwm.start(0)

# Ultrasonic and Buzzer Setup
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(BUZZER, GPIO.OUT)
buzzer_pwm = GPIO.PWM(BUZZER, 2000)
buzzer_pwm.start(0)

# LCD Initialization
def setup_lcd():
    common_addresses = [0x27, 0x3F, 0x20, 0x38]
    for addr in common_addresses:
        try:
            return PCF8574_GPIO(addr)
        except: continue
    raise RuntimeError("LCD not found!")

try:
    mcp = setup_lcd()
    lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)
    mcp.output(3, 1)
    lcd.begin(16, 2)
    lcd.message("System Ready")
except Exception as e:
    print(f"LCD Error: {str(e)}")
    lcd = None

# Shared variables
current_direction = "stop"
ai_active = False
sensor_data = {'ax': 0, 'ay': 0, 'gx': 0, 'gy': 0, 'torque': [0,0,0,0]}
obstacle_near = False
lock = Lock()
plot_data = {'gx': [], 'gy': [], 'timestamps': []}

def mpu_init():
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
    bus.write_byte_data(DEVICE_ADDRESS, FSYNC, 0)
    bus.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
    bus.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0)
    time.sleep(0.1)

def read_raw_data(addr):
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low = bus.read_byte_data(DEVICE_ADDRESS, addr+1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    timeout = pulse_start + 0.1
    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        pulse_start = time.time()

    pulse_end = time.time()
    timeout = pulse_end + 0.1
    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        pulse_end = time.time()

    distance = (pulse_end - pulse_start) * 17150
    return round(max(0, min(distance, 300)), 2)

def update_lcd(distance):
    if lcd:
        try:
            lcd.clear()
            lcd.setCursor(0, 0)
            lcd.message(f"Distance: {distance}cm")
            lcd.setCursor(0, 1)
            lcd.message("OBSTACLE NEAR!" if distance <= 10 else "All clear      ")
        except Exception as e:
            print(f"LCD Error: {str(e)}")

def distance_monitor():
    global obstacle_near
    while True:
        distance = get_distance()
        with lock:
            obstacle_near = distance <= 10
        buzzer_pwm.ChangeDutyCycle(95 if obstacle_near else 0)
        update_lcd(distance)
        time.sleep(0.1)

def read_sensors():
    global sensor_data
    mpu_init()
    while True:
        try:
            ax = read_raw_data(0x3B)/16384.0
            ay = read_raw_data(0x3D)/16384.0
            gx = read_raw_data(0x43)/131.0
            gy = read_raw_data(0x45)/131.0

            torque_values = [0.0, 0.0, 0.0, 0.0]

            if ai_active:
                input_data = np.array([[ax, ay, gx, gy, 0]])
                scaled_input = scaler.transform(input_data)
                
                duties = model.predict(scaled_input, verbose=0)
                torque_values = duties[0].tolist()
                print("Raw torque values:", torque_values)
                
                clipped_duties = np.clip(torque_values, 0, 100)
                
                with lock:
                    if obstacle_near:
                        clipped_duties = [0, 0, 0, 0]
                
                for i, duty in enumerate(clipped_duties):
                    pwms[i].ChangeDutyCycle(duty)

            with lock:
                sensor_data = {
                    'ax': round(ax, 2),
                    'ay': round(ay, 2),
                    'gx': round(gx, 2),
                    'gy': round(gy, 2),
                    'torque': [round(float(val), 2) for val in torque_values]
                }
                plot_data['gx'].append(gx)
                plot_data['gy'].append(gy)
                plot_data['timestamps'].append(time.time())
                if len(plot_data['gx']) > 50:
                    plot_data['gx'].pop(0)
                    plot_data['gy'].pop(0)
                    plot_data['timestamps'].pop(0)

        except Exception as e:
            print(f"Sensor error: {str(e)}")
        time.sleep(0.05)

def set_motor_speed(motor_idx, speed, forward=True):
    speed = max(0, min(100, abs(speed)))
    pin1 = motor_pins[motor_idx]['in1']
    pin2 = motor_pins[motor_idx]['in2']
    
    if forward:
        GPIO.output(pin1, GPIO.HIGH)
        GPIO.output(pin2, GPIO.LOW)
    else:
        GPIO.output(pin1, GPIO.LOW)
        GPIO.output(pin2, GPIO.HIGH)
    
    pwms[motor_idx].ChangeDutyCycle(speed)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/move", methods=["POST"])
def move():
    global current_direction
    direction = request.json.get('direction')
    speed = int(request.json.get('speed', 90))
    
    with lock:
        current_direction = direction
        if obstacle_near:
            speed = 0
    
    if direction == "forward":
        for i in range(4):
            set_motor_speed(i, speed, True)
    elif direction == "backward":
        for i in range(4):
            set_motor_speed(i, speed, False)
    elif direction == "left":
        set_motor_speed(0, speed, True)   # Front Right forward
        set_motor_speed(1, speed, False)  # Rear Left backward
        set_motor_speed(2, speed, True)   # Rear Right backward
        set_motor_speed(3, speed, False)  # Front Left forward
    elif direction == "right":
        set_motor_speed(0, speed, False)  # Front Right backward
        set_motor_speed(1, speed, True)   # Rear Left forward
        set_motor_speed(2, speed, False)  # Rear Right forward
        set_motor_speed(3, speed, True)   # Front Left backward
    else:  # stop
        for i in range(4):
            set_motor_speed(i, 0)
            
    return jsonify({"status": "success"})

@app.route("/toggle_ai", methods=["POST"])
def toggle_ai():
    global ai_active
    ai_active = not ai_active
    if not ai_active:
        for i in range(4):
            set_motor_speed(i, 0)
    return jsonify({"ai_active": ai_active})

@app.route("/sensor_data")
def get_sensor_data():
    with lock:
        return jsonify({
            'sensors': sensor_data,
            'plot_data': plot_data
        })

if __name__ == "__main__":
    try:
        sensor_thread = Thread(target=read_sensors)
        distance_thread = Thread(target=distance_monitor)
        sensor_thread.daemon = True
        distance_thread.daemon = True
        sensor_thread.start()
        distance_thread.start()
        app.run(host="0.0.0.0", port=5000, debug=False)
    finally:
        buzzer_pwm.stop()
        if lcd:
            lcd.clear()
        for pwm in pwms:
            pwm.stop()
        GPIO.cleanup()
