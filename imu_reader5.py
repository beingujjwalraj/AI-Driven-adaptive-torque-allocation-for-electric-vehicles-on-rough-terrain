import subprocess
import json
import csv
import math
import time
import numpy as np
from scipy.optimize import minimize

# Constants
VEHICLE_MASS = 1326  # kg
WHEEL_RADIUS = 0.3  # meters
TORQUE_REQUEST = 500  # Nm driver request
MAX_TORQUE = 200  # Nm per wheel
C_SLIP = 0.1  # Slip coefficient for ki calculation
MOTOR_KV = 120  # RPM per volt (motor constant)
BATTERY_VOLTAGE = 12  # volts
MAX_RPM = 100000  # Maximum RPM for duty cycle calculation

def compute_velocity(acceleration, prev_velocity, dt):
    return prev_velocity + acceleration * dt

def compute_force(acceleration):
    return acceleration * VEHICLE_MASS

def wheel_slip_angle(vx_wheel, vy_wheel):
    return math.atan2(vy_wheel, vx_wheel) if vx_wheel != 0 else 0

#rigid body velocity equations
def calculate_wheel_velocities(vx, vy, wz, wheel_positions):
    """Calculate individual wheel velocities based on vehicle dynamics"""
    wheel_velocities = []
    for (x, y) in wheel_positions:
        vx_wheel = vx - wz * y
        vy_wheel = vy + wz * x
        wheel_velocities.append((vx_wheel, vy_wheel))
    return wheel_velocities

def calculate_ki(slip_angle):
    """Slip adaptation factor calculation"""
    return max(0, 1 - C_SLIP * abs(slip_angle))

def torque_to_rpm(torque, ki):
    """Convert torque to RPM considering motor characteristics"""
    if torque == 0: return 0
    effective_torque = torque * ki
    return (effective_torque * MOTOR_KV * BATTERY_VOLTAGE) / (2 * math.pi * WHEEL_RADIUS)

def allocate_torque(k_factors):
    """Optimized torque allocation with slip adaptation"""
    def objective(T):
        torque_error = (TORQUE_REQUEST - np.sum(T))**2
        balance_error = (np.sum(T[::2]) - np.sum(T[1::2]))**2  # Front-rear balance
        return torque_error + balance_error

    constraints = [
        {'type': 'ineq', 'fun': lambda T: T},
        {'type': 'ineq', 'fun': lambda T: MAX_TORQUE - T}
    ]

    bounds = [(0, MAX_TORQUE * k) for k in k_factors]
    initial_guess = np.array([TORQUE_REQUEST/4 * k for k in k_factors])
    
    result = minimize(objective, initial_guess, 
                     constraints=constraints, 
                     bounds=bounds,
                     method='SLSQP')
    
    return result.x if result.success else np.zeros(4)

# Wheel positions (relative to CG) [front_left, front_right, rear_left, rear_right]
WHEEL_POSITIONS = [(-1, 0.75), (-1, -0.75), (1, 0.75), (1, -0.75)]

# CSV configuration
csv_filename = "training_data1.csv"
headers = ["ax", "ay", "az", "wx", "wy", "wz",  # Raw IMU data
           "fx", "fy", "vx", "vy",              # Calculated forces/velocities
           "fl_slip", "fr_slip", "rl_slip", "rr_slip",
           "fl_ki", "fr_ki", "rl_ki", "rr_ki",
           "fl_rpm", "fr_rpm", "rl_rpm", "rr_rpm",
           "fl_duty", "fr_duty", "rl_duty", "rr_duty"]

def parse_and_store_imu():
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)

        imu_process = subprocess.Popen([
            "gz", "topic", "-e", "-t", "/imu", "--json-output"
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        prev_time = None
        vx, vy, wz = 0, 0, 0

        try:
            for line in imu_process.stdout:
                if line.strip():
                    data = json.loads(line.strip())
                    # Raw IMU measurements
                    ax = data["linearAcceleration"]["x"]
                    ay = data["linearAcceleration"]["y"]
                    az = data["linearAcceleration"]["z"]
                    wx = data["angularVelocity"]["x"]
                    wy = data["angularVelocity"]["y"]
                    wz = data["angularVelocity"]["z"]

                    current_time = time.time()
                    dt = current_time - prev_time if prev_time else 0.01
                    prev_time = current_time

                    # Calculate vehicle dynamics
                    vx = compute_velocity(ax, vx, dt)
                    vy = compute_velocity(ay, vy, dt)
                    fx = compute_force(ax)
                    fy = compute_force(ay)

                    # Calculate individual wheel velocities
                    wheel_vels = calculate_wheel_velocities(vx, vy, wz, WHEEL_POSITIONS)
                    
                    # Calculate slip angles and ki factors for each wheel
                    slip_angles = []
                    k_factors = []
                    for vx_wheel, vy_wheel in wheel_vels:
                        alpha = wheel_slip_angle(vx_wheel, vy_wheel)
                        slip_angles.append(alpha)
                        k_factors.append(calculate_ki(alpha))

                    # Torque allocation and RPM calculation
                    torques = allocate_torque(k_factors)
                    rpms = [torque_to_rpm(t, k) 
                           for t, k in zip(torques, k_factors)]
                    
                    # Calculate duty cycles
                    duty_cycles = [(abs(rpm) / MAX_RPM) * 100 for rpm in rpms]

                    # Prepare data row with all parameters
                    row = [
                        # Raw IMU data
                        ax, ay, az, wx, wy, wz,
                        # Calculated parameters
                        fx, fy, vx, vy,
                        # Slip angles
                        *slip_angles,
                        # Slip factors
                        *k_factors,
                        # RPM values
                        *rpms,
                        # Duty cycles
                        *duty_cycles
                    ]
                    writer.writerow(row)
                    print(f"Recorded: Duties {duty_cycles} | RPMs {rpms}")

        except KeyboardInterrupt:
            print("\nData collection stopped.")
        finally:
            imu_process.terminate()

if __name__ == "__main__":
    print("Starting data collection...")
    parse_and_store_imu()