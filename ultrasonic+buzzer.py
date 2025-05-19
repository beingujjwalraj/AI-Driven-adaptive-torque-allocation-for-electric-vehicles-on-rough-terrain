import serial
import time

# Update with your correct port
PORT = '/dev/cu.usbserial-0001'  # Use cu.* on macOS
BAUDRATE = 115200  # Default for LDS-01

try:
    # Open the serial connection
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"Connected to {PORT} at {BAUDRATE} baud.\nReading data...\n")

    while True:
        if ser.in_waiting:
            data = ser.read(42)  # LDS-01 sends 42-byte packets
            print(data)  # raw bytes; can decode further if needed

except KeyboardInterrupt:
    print("\nStopped by user.")

except serial.SerialException as e:
    print(f"Serial error: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")