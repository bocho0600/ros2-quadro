import serial
import time

# Adjust to the correct serial port
# Check with: ls /dev/tty*
ser = serial.Serial('/dev/serial0', 230400, timeout=1)

time.sleep(2)  # Give ESP32 time to reset

try:
    while True:
        while ser.in_waiting:
            response = ser.readline().decode(errors='replace').strip()
            print(response)
        time.sleep(0.001)  # Reduce sleep for faster polling

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
