import serial
import time

# Adjust to the correct serial port
# Check with: ls /dev/tty*
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

time.sleep(2)  # Give ESP32 time to reset

try:
    while True:
        # Send message to ESP32
        ser.write(b'Hello from Raspberry Pi\n')
        print("Sent: Hello from Raspberry Pi")

        # Read response
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print("Received:", response)

        time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
