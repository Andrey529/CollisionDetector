import RPi.GPIO as GPIO
import time
import serial
# GPIO pin numbers for ultrasonic sensors (JSN-SR04T)
ultrasonic_pins = {
    'ultrasonic_sensor1': {'trigger': 35, 'echo': 37},
    'ultrasonic_sensor2': {'trigger': 31, 'echo': 33}
}

lidar_ports = {
    'lidar_sensor1': {'RX': 38, 'TX': 40},  # Assuming TFmini Plus 1 RX/TX pins are connected to GPIO 38/40
    'lidar_sensor2': {'RX': 32, 'TX': 36}   # Assuming TFmini Plus 2 RX/TX pins are connected to GPIO 32/36
}


# Initialize GPIO
GPIO.setmode(GPIO.BOARD)

# Initialize ultrasonic sensors
for sensor, pins in ultrasonic_pins.items():
    GPIO.setup(pins['trigger'], GPIO.OUT)
    GPIO.setup(pins['echo'], GPIO.IN)

lidar_sensors = {}
for sensor, pins in lidar_ports.items():
    ser = serial.Serial()
    ser.port = '/dev/ttyS0'  # Serial port for Raspberry Pi (default is ttyAMA0)
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE

    ser.open()
    lidar_sensors[sensor] = ser

def measure_ultrasonic_distance(trigger_pin, echo_pin):
    print("start")
    GPIO.output(trigger_pin, False)
    time.sleep(0.1)

    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    pulse_start = time.time()
    pulse_timeout = pulse_start + 0.1
    pulse_end = time.time()

    while GPIO.input(echo_pin) == 0 and pulse_start <= pulse_timeout:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == 1 and pulse_end <= pulse_timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound is 34300 cm/s
    distance = round(distance, 2) if pulse_duration < 0.1 else -1
    print("end")
    return distance

def measure_lidar_distance(ser):
    try:
        ser.write(b'\x42\x57\x02\x00\x00\x00\x01\x25')  # Send command to TFmini Plus
        ser.timeout = 0.1
        data = ser.read(9)  # Read data (9 bytes expected)

        if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
            distance = (data[2] + data[3] * 256) / 100.0
            return distance
        else:
            return -1  # Return -1 if data format is invalid or timeout
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
        return -1
    

try:
    while True:
        # Measure distance using ultrasonic sensors (JSN-SR04T)
        for sensor, pins in ultrasonic_pins.items():
            dist = measure_ultrasonic_distance(pins['trigger'], pins['echo'])
            if dist != -1:
                print(f"Ultrasonic {sensor} - Distance: {dist} cm")
            else:
                print(f"Ultrasonic {sensor} - Distance: Measurement timeout")
            time.sleep(0.1)

          # Additional delay between ultrasonic measurements

        for sensor, ser in lidar_sensors.items():
            print("start")
            lidar_dist = measure_lidar_distance(ser)
            print("end")
            if lidar_dist != -1:
                print(f"TFmini Plus LIDAR {sensor} - Distance: {lidar_dist} m")
            else:
                print(f"TFmini Plus LIDAR {sensor} - Distance: Measurement error")
            time.sleep(0.1)            
        time.sleep(1)  # Delay between measurements

except KeyboardInterrupt:
    print("Measurement stopped by user")
finally:
    # Clean up GPIO
    GPIO.cleanup()
