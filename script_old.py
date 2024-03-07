import RPi.GPIO as GPIO
import time

# GPIO pin numbers for ultrasonic sensors (JSN-SR04T)
ultrasonic_pins = {
    'ultrasonic_sensor1': {'trigger': 35, 'echo': 37},
    'ultrasonic_sensor2': {'
# Initialize GPIO
GPIO.setmode(GPIO.BOARD)

# Initialize ultrasonic sensors
for sensor, pins in ultrasonic_pins.items():
    GPIO.setup(pins['trigger'], GPIO.OUT)
    GPIO.setup(pins['echo'], GPIO.IN)

# Initialize infrared sensors
for sensor, pins in infrared_pins.items():
    GPIO.setup(pins['pin1'], GPIO.IN)
    GPIO.setup(pins['pin2'], GPIO.IN)

def measure_ultrasonic_distance(trigger_pin, echo_pin):
    print("start")
    GPIO.output(trigger_pin, False)
    time.sleep(0.1)

    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, Fals:e)

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

try:
    while True:
        # Measure distance using ultrasonic sensors (JSN-SR04T)
        for sensor, pins in ultrasonic_pins.items():
            dist = measure_ultrasonic_distance(pins['trigger'], pins['echo'])
            if dist != -1:
                print(f"Ultrasonic {sensor} - Distance: {dist} cm")
            else:
                print(f"Ultrasonic {sensor} - Distance: Measurement timeout")

            time.sleep(0.1)  # Additional delay between ultrasonic measurements
        # Measure proximity using infrared sensors
#        for sensor, pins in infrared_pins.items():
#            proximity1 = GPIO.input(pins['pin1'])
#            proximity2 = GPIO.input(pins['pin2'])
#            print(f"Infrared {sensor} - Proximity1: {'Detected' if proximity1 == GPIO.LOW else 'Not Detected'}")
#            print(f"Infrared {sensor} - Proximity2: {'Detected' if proximity2 == GPIO.LOW else 'Not Detected'}")

        time.sleep(1)  # Delay between measurements

except KeyboardInterrupt:
    print("Measurement stopped by user")
finally:
    # Clean up GPIO
    GPIO.cleanup()
