import RPi.GPIO as GPIO
import time
import serial
import sys
import os
import output_matrix_calculation
import matrix_visualisation as matrix_visualisation
import visual
# GPIO pin numbers for ultrasonic sensors (JSN-SR04T)
ultrasonic_pins = {
    'ultrasonic_sensor1': {'trigger': 35, 'echo': 37},
    'ultrasonic_sensor2': {'trigger': 31, 'echo': 33}
}
lidars_config = {
    'lidar': {'port': '/dev/ttyUSB0', 'braudrate': '115200'},
    'lidar1': {'port': '/dev/ttyAMA0', 'braudrate': '115200'}
}

def print_sectors():
    art = [
        "                      ..:==*******==:.",
        "                  ..-*=:..   .%    ..:+*.",
        "                  .++..      .%       ..+-",
        "                  .:#.       .%        .#:",
        "                  ..%.       .%        .#.",
        "                    #:   1   .%   2    :#.",
        "                    +-       .%        *+",
        "                    -=.      .%       .#:",
        "                    :#.      .%       .%.",
        "                    .#.      .%       -#.",
        "                    .#.      .%       ==.",
        "                    .*=      .%       +-",
        "                     -%      .%       #:",
        "                     .%.     .%      .#.",
        "                     .%:     .%     ..*.",
        "                     .==     .%     .*+.",
        "                     .:=     .%     .%:.",
        "                     .:*     .%     .%..",
        "                    ...#......%.....:%...",
        "              ..*%%*=::*::::=*@+-:::++::+#%%+.",
        "         ..=#*-.....  .*-.   -@    .=:  ......=#*-...",
        "       .:#+.          .=#.   -@    .*:           .*#:.",
        "     .=#:..           ..@. 5 -@  6 .#.            ..:@-",
        "   .-#:.              ..@:   -@    .*.               .-%-.",
        "  .#=                   #=   -@    -+.                 .++..",
        "  .=#         3         =+   -@    #=        4         .#:.",
        "   .:#..                .*.  -@   .@.                .:%:.",
        "     -%.                .#.  -@  .:@.                :#.",
        "      .%:               .*.  -@  .=#.              .-%.",
        "      .:%:.             .+:  -@  .+:.             .-*..",
        "       ..#-.            .=#  -@  .*.             .=*..",
        "         .#-.            :@  -@  .#            ..=+.",
        "          .+=            .@..-@ .:*            .*+.",
        "           .+*.          .%-.-@ .-+           .*-",
        "           ..=*.         .-*.-@ .#=         ..%-..",
        "             .-#..       ..*.-@ .%:        ..#..",
        "              .:#.        .#.-@ .%.        :@:",
        "                :%..       *:-@ -#.     ..:#.",
        "                ..%:       =:-@ *-      .-%..",
        "                  .#:.     ==-@.*.     .-*.",
        "                  ..#-.    -#-@.#.    .+*.",
        "                    .#=.   .%-@:*.  ..+=.",
        "                     .+=   .%=@:=.  .*=.",
        "                      .+*.  +#@+=  .*-",
        "                        :*. :@@%-..@-",
        "                        .-%..@@%..#..",
        "                         ..#.%@%:%:",
        "                           :%+@##.",
        "                            .%@%.",
        "                            ..:.."
    ]

    for line in art:
        print(line)

def clear_lines_from_bottom(num_lines):
    # Move cursor up and clear lines
    for _ in range(num_lines):
        sys.stdout.write("\033[A")  # Move cursor up one line
        sys.stdout.write("\033[K")  # Clear line

def getSectors(numbers):
    left_inf, right_inf, left_lid, right_lid = numbers
    result = ""

    if left_inf > 0:
        result += "3"
        if left_inf > 0 and left_lid > 0 and left_lid < left_inf:
            result += " 5"

    if right_inf > 0:
        result += " 4"
        if right_inf > 0 and right_lid > 0 and right_lid < right_inf:
            result += " 6"

    if left_lid > 0 and left_lid > left_inf:
        result += " 1"

    if right_lid > 0 and right_lid > right_inf:
        result += " 2"

    return result


# Initialize GPIO
GPIO.setmode(GPIO.BOARD)

# Initialize ultrasonic sensors
for sensor, pins in ultrasonic_pins.items():
    GPIO.setup(pins['trigger'], GPIO.OUT)
    GPIO.setup(pins['echo'], GPIO.IN)

lidars_serials = {}

for name, options in lidars_config.items():
    ser = serial.Serial()
    ser.port = options['port']
    ser.baudrate = options['braudrate']
    ser.open()
    lidars_serials[name] = ser
    


def getUltrasonicDistance(trigger_pin, echo_pin):
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
    if (distance < 25):
        return -2
    elif (distance > 450):
        return -3
    elif (pulse_duration < 0.1):
        return  round(distance,2)
    else:
        return - 1

def getLidarDistance(ser):
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)   
        ser.reset_input_buffer() 
        if recv[0] == 0x59 and recv[1] == 0x59:
            print(recv)
            distance = recv[2] + recv[3] * 256
            ser.reset_input_buffer()
            return distance
        else:
            return -1
    else:
        return -2 

try:
    print_sectors()
    distances = []
    while True:
        # Measure distance using ultrasonic sensors (JSN-SR04T)
        for sensor, pins in ultrasonic_pins.items():
            dist = getUltrasonicDistance(pins['trigger'], pins['echo'])
            if dist == -1:
                print(f"Ultrasonic {sensor} - Distance: Measurement timeout")
                distances.append(0)
            elif dist == -2:
                print(f"Ultrasonic {sensor} - Distanse: Too low")
                distances.append(0)
            elif dist == -3:
                print(f"Ultrasonic {sensor} - Distanse: Too hign")
                distances.append(0)
            else:
                print(f"Ultrasonic {sensor} - Distanse: {dist} cm")
                distances.append(dist)
                
        for name, ser in lidars_serials.items():
            dist = getLidarDistance(ser)
            if dist == -1:
                print(f"lidar {name} - Distance: distance error ")
                distances.append(0)
            elif dist == -2:
                print(f"lidar {name} - Distance: serial port not ready")
                distances.append(0)
            else:
                print(f"lidar {name} - Distance: {dist} cm")
                distances.append(dist)
        
        sectors = getSectors(distances)
        output_data = output_matrix_calculation.format_output_data(distances)
        matrix_visualisation.print_data(output_data)
        print(f"Object detected at sectors: {sectors}")
        time.sleep(0.5)
        distances = []
        clear_lines_from_bottom(6)
except KeyboardInterrupt:
    print("Measurement stopped by user")
finally:
    #os.system('clear')
    GPIO.cleanup()
    for name, ser in lidars_serials.items():
        ser.close()
