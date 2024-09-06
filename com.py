import Jetson.GPIO as GPIO
import select
import os
import time
import threading
import serial
import numpy as np

# serial port
SERPORT = '/dev/ttyUSB1'
# named pipe filename
TX_FILENAME = './result'
RX_FILENAME = './direction'
# constants
DIR_LEFT = b'\x00'
DIR_RIGHT = b'\xFF'
DIR_CENTER = b'\x0F'
STOP_SIGNAL = b'\xFF'  # Define a stop signal, e.g., \xFF
# global variables
receive_flag = False
presentdir = b'\x0F'
immergstop_flag = False
moveend_flag = False

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO_TRIGGER = 19
GPIO_ECHO = 21
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def get_distance():
    # Send trigger pulse
    GPIO.output(GPIO_TRIGGER, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, GPIO.LOW)
    # Wait for echo start
    while GPIO.input(GPIO_ECHO) == 0:
        start = time.time()
    # Wait for echo end
    while GPIO.input(GPIO_ECHO) == 1:
        stop = time.time()
    # Calculate distance
    check_time = stop - start
    distance = check_time * 34300 / 2
    return distance

def sensorThread():
    global immergstop_flag, moveend_flag
    measurements = []
    filter_size = 5  # Number of measurements for filtering
    
    print("Distance measurement in process")
    GPIO.output(GPIO_TRIGGER, GPIO.LOW)
    print("Sensor is starting!")
    time.sleep(2)
    
    while not moveend_flag:
        distance = get_distance()
        measurements.append(distance)
        # Keep only the latest filter_size measurements
        if len(measurements) > filter_size:
            measurements.pop(0)
        # Calculate the average distance
        avg_distance = np.mean(measurements)
        avg_distance = np.minimum(100, avg_distance)
        print("Distance: %.2f cm" % avg_distance)
        
        # Check for obstacles
        if avg_distance < 20:  # Example: consider as obstacle if less than 20 cm
            immergstop_flag = True
        else:
            immergstop_flag = False
        
        time.sleep(0.1)
    
    print("Distance measurement completed")

# Main #
try:
    ser = serial.Serial(SERPORT, 115200, timeout=1)
    if not os.path.exists(TX_FILENAME):
        os.mkfifo(TX_FILENAME)

    # Start the sensor thread
    sensor_thread = threading.Thread(target=sensorThread)
    sensor_thread.start()

    while True:
        with open(RX_FILENAME, "rb") as rx:
            # Receive direction data via FIFO
            data = rx.read(1)
            # Transmit operation via UART
            if data == b'\x00':
                result = b'l'
            elif data == b'\xff':
                result = b'r'
            elif data == b'\x0f':
                result = b'c'
            else:
                continue

            # If emergency stop flag is set, send stop signal and ignore direction
            if immergstop_flag:
                result = STOP_SIGNAL
                print("Emergency stop triggered!")

            ser.write(result)
            
            # Receive operation result via UART
            result = ser.read(1)
            print("Operation result: {}".format(result))
            
            # Transmit operation result via FIFO
            with open(TX_FILENAME, "wb") as tx:
                if result == b'l':
                    bd = b'\x00'
                elif result == b'r':
                    bd = b'\xff'
                elif result == b'c':
                    bd = b'\x0f'
                elif result == STOP_SIGNAL:
                    bd = STOP_SIGNAL
                else:
                    continue
                tx.write(bd)
            
            # Check for emergency stop
            if immergstop_flag:
                print("Emergency stop! Obstacle detected!")
                # Here you can add additional logic if needed

except KeyboardInterrupt:
    moveend_flag = True
    # Instead of joining the sensor thread, you can set a flag and let it finish
    # sensor_thread.join()  # Wait for the sensor thread to finish
    # Set moveend_flag to stop the sensor thread
finally:
    GPIO.cleanup()
    ser.close()
