import Jetson.GPIO as GPIO
import select
import os.path
import time
import threading

# named pipe filename
TX_FILENAME = './result'
RX_FILENAME = './direction'

# constants
RELAY_PIN = 21
DIR_LEFT = b'\x00'
DIR_RIGHT = b'\xFF'
DIR_CENTER = b'\x0F'

# global variables
receive_flag = True
presentdir = b'\x0F'
immergstop_flag = False
moveend_flag = False


def init_GPIO():
    GPIO.output(RELAY_PIN, GPIO.LOW)


def turnon_Magnet():
    GPIO.output(RELAY_PIN, GPIO.HIGH)


def turnoff_Magnet():
    GPIO.output(RELAY_PIN, GPIO.LOW)


def moveThread(movedir):
    global presentdir, moveend_flag, immergstop_flag, ser

    # 움직일지 말지 정보
    # if sign == "MOVE":
    #     move = b'm\n'
    # elif sign == "STOP":
    #     move = b's\n'
    # else:
    #     move = b'INVALID\n'

    # 도달 목적지 정보
    if movedir == "LEFT":
        command_bytes = b'l'
    elif movedir == "RIGHT":
        command_bytes = b'r'
    elif movedir == "CENTER":
        command_bytes = b'c'
    elif movedir == "STOP":
        command_bytes = b's'
    else:
        command_bytes = b'x'

    # 신호 전송
    ser.write(command_bytes)

    time.sleep(1)  # STM32가 응답대기시간 1초
    if ser.in_waiting > 0:
        receive = ser.read(1)
        print(f"Received from STM32: {receive}")
        presentdir = receive  # `presentdir`에 응답값 저장 이거는 과거 상황과 현재 상황을 위한 전역변수
    else:
        receive = "NO"  # 못받았을 경우

    moveend_flag = True
    return receive


def sensorThread():
    while True:
        # check obstacle
        # if there is obstacle, set immergstop_flag True
        if moveend_flag:
            break


# def send_result(result):
#     with open(TX_FILENAME, "wb") as tx:
#         tx.write(result.encode())


if not os.path.exists(TX_FILENAME):
    os.mkfifo(TX_FILENAME)
if os.path.exists(TX_FILENAME):
    try:
        while True:
            with open(RX_FILENAME, "rb") as rx:
                print("receiving from detection_code")
                # receive direction data
                data = rx.read(1)
                movTh = threading.Thread(target=moveThread, args=(data))
                ultTh = threading.Thread(target=sensorThread)

                # moving
                ultTh.start()
                movTh.start()
                ultTh.join()
                result=movTh.join()

                # 움직임이 끝나고 결과를 전송
                # result = f"Movement completed. Direction: {presentdir}, Emergency Stop: {immergstop_flag}"
                with open(TX_FILENAME, "wb") as tx:
                    if result == 'l':
                        bd = b'\x00'
                    elif result == 'r':
                        bd = b'\xff'
                    elif result == 'c':
                        bd = b'\x0f'
                    else:
                        print("invalid access")
                        continue


                print("전송완료")
                immergstop_flag = False
                moveend_flag = False

                # ignore data received while moving
                rlist, _, _ = select.select([rx], [], [], 0)
                if rlist:
                    data = os.read(rx.fileno(), 4096)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
