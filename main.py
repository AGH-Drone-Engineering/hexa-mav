import socket
import threading
import time
from pymavlink import mavutil

HOST = "127.0.0.1"
PORT = 4343

global_destination = ""
the_connection = None

# definicje
ACK_MSG = "ACK"
GOTO_MSG = "GOTO"
GETPOS_MSG = "GETPOS"
SHOOT_MSG = "SHOOT"
PIN_STRZAL = 11
PIN_1 = 9
PIN_2 = 10
DEVICE = 'com8'
BAUD = 57600

thrd_num = 0
conn = None

def map_val(x,in_min, in_max, out_min, out_max) :
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
def map_servo(x):
    return int(map_val(x,0, 180, 500, 2500))

def getpos():
    xx = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
    lat = xx['lat']/10**7
    log = xx['lon']/10**7
    return str(lat) + "," + str(log)


def getheading():
    return str(the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()['hdg']/100)


def getalt():
    return str(the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()['relative_alt']/1000)

def setpos():
    xx = global_destination.split(",")
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                                                                       the_connection.target_component,
                                                                       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                       int(0b110111111000), int(float(xx[0]) * 10 ** 7),
                                                                       int(float(xx[1]) * 10 ** 7), float(getalt()),
                                                                       0, 0, 0, 0, 0, 0, 0, 0))

def moveServo(odl, pin):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, pin, odl, 0, 0, 0, 0, 0)
def strzelaj(kulka):
    if kulka == 1:
        moveServo(2500, PIN_1)
    elif kulka == 2:
        moveServo(2500, PIN_2)
    else:
        print("zly numer kulki")
        return
    moveServo(1400, PIN_STRZAL)
    time.sleep(3)
    moveServo(1100, PIN_STRZAL)
    moveServo(500, PIN_2)
    moveServo(500, PIN_1)

def watcher():
    global thrd_num

    while True:
        dist = int(the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True).to_dict()['wp_dist'])
        if dist < 1:
            break
        else:
            time.sleep(1)

    thrd_num -= 1
    if thrd_num == 0:
        conn.sendall(b"ACK\n")


def start_ack_watcher(x):
    global thrd_num
    thrd = threading.Thread(target=watcher, name="ack pos watcher")
    thrd.start()
    thrd_num+=1


#the_connection = mavutil.mavlink_connection(DEVICE)
the_connection = mavutil.mavlink_connection(DEVICE, BAUD)

the_connection.wait_heartbeat()
print("Odebrano heartbeat")
moveServo(1100, PIN_STRZAL)
print("Serwo IDLE")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sck:
    sck.bind((HOST, PORT))
    sck.listen()
    print("Zbindowane... nasluchiwanie na", PORT)
    conn, addr = sck.accept()
    with conn:
        print("Podlaczono do", addr)
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    break

                data = data.decode()
                if data.endswith("\n"):
                    data = data[:-1]

                if data == GETPOS_MSG:
                    pos = "POS " + getpos() + "," + getheading() + "," + getalt() + "\n"
                    conn.sendall(pos.encode())
                    print("Wysylanie pozycji ", pos)
                    continue

                if data.startswith(GOTO_MSG):
                    if len(data) > len(GOTO_MSG) + 1:
                        pos = data[len(GOTO_MSG) + 1:]
                        global_destination = pos
                        setpos()
                        print("Lot do", global_destination)
                        start_ack_watcher(conn)
                    else:
                        print("[OSTRZEZENIE] Odebrano niepoprawny format komendy", GOTO_MSG)
                    continue
                if data.startswith(SHOOT_MSG):
                    if len(data) > len(SHOOT_MSG) + 1:
                        id_kulki = data[len(SHOOT_MSG) + 1:]
                        print("Strzelam kulkom",id_kulki)
                        strzelaj(int(id_kulki))
                    else:
                        print("[OSTRZEZENIE] Odebrano niepoprawny format komendy", SHOOT_MSG)

                if data.startswith("SERVO"):
                    moveServo(int(data[len("SERVO") + 1:]),PIN_1)
                    print("Poruszam servem",map_servo(int(data[len("SERVO") + 1:])) , PIN_1)

                if data == "ARM":
                    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
                if data == "DISARM":
                    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0, 0, 0, 0, 0)

            except ConnectionError:
                print("Polaczenie zostalo zamkniete",map_servo(int(data[len("SERVO") + 1:])),PIN_1)
                break