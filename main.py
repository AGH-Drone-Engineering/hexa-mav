import socket
import threading
import time
from pymavlink import mavutil

HOST = "127.0.0.1"
PORT = 4344

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
#DEVICE = 'udpout:192.168.1.101:14560'
DEVICE = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_270022000D51393239383638-if00'
BAUD = 115200
# BAUD = 57600
# BAUD = 9600
TARGET_ALTITUDE = 2

conn = None
watcher_running = 0


def check_ok():
    # print(the_connection.recv_match(type='COMMAND_ACK', blocking=True).to_dict())
    return int(the_connection.recv_match(type='COMMAND_ACK', blocking=True).to_dict()['result']) == 0

def get_ok():
    ok = the_connection.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    print(ok)
    return int(ok['result'])

def map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def map_servo(x):
    return int(map_val(x, 0, 180, 500, 2500))

def wait_conn():
    msg = None
    while not msg:
       the_connection.mav.ping_send(int(time.time() * 1e6), 0,  0,  0 )
       msg = the_connection.recv_match()
       time.sleep(0.5)

def getpos():
    print("test")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0, 33, 0, 0, 0, 0, 0, 0)
    xx = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
    print("test")
    print(xx)
    lat = xx['lat'] / 10 ** 7
    log = xx['lon'] / 10 ** 7
    return str(lat) + "," + str(log)


def getheading():
    return str(the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()['hdg'] / 100)


def getalt():
    return str(the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()['relative_alt'] / 1000)


def setpos():
    xx = global_destination.split(",")
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                                                                       the_connection.target_component,
                                                                       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,0, int(0b110111111000), int(float(xx[0]) * 10 ** 7),
                                                                       int(float(xx[1]) * 10 ** 7), float(getalt()),
                                                                       0, 0, 0, 0, 0,0 ,0))


def moveServo(odl, pin):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0, pin, odl, 0, 0, 0, 0)


def strzelaj(kulka):
    if kulka == 1:
        moveServo(2500, PIN_1)
    elif kulka == 2:
        moveServo(2500, PIN_2)
    else:
        print("Zly numer kulki")
        return
    moveServo(1400, PIN_STRZAL)
    time.sleep(3)
    moveServo(1100, PIN_STRZAL)
    moveServo(500, PIN_2)
    moveServo(500, PIN_1)


def arm(force=21196):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0 ,1, 0, 0, 0, 0, 0, 0)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 1, force, 0, 0, 0, 0, 0)
    s = check_ok()
    if s:
        print("Uzbrojono pojazd")
    else:
        print("Nie uzbrojono pojazdu")


def disarm():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0, 0, 0, 0, 0, 0, 0, 0)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 0, 1, 21196, 0, 0, 0, 0)
    the_connection.set_mode_manual()


def rth():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0, 0, 0, 0, 0, 0, 0, 0)


def takeoff():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE)


def land():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LAND,0, 0, 0, 0, 0, 0, 0, 0)

def getfullpos():
    print("Nadawanie prosby o pozycje")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0, 33, 0, 0, 0, 0, 0, 0)
    xx = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
    print("Odebrano pozycje drona")
    print(xx)
    lat =   int(xx['lat']) / 10 ** 7
    log =   int(xx['lon']) / 10 ** 7
    head =  int(xx['hdg']) / 100
    alt =   int(xx['relative_alt']) / 1000
    pos = "POS " + str(lat) + "," + str(log) + "," + str(head) + "," + str(alt) + "\n"
    return pos

def print_ok():
    rslt = get_ok()
    if rslt == 0:
        print("Odebrano komende poprawnie")
    else:
        print("Komena zwrocila status",rslt)
    print()

def watcher():
    global thrd_num

    while True:
        dist = int(the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True).to_dict()['wp_dist'])
        if dist < 1:
            break
        else:
            time.sleep(1)

        conn.sendall(b"ACK\n")
        watcher_running = 0


def start_ack_watcher(x):
    global thrd_num
    if not watcher_running:
        thrd = threading.Thread(target=watcher, name="ack pos watcher")
        thrd.start()
        watcher_running = 1


# the_connection = mavutil.mavlink_connection(DEVICE)
the_connection = mavutil.mavlink_connection(DEVICE, BAUD)

print("Start!, czekam na ping od drona")
wait_conn()
print("Odebrano PING")
the_connection.wait_heartbeat()
print("Odebrano heartbeat")
moveServo(500, PIN_2)
moveServo(500, PIN_1)
moveServo(1100, PIN_STRZAL)
print("Ustawiono silniki/serwa w tryb IDLE")
#print("Czekam na GPS")
#the_connection.wait_gps_fix()
#print("GPS ustawiony")
# print("Serwo IDLE")

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0 , 62, 1000000,0,0,0,0,0)
                                         
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0 , 33, 1000000,0,0,0,0,0)

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
                    pos = getfullpos()
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
                        print("Strzelam kulkom", id_kulki)
                        strzelaj(int(id_kulki))
                    else:
                        print("[OSTRZEZENIE] Odebrano niepoprawny format komendy", SHOOT_MSG)

                if data.startswith("SERVO-RAW"):
                    moveServo(int(data[len("SERVO-RAW") + 1:]), PIN_1)
                    print("Poruszam servem", int(data[len("SERVO_RAW") + 1:]), PIN_1)

                if data.startswith("SERVO"):
                    a = int(map_servo(int(data[len("SERVO-RAW") + 1:])))
                    moveServo(a, PIN_1)
                    print("Poruszam servem", a, PIN_1)

                if data == "FORCE-ARM":
                    arm()

                if data == "ARM":
                    arm(0)

                if data == "DISARM":
                    disarm()
                    print_ok()

                if data == "LAND":
                    land()
                    print_ok()

                if data == "RTH":
                    rth()
                    print_ok()
                    
                if data == "TAKEOFF":
                    takeoff()
                    print_ok()

            except ConnectionError:
                print("Polaczenie zostalo zamkniete")
                break
