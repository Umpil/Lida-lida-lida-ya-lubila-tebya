from dynamixel_sdk import port_handler
from dynamixel_sdk import protocol2_packet_handler
from rplidar import RPLidar

import os, sys
from time import sleep

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()


ESC_ASCII_VALUE = 0x1b

device_name = "COM7"
baudrate = 57600
DXL_MOVING_STATUS_THRESHOLD = 10
device_id = 1

torq_enable = 1
torq_disable = 0

torq_code = 64
goal_pos_code = 116
led_code = 65
preset_pos_code = 132

port_number = port_handler.PortHandler(device_name)

minimum_position_val = 1023
maximum_position_val = 3073

min_max_positions = [minimum_position_val, maximum_position_val]


if port_number.is_open:
    port_number.closePort()

if port_number.openPort():
    print("Opened")
else:
    print("Something went wrong")

if port_number.setBaudRate(baudrate):
    print("Set baudrate")
else:
    print("No baudrate")

packet_hand_2 = protocol2_packet_handler.Protocol2PacketHandler()

packet_hand_2.write1ByteTxRx(port_number, device_id, torq_code, data=torq_enable)

packet_hand_2.write1ByteTxRx(port_number, device_id, led_code, data=1)
#packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, 2048)
sleep(1)

index = 1
go = False
if go:
    while 1:
        dxl_present_pos = packet_hand_2.read4ByteTxRx(port_number, device_id, preset_pos_code)
        packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, min_max_positions[1])
        print(f"ID:{device_id} GoalPos:{min_max_positions[index]}  PresPos:{dxl_present_pos}")
        sleep(1)
        if not (abs(min_max_positions[index] - dxl_present_pos[0]) > DXL_MOVING_STATUS_THRESHOLD):
            break
else:
    packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, min_max_positions[0])
    sleep(2.5)

packet_hand_2.write1ByteTxRx(port_number, device_id, led_code, data=0)
#packet_hand_2.write1ByteTxRx(port_number, device_id, torq_code, data=torq_disable)
#port_number.closePort()

lidar = RPLidar("COM3")
info = lidar.get_info()
lidar.connect()
health = lidar.get_health()
print(health)
all_scans = {}
movement = 15

for i, scan in enumerate(lidar.iter_scans()):
    current = packet_hand_2.read4ByteTxRx(port_number, device_id, preset_pos_code)
    if current[0] >= min_max_positions[1]:
        break
    all_scans[current[0]] = scan
    packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, current[0] + movement)
    print(i)
    #print('%d: Got %d measurments' % (i, len(scan)))

print(all_scans)
lidar.stop()
lidar.disconnect()
packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, min_max_positions[0])
sleep(1.75)
packet_hand_2.write1ByteTxRx(port_number, device_id, torq_code, data=torq_disable)
port_number.closePort()


