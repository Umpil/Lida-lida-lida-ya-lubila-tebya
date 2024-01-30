from dynamixel_sdk import port_handler
from dynamixel_sdk import protocol2_packet_handler
from rplidar import RPLidar
import os
import csv
from time import sleep
import time

# Надо написать функцию, для остановки программы, на всякий случай, по нажатию клавиши
# ESC_VALUE = 0x1b

# Переменные для сервопривода(берутся из проги серв)
device_name = "COM7"
baudrate = 57600
device_id = 1

torq_code = 64  # Вкл-выкл

goal_pos_code = 116  # Позиция, в которую должен придти сервопривод

led_code = 65  # Код для вкл-выкл лампочки

preset_pos_code = 132  # Код нынешней позиции

DXL_MOVING_STATUS_THRESHOLD = 10  # Переменная точности передвижения для проверки работы

torq_enable = 1
torq_disable = 0

minimum_position_val = 1023  # minimum = 180° maximum=0°, относительно регулярной окружности для человека
maximum_position_val = 3073
min_max_positions = [minimum_position_val, maximum_position_val]

port_number = port_handler.PortHandler(device_name)  # Запись порта, с которым будем работать

packet_hand_2 = protocol2_packet_handler.Protocol2PacketHandler()  # Необходимые функции, описанные в классе протокола 2

# Проверка порта
if port_number.is_open:
    port_number.closePort()

# Пытаемся открыть порт для передачи данных и установить скорость передачи данных
if port_number.openPort():
    print("Opened")
else:
    print("Something went wrong(")

if port_number.setBaudRate(baudrate):
    print("Set baudrate")
else:
    print("No baudrate(")

# Передаём 1 байт информации, что надо по порту обратиться к переменной включения и включить серву
packet_hand_2.write1ByteTxRx(port_number, device_id, torq_code, data=torq_enable)
# Грубо говоря, происходит это по схеме:
# packet_hand_2./Функция "write"/(port_handler, id сервы, код переменной сервы, новое значение переменной)
# А чтобы получить данные из:
# var = packet_hand_2./Функция "read"/(port_handler, id сервы, код переменной сервы)

packet_hand_2.write1ByteTxRx(port_number, device_id, led_code, data=1)
sleep(1)

# Переводим сервопривод в начальное положение
index = 1
dxl_present_pos = packet_hand_2.read4ByteTxRx(port_number, device_id, preset_pos_code)
if abs(dxl_present_pos - min_max_positions[0]) >= 5:
    packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, min_max_positions[0])
    sleep(2.5)

lidar = RPLidar("COM3")  # Задаём лидар на определённом порте

lidar.connect()  # Подключаемся к нему

health = lidar.get_health()  # Получаем статус лидара(Good - гуд)
print(health)

all_scans = {}  # Задаём пустой словарь

movement = 15  # Задаём смещение сервопривода(<15 зависает на 135°(45°), надо смазать)

# Бесконечный итератор сканов лидара
for i, scan in enumerate(lidar.iter_scans()):
    # Получаем текущую позицию сервопривода
    current = packet_hand_2.read4ByteTxRx(port_number, device_id, preset_pos_code)
    
    # Если выходим за пределы 0° в отрицательную часть, то останавливаем итератор
    if current[0] >= min_max_positions[1]:
        break
        
    # Записываем в словарь, по ключу позиции сервопривода, значение сканов
    # Тут надо сделать перевод единиц сервы в градусы
    all_scans[current[0]] = scan
    
    # Смещаем сервопривод
    packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, current[0] + movement)

print(all_scans)

lidar.stop()  # Прекращаем передачу сканов и отключаемся от лидара
lidar.disconnect()

# Тут написать код для перевода в csv file
field_names = ["angle_serv", "id", "angle", "lenght"]
file_name = f"scan_{time.strftime('%d.%m.%Y %H:%M:%S')}"

with open(file_name, "w", encoding="utf-8") as file:
    writer = csv.writer(file, delimiter=",", lineterminator="\r")
    writer.writerow(field_names)
    for angle_serv, lidar_data in all_scans.items():
        for mini_lidar_data in lidar_data:
            writer.writerow((angle_serv,) + mini_lidar_data)

# Переводим сервопривод в начальное положение
packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, min_max_positions[0])
sleep(1.75)

# Выключаем сервопривод и отключаемся от сервы
packet_hand_2.write1ByteTxRx(port_number, device_id, led_code, data=0)
packet_hand_2.write1ByteTxRx(port_number, device_id, torq_code, data=torq_disable)
port_number.closePort()


