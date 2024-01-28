from dynamixel_sdk import port_handler
from dynamixel_sdk import protocol2_packet_handler
from rplidar import RPLidar
import os, sys
from time import sleep

#Надо написать функцию, для остановки программы, на всякий случай, по нажатию клавиши
#ESC_VALUE = 0x1b

#Переменные для сервопривода(берутся из проги серв)
device_name = "COM7"
baudrate = 57600
device_id = 1
#Вкл-выкл
torq_code = 64
#Позиция, в которую должен придти сервопривод
goal_pos_code = 116
#Код для вкл-выкл лампочки
led_code = 65
#Код нынешней позиции
preset_pos_code = 132


#Переменная точности передвижения для проверки работы
DXL_MOVING_STATUS_THRESHOLD = 10


torq_enable = 1
torq_disable = 0

#minimum = 180° maximum=0°, относительно регулярной окружности для человека
minimum_position_val = 1023
maximum_position_val = 3073

min_max_positions = [minimum_position_val, maximum_position_val]

#Запись порта, с которым будем работать
port_number = port_handler.PortHandler(device_name)

#Необходимые функции, описанные в классе протокола 2
packet_hand_2 = protocol2_packet_handler.Protocol2PacketHandler()

#Проверка порта
if port_number.is_open:
    port_number.closePort()


#Пытаемся открыть порт для передачи данных и установить скорость передачи данных
if port_number.openPort():
    print("Opened")
else:
    print("Something went wrong(")

if port_number.setBaudRate(baudrate):
    print("Set baudrate")
else:
    print("No baudrate(")

#Передаём 1 байт информации, что надо по порту обратиться к переменной включения и включить серву
packet_hand_2.write1ByteTxRx(port_number, device_id, torq_code, data=torq_enable)
#Грубо говоря, происходит это по схеме:
#packet_hand_2./Функция "write"/(port_handler, id сервы, код переменной сервы, новое значение переменной)
#А чтобы получить данные из:
#var = packet_hand_2./Функция "read"/(port_handler, id сервы, код переменной сервы)

packet_hand_2.write1ByteTxRx(port_number, device_id, led_code, data=1)
#packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, 2048)
sleep(1)

#Надо было для проверки работоспособности и углов
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

#Задаём лидар на определённом порте
lidar = RPLidar("COM3")
#Подключаемся к нему
lidar.connect()
#Получаем статус лидара(Good - гуд)
health = lidar.get_health()
print(health)
#Задаём пустой словарь
all_scans = {}
#Задаём смещение сервопривода(<15 зависает на 135°(45°), надо смазать) 
movement = 15

#Бесконечный итератор сканов лидара 
for i, scan in enumerate(lidar.iter_scans()):
    #Получаем текущую позицию сервопривода
    current = packet_hand_2.read4ByteTxRx(port_number, device_id, preset_pos_code)
    
    #Если выходим за пределы 0° в отрицательную часть, то останавливаем итератор
    if current[0] >= min_max_positions[1]:
        break
        
    #Записываем в словарь, по ключу позиции сервопривода, значение сканов
    #Тут надо сделать перевод единиц сервы в градусы
    all_scans[current[0]] = scan
    
    #Смещаем сервопривод
    packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, current[0] + movement)
    #print(i)

print(all_scans)
#Прекащаем передачу сканов и отключаемся от лидара
lidar.stop()
lidar.disconnect()
#Переводим сервопривод в начальное положение
packet_hand_2.write4ByteTxRx(port_number, device_id, goal_pos_code, min_max_positions[0])
sleep(1.75)
#Выключаем сервопривод и отключаемся от сервы
packet_hand_2.write1ByteTxRx(port_number, device_id, torq_code, data=torq_disable)
port_number.closePort()


