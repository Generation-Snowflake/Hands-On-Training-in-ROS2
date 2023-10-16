import requests
import threading

esp_ip = "192.168.0.62"
CO_B = 0
CO_A = 0
motor_speed = [0, 0, 0, 0] 
max_speed = 50
_r = 21.053 #wheel diameter = 9.5 cm
#fl(front left) = M3 /fr(front righr) = M2 / rl(rear left) = M4 / rr(rear right) = M1
def cor2speed(x,y):
    try:
        speed_x = (CO_A/abs(CO_A)) * (abs(CO_A)/max(abs(CO_B), abs(CO_A))) * max_speed
    except:
        speed_x = 0
    try:
        speed_y = (CO_B/abs(CO_B)) * (abs(CO_B)/max(abs(CO_B), abs(CO_A))) * max_speed
    except:
        speed_y = 0
    return speed_x, speed_y
def process():
    global CO_A, CO_B
    while True:
        speed_x, speed_y = cor2speed(CO_A, CO_B)
        motor_speed[0] = _r * (speed_x - speed_y) #Wrr
        motor_speed[1] = _r * (speed_x + speed_y) #Wfr
        motor_speed[2] = _r * (speed_x - speed_y) #Wfl
        motor_speed[3] = _r * (speed_x + speed_y)#Wrl
        #A=M1, B=,M2, C=M3, D=M4
        try:
            url="http://" + esp_ip + "/?A="+str(motor_speed[0])+"&B="+str(motor_speed[1])+"&C="+str(motor_speed[2])+"&D="+str(motor_speed[3])
            #print(url)
            requests.get(url)
        except Exception as e:
            pass

x = threading.Thread(target = process)
x.start()

while True:
    CO_A = int(input('Input X: '))
    CO_B = int(input('Input Y: '))
   # process()