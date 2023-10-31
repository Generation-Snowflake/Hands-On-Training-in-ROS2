import requests
import threading
import numpy as np
import math
esp_ip = "192.168.0.62"
CO_B = 0
CO_A = 0
motor_speed = [0, 0, 0, 0] 
robot_speed = 50.0 #rpm = 50 -> v = 0.2487 m/s
rotation_speed = 1.3785  #rad/s -> wz
wheel_radius = 0.052
lx = 0.105 #21cm / 2
ly = 0.0925 #18.5cm / 2
la = lx + ly
_r = 1 /wheel_radius #wheel diameter = 9.5 cm
travel_time = 0
#fl(front left) = M3 /fr(front righr) = M2 / rl(rear left) = M4 / rr(rear right) = M1
def cor2speed(x,y):
    try:
        speed_x = (CO_A/abs(CO_A)) * (abs(CO_A)/max(abs(CO_B), abs(CO_A))) * angular2linear(robot_speed)
    except:
        speed_x = 0
    try:
        speed_y = (CO_B/abs(CO_B)) * (abs(CO_B)/max(abs(CO_B), abs(CO_A))) * angular2linear(robot_speed)
    except:
        speed_y = 0
    return speed_x, speed_y
def angular2linear(RPM):
    RPS = RPM / 60.0 #rpm
    w = 2 * np.pi * RPS #radians/sec
    v = w * wheel_radius
    return v
def w2rpm(w):
    RPS = w /(2.0*np.pi)
    RPM = RPS*60.0
    return math.floor(RPM)
def calTime(mx,my,vx,vy):
    meter = math.sqrt((mx**2)+(my**2))
    print(meter)
    time = meter / (math.sqrt((vx**2) + (vy**2))) #sec
    return math.floor(1000 * time) #ms

def ceta2rad(ceta):
    rad = ceta*(np.pi/180)
    return rad
def calRotateTime(ceta):
    radian = ceta2rad(ceta)
    print(radian)
    time = abs(radian) / rotation_speed
    return math.floor(1000 * time) #ms
def send_drive_command(x,y):
        speed_x, speed_y = cor2speed(x, y)
        #print(speed_y)
        motor_speed[0] = _r * (speed_x - speed_y) #Wrr
        motor_speed[1] = _r * (speed_x + speed_y) #Wfr
        motor_speed[2] = _r * (speed_x - speed_y) #Wfl
        motor_speed[3] = _r * (speed_x + speed_y) #Wrl
        travel_time = calTime(abs(x),abs(y),speed_x,speed_y)
        #print(travel_time)
        #A=M1, B=,M2, C=M3, D=M4
        try:
            url="http://" + esp_ip + "/?A="+str(w2rpm(motor_speed[0]))+"&B="+str(w2rpm(motor_speed[1]))+"&C="+str(w2rpm(motor_speed[2]))+"&D="+str(w2rpm(motor_speed[3]))+"&T="+str(travel_time)
            #url="http://" + esp_ip + "/?A="+str(motor_speed[0])+"&B="+str(motor_speed[1])+"&C="+str(motor_speed[2])+"&D="+str(motor_speed[3])
            #url="http://" + esp_ip + "/?A="+str(100)+"&B="+str(0)+"&C="+str(0)+"&D="+str(0)
            print(url)
            requests.get(url)
        except Exception as e:
            pass

def send_angle_command(ceta):
        motor_speed[0] = _r * (la*rotation_speed) #Wrr
        motor_speed[1] = _r * (la*rotation_speed) #Wfr
        motor_speed[2] = _r * (-(la*rotation_speed)) #Wfl
        motor_speed[3] = _r * (-(la*rotation_speed)) #Wrl
        rotation_time = calRotateTime(ceta)
        #print(ceta)
        try:
            url="http://" + esp_ip + "/?A="+str(w2rpm(motor_speed[0]))+"&B="+str(w2rpm(motor_speed[1]))+"&C="+str(w2rpm(motor_speed[2]))+"&D="+str(w2rpm(motor_speed[3]))+"&T="+str(rotation_time)
            #url="http://" + esp_ip + "/?A="+str(motor_speed[0])+"&B="+str(motor_speed[1])+"&C="+str(motor_speed[2])+"&D="+str(motor_speed[3])
            #url="http://" + esp_ip + "/?A="+str(100)+"&B="+str(0)+"&C="+str(0)+"&D="+str(0)
            print(url)
            requests.get(url)
        except Exception as e:
            pass



#x = threading.Thread(target = process)
#.start()

while True:
    MODE = int(input('Mode? (1.Drive/2.Rotate): '))
    if(MODE == 1):
        CO_A = float(input('Input X: '))
        CO_B = float(input('Input Y: '))
        send_drive_command(CO_A,CO_B)
    elif(MODE == 2):
        ANGLE = float(input('Input Angle: '))
        send_angle_command(ANGLE)