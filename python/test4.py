import requests
import threading

CO_B = 0
CO_A = 0
speed = [0, 0, 0, 0] 
max_speed = 250
def process():
    global CO_X, CO_Y
    while True:
        try:
            speed_x = (CO_A/abs(CO_A)) * (abs(CO_A)/max(abs(CO_B), abs(CO_A)))
        except:
            speed_x = 0
        try:
            speed_y = (CO_B/abs(CO_B)) * (abs(CO_B)/max(abs(CO_B), abs(CO_A)))
        except:
            speed_y = 0
        speed[0] = int(((speed_y+speed_x)/2)*max_speed)
        speed[1] = int(((speed_y-speed_x)/2)*max_speed)
        speed[2] = int(((speed_y+speed_x)/2)*max_speed)
        speed[3] = int(((speed_y-speed_x)/2)*max_speed)
        try:
            url="http://192.168.0.62/?A="+str(speed[0])+"&B="+str(speed[1])+"&C="+str(speed[2])+"&D="+str(speed[3])
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