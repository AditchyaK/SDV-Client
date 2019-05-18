import time, os, socket

#External thermometer address: 28-031897792ede

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

temp_sensor = '/sys/bus/w1/devices/28-031897792ede/w1_slave'

def temp_raw():
    f = open(temp_sensor, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    
    lines = temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        lines = temp_raw()
        
    temp_output = lines[1].find('t=')
    
    if temp_output != -1:
        temp_string = lines [1].strip()[temp_output+2:]
        temp_c = float(temp_string) / 1000.0
        return round(temp_c, 1)
    
while True:
    print(read_temp())
