from multiprocessing import Process
import serial

ser = serial.Serial('COM3', 9600, timeout=1)

def f(name):
    while(1):
        text=ser.readline()
        dis = int(text.decode())
    return

if __name__ == '__main__':
    p = Process(target=f)
    p.start()
    p.join()
