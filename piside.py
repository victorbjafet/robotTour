import serial
import multiprocessing
import time
from simple_pid import PID


leftEncoder = 0
rightEncoder = 0


def unoStream():
    uno = serial.Serial("/dev/ttyACM0",115200,timeout=None) #COM4 for windows
    global leftEncoder
    global rightEncoder

    time.sleep(1) # wait for the arduino to stop sending the initial garbage data
    while True:
        if uno.isOpen() == False:
            raise Exception("Uno disconnected") # makes sure that the serial is connected
        

        recieved = str(uno.readline())
        try: #sometimes the arduino sends a blank line, so this is to catch that
            leftEncoder = int(recieved[2:-5])
        except:
            pass
            print("broke")

        recieved = str(uno.readline())
        try: #sometimes the arduino sends a blank line, so this is to catch that
            rightEncoder = int(recieved[2:-5])
        except:
            pass
            print("broke")

        print("left:", leftEncoder, "right:", rightEncoder)


# def leftMotor



if __name__ == '__main__':



    unoStreamProc = multiprocessing.Process(target = unoStream)

    unoStreamProc.start()


    mbot = serial.Serial("/dev/ttyUSB0",115200,timeout=None) #COM6 for windows
    while True:
        if mbot.isOpen() == False:
            raise Exception("Mbot disconnected") # makes sure that the serial is connected
        
        while start == False:
            recieved = str(mbot.readline())
            if recieved == "b'start\\r\\n'":
                start = True
                print("Starting")
                

        

