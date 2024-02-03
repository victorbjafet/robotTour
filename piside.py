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
        


        recieved = str(uno.readline())[2:-5]
        try: #sometimes the arduino sends a blank line, so this is to catch that
            value = int(recieved[1:])
            if recieved[0] == "l":
                leftEncoder = value
            elif recieved[0] == "r":
                rightEncoder = value
        except:
            print("broke")

        print("left:", leftEncoder, "right:", rightEncoder)




def motorWrite(lMotor, rMotor):
    if lMotor >= 0:
        lWrite = str(lMotor).rjust(5, "0")
    else:
        lWrite = "-" + str(lMotor)[1:].rjust(4, "0")
    print("lWrite:", lWrite)

    if rMotor >= 0:
        rWrite = str(rMotor).rjust(5, "0")
    else:
        rWrite = "-" + str(rMotor)[1:].rjust(4, "0")
    print("rWrite:", rWrite)

    mbot.write(bytearray(lWrite + " " + rWrite, "ascii"))



if __name__ == '__main__':

    unoStreamProc = multiprocessing.Process(target = unoStream)

    unoStreamProc.start()


    mbot = serial.Serial("/dev/ttyUSB0",115200,timeout=None) #COM6 for windows

    

    start = False

    while start == False:
        recieved = str(mbot.readline())
        if recieved == "b'start\\r\\n'":
            start = True
            print("Starting")

    time.sleep(2) # let me take my hand off the robot

    print("started")

    # 20.2 cm is the circumference of the wheels


    left_kP = 1
    left_kI = 0
    left_kD = 0
    
    left_PID = PID(left_kP, left_kI, left_kD, setpoint=0, output_limits=(-1023, 1023))



    right_kP = 1
    right_kI = 0
    right_kD = 0
    
    right_PID = PID(right_kP, right_kI, right_kD, setpoint=0, output_limits=(-1023, 1023))




    while True:
        print("pid")
        left_PID.setpoint = -131
        right_PID.setpoint = 131

        left_PID_out = left_PID(leftEncoder)
        right_PID_out = right_PID(rightEncoder)

        motorWrite(left_PID_out, right_PID_out)
    

        

