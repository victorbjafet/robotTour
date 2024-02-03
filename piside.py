import serial
import multiprocessing
import time
from simple_pid import PID


leftEncoder = multiprocessing.Value('i', 0) #makes the values able to be shared between the processes
rightEncoder = multiprocessing.Value('i', 0) #type int value 0


def unoStream(leftEncoder, rightEncoder):
    uno = serial.Serial("/dev/ttyACM0",115200,timeout=None) #COM4 for windows

    time.sleep(1) # wait for the arduino to stop sending the initial garbage data
    while True:
        if uno.isOpen() == False:
            raise Exception("Uno disconnected") # makes sure that the serial is connected
        


        recieved = str(uno.readline())[2:-5]
        try: #sometimes the arduino sends a blank line, so this is to catch that
            value = int(recieved[1:])
            if recieved[0] == "l":
                leftEncoder.value = value
            elif recieved[0] == "r":
                rightEncoder.value = value
        except:
            print("broke")

        print("left:", leftEncoder.value, "right:", rightEncoder.value)




def motorWrite(lMotor, rMotor):
    if lMotor >= 0:
        lWrite = str(lMotor).rjust(5, "0")
    else:
        lWrite = "-" + str(lMotor)[1:].rjust(4, "0")

    if rMotor >= 0:
        rWrite = str(rMotor).rjust(5, "0")
    else:
        rWrite = "-" + str(rMotor)[1:].rjust(4, "0")

    mbot.write(bytearray(lWrite + " " + rWrite, "ascii"))







# 20.2 cm is the circumference of the wheels
#180 ticks is one rotation

left_kP = 1
left_kI = 0
left_kD = 1

left_PID = PID(left_kP, left_kI, left_kD, setpoint=0, output_limits=(-1023, 1023))



right_kP = 0.2
right_kI = 0
right_kD = 0

right_PID = PID(right_kP, right_kI, right_kD, setpoint=0, output_limits=(-1023, 1023))

def goto(leftSetPoint, rightSetPoint):
    while leftEncoder.value != leftSetPoint and rightEncoder.value != rightSetPoint:
        left_PID.setpoint = leftSetPoint
        right_PID.setpoint = rightSetPoint

        left_PID_out = left_PID(leftEncoder.value)
        right_PID_out = right_PID(rightEncoder.value)

        motorWrite(left_PID_out, right_PID_out)

        print("left write:", left_PID_out, "right write:", right_PID_out)
        # print("left think:", leftEncoder.value, "right think:", rightEncoder.value)

        time.sleep(0.2)







if __name__ == '__main__':

    unoStreamProc = multiprocessing.Process(target = unoStream, args=(leftEncoder,rightEncoder))

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


    goto(-131, 131)

        

