import serial
import multiprocessing
import time
from simple_pid import PID


leftEncoder = multiprocessing.Value('i', 0) #makes the values able to be shared between the processes
rightEncoder = multiprocessing.Value('i', 0) #type int value 0
done = multiprocessing.Value('i', 0) #will terminate the seperate process if the program is done

def unoStream(leftEncoder, rightEncoder):
    try:
        uno = serial.Serial("/dev/ttyACM0",115200,timeout=None) #COM4 for windows

        time.sleep(1) # wait for the arduino to stop sending the initial garbage data
        while not done.value:
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
    except:
        pass


minSpeed = 50

def motorWrite(lMotor, rMotor):

    if lMotor == 0 and rMotor == 0:
        for i in range(30):
            mbot.write(bytearray("00000 00000", "ascii"))
    else:
        try:
            if lMotor >= 0:
                lWrite = str(max(lMotor,minSpeed)).rjust(5, "0")
            else:
                lWrite = "-" + str(max(int(str(lMotor)[1:]),minSpeed)).rjust(4, "0")

            if rMotor >= 0:
                rWrite = str(max(rMotor,minSpeed)).rjust(5, "0")
            else:
                rWrite = "-" + str(max(int(str(rMotor)[1:]),minSpeed)).rjust(4, "0")

            mbot.write(bytearray(lWrite + " " + rWrite, "ascii"))
        except:
            pass







# 20.2 cm is the circumference of the wheels
#180 ticks is one rotation
        

speedLimit = 100



left_kP = 1
left_kI = 0
left_kD = 7

left_PID = PID(left_kP, left_kI, left_kD, setpoint=0, output_limits=(-speedLimit, speedLimit))



right_kP = left_kP
right_kI = left_kI
right_kD = left_kD

right_PID = PID(right_kP, right_kI, right_kD, setpoint=0, output_limits=(-speedLimit, speedLimit))

def goto(leftSetPoint, rightSetPoint):
    while leftEncoder.value != leftSetPoint and rightEncoder.value != rightSetPoint:
        left_PID.setpoint = leftSetPoint
        right_PID.setpoint = rightSetPoint

        left_PID_out = int(left_PID(leftEncoder.value))
        right_PID_out = int(right_PID(rightEncoder.value))

        motorWrite(-left_PID_out, right_PID_out) #left is flipped for some reason

        print("left write:", left_PID_out, "right write:", right_PID_out)
        # print("left think:", leftEncoder.value, "right think:", rightEncoder.value)

        time.sleep(0.1)

    motorWrite(0, 0)
    done.value = 1
    print("done")
    time.sleep(2)
    motorWrite(0, 0)
    print("ok actually done")
        
        









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

        

