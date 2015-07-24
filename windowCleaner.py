import serial
import time
import datetime
import math
from itg3200 import SensorITG3200
from adxl345 import SensorADXL345
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN)
GPIO.setup(22,GPIO.IN)
GPIO.setup(23,GPIO.IN)
GPIO.setup(27,GPIO.IN)
GPIO.setup(24,GPIO.IN)

GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setwarnings(False)
State = "pause"
state = "stop"
state_1="forward"
prev_state = "forward"

turn_dir = "left"
turnNum = 0

prev_input1 = 0
prev_input2 = 0
prev_input3 = 0
prev_input4 = 0
prev_inputS = 0
switch=[0,0,0,0,0]

s1=0
s2=0
s3=0
s4=0
#sf_start_time = datetime.datetime.now()
#sf_time_diff = datetime.datetime.now()
sf_start_time = time.time()
sf_time_diff = time.time()
forward_time=time.time()

prev_gx=0
prev_gy=0
prev_gz=0
offset = 0
scaleW = .512
w = 0;
angle = 0
#last_time = datetime.datetime.now()
#curr_time = datetime.datetime.now()
last_time = time.time()
curr_time = time.time()
print_time = time.time()
time_diff = 0
dt = .2
enable = 0
gz = 0
angle_Initial = 0
flag = 0
short = 0
thre=0.1
sign=0
back = 0
back_flag = 0
fix_flag=0
finish_flag = 0
right_flag=0
left_flag = 0


CONST_SF_TIME_DELAY = 1
CONST_LEFT_90 = 80.0
CONST_RIGHT_90 = -82.0


arduinoSerialPort = serial.Serial('/dev/ttyACM0', 115200)
switch_input = GPIO.input(17)
#sensor = SensorITG3200(1, 0x68) # update with your bus number and address
#sensor.default_init()
sensor1 = SensorADXL345(1, 0x53)
sensor1.default_init()
#time.sleep(2)

def getAngle():
    #time.sleep(0.1)
    
   # gx,gy,gz = sensor.read_data()
    time.sleep(0.1)
    ax, ay, az = sensor1.read_data()
    sensor1.standby()
    
    
   # gz/=14.375
    
    ax/=256.0
    ay/=256.0
    az/=256.0
    #print ax, ay, az
    #print gx, gy, gz
    A=math.sqrt( ax*ax + ay*ay+ az*az)
    while (A == 0):
        #sensor.default_init()
        #sensor1.default_init()
        #gx,gy,gz = sensor.read_data()
        time.sleep(0.1)
        ax, ay, az = sensor1.read_data()
        sensor1.standby()    
        #gz/=14.375
        ax/=256.0
        ay/=256.0
        az/=256.0
        A=math.sqrt( ax*ax + ay*ay+ az*az)
        print A
    Ax=ax/A
    Ay=ay/A
    Az=az/A
    Angle_xy = math.atan2(Ay, Ax)    
  #  if abs(gz)>0.5:
        #Angle_xy = Angle_xy + gz*dt*math.pi/180.0
    if (Angle_xy>math.pi):
        Angle_xy = math.pi
    if (Angle_xy<-math.pi):
        Angle_xy = - math.pi
    #print Angle_xy
    return Angle_xy

def Led(pin):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.5)

# Blink for 5 times when robot is ready

for i in range(0,5):
    Led(5)
GPIO.cleanup()

GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN)
GPIO.setup(22,GPIO.IN)
GPIO.setup(23,GPIO.IN)
GPIO.setup(27,GPIO.IN)
GPIO.setup(24,GPIO.IN)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
#GPIO.cleanup()
while(True):
    
    
    # update time
    curr_time = time.time()
    time_diff=curr_time-last_time

    # Get angle from IMU
    if (time_diff > dt):
        Angle_xy=getAngle()
        last_time = curr_time
        
    # check switch state
    
    switch_input1 = GPIO.input(17) #FL
    switch_input2 = GPIO.input(22) #FR
    switch_input3 = GPIO.input(23) #BR
    switch_input4 = GPIO.input(27) #BL
    switch_inputS = GPIO.input(24) #Start
    #time.sleep(0.1)
    if ((not prev_input1) and switch_input1):
        sw_1 = 1;
    
    elif ((not prev_input1) and (not switch_input1)):
        sw_1 = 0;  
    prev_input1 = switch_input1
    
    if ((not prev_input2) and switch_input2):
        sw_2 = 1;
    #s1=sw_1
    elif((not prev_input2) and (not switch_input2)) :
        sw_2 = 0;  
    prev_input2 = switch_input2

    if ((not prev_input3) and switch_input3):
        sw_3 = 1;
    #s1=sw_1
    elif((not prev_input3) and (not switch_input3)) :
        sw_3 = 0;  
    prev_input3 = switch_input3
        
    if ((not prev_input4) and switch_input4):
        sw_4 = 1;
    #s1=sw_1
    elif((not prev_input4) and (not switch_input4)) :
        sw_4 = 0;  
    prev_input4 = switch_input4
    
    if ((not prev_inputS) and switch_inputS):
        sw_S = 1;
    #s1=sw_1
    else :
        sw_S = 0;  
    prev_inputS = switch_inputS
    
    switch = [sw_1,sw_2,sw_3,sw_4,sw_S]
    ##########################################################
    s1=switch[0]  # FL
    s2=switch[1]  # FR
    s3=switch[2] # R
    s4=switch[3]  # L
    
    sS=switch[4] # s4 is the start/pause button
    
    
        
    if(sS == 1 and State == "pause"):
        State="start"
        state = prev_state
        S=sS           
        sS = 0
        # Blink for 3 times when start
        for i in range(0,3):
            Led(12)
        #GPIO.cleanup()
    if (sS==1 and State == "start"):
        State = "pause"
        prev_state = state
        state = "stop"
        S=sS            
        sS = 0
        
    
        
    
    if (flag == 0 and State == "start"):  
        state = "initial"
    # initial flag, find the upright direction to start
    if (state == "initial"):
        
        if (Angle_xy<=0):
            angle_Initial=-0.5*math.pi
            if(Angle_xy>thre+angle_Initial):
                arduinoSerialPort.write("4") # turn left
            if(Angle_xy<angle_Initial-thre):
                arduinoSerialPort.write("3") # turn right
            if(angle_Initial-thre<Angle_xy<angle_Initial+thre):
                arduinoSerialPort.write("0")
                flag=1
                state="forward"
                turn_dir="left"
                print "Go Left"
                time.sleep(1)
                                
        if (Angle_xy>0):
            angle_Initial=0.5*math.pi
            if(Angle_xy<angle_Initial-thre):
                arduinoSerialPort.write("3") # turn right
            if(angle_Initial+thre<Angle_xy):
                arduinoSerialPort.write("4") # turn left
            if(angle_Initial-thre<Angle_xy<angle_Initial+thre):
                arduinoSerialPort.write("0")
                flag=1
                state="forward"
                turn_dir="right"
                print "Go Right"
                time.sleep(1)
    
    
        
       
    print_diff=curr_time-print_time
    if (print_diff>1):
        #num+=1
        print print_diff,State,state,state_1,Angle_xy,angle_Initial
        #print print_diff,State,state,s1,S,N,num
        print_time = curr_time
    
###############################################
#   Side way
###############################################
    
    if (state == "forward"):
        state_1 =state
        short = 0
        fix_flag = 0
        back_flag = 0 
        finish_flag = 0
        # Once triggering the limit switch, first move backward for a while 
        if (s1 == 1 and s2==1):
            if (finish_flag == 1):
                state = "finish"
            else:
                state = "backward"
        if (s1 == 1 ):
            state = "FL"
        if (s2 == 1 ):
            state = "FR"
            #s1=0 # reset limit switch
        if (s3==1 and angle_Initial>0):
            state = "last_R"
            finish_flag=1
        if (s4==1 and angle_Initial<0):
            state = "last_L"
            finish_flag=1             
            
            
        
            
        # Maintain the direction when robot bias to the left
        if(2.5*thre < Angle_xy-angle_Initial < 0.5 * math.pi):
           state_1="left"
         # Maintain the direction when robot move downward           
        if (Angle_xy-angle_Initial<-2.5*thre):
            state_1="right"
        
       
                
        #  Deviation to right, need to move left
        if (state_1 == "left"):
            arduinoSerialPort.write("4") # forward left
            if (-thre<Angle_xy-angle_Initial<0 ):
                state_1 = "forward"
        # Deviation to left, need to move right
        if (state_1 == "right"):
            arduinoSerialPort.write("3") # forward right
            if (0<Angle_xy-angle_Initial<thre):
                state_1 = "forward"
               
        # Go straight
        if (state_1 == "forward"):
            arduinoSerialPort.write("1")
            
###############################################

    if (state == "short_forward"):
        
        if (s1 or s2):
            finish_flag = 1
        arduinoSerialPort.write("0")
        short = 1
        back_flag = 0    # reset backward flag
        back = 0
        
        sf_time_diff = curr_time - sf_start_time
        
        if(sf_time_diff > CONST_SF_TIME_DELAY):
            #arduinoSerialPort.write("2")
            if(turn_dir == "left"):
                
                state = "turn_left"
                turn_dir = "right"
            else: 
                state = "turn_right"
                turn_dir = "left"
                
#############################################

    if (state == "turn_left"):
        if (s1 or s2):
            if (finish_flag==1):
                state = "finish"
            state = "backward"
            finish_flag = 1
            turn_dir = "left"
        
        if (turnNum==1):
            if (0.5*math.pi>Angle_xy>0):
                arduinoSerialPort.write("0")
                angle_Initial=-angle_Initial
                turnNum = 0
                state = "forward"
            else:
                arduinoSerialPort.write("4")
        elif(turnNum==0):
            if(math.pi-thre<math.fabs(Angle_xy)):
                arduinoSerialPort.write("0")
                turnNum = 1
                sf_start_time = curr_time
                state = "short_forward"
            else:
                arduinoSerialPort.write("4")

##############################################

    if (state == "turn_right"):
        if (s2 or s1):
            if (finish_flag == 1):
                state = "finish"
            state = "backward"
            turn_dir = "right"
            finish_flag = 1
            
        
        if (turnNum==1):
            if (-0.5*math.pi<Angle_xy<0):
                arduinoSerialPort.write("0")
                angle_Initial=-angle_Initial
                turnNum = 0
                state = "forward"
            else:
                arduinoSerialPort.write("3")
        elif(turnNum==0):
            if(math.pi-thre<math.fabs(Angle_xy)):
                arduinoSerialPort.write("0")
                turnNum = 1
                sf_start_time = curr_time
                state = "short_forward"
            else:
                arduinoSerialPort.write("3")
                
##############################################

    if (state=="stop"):    
        arduinoSerialPort.write("0")
        # Blink for 5 times when pause
        #for i in range(0,5):
         #   Led(12)
        #GPIO.cleanup()
        

##############################################
    
    if (state == "backward"):
        
        if (back_flag == 0):
            arduinoSerialPort.write("2")
            back_start = curr_time
            back_flag = 1
        back_tdiff = curr_time - back_start 
        if (back_tdiff>3):
            back = 1
           
            if ((not s1) and (not s2)):
                arduinoSerialPort.write("0")
                back_start = curr_time
		if (turn_dir == "left"):
                    state = "turn_left"                        
		elif(turn_dir == "right"):
                    state = "turn_right"
            
    if (state == "FL"):
        arduinoSerialPort.write("9") # stop left wheel, only right wheel move forward
        if (Angle_xy-angle_Initial<-2*thre):
            state = "fix"
        if (s2):
            state = "backward"
    if (state == "FR"):
        arduinoSerialPort.write("8") # stop the right wheel, only left wheel move forward
        if (Angle_xy-angle_Initial>2*thre):
            state = "fix"
        if (s1):
            state = "backward"

    if (state == "fix"):
        if (fix_flag == 0):
            arduinoSerialPort.write("2")
            fix_start = curr_time
            fix_flag = 1
        fix_tdiff = curr_time - fix_start 
        if (fix_tdiff>2):
            if ((not s1) and (not s2)):
                arduinoSerialPort.write("0")
                time.sleep(0.5)
                arduinoSerialPort.write("3")
                time.sleep(1)
                state = "forward"
                fix_start = curr_time

                
    if (state == "last_R"):
        if (fix_flag == 0):
            arduinoSerialPort.write("2")
            fix_start = curr_time
            fix_flag = 1
        fix_tdiff = curr_time - fix_start 
        if (fix_tdiff>2):
            if ( not s3):
                if (Angle_xy-thre<angle_Initial):
                    arduinoSerialPort.write("4") # turn left
                else:
                    state = "forward"
                    
    if (state == "last_L"):
        if (fix_flag == 0):
            arduinoSerialPort.write("2")
            fix_start = curr_time
            fix_flag = 1
        fix_tdiff = curr_time - fix_start 
        if (fix_tdiff>2):
            if ( not s3):
                if (Angle_xy-thre>angle_Initial):
                    arduinoSerialPort.write("3") # turn right
                else:
                    state = "forward"                
                
                
    if (state == "finish"):
        arduinoSerialPort.write("0")
        finish_flag=0
        for i in range (0,8):
            Led(6)
            #time.sleep(0.2)
            Led(5)
        GPIO.cleanup()
    

    
        
