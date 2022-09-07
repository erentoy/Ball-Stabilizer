import cv2
import numpy as np
import time
import serial
import time
import pigpio
import imutils

GPIO = pigpio.pi()
prevTime = 0
maxSwing = 50
xErrPrev = 0
yErrPrev = 0

xErrSum = 0
yErrSum = 0

xSetPoint = 45
ySetPoint = 45

# Kp = 1.09 	# 1.1
# Ki = 0.005	# 0.005
# Kd = 1.45 	# 0.9

Kp = 1.09
Ki = 0.005
Kd = 1.45

def GetPidOutputs(inputX, inputY):
    global prevTime, xErrPrev, yErrPrev, xErrSum, yErrSum
    dt = 0.1
    currTime = time.time()

    xErr = xSetPoint - inputX
    xErrDt = xErr - xErrPrev
    xErrPrev = xErr
    xErrSum += xErr * dt
    outputX = Kp * xErr + Ki * xErrSum + Kd * xErrDt / dt
    
    if (outputX > maxSwing):
        outputX = maxSwing
    elif (outputX < -maxSwing):
        outputX = -maxSwing
        
    yErr = ySetPoint - inputY
    yErrDt = yErr - yErrPrev
    yErrPrev = yErr   
    yErrSum += yErr * dt  
    outputY = Kp * yErr + Ki * yErrSum + Kd * yErrDt / dt    

    if (outputY > maxSwing):
        outputY = maxSwing
    elif (outputY < -maxSwing):
        outputY = -maxSwing

    return outputX, outputY

def angle_to_duty(angle):
    return ((angle/18.0)+2.5)*10000

def dummy_func(x):
    pass

def SerialUSB(x,y):
    ser2 = serial.Serial('/dev/ttyS0', 115200, timeout=0.05)
    
    
    num_x = str(x)
    num_y = str(y)
    
    ser2.write(str.encode(num_x))
    ser2.write(str.encode(' '))
    ser2.write(str.encode(num_y))

    return
    
#green

def x_y_calc(xs,ys,xloc,yloc):
    step_sizex = xs/100
    step_sizey = ys/100
    x = (xloc-(xs/2))/step_sizex
    y = (yloc-(ys/2))/step_sizey

    return (np.floor(x) + 46, np.floor(y) + 46)


ser = serial.Serial(
    port='/dev/rfcomm0',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)

ser.isOpen()


cap = cv2.VideoCapture(0)


                                                                         
initialX = 70
initialY = 80

GPIO.set_mode(12, pigpio.OUTPUT)
GPIO.set_mode(13, pigpio.OUTPUT)

GPIO.set_PWM_frequency(12,50)
GPIO.set_PWM_frequency(13,50)

GPIO.hardware_PWM(12,50,int(angle_to_duty(initialX)))
GPIO.hardware_PWM(13,50,int(angle_to_duty(initialY)))

max_outX = 113
min_outX = 33

max_outY = 120
min_outY = 40


setpointX = 45.0
setpointY = 45.0

topyok_Flag = 0


initial_color_val = 0
while True:
    
    #Görüntüyü alıyoruz
    ret, frame = cap.read()

    # roi = frame[10:455,90:535]
    roi = imutils.resize(frame[20:445,100:525], width = 300)
    hs,ws,_ = roi.shape
    

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                                                           
    if initial_color_val == 0:
        lower_bound = np.array([50, 20, 20])
        upper_bound = np.array([100, 255, 255])

    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, hierachy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours :
        area = cv2.contourArea(cnt)
        print(area)
        #sabit arka planda alan thesholdunu koyduğumuzda şimdilik değişimin sadece topta olduğunu kabul ederek konum değerlerini gönderiyoruz
        if area > 300 and area < 1500:

            
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 3)
            x_konum, y_konum = x_y_calc(ws,hs,x+w/2,y+h/2)
            out_x, out_y = GetPidOutputs(x_konum, y_konum)
            print(out_x, out_y)
            if abs(x_konum - 45) == 1 and abs(y_konum - 45) == 1:
                GPIO.hardware_PWM(12,50,int(angle_to_duty(initialX)))
                GPIO.hardware_PWM(12,50,int(angle_to_duty(initialX)))
            else:
                GPIO.hardware_PWM(12,50,int(angle_to_duty(initialX + out_x)))
                GPIO.hardware_PWM(13,50,int(angle_to_duty(initialY + out_y)))
            print((x_konum,y_konum))
            SerialUSB(int(x_konum), int(y_konum))
            bluetooth_input = str(x_konum) + "-" + str(y_konum)
            user_input = bluetooth_input
    
            user_input = user_input + '\r'
            ser.write(user_input.encode())
            recv = ''
            topyok_Flag = 0
            
            
            
    topyok_Flag += 1
    if topyok_Flag == 10:
		
        user_input = "500-500"
        user_input = user_input + '\r'
        ser.write(user_input.encode())
        recv = ''
        topyok_Flag = 0

				
    
    cv2.imshow('Maske', mask)
    cv2.imshow('Out ROI',roi)
    c = cv2.waitKey(1)
    
    if c == 32:

        initial_color_val = 1
 
        image_bar = cv2.imread('1943044.jpg')
        hsv_bar = cv2.cvtColor(image_bar, cv2.COLOR_BGR2HSV)

        tr = cv2.namedWindow("Tracking")

        cv2.createTrackbar("LH", "Tracking", 0, 255, dummy_func)
        cv2.createTrackbar("LS", "Tracking", 0, 255, dummy_func)
        cv2.createTrackbar("LV", "Tracking", 0, 255, dummy_func)
        cv2.createTrackbar("UH", "Tracking", 255, 255, dummy_func)
        cv2.createTrackbar("US", "Tracking", 255, 255, dummy_func)
        cv2.createTrackbar("UV", "Tracking", 255, 255, dummy_func)
		
        while True:



            l_h = cv2.getTrackbarPos("LH", "Tracking")
            l_s = cv2.getTrackbarPos("LS", "Tracking")
            l_v = cv2.getTrackbarPos("LV", "Tracking")

            u_h = cv2.getTrackbarPos("UH", "Tracking")
            u_s = cv2.getTrackbarPos("US", "Tracking")
            u_v = cv2.getTrackbarPos("UV", "Tracking")

            lower_bound = np.array([l_h, l_s, l_v])
            upper_bound = np.array([u_h, u_s, u_v])

            mask_bar = cv2.inRange(hsv_bar, lower_bound, upper_bound)
            res_bar = cv2.bitwise_and(image_bar, image_bar, mask=mask_bar)
            cv2.imshow('BAR', res_bar)

            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            cv2.imshow('MASK_BAR', mask)
            c = cv2.waitKey(1)

            if c == 13:
                cv2.destroyWindow("MASK_BAR")
                cv2.destroyWindow("BAR")
                cv2.destroyWindow("Tracking")
                break
    
    
    if c == 27:
        break
cv2.destroyAllWindows()


