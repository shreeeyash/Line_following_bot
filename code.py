import cv2
import urllib.request
import picamera
import picamera.array
import numpy
import RPi.GPIO as GPIO

ml1 = 22  # 15
ml2 = 5   # 29
mr1 = 16  #36
mr2 = 20  # 38
kpx = 0.5
kpa = 0.25
kdx = 0.4
kda = 0.2
motor_s = 250
corr =0
max_rpm = int(0.01234*((kpx*120) + (kpa*90) + (kdx*150) + (kda*90) + 300))
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12,GPIO.OUT) #32
GPIO.setup(13,GPIO.OUT) #33
#def init():
GPIO.setup(ml2, GPIO.OUT)
GPIO.setup(ml1, GPIO.OUT)
GPIO.setup(mr1, GPIO.OUT)
GPIO.setup(mr2, GPIO.OUT)

def forward():
    GPIO.output(ml1,GPIO.HIGH)
    GPIO.output(ml2,GPIO.LOW)
    GPIO.output(mr1,GPIO.HIGH)
    GPIO.output(mr2,GPIO.LOW)


mr_pwm = GPIO.PWM(13,100)   # 
ml_pwm = GPIO.PWM(12,100)   # 

# Replace the URL with your own IPwebcam shot.jpg IP:port
url='http://192.168.0.105:8080/shot.jpg'
pre_err_a = 0
pre_err_x = 0
mr_pwm.start(100)
ml_pwm.start(100)
while True:

    # Use urllib to get the image and convert into a cv2 usable format
    imgResp=urllib.request.urlopen(url)
    imgNp=numpy.array(bytearray(imgResp.read()),dtype=numpy.uint8)
    frame=cv2.imdecode(imgNp,-1)
    thresh = cv2.inRange(frame, (0, 0, 0), (70, 70, 70))
    kernel = numpy.ones((3, 3), numpy.uint8)
    thresh = cv2.erode(thresh, kernel, 5)
    thresh = cv2.dilate(thresh, kernel, 9)
    ret, contour, hier = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contour) > 0:
        cnt = contour[0]

        # making tilted rectangle to measure angle error
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = numpy.int0(box)
        cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
        if rect[1][0] < rect[1][1]:
            err_a = (rect[2])
            print('ANGLE : ' + str(err_a))
        else:
            err_a = (90 + rect[2])
            print('ANGLE : ' + str(err_a))

        # making simple rectangle to find distance error
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.line(thresh, (x + (w // 2), y), (x + (w // 2), y + h), (255, 0, 0), 2)
        err_x = (x + (w // 2) - 120)
        print('DISTANCE : ' + str(err_x))

        # calculating error and code for PID
        d_err_x = err_x - pre_err_x
        d_err_a = err_a - pre_err_a
        corr = (kpx * err_x) + (kpa * err_a) + (kdx * d_err_x) + (kda * d_err_a)
        pre_err_x = err_x
        pre_err_a = err_a
        lms_p = (motor_s + corr) // max_rpm
        rms_p = (motor_s - corr) // max_rpm
        #forward()
        #print('rms_p : '+str(rms_p))
        #print('lms_p : '+ str(lms_p))
        if all([rms_p<100,lms_p<100]):
            mr_pwm.ChangeDutyCycle(rms_p)
            ml_pwm.ChangeDutyCycle(lms_p)
    cv2.imshow('stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
