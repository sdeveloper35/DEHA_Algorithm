# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import math


target = False

land_sensivity = 50 #pixel

#Dronekit

connection_address = '/dev/ttyACM0' #kontrol et
baud_rate = 115200
take_off_altitude = 5 #in meter
ground_speed = 5  # m/s
air_speed = 5  # m/s
land_speed= 60 # cm/s
rtl_altitude = 5 #in meter

Velocity_x = 1 #X ekseni hızı
Velocity_y = 0 #Y ekseni hızı
Velocity_z = 0 #Z ekseni hızı

# Hızların değişimini kontrol etmek için eksen hızlarını tutan değişkenler
Velx_d = Velocity_x 
Vely_d = Velocity_y
Velz_d = Velocity_z

alt_sensivity = 0.2

# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


# -- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # -- POSITION
        vx, vy, vz,  # -- VELOCITY
        0, 0, 0,  # -- ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def yukseklik_ayarla(vehicle, yukseklik, hiz, Vx, Vy):
    v_altitude = vehicle.location.global_relative_frame.alt
    
    if yukseklik > v_altitude:
        print("Yukseliniyor...")
        set_velocity_body(vehicle, Vx, Vy, -hiz)
        while True:
            v_alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m" % v_alt)
            if v_alt >= yukseklik - alt_sensivity:
                print("Target altitude reached")
                break
    else:
        print("Alcaliniyor...")
        set_velocity_body(vehicle, Vx, Vy, hiz)
        while True:
            v_alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m" % v_alt)
            if v_alt <= yukseklik + alt_sensivity:
                print("Target altitude reached")
                break


contour_area = 0
cX=0
cY=0
def findRedContours():
    global target
    global contour_area #alanı global değişken olarak tanımladım
    global cX
    global cY
    global ptin_contour
    
    # En büyük contouru seçiyoruz
    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)  #maximum alana sahip contour

        contour_area = int(cv2.contourArea(c))
        #if showCircleArea:
#             print(contour_area)

        if contour_area >= 3000 and contour_area <= 180000:
            target = True

            Contour_Check = cv2.pointPolygonTest(c, (img_Center_X , img_Center_Y),False)

            #print(Contour_Check)
            if Contour_Check >= 0:
                ptin_contour = True
                contour_color = (0 , 255 , 0)
                #pool_font_color = (255 , 255 , 0)      #Havuz yazısının rengi tek renk olsun diye bu satırı çıkardım
            else:
                ptin_contour = False
                contour_color  = (255 , 0 , 0)
                #pool_font_color = (0 , 0 , 255)      #Havuz yazısının rengi tek renk olsun diye bu satırı çıkardım

            # Calculate Moments for each contour
            M = cv2.moments(c)

            if M["m00"] != 0:
                # Calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                cv2.drawContours(image, [c], 0, contour_color, 3)
                cv2.arrowedLine(image, (320, 240), (cX, cY), (255, 255, 0), 2)
                cv2.putText(image,'Havuz', (cX+10, cY+10),cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2, cv2.LINE_AA)
        else:
            target = False
            ptin_contour = False
    #else:
        #target = False


upt = False
ptin_contour = False
circle_color = (0 , 255 , 0)
contour_color = (0 , 255 , 0)
pool_font_color = (255 , 255 , 0)
Use_Circle_Check = False
Aim_Length = 40
total_Mean_Check = False
hsv_Mean_limit = 60
showCircleArea = True
ShowMessage = True #içinde olup olmadığı (mesajın bir kere yazması için anahtar) (logging)
ShowMessageTarget = True #daireyi görüp görmediği (mesajın bir kere yazması için anahtar) (logging)




# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

print("Kod basladi...")
if camera:
    print("kamera baglanildi")

#Dronekit
#Set ground speed
#vehicle.groundspeed = ground_speed
#print (" Ground speed is set to " + str(ground_speed) + " m/s")

#Set air speed
#vehicle.airspeed = air_speed
#print ("Air speed is set to " + str(air_speed) + " m/s")

#Set rtl altitude
#vehicle.parameters['RTL_ALT'] = rtl_altitude

#Set landing speed
#vehicle.parameters['LAND_SPEED'] = land_speed


firstMessage = True #ilk frame alındı mesajını 1 kere yazdırmak için değişken (ilk kare gelince konsola print atar ve false olur böylelikle birdaha print yazmaz)
frame_counter = 0
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    #print("Frame alindi...") 
    img_Center_X = int(320)
    img_Center_Y = int(240)

    if firstMessage:
        print("ilk frame alindi...")
        firstMessage = False
    frame_counter = frame_counter+1
    if frame_counter >= 300:
        print("Frame alındı (300 frame)")
        frame_counter = 0
    
    #HSV'ye dönüştür
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower_red = np.array([150, 50, 50])
    upper_red = np.array([200, 255, 255])

    #Thresholding
    red_hue_range = cv2.inRange(hsv, lower_red, upper_red)

    #Threshold ile ayrılan resmi tekrar maskele
    res = cv2.bitwise_and(image, image, mask=red_hue_range)

    #3x3 blurla (Kullanılmıyor)
    gray_blurred1 = cv2.blur(red_hue_range,(3,3))

    #kırmızı ayıkladığın yuvarlağı gray yap
    gray_blurred2 = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

    #3x3 blue uygula (üstteki gereksiz)
    gray_blurred = cv2.blur(gray_blurred2,(10,10))


    #kenarları bul - KULLANMADIN
    #canny_edge = cv2.Canny(red_hue_range,50,240)
    #--------------------------------------------------

    #detected_circles = cv2.HoughCircles(image=gray_blurred,method=cv2.HOUGH_GRADIENT,dp=1,minDist=600,param1=50,param2=30,minRadius=50,maxRadius=150)

    contours, hierarchy = cv2.findContours(red_hue_range, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    # resmin hsv değerlerinin ortalamasını alma
    hsv2 = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
    # Disassemble for each channel
    h, s, v = cv2.split(hsv2)
    mean1 = h.mean()
    mean2 = s.mean()
    mean3 = v.mean()
    #---print('H:% .1f S:% .1f V:% .1f' % (mean1, mean2, mean3))
    # ek (toplam ortalama)
    total_mean = (mean1 + mean2 + mean3) / 3

    if total_mean <= hsv_Mean_limit:
        total_Mean_Check = True
    else:
        total_Mean_Check = False

    if Use_Circle_Check:
        if total_mean <= hsv_Mean_limit:
            total_Mean_Check = True
            #draw circles that are detected--------------------------------------
            if detected_circles is not None:
                target = True
                upt=True
                #convert the circle parameters a, b and r to integers
                detected_circles = np.uint16(np.around(detected_circles))

                for pt in detected_circles[0, :]:
                    a, b, r = pt[0], pt[1], pt[2]

                    #Draw the circumference of the circle
                    cv2.circle(image, (a,b), r, circle_color,3)

                    #Draw a small circle to show th ecenter
                    cv2.circle(image, (a,b), 1, (0,0,255),3)
            #Belirlenen daireleri çizme buraya kadar---------------------------- (Altta contour muhabbeti var)
                    cv2.putText(image,'Havuz', (a+10, b+10),cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2, cv2.LINE_AA)
            else:
                target = False
                upt = False
                findRedContours()

        else:
            findRedContours()
            total_Mean_Check = False

 
    else:
       # if target:
           # print("Kirmizi tespit edilemedi...")
        #else
           # print("Kirmizi tespit edildi...")
        findRedContours()
        #v_alt = vehicle.location.global_relative_frame.alt
        if target:
            if ShowMessageTarget:
                print("Daire tespit EDILDI")
                ShowMessageTarget = False
            if ptin_contour == True:
                if ShowMessage:
                    print("Daire'nin ICINDE")
                    ShowMessage = False
                #v_alt = vehicle.location.global_relative_frame.alt
                #Alçalmaya başla
                #if v_alt >= 2:
                #    Velocity_z = 0.2
                #else:
                 #   Velocity_z = 0
                    
                if abs(cX-320) > land_sensivity:
                    if (cX-320) > 0:
                        Velocity_y = -0.2
                    else:
                        Velocity_y = 0.2
                else:
                    Velocity_y = 0
                
                if abs(cY-240) > land_sensivity:
                    if (cY-240) > 0:
                        Velocity_x = 0.2
                    else:
                        Velocity_x = -0.2
                else:
                    Velocity_x = 0
            
            #Dairenin içinde değilse
            else:
                if not ShowMessage:
                    print("Daire'nin DISINDA")
                    ShowMessage = True
                
                Velocity_z = 0
                
                if abs(cX-320) > land_sensivity:
                    if (cX-320) > 0:
                        Velocity_y = -1
                    else:
                        Velocity_y = 1
                else:
                    Velocity_y = 0
                
                if abs(cY-240) > land_sensivity:
                    if (cY-240) > 0:
                        Velocity_x = 1
                    else:
                        Velocity_x = -1
                else:
                    Velocity_x = 0
                #v_alt = vehicle.location.global_relative_frame.alt
                #if v_alt <= 2:
                 #   Velocity_z = 0
        else:
            if not ShowMessageTarget:
                print("Daire tespit EDILEMEDI")
                ShowMessageTarget = True   
        #if (v_alt - 0.2) <= 2 and abs(cY-240) <= land_sensivity and abs(cX-320) <= land_sensivity:
         #   print('Vehicle landing...')
            #vehicle.mode = VehicleMode("LAND")
        if Velocity_x != Velx_d or Velocity_y != Vely_d or Velocity_z != Velz_d:
            print("X hizi : %f - Y hizi : %f - Z hizi %f" % (Velocity_x,Velocity_y,Velocity_z))
            print("cX : %d - cY : %d  - abs x = %d - abs y = %d" % (cX,cY,abs(cX-320),abs(cY-240)))
            Velx_d = Velocity_x 
            Vely_d = Velocity_y
            Velz_d = Velocity_z
        #set_velocity_body(vehicle, Velocity_x, Velocity_y, Velocity_z)

    #Dairenin içinde mi?
    if upt:
        distance = (((a-img_Center_X)**2)+((b-img_Center_Y)**2))**0.5
        if abs(distance) < r:
            circle_color = (0,255,0)
        else:
            circle_color = (255,0,0)
            cv2.arrowedLine(image,(img_Center_X , img_Center_Y) , (a,b) , (255,255,0) , 2)

    #hsv ortalamalarını yazdır------------------------------
    image = cv2.putText(image,'H:% .1f S:% .1f V:% .1f  Tot: % .1f' % (mean1, mean2, mean3,total_mean),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2,cv2.LINE_AA   )

    if showCircleArea:#Tamamlanmadı
        cv2.putText(image, 'Kirmizi Alan : %d' % contour_area, (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    #Kameranın pist tesğit edip etmediği
    #if total_Mean_Check:
     #   if target:
      #      image = cv2.putText(image, 'Pist Tespit Edildi' , (50, 75),
       #                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
       # else:
        #    image = cv2.putText(image, 'Pist Tespit Edilemedi', (50, 75),
         #                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)


    # Merkez noktası ekle (Drone Aim)
    #cv2.rectangle(image, (img_Center_X - int(Aim_Length / 2), img_Center_Y - int(Aim_Length / 2)),
    #             (img_Center_X + int(Aim_Length / 2), img_Center_Y + int(Aim_Length / 2)), (0, 255, 255), 1)
    # cv2.circle(image , ( img_Center_X , img_Center_Y ) , int(Aim_Length/2) , (50,255,255) , 2)
    cv2.line(image , ( img_Center_X , img_Center_Y - int(Aim_Length/2) ) , ( img_Center_X , img_Center_Y + int(Aim_Length/2) ) , (50,255,255) , 2)
    cv2.line(image , ( img_Center_X - int(Aim_Length/2) , img_Center_Y ) , ( img_Center_X + int(Aim_Length/2) , img_Center_Y ) , (50,255,255) , 2)

    #cv2.namedWindow('DETECTED CIRCLE',cv2.WINDOW_NORMAL)    #Window oluştur (Bu satır olmadan sadece imshow() ile de yapılabilir)
    #cv2.resizeWindow('DETECTED CIRCLE',1280,920)           #Windowun size'ını değiştir

    #video.write(image)     #Şuan video kaydetmek istemediğim için kullanmıyorum

    #cv2.imshow('DETECTED CIRCLE',image)
    #cv2.imshow('Gray',gray_blurred)
    #cv2.imshow('MASK', res)
    #cv2.imshow('CANNY',canny_edge)
    #cv2.imshow('RedHue',red_hue_range)
    #cv2.setMouseCallback('DETECTED CIRCLE', on_EVENT_LBUTTONDOWN)
    #cv2.waitKey(0)
    #key=cv2.waitKey(10)
    #if key & 0xFF == ord('q'):
        #break
    #if key & 0xFF == ord('p'):
        #while(True):
            #if cv2.waitKey(10) & 0xFF == ord('s'):
               # break
    

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

#cv2.destroyAllWindows()





