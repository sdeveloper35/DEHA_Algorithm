# import the necessary packages
# UPT hatası aldık -Global belirlenmemişti
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import math
import RPi.GPIO as GPIO


acik = 45
kapali = 100

def get_pwm(angle):
    return (angle / 18) + 2.5


def servo_go(servo, angle):
    servo.ChangeDutyCycle(get_pwm(angle))


GPIO.setwarnings(False)

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50)  # GPIO 17 pini 50Hz ile pwm olarak ayarlandı
p.start(2.5)  # Initialization 20ms

ReturnHome = False

# servo_go(p , 0)

# p.stop()
# GPIO.cleanup()

Land_Target = False
target = False
land_sensivity = 50  # pixel
frame_counter = 0
# Dronekit

# --------- PID ------
Kp = 0.0044
Ki = 0
Kd = 0.022
LastErrorX = 0
LastErrorY = 0
integralX = 0
integralY = 0
derivativeX = 0
derivativeY = 0
PID_Velocity_X = 1
PID_Velocity_Y = 0

max_Vel = 1.5
#-------------------------------------------------------------------
Velocity_x = 1  # X ekseni hızı
Velocity_y = 0  # Y ekseni hızı
Velocity_z = 0  # Z ekseni hızı

# Hızların değişimini kontrol etmek için eksen hızlarını tutan değişkenler
Velx_d = 0
Vely_d = Velocity_y
Velz_d = Velocity_z
Show_Velocities_onScreen = True

alt_sensivity = 0.3
alt_speed = 0.3
TargetCompleted = False
ortalandi = False
Target_Location = (0.00, 0.00, 0)


################
# -----PARAMETERS
################
# ---- Vehicle
# connection_address = 'tcp:192.168.4.1:23'
# connection_address = 'tcp:127.0.0.1:5763'
connection_address = '/dev/ttyACM0'
baud_rate = 115200
take_off_altitude = 5  # in meter
ground_speed = 5  # m/s
air_speed = 5  # m/s
land_speed = 60  # cm/s
rtl_altitude = 5  # in meter

# ---- Waypoints
waypoint_havuz = LocationGlobalRelative(38.3714838, 27.2006498, 5)  # longitude, lattitude, altitude
waypoint_havuz_1m = LocationGlobalRelative(38.3714838, 27.2006498, 1.5)

waypoint_target = LocationGlobalRelative(38.3714291, 27.2006968, 5)  # longitude, lattitude, altitude
waypoint_target_2m = LocationGlobalRelative(38.3714291, 27.2006968, 3)

waypoint_land = LocationGlobalRelative(38.3714344, 27.2006552, 5)


# ---------Functions
# ---Print Parameters of Connected Vehicle
def print_vehicle_parameters():
    # Get all vehicle attributes (state)
    print("\nGet all vehicle attribute values:")
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Attitude: %s" % vehicle.attitude)
    print(" Velocity: %s" % vehicle.velocity)
    print(" GPS: %s" % vehicle.gps_0)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Rangefinder: %s" % vehicle.rangefinder)
    print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print(" Heading: %s" % vehicle.heading)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Groundspeed: %s" % vehicle.groundspeed)
    print(" Airspeed: %s" % vehicle.airspeed)
    print(" Mode: %s" % vehicle.mode.name)
    print(" Armed: %s" % vehicle.armed)

def Check_alt(TargetAltitude):
    if vehicle.location.global_relative_frame.alt < TargetAltitude:
        print("You are Ascending...")

        #        İf target altitude is higher than our altitude

        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= TargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    else:
        print("You are Descending...")

        #        İf target altitude is lower than our altitude

        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt <= TargetAltitude * 1.05:
                print("Reached target altitude")
                break
            time.sleep(1)


# ----Arm and Take Off
# TODO: take off and landing speed will be adjusted via a parameter
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


# ---- Get Distance in Meters
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def yukseklik_ayarla(vehicle, yukseklik, hiz, Vx, Vy):
    if not Vx:
        Vx = 0
    if not Vy:
        Vy = 0
    v_altitude = vehicle.location.global_relative_frame.alt

    if yukseklik > v_altitude:
        print("Yukseliniyor...")
        while True:
            set_velocity_body(vehicle, Vx, Vy, -hiz)
            v_alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m" % v_alt)
            if v_alt >= yukseklik - alt_sensivity:
                break
    else:
        print("Alcaliniyor...")
        while True:
            set_velocity_body(vehicle, Vx, Vy, hiz)
            v_alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m" % v_alt)
            if v_alt <= yukseklik + alt_sensivity:
                break

    print("Target altitude reached")
    set_velocity_body(vehicle, 0, 0, 0)

# ---- Distance Between The Vehicle and Given Waypoint
def waypoint_distance(waypoint):
    distance = get_distance_metres(vehicle.location.global_frame, waypoint)
    return distance


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    print("dlat, dlon", dLat, dLon)
    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return (newlat, newlon)

#---------------------------Opencv Variables------------------------------
contour_area = 0
cX = 0
cY = 0

upt = False
ptin_contour = False
circle_color = (0, 255, 0)
contour_color = (0, 255, 0)
pool_font_color = (255, 255, 0)
Use_Circle_Check = False
Aim_Length = 40
total_Mean_Check = False
hsv_Mean_limit = 60
showCircleArea = True
ShowMessage = True  # içinde olup olmadığı (mesajın bir kere yazması için anahtar) (logging)
ShowMessageTarget = True  # daireyi görüp görmediği (mesajın bir kere yazması için anahtar) (logging)
firstMessage = True  # ilk frame alındı mesajını 1 kere yazdırmak için değişken (ilk kare gelince konsola print atar ve false olur böylelikle birdaha print yazmaz)



#------------------------------- Opencv Functions-------------------------

def findRedContours(image,contours,img_Center_X,img_Center_Y):
    global target
    global contour_area  # alanı global değişken olarak tanımladım
    global cX
    global cY
    global ptin_contour

    # En büyük contouru seçiyoruz
    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)  # maximum alana sahip contour

        contour_area = int(cv2.contourArea(c))
        # if showCircleArea:
        #             print(contour_area)

        if contour_area >= 2000 and contour_area <= 180000:
            target = True

            Contour_Check = cv2.pointPolygonTest(c, (img_Center_X, img_Center_Y), False)

            # print(Contour_Check)
            if Contour_Check >= 0:
                ptin_contour = True
                contour_color = (0, 255, 0)
                # pool_font_color = (255 , 255 , 0)      #Havuz yazısının rengi tek renk olsun diye bu satırı çıkardım
            else:
                ptin_contour = False
                contour_color = (255, 0, 0)
                # pool_font_color = (0 , 0 , 255)      #Havuz yazısının rengi tek renk olsun diye bu satırı çıkardım

            # Calculate Moments for each contour
            M = cv2.moments(c)

            if M["m00"] != 0:
                # Calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                cv2.drawContours(image, [c], 0, contour_color, 3)
                cv2.arrowedLine(image, (320, 240), (cX, cY), (255, 255, 0), 2)
                cv2.putText(image, 'Havuz', (cX + 10, cY + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2,
                            cv2.LINE_AA)
        else:
            target = False
            ptin_contour = False

def Mark_GPS_of_Target(camera,vehicle,waypoint):
    global Use_Circle_Check
    global Target_Location
    global Kp
    global Ki
    global Kd
    global LastErrorX
    global LastErrorY
    global integralX
    global integralY
    global derivativeX
    global derivativeY
    global PID_Velocity_X
    global PID_Velocity_Y
    global ShowMessage
    global ShowMessageTarget
    global land_sensivity
    global target
    global ptin_contour
    global Velocity_x
    global Velocity_y
    global Velocity_z
    global Velx_d
    global Vely_d
    global Velz_d
    global alt_speed
    global TargetCompleted
    global frame_counter
    global firstMessage

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text

        image = frame.array

        # ret, image = cap.read()

        if firstMessage:
            print("ilk frame alindi...")
            firstMessage = False
        frame_counter = frame_counter + 1
        if frame_counter >= 300:
            print("Frame alındı (300 frame)")
            frame_counter = 0

        img_Center_X = int(320)
        img_Center_Y = int(240)

        # HSV'ye dönüştür
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Kırmızı daire (Normal)
        lower_red = np.array([150, 50, 50])
        upper_red = np.array([200, 255, 255])

        # for test - (karton için)
        # lower_red = np.array([0, 101, 0])
        # upper_red = np.array([179, 255, 255])

        # Thresholding
        red_hue_range = cv2.inRange(hsv, lower_red, upper_red)

        # Threshold ile ayrılan resmi tekrar maskele
        res = cv2.bitwise_and(image, image, mask=red_hue_range)

        # 3x3 blurla (Kullanılmıyor)
        gray_blurred1 = cv2.blur(red_hue_range, (3, 3))

        # kırmızı ayıkladığın yuvarlağı gray yap
        gray_blurred2 = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        # 3x3 blue uygula (üstteki gereksiz)
        gray_blurred = cv2.blur(gray_blurred2, (10, 10))

        # kenarları bul - KULLANMADIN
        # canny_edge = cv2.Canny(red_hue_range,50,240)
        # --------------------------------------------------

        # detected_circles = cv2.HoughCircles(image=gray_blurred,method=cv2.HOUGH_GRADIENT,dp=1,minDist=600,param1=50,param2=30,minRadius=50,maxRadius=150)

        contours, hierarchy = cv2.findContours(red_hue_range, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # resmin hsv değerlerinin ortalamasını alma
        hsv2 = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
        # Disassemble for each channel
        h, s, v = cv2.split(hsv2)
        mean1 = h.mean()
        mean2 = s.mean()
        mean3 = v.mean()
        # ---print('H:% .1f S:% .1f V:% .1f' % (mean1, mean2, mean3))
        # ek (toplam ortalama)
        total_mean = (mean1 + mean2 + mean3) / 3

        if total_mean <= hsv_Mean_limit:
            total_Mean_Check = True
        else:
            total_Mean_Check = False

        if Use_Circle_Check:
            if total_mean <= hsv_Mean_limit:
                total_Mean_Check = True
                # draw circles that are detected--------------------------------------
                if detected_circles is not None:
                    target = True
                    upt = True
                    # convert the circle parameters a, b and r to integers
                    detected_circles = np.uint16(np.around(detected_circles))

                    for pt in detected_circles[0, :]:
                        a, b, r = pt[0], pt[1], pt[2]

                        # Draw the circumference of the circle
                        cv2.circle(image, (a, b), r, circle_color, 3)

                        # Draw a small circle to show th ecenter
                        cv2.circle(image, (a, b), 1, (0, 0, 255), 3)
                        # Belirlenen daireleri çizme buraya kadar---------------------------- (Altta contour muhabbeti var)
                        cv2.putText(image, 'Havuz', (a + 10, b + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2,
                                    cv2.LINE_AA)
                else:
                    target = False
                    upt = False
                    findRedContours(image,contours,img_Center_X,img_Center_Y)

            else:
                findRedContours(image,contours,img_Center_X,img_Center_Y)
                total_Mean_Check = False


        else:
            findRedContours(image,contours,img_Center_X,img_Center_Y)
            v_alt = vehicle.location.global_relative_frame.alt
            if target:
                # if ShowMessageTarget:
                print("Daire tespit EDILDI")
                # ShowMessageTarget = False
                if ptin_contour == True:
                    if ShowMessage:
                        print("Daire'nin ICINDE")
                        ShowMessage = False
                        Target_Location = vehicle.location.global_relative_frame
                        print("Location Found Global Location : %s" %vehicle.location.global_relative_frame)
                        TargetCompleted = True
                        break
        rawCapture.truncate(0)
        if waypoint_distance(waypoint) <= 1:
            print("Target Reached")
            break

def FindTarget_WaterDropPool(camera,vehicle,waypoint,waypointBool):
    global Kp
    global Ki
    global Kd
    global LastErrorX
    global LastErrorY
    global integralX
    global integralY
    global derivativeX
    global derivativeY
    global PID_Velocity_X
    global PID_Velocity_Y
    global ShowMessage
    global ShowMessageTarget
    global land_sensivity
    global target
    global ptin_contour
    global Velocity_x
    global Velocity_y
    global Velocity_z
    global Velx_d
    global Vely_d
    global Velz_d
    global alt_speed
    global TargetCompleted
    global ortalandi
    global frame_counter
    global firstMessage
    global Use_Circle_Check
    global Land_Target
    global upt
    PID_Velocity_X = 0
    PID_Velocity_Y = 0
    Velocity_z = 0
    Velx_d = PID_Velocity_X
    Vely_d = PID_Velocity_Y
    Velz_d = Velocity_z

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text

        image = frame.array

        # ret, image = cap.read()

        if firstMessage:
            print("ilk frame alindi...")
            firstMessage = False
        frame_counter = frame_counter + 1
        if frame_counter >= 300:
            print("Frame alındı (300 frame)")
            frame_counter = 0

        img_Center_X = int(320)
        img_Center_Y = int(240)

        # HSV'ye dönüştür
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Kırmızı daire (Normal)
        lower_red = np.array([150, 50, 50])
        upper_red = np.array([200, 255, 255])

        # for test - (karton için)
        # lower_red = np.array([0, 101, 0])
        # upper_red = np.array([179, 255, 255])

        # Thresholding
        red_hue_range = cv2.inRange(hsv, lower_red, upper_red)

        # Threshold ile ayrılan resmi tekrar maskele
        res = cv2.bitwise_and(image, image, mask=red_hue_range)

        # 3x3 blurla (Kullanılmıyor)
        gray_blurred1 = cv2.blur(red_hue_range, (3, 3))

        # kırmızı ayıkladığın yuvarlağı gray yap
        gray_blurred2 = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        # 3x3 blue uygula (üstteki gereksiz)
        gray_blurred = cv2.blur(gray_blurred2, (10, 10))

        # kenarları bul - KULLANMADIN
        # canny_edge = cv2.Canny(red_hue_range,50,240)
        # --------------------------------------------------

        # detected_circles = cv2.HoughCircles(image=gray_blurred,method=cv2.HOUGH_GRADIENT,dp=1,minDist=600,param1=50,param2=30,minRadius=50,maxRadius=150)

        contours, hierarchy = cv2.findContours(red_hue_range, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # resmin hsv değerlerinin ortalamasını alma
        hsv2 = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
        # Disassemble for each channel
        h, s, v = cv2.split(hsv2)
        mean1 = h.mean()
        mean2 = s.mean()
        mean3 = v.mean()
        # ---print('H:% .1f S:% .1f V:% .1f' % (mean1, mean2, mean3))
        # ek (toplam ortalama)
        total_mean = (mean1 + mean2 + mean3) / 3

        if total_mean <= hsv_Mean_limit:
            total_Mean_Check = True
        else:
            total_Mean_Check = False

        if Use_Circle_Check:
            if total_mean <= hsv_Mean_limit:
                total_Mean_Check = True
                # draw circles that are detected--------------------------------------
                if detected_circles is not None:
                    target = True
                    upt = True
                    # convert the circle parameters a, b and r to integers
                    detected_circles = np.uint16(np.around(detected_circles))

                    for pt in detected_circles[0, :]:
                        a, b, r = pt[0], pt[1], pt[2]

                        # Draw the circumference of the circle
                        cv2.circle(image, (a, b), r, circle_color, 3)

                        # Draw a small circle to show th ecenter
                        cv2.circle(image, (a, b), 1, (0, 0, 255), 3)
                        # Belirlenen daireleri çizme buraya kadar---------------------------- (Altta contour muhabbeti var)
                        cv2.putText(image, 'Havuz', (a + 10, b + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2,
                                    cv2.LINE_AA)
                else:
                    target = False
                    upt = False
                    findRedContours(image,contours,img_Center_X,img_Center_Y)

            else:
                findRedContours(image,contours,img_Center_X,img_Center_Y)
                total_Mean_Check = False


        else:
            findRedContours(image,contours,img_Center_X,img_Center_Y)
            v_alt = vehicle.location.global_relative_frame.alt
            if target:
                errorX = cX - 320
                errorY = cY - 240

                if LastErrorX != 0:
                    derivativeX = errorX - LastErrorX
                if LastErrorY != 0:
                    derivativeY = errorY - LastErrorY

                integralX = integralX + errorX
                integralY = integralY + errorY

                LastErrorX = errorX
                LastErrorY = errorY

                power_difference_X = errorX * Kp + integralX * Ki + derivativeX * Kd
                power_difference_Y = errorY * Kp + integralY * Ki + derivativeY * Kd

                PID_Velocity_Y = -power_difference_X
                PID_Velocity_X = power_difference_Y

                if PID_Velocity_X > max_Vel:
                    PID_Velocity_X = max_Vel
                if PID_Velocity_X < -max_Vel:
                    PID_Velocity_X = (-1 * max_Vel)

                if PID_Velocity_Y > max_Vel:
                    PID_Velocity_Y = max_Vel
                if PID_Velocity_Y < -max_Vel:
                    PID_Velocity_Y = (-1 * max_Vel)

                # Rounding---------------------
                xR = int(PID_Velocity_X * 10)
                PID_Velocity_X = xR / 10
                yR = int(PID_Velocity_Y * 10)
                PID_Velocity_Y = yR / 10
                # if ShowMessageTarget:
                print("Daire tespit EDILDI")
                # ShowMessageTarget = False
                if ptin_contour == True:
                    if ShowMessage:
                        print("Daire'nin ICINDE")
                        ShowMessage = False
                    # v_alt = vehicle.location.global_relative_frame.alt
                    # Alçalmaya başla
                    if v_alt >= 2:
                        Velocity_z = alt_speed
                    else:
                        Velocity_z = 0

                    """
                    if abs(cX - 320) > land_sensivity:
                        if (cX - 320) > 0:
                            Velocity_y = -0.2
                        else:
                            Velocity_y = 0.2
                    else:
                        Velocity_y = 0

                    if abs(cY - 240) > land_sensivity:
                        if (cY - 240) > 0:
                            Velocity_x = 0.2
                        else:
                            Velocity_x = -0.2
                    else:
                        Velocity_x = 0
                    """
                # Daire'nin dışında ise
                else:
                    if not ShowMessage:
                        print("Daire'nin DISINDA")
                        ShowMessage = True

                    Velocity_z = 0
                    """
                    if abs(cX - 320) > land_sensivity:
                        if (cX - 320) > 0:
                            Velocity_y = -1
                        else:
                            Velocity_y = 1
                    else:
                        Velocity_y = 0

                    if abs(cY - 240) > land_sensivity:
                        if (cY - 240) > 0:
                            Velocity_x = 1
                        else:
                            Velocity_x = -1
                    else:
                        Velocity_x = 0
                    # v_alt = vehicle.location.global_relative_frame.alt
                    # if v_alt <= 2:
                    #   Velocity_z = 0
                """

                if v_alt <= 2:
                    Velocity_z = 0
                    # print("Arac 2m'ye indi zorunlu LAND atiyor...")
                    # vehicle.mode = VehicleMode("LAND")

                # Araç 2m'ye inmiş ve daireyi ortalamış ise
                if (v_alt - 0.2) <= 2 and abs(cY - 240) <= land_sensivity and abs(cX - 320) <= land_sensivity:
                    if Land_Target:
                        print("Vehicle Landing...")
                        vehicle.mode = VehicleMode("LAND")
                    else:
                        print('Breaking Loop...')
                        ortalandi = True
                        set_velocity_body(vehicle, 0, 0, 0)
                        break

            # Eğer daireyi görmüyorsa
            else:
                # if not ShowMessageTarget:
                print("Daire tespit EDILEMEDI")
                # ShowMessageTarget = True
                # araç 2m'ye inmiş ve daireyi görmüyorsa
                #-----------------------------Yükselme Eklenecek----------------------------------------
                if TargetCompleted:
                    vehicle.mode = VehicleMode("LAND")#Hiç daire görmüyorsa land atsın
                #---------------------------------------------------------------------------------------
                if v_alt <= 1 and not ptin_contour:
                    Velocity_z = 0
                    print("Arac 2m'ye indi zorunlu LAND atiyor...")
                    vehicle.mode = VehicleMode("LAND")
            # Herhangi bir hız değeri değişti ise
            if PID_Velocity_X != Velx_d or PID_Velocity_Y != Vely_d or Velocity_z != Velz_d:
                print("X hizi : %f - Y hizi : %f - Z hizi %f" % (PID_Velocity_X, PID_Velocity_Y, Velocity_z))
                print("cX : %d - cY : %d  - abs x = %d - abs y = %d" % (cX, cY, abs(cX - 320), abs(cY - 240)))
                Velx_d = PID_Velocity_X
                Vely_d = PID_Velocity_Y
                Velz_d = Velocity_z
                # Eğer herhangi bir hız değişti ise gönder
                set_velocity_body(vehicle, PID_Velocity_X, PID_Velocity_Y, Velocity_z)
        # Dairenin içinde mi?
        if upt:
            distance = (((a - img_Center_X) ** 2) + ((b - img_Center_Y) ** 2)) ** 0.5
            if abs(distance) < r:
                circle_color = (0, 255, 0)
            else:
                circle_color = (255, 0, 0)
                cv2.arrowedLine(image, (img_Center_X, img_Center_Y), (a, b), (255, 255, 0), 2)

        if Show_Velocities_onScreen:
            # hsv ortalamalarını yazdır------------------------------
            image = cv2.putText(image, 'H:% .1f S:% .1f V:% .1f  Tot: % .1f' % (mean1, mean2, mean3, total_mean),
                                (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
        else:
            image = cv2.putText(image, 'Vx:% .1f Vy:% .1f Vz:% .1f' % (PID_Velocity_X, PID_Velocity_Y, Velocity_z),
                                (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 0), 2, cv2.LINE_AA)

        if showCircleArea:  # Tamamlanmadı
            cv2.putText(image, 'Kirmizi Alan : %d' % contour_area, (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Kameranın pist tesğit edip etmediği
        # if total_Mean_Check:
        #   if target:
        #      image = cv2.putText(image, 'Pist Tespit Edildi' , (50, 75),
        #                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # else:
        #    image = cv2.putText(image, 'Pist Tespit Edilemedi', (50, 75),
        #                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Merkez noktası ekle (Drone Aim)
        # cv2.rectangle(image, (img_Center_X - int(Aim_Length / 2), img_Center_Y - int(Aim_Length / 2)),
        #             (img_Center_X + int(Aim_Length / 2), img_Center_Y + int(Aim_Length / 2)), (0, 255, 255), 1)
        # cv2.circle(image , ( img_Center_X , img_Center_Y ) , int(Aim_Length/2) , (50,255,255) , 2)
        cv2.line(image, (img_Center_X, img_Center_Y - int(Aim_Length / 2)),
                 (img_Center_X, img_Center_Y + int(Aim_Length / 2)), (50, 255, 255), 2)
        cv2.line(image, (img_Center_X - int(Aim_Length / 2), img_Center_Y),
                 (img_Center_X + int(Aim_Length / 2), img_Center_Y), (50, 255, 255), 2)

        # cv2.namedWindow('DETECTED CIRCLE',cv2.WINDOW_NORMAL)    #Window oluştur (Bu satır olmadan sadece imshow() ile de yapılabilir)
        # cv2.resizeWindow('DETECTED CIRCLE',1280,920)           #Windowun size'ını değiştir

        # video.write(image)     #Şuan video kaydetmek istemediğim için kullanmıyorum

        # cv2.imshow('DETECTED CIRCLE', image)
        # cv2.imshow('Gray',gray_blurred)
        # cv2.imshow('MASK', res)
        # cv2.imshow('CANNY',canny_edge)
        # cv2.imshow('RedHue',red_hue_range)
        # cv2.setMouseCallback('DETECTED CIRCLE', on_EVENT_LBUTTONDOWN)
        # cv2.waitKey(0)
        """
        key = cv2.waitKey(10)
        if key & 0xFF == ord('q'):
            break
        if key & 0xFF == ord('p'):
            while (True):
                if cv2.waitKey(10) & 0xFF == ord('s'):
                    break
        if key & 0xFF == ord('v'):
            if Show_Velocities_onScreen:
                Show_Velocities_onScreen = False
            else:
                Show_Velocities_onScreen = True
        """
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        if waypointBool:
            if waypoint_distance(waypoint) <= 1:
                print("Target Reached")
                break

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)


print("\nConnecting to vehicle on: " + connection_address + " with baud rate: " + str(baud_rate))
vehicle = connect(connection_address, wait_ready=True, baud=baud_rate)

# Set first arming for rtl
waypoint_home = vehicle.location.global_frame
# Print all vehicle parameters
print_vehicle_parameters()
# TODO: After printing vehicle state, to continue the program ask for user approval and serve 3 option: refresh vehicle state, arm the vehicle, terminate the program
# Set ground speed
#vehicle.groundspeed = ground_speed
#print(" Ground speed is set to " + str(ground_speed) + " m/s")
# Set air speed
#vehicle.airspeed = air_speed
#print("Air speed is set to " + str(air_speed) + " m/s")
# Set rtl altitude
#vehicle.parameters['RTL_ALT'] = rtl_altitude
# Set landing speed
#vehicle.parameters['LAND_SPEED'] = land_speed

print("Land_Target : " + str(Land_Target))
user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")
while not (user_approval == "arm" or user_approval == "cancel"):
    print("Invalid input, please type again...")
    user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")
if user_approval == "arm":
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    arm_and_takeoff(take_off_altitude)
    vehicle.simple_goto(waypoint_havuz)
    print('Vehicle is going to waypoint_havuz')
    while waypoint_distance(waypoint_havuz) >= 1:
        print('Distance to waypoint havuz : %s' % waypoint_distance(waypoint_havuz))
        time.sleep(1)

    print("Water mechanism open...")
    servo_go(p, acik)
    vehicle.simple_goto(waypoint_havuz_1m)
    print("Descending to 1 meter...")
    print("10 seconds just started")
    Check_alt(1.5)
    time.sleep(5)
    print("Water mechanism closed...")
    servo_go(p, kapali)
    time.sleep(2)
    vehicle.simple_goto(waypoint_havuz)
    print("Vehicle Ascending to 5 meter again...")
    Check_alt(5)
    # time.sleep(7)

    vehicle.simple_goto(waypoint_target)
    print('Vehicle is going to waypoint target')
    while waypoint_distance(waypoint_target) >= 1:
        print('Distance to waypoint target : %s' % waypoint_distance(waypoint_target))
        time.sleep(1)
    FindTarget_WaterDropPool(camera, vehicle, waypoint_target, False)
    if ortalandi:
        print("Water mechanism opening...")
        print("10 sec. to dropping water...")
        servo_go(p, acik)
        time.sleep(10)
        print("Water mechanism closing...")
        servo_go(p, kapali)
        time.sleep(2)
        print("Ascending for 5 meter...")
        yukseklik_ayarla(vehicle, 5, 1.5, 0, 0)
        Check_alt(5)


    print("Inis konumuna gidiliyor...")
    if ReturnHome:
        vehicle.simple_goto(waypoint_home)
        while waypoint_distance(waypoint_home) >= 1:
            print('Distance to waypoint home : %s' % waypoint_distance(waypoint_home))
        time.sleep(1)

    else:
        vehicle.simple_goto(waypoint_land)
        # print ('Vehicle is going to waypoint land')
        while waypoint_distance(waypoint_land) >= 1:
            print('Distance to waypoint land : %s' % waypoint_distance(waypoint_land))
            time.sleep(1)

    # Update home location to first take off point
    vehicle.home_location = waypoint_home
    print('Vehicle landing...')
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed == True:
        print(" Altitude: %s"  %vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    print('Landing Completed')
    print("Close vehicle object")
    vehicle.close()
else:
    print("Mission denied...")
    print("Close vehicle object")
    vehicle.close()

p.stop()
GPIO.cleanup()
print("PWM pini kapatildi")


