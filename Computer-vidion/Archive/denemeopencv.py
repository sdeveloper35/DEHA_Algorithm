import cv2
import numpy as np

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        cv2.putText(frame, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,1.0, (0, 0, 0), thickness=1)
        print(x,y)
        cv2.imshow('DETECTED CIRCLE', frame)

        RGB = frame
        hsv = cv2.cvtColor(RGB,cv2.COLOR_BGR2HSV)
        #print('hsv -> ',hsv[x,y])
        s0,s1,s2=cv2.split(hsv)
        #split() h, s ve v değerlerinin olduğu matrixler dönderiyor(her matrixte tüm pixellerin ayrıştırılmış
        #ayrı ayrı h s ve v değerleri var)(Ekrandaki tüm pixellerin)
        print("H:",s0[x][y],"   S:",s1[x][y],"    V:",s2[x][y])

target = False

def findRedContours():
    global target
    global contour_area #alanı global değişken olarak tanımladım
    # En büyük contouru seçiyoruz
    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)  #maximum alana sahip contour

        contour_area = int(cv2.contourArea(c))
        if showCircleArea:
            print(contour_area)

        if contour_area >= 100:
            if contour_area <= 130000:
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
                    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.drawContours(frame, [c], 0, contour_color, 3)
                    cv2.arrowedLine(frame, (320, 240), (cX, cY), (255, 255, 0), 2)
                    cv2.putText(frame,'Havuz', (cX+10, cY+10),cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2, cv2.LINE_AA)
    else:
        target = False

cap = cv2.VideoCapture('drone1.mp4')
upt = False
ptin_contour = False
circle_color = (0 , 255 , 0)
contour_color = (0 , 255 , 0)
pool_font_color = (255 , 255 , 0)
Use_Circle_Check = False
Aim_Length = 40
total_Mean_Check = False
hsv_Mean_limit = 60
showCircleArea = False


frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

size = (frame_width, frame_height)
video = cv2.VideoWriter('DEHA.avi', cv2.VideoWriter_fourcc(*'MJPG'),30, size)

while(True):
    ret,frame = cap.read()

    #İmage Size
    dimentions = frame.shape    #shape[0] --> resmin boyu ---------- shape[1] --> Resmin eni --------- shape[2] ---> resimdeki channel sayısı
    #print(dimentions[0],dimentions[1])
    boy,en = dimentions[0],dimentions[1]
    img_Center_X = int(en/2)
    img_Center_Y = int(boy/2)


    #HSV'ye dönüştür
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    lower_red = np.array([150, 50, 50])
    upper_red = np.array([200, 255, 255])

    #Thresholding
    red_hue_range = cv2.inRange(hsv, lower_red, upper_red)

    #Threshold ile ayrılan resmi tekrar maskele
    res = cv2.bitwise_and(frame, frame, mask=red_hue_range)

    #3x3 blurla (Kullanılmıyor)
    gray_blurred1 = cv2.blur(red_hue_range,(3,3))

    #kırmızı ayıkladığın yuvarlağı gray yap
    gray_blurred2 = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

    #3x3 blue uygula (üstteki gereksiz)
    gray_blurred = cv2.blur(gray_blurred2,(10,10))


    #kenarları bul - KULLANMADIN
    canny_edge = cv2.Canny(red_hue_range,50,240)
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
                    cv2.circle(frame, (a,b), r, circle_color,3)

                    #Draw a small circle to show th ecenter
                    cv2.circle(frame, (a,b), 1, (0,0,255),3)
            #Belirlenen daireleri çizme buraya kadar---------------------------- (Altta contour muhabbeti var)
                    cv2.putText(frame,'Havuz', (a+10, b+10),cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2, cv2.LINE_AA)
            else:
                target = False
                upt = False
                findRedContours()

        else:
            findRedContours()
            total_Mean_Check = False

    else:
        findRedContours()

    #Dairenin içinde mi?
    if upt:
        distance = (((a-img_Center_X)**2)+((b-img_Center_Y)**2))**0.5
        if abs(distance) < r:
            circle_color = (0,255,0)
        else:
            circle_color = (255,0,0)
            cv2.arrowedLine(frame,(img_Center_X , img_Center_Y) , (a,b) , (255,255,0) , 2)

    #hsv ortalamalarını yazdır------------------------------
    frame = cv2.putText(frame,'H:% .1f S:% .1f V:% .1f  Tot: % .1f' % (mean1, mean2, mean3,total_mean),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2,cv2.LINE_AA   )

    if showCircleArea:#Tamamlanmadı
        cv2.putText(frame, 'H:% .1f S:% .1f V:% .1f  Tot: % .1f' % (mean1, mean2, mean3, total_mean), (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

    #Kameranın pist tesğit edip etmediği
    #if total_Mean_Check:
     #   if target:
      #      frame = cv2.putText(frame, 'Pist Tespit Edildi' , (50, 75),
       #                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
       # else:
        #    frame = cv2.putText(frame, 'Pist Tespit Edilemedi', (50, 75),
         #                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)


    # Merkez noktası ekle (Drone Aim)
    cv2.rectangle(frame, (img_Center_X - int(Aim_Length / 2), img_Center_Y - int(Aim_Length / 2)),
                  (img_Center_X + int(Aim_Length / 2), img_Center_Y + int(Aim_Length / 2)), (0, 255, 255), 1)
    # cv2.circle(frame , ( img_Center_X , img_Center_Y ) , int(Aim_Length/2) , (50,255,255) , 2)
    # cv2.line(frame , ( img_Center_X , img_Center_Y - int(Aim_Length/2) ) , ( img_Center_X , img_Center_Y + int(Aim_Length/2) ) , (50,255,255) , 2)
    # cv2.line(frame , ( img_Center_X - int(Aim_Length/2) , img_Center_Y ) , ( img_Center_X + int(Aim_Length/2) , img_Center_Y ) , (50,255,255) , 2)

    #cv2.namedWindow('DETECTED CIRCLE',cv2.WINDOW_NORMAL)    #Window oluştur (Bu satır olmadan sadece imshow() ile de yapılabilir)
    #cv2.resizeWindow('DETECTED CIRCLE',1280,920)           #Windowun size'ını değiştir

    #video.write(frame)     #Şuan video kaydetmek istemediğim için kullanmıyorum

    cv2.imshow('DETECTED CIRCLE',frame)
    cv2.imshow('Gray',gray_blurred)
    cv2.imshow('MASK', res)
    cv2.imshow('CANNY',canny_edge)
    cv2.imshow('RedHue',red_hue_range)
    cv2.setMouseCallback('DETECTED CIRCLE', on_EVENT_LBUTTONDOWN)
    #cv2.waitKey(0)
    key=cv2.waitKey(10)
    if key & 0xFF == ord('q'):
        break
    if key & 0xFF == ord('p'):
        while(True):
            if cv2.waitKey(10) & 0xFF == ord('s'):
                break
cv2.destroyAllWindows()
video.release()
cap.release()
