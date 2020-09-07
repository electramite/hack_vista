import cv2
import numpy as np
import pyttsx3
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
button = 12
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
cap = cv2.VideoCapture(0)
while cap.isOpened():
    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red_lower = np.array([136, 87, 111])
    red_upper = np.array([180, 255, 255])

    blue_lower = np.array([99, 115, 150])
    blue_upper = np.array([110, 255, 255])

    yellow_lower = np.array([22, 60, 200], np.uint8)
    yellow_upper = np.array([60, 255, 255], np.uint8)

    red = cv2.inRange(hsv, red_lower, red_upper)
    blue = cv2.inRange(hsv, blue_lower, blue_upper)
    yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

    kernal = np.ones((5, 5), "uint8")
    red = cv2.dilate(red, kernal)
    res = cv2.bitwise_and(frame, frame, mask=red)

    blue = cv2.dilate(blue, kernal)
    res1 = cv2.bitwise_and(frame, frame, mask=blue)

    yellow = cv2.dilate(yellow, kernal)
    res2 = cv2.bitwise_and(frame, frame, mask=yellow)
     #traking of red clothes
    (_, contours, hierarchy) = cv2.findContours(red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        engine = pyttsx3.init()
        area = cv2.contourArea(contour)
        if (area > 500):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "RED", (x, y), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.7, (0, 0, 255))
            # os.system('start red.mp3')
            if GPIO.input(button) == 0 and area >= 500:
                engine.say('red')
                engine.runAndWait()
    # traking of blue clothes
    (_, contours, hierarchy) = cv2.findContours(blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 500):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, "BLUE", (x, y), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.7, (255, 0, 0))
            if GPIO.input(button) == 0 and area >= 500:
                engine.say('blue')
                engine.runAndWait()
        # os.system('start blue.mp3')
    # traking of yellow clothes
    (_, contours, hierarchy) = cv2.findContours(yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 500):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, "yellow  color", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
            if GPIO.input(button) == 0 and area >= 500:
                engine.say('blue')
                engine.runAndWait()

    cv2.imshow("clothes", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break