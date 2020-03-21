import cv2
import numpy as np

def nothing(x):
    #anything here
    pass

cap = cv2.VideoCapture(0)


# setup for blurple detection right now
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 110, 255, nothing)
cv2.createTrackbar("L-S", "Trackbars", 110, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 56, 180, nothing)
cv2.createTrackbar("U-H", "Trackbars", 160, 255, nothing)
cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 180, 180, nothing)

font = cv2.FONT_HERSHEY_COMPLEX

'''
#setup for black detection right now
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L-S", "Trackbars", 176, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 0, 180, nothing)
cv2.createTrackbar("U-H", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 73, 180, nothing)
'''


while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L-H", "Trackbars")
    l_s = cv2.getTrackbarPos("L-S", "Trackbars")
    l_v = cv2.getTrackbarPos("L-V", "Trackbars")
    u_h = cv2.getTrackbarPos("U-H", "Trackbars")
    u_s = cv2.getTrackbarPos("U-S", "Trackbars")
    u_v = cv2.getTrackbarPos("U-V", "Trackbars")

    #print(l_h)

    #lower_blue = np.array([l_h, l_s, l_v])
    #upper_blue = np.array([u_h, u_s, u_v])

    lower_blue = np.array([110, 110, 56])
    upper_blue = np.array([160, 255, 180])


    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)

    # Contours detection
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 400:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 3:
                cv2.putText(frame, "Triangle", (x, y), font, 1, (0, 0, 0))
                #print("I see a triangle"+ "\n")

            elif len(approx) == 4:
                # cv2.drawContours(frame, [approx], 0, (0, 180, 255), 5)
                cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))
                #print("I see a rectangle"+ "\n")
            '''elif 10 < len(approx) < 20:
                cv2.putText(frame, "Circle", (x, y), font, 1, (0, 0, 0))'''


    # show all results
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
