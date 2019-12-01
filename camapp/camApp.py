import cv2
import time
import math


def calcD(h,D=1335):
    f=1550
    return round((D/f)*h,1)

def findCorners(cnt):
    maxX = 0
    minX = 99999
    maxY = 0
    minY = 99999

    pairmaxX = []
    pairminX = []
    pairmaxY = []
    pairminY = []


    for tcour in cnt:
        cour=tcour[0]
        # maxX=max(maxX,cour[0])
        if cour[0]>maxX :
            maxX= cour[0]
            pairmaxX= cour
        # minX=min(minX,cour[0])
        if cour[0]<minX :
            minX= cour[0]
            pairminX= cour
        #maxY=max(maxY,cour[1])
        if cour[1]>maxY :
            maxY= cour[1]
            pairmaxY= cour
        # minY=min(minY,cour[1])
        if cour[1]<minY :
            minY= cour[1]
            pairminY= cour

    return pairminX,pairminY,pairmaxX,pairmaxY

def findDiameter(minX, minY, maxX, maxY):

    def dist(A,B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

    if dist(minX, minY) < dist(minY, maxX):
        return dist(minX, minY)
    elif dist(minX, minY) > dist(minY, maxX):
        return dist(minY, maxX)
    else:
        return None

def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image

    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)

    else:
        r = width / float(w)
        dim = (width, int(h * r))

    resized = cv2.resize(image, dim, interpolation = inter)

    return resized

def app():
    # video= cv2.VideoCapture('http://192.168.2.28:8080/videofeed')
    #video= cv2.VideoCapture('http://192.168.137.63:8080/videofeed')
    video= cv2.VideoCapture(0)
    items = 0
    first_frame= None
    time.sleep(2.0)
    while True:
        key = cv2.waitKey(1)
        check, frame = video.read()
        if check:

            gray= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray=cv2.GaussianBlur(gray, (19, 19), 1.21)
            if first_frame is None:
                first_frame= gray
                continue

            delta_frame= cv2.absdiff(first_frame,gray)

            thresh_frame= cv2.threshold(delta_frame, 60, 255, cv2.THRESH_BINARY)[1]
            thresh_frame= cv2.dilate(thresh_frame, None, iterations=2)
            cnts= cv2.findContours(thresh_frame.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            print(cnts)
            hier= cnts[2]
            cnts= cnts[1]
            cv2.putText(
                frame,
                'Items : '+str(items),
                (120, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                5,
                (80, 100, 255, 255),5)
            items= 0
            for i, contour in enumerate(cnts):
                if hier[0][i][3] == -1 and cv2.contourArea(contour) > 1100:
                    items+=1
                    minX, minY, maxX, maxY= findCorners(contour)
                    diameter= findDiameter(minX, minY, maxX, maxY)
                    if diameter is None:
                        (x, y, w, h) = cv2.boundingRect(contour)
                        diameter=h

                    cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)
                    print('diameter : '+str(diameter))
                    cv2.putText(
                        frame,
                        str(calcD(diameter))+'(mm)',
                        (minX[0],minX[1]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        2,
                        (220, 220, 80, 255))


            frame= image_resize(frame,height=768,width=1024)
            cv2.imshow("color", frame)
            if key == ord('r'):
                first_frame = gray

        if key==ord('q'):
            break


    video.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    app()