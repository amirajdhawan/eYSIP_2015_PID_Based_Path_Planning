import numpy as np
import cv2

# Initialize camera 

############################################

def nothing(x):
    pass
# Creating a window for later use
cap=cv2.VideoCapture(0)
cv2.namedWindow('Hsv_Color')

# Starting with 100's to prevent error while masking
Lh,Ls,Lv = [0,0,0]
Uh,Us,Uv = [120,120,120]
# Creating track bar
cv2.createTrackbar('Lh', 'Hsv_Color',Lh,179,nothing)
cv2.createTrackbar('Ls', 'Hsv_Color',Ls,255,nothing)
cv2.createTrackbar('Lv', 'Hsv_Color',Lv,255,nothing)
cv2.createTrackbar('Uh', 'Hsv_Color',Uh,179,nothing)
cv2.createTrackbar('Us', 'Hsv_Color',Us,255,nothing)
cv2.createTrackbar('Uv', 'Hsv_Color',Uv,255,nothing)

while(1):
    ## Read the image
    #img = cv2.imread('output_image.jpg')
    ret,img=cap.read()
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv',hsv)

    # get info from track bar and appy to result
    Lh = cv2.getTrackbarPos('Lh','Hsv_Color')
    Ls = cv2.getTrackbarPos('Ls','Hsv_Color')
    Lv = cv2.getTrackbarPos('Lv','Hsv_Color')
    Uh = cv2.getTrackbarPos('Uh','Hsv_Color')
    Us = cv2.getTrackbarPos('Us','Hsv_Color')
    Uv = cv2.getTrackbarPos('Uv','Hsv_Color')
    # Normal masking algorithm
    lower = np.array([Lh,Ls,Lv])
    upper = np.array([Uh,Us,Uv])

    mask = cv2.inRange(hsv,lower, upper)
    cv2.imshow('mask',mask)
    res = cv2.bitwise_and(img, img, mask= mask)
    #cv2.imshow('res',res)
    kernel = np.ones((70,50),np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    erosion = cv2.erode(mask,kernel,iterations = 1)
    dilation = cv2.dilate(erosion,kernel,iterations = 1)
    cv2.imshow('image1',closing)
    #cv2.imwrite('threshA.jpg',dilation)


    if cv2.waitKey(100)==27:
        break

print len(contours)
## Show the image
#cv2.imshow('image',img)

    
############################################

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
