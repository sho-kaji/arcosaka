#!/usr/bin/env python
# -*-coding:utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt


#=======================================================================================================================
# Display Mode(for Debug)
dispMode  = 0                   # display 1:ON 0:OFF


#=======================================================================================================================
# Connecting Camera
capL = cv2.VideoCapture( 1 )
capR = cv2.VideoCapture( 0 )

# Setting Camera
capWidth  = 320                 # 480x360 not display
capHeight = 240                 # 640x360 select timeout a little
capL.set(  3, capWidth  )
capL.set(  4, capHeight )
capR.set(  3, capWidth  )
capR.set(  4, capHeight )
capL_CenterX     = 174          # Left  Camera CenterX
capR_CenterX     = 144          # Right Camera CenterX
cap_CenterY      = 119          # Both Camera CenterY
cap_CenterOffset = 5            # Both Camera CenterOffset

# Mapping Param
imgL = np.zeros( (capHeight, capWidth, 3), np.uint8 )
imgR = np.zeros( (capHeight, capWidth, 3), np.uint8 )
dispar = 32                     # multiple of a unit = 16
sadws  = 21                     # odd within 5...255

# Tomato Position/Distance
tomatoPosX = 0
tomatoPosY = 0
tomatoDist = 0


#=======================================================================================================================
# Get Disparity Map
def GetDisparityMap():
    imgGrayL = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )
    imgGrayR = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )

    stereo = cv2.StereoBM_create( numDisparities=dispar, blockSize=sadws )
    return stereo.compute( imgGrayL, imgGrayR )


#=======================================================================================================================
# Get Tomato
def GetRedDetect( img ):
    # HSV space
    hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

    # thresh1
    hsv_min = np.array( [  0, 180, 100] )
    hsv_max = np.array( [ 10, 255, 255] )
    mask1   = cv2.inRange( hsv, hsv_min, hsv_max )

    # thresh2
    hsv_min = np.array( [169, 180, 100] )
    hsv_max = np.array( [179, 255, 255] )
    mask2   = cv2.inRange( hsv, hsv_min, hsv_max )

    binary_img = mask1 + mask2

    if dispMode == 1:    
        cv2.imshow( "Red", binary_img )

    image, contours, hierarchy = cv2.findContours( binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE )
    max_index   = 0
    max_contour = cv2.contourArea( contours[max_index] )

    for i in range( 0, len( contours ) ):
        if len( contours[i] ) > 0:
            tmp = cv2.contourArea( contours[i] )

            # remove small objects
            if tmp < 10:
                continue

            # Max RedDetect Contours
            if tmp > max_contour:
                max_index    = i
                max_contour  = tmp

    return contours[max_index]


#=======================================================================================================================
# Loop
while True:
    capL.read( imgL )
    capR.read( imgR )

    if dispMode == 1:
        cv2.imshow( "Left  Camera", imgL )
        cv2.imshow( "Right Camera", imgR )
        key = cv2.waitKey( 1 ) & 0xFF
    else:
        tmp = raw_input('>>')
        key = ord(tmp)

    if key == ord('q'):
        break
    else:
        #-- DiparityMap(for Debug) --
        if key == ord('d'):
            disparity = GetDisparityMap()
            disp = cv2.normalize( disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U )
            if dispMode == 1:
                cv2.imshow( "disp", disp )
        #-- Print tomatoDist(for Debug) --
        if key == ord('p'):
            disparity = GetDisparityMap()
            print disparity[tomatoPosY][tomatoPosX]
        #-- Up ndisparities --
        if key == ord('a'):
            dispar = dispar + 16
            print dispar
        #-- Down ndisparities --
        if key == ord('z'):
            if( dispar > 16 ):
                dispar = dispar - 16
            print dispar
        #-- UP SADWindowSize --
        if key == ord('b'):
            if( sadws < 255 ):
                sadws = sadws + 2
            print sadws
        #-- Down SADWindowSize --
        if key == ord('y'):
            if( sadws > 5 ):
                sadws = sadws - 2
            print sadws
        #-- Cal tomatoDistance --
        if key == ord( 'c' ):
            disparity = GetDisparityMap()

            capL_target = GetRedDetect( imgL )
            Lx, Ly, Lw, Lh = cv2.boundingRect( capL_target )

            capR_target = GetRedDetect( imgR )
            Rx, Ry, Rw, Rh = cv2.boundingRect( capR_target )

            #tomatoPosX = (Lx + Rx) / 2 + ((Lw + Rw) / 2) / 2
            #tomatoPosY = (Ly + Ry) / 2 + ((Lh + Rh) / 2) / 2
            tomatoPosX = Lx + (Lw / 2)
            tomatoPosY = Ly + (Lh / 2)

            # from disparity to dist
            # Z = f x T / um / disparity
            tomatoDist = 6.6 * 62 / 0.003 / disparity[tomatoPosY][tomatoPosX]
            print tomatoDist
        #-- Disp tomatoCenter --
        if key == ord( 'r' ):
            capL_target = GetRedDetect( imgL )
            Lx, Ly, Lw, Lh = cv2.boundingRect( capL_target )

            capR_target = GetRedDetect( imgR )
            Rx, Ry, Rw, Rh = cv2.boundingRect( capR_target )

            if dispMode == 1:
                cv2.rectangle( imgL, (Lx, Ly), (Lx + Lw, Ly + Lh), (0, 0, 255), 1 )
                cv2.imshow( "Left Tomato", imgL )
                cv2.rectangle( imgR, (Rx, Ry), (Rx + Rw, Ry + Rh), (0, 0, 255), 1 )
                cv2.imshow( "Right Tomato", imgR )

            Lpos = Lx + (Lw / 2)
            Rpos = Rx + (Rw / 2)
            print Lpos, Rpos


#=======================================================================================================================
# Stop
capL.release()
capR.release()
cv2.destroyAllWindows()

