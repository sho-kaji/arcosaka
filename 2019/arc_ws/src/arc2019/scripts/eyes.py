#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import cv2
import numpy as np
from matplotlib import pyplot as plt


#=======================================================================================================================
# Display Mode(for Debug)
dispMode  = 1                   # display 1:ON 0:OFF


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

    stereo = cv2.StereoBM( cv2.STEREO_BM_BASIC_PRESET, ndisparities=dispar, SADWindowSize=sadws)
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
        cv2.imshow( "detect", binary_img )

# connectedComponentsWithStats is not usable ver2.4.x
#    label = cv2.connectedComponentsWithStats( binary_img )
#    n      = label[0] - 1
#    data   = np.delete( label[2], 0, 0 )
#    center = np.delete( label[3], 0, 0 )
#    max_index = np.argmax( data[:, 4] )
#    maxblob = {}
#    maxblob["upper_left"] = ( data[:, 0][max_index], data[:, 1][max_index] )
#    maxblob["width"     ] = data[:, 2][max_index]
#    maxblob["height"    ] = data[:, 3][max_index]
#    maxblob["area"      ] = data[:, 4][max_index]
#    maxblob["center"    ] = center[max_index]
#    return maxblob

    contours, hierarchy = cv2.findContours( binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE )
    max_index   = 0
    max_contour = cv2.contourArea( contours[max_index] )

    for i in range( 0, len( contours ) ):
        if len( contours[i] ) > 0:
            tmp = cv2.contourArea( contours[i] )

            # remove small objects
            if tmp < 10:
                continue

            # Max Tomato Contours
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
            x, y, w, h = cv2.boundingRect( capL_target )
            tomatoPosX = x + (w / 2)
            tomatoPosY = y + (h / 2)

            # from disparity to dist
            # Z = f x T / um / disparity
            tomatoDist = 6.6 * 62 / 0.003 / disparity[tomatoPosY][tomatoPosX]
            print tomatoDist
        #-- Cal tomatoCenter --
        if key == ord( 'r' ):
            # Use Left Camera
            target = GetRedDetect( imgL )

            # target Position
            x, y, w, h = cv2.boundingRect( target )

            if dispMode == 1:
                cv2.rectangle( imgL, (x, y), (x + w, y + h), (0, 0, 255), 1 )
                cv2.imshow( "red", imgL )

            tomatoPosX = x + (w / 2)
            tomatoPosY = y + (h / 2)
            print tomatoPosX, tomatoPosY

            
            if tomatoPosX >= ( capL_CenterX - cap_CenterOffset ):
                if tomatoPosX <= ( capL_CenterX + cap_CenterOffset ):
                    print ('Center')
                else:
                    print ('Turn Left')
            else:
                print ('Turn Right')


#=======================================================================================================================
# Stop
capL.release()
capR.release()
cv2.destroyAllWindows()

