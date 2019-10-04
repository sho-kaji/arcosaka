#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import cv2
import numpy as np
from matplotlib import pyplot as plt

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

from params import MODE, TARGET, CAMERA
from eye_consts import PUB_RATE, EYE2_DISP
from eyes import Eyes

#=======================================================================================================================
# Display Mode(for Debug)
dispMode  = EYE2_DISP                   # display 1:ON 0:OFF


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

#  Side Buds and Weed Position
sidebudsPosX = 0
sidebudsPosY = 0
#sidebudsPosDist = 0
weedPosX     = 0
weedPosY     = 0
#weedPosDist  = 0


#=======================================================================================================================
# Get Disparity Map
def GetDisparityMap():
    imgGrayL = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )
    imgGrayR = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )

    stereo = cv2.StereoBM_create( numDisparities=dispar, blockSize=sadws )
    return stereo.compute( imgGrayL, imgGrayR )


#=======================================================================================================================
# Get Green Detect
def GetGreenDetect( img ):
    # HSV space
    hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

    # thresh1
    hsv_min = np.array( [ 40,  80, 100] )
    hsv_max = np.array( [ 80, 255, 255] )
    mask1   = cv2.inRange( hsv, hsv_min, hsv_max )

    binary_img = mask1

    if dispMode == 1:    
        cv2.imshow( "detect", binary_img )

    unused_image, contours, unused_hierarchy = cv2.findContours( binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE )
    max_index   = 0
    max_contour = cv2.contourArea( contours[max_index] )

    for i in range( 0, len( contours ) ):
        if len( contours[i] ) > 0:
            tmp = cv2.contourArea( contours[i] )

            # remove small objects
            if tmp < 10:
                continue

            # Max Green Contours
            if tmp > max_contour:
                max_index    = i
                max_contour  = tmp

    return contours[max_index]

#=======================================================================================================================
def eyes_py(id):
    global sidebudsPosX, sidebudsPosY, dispar, sadws

    # 初期化宣言 : このソフトウェアは"eyes_py_node"という名前
    rospy.init_node('eyes_py_node', anonymous=True)
    # インスタンスの作成 
    eyes = Eyes(id)
    # 処理周期の設定
    r = rospy.Rate(PUB_RATE)

    print("[eyes2] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():

        #==================================================
        # Loop
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
                print disparity[sidebudsPosY][sidebudsPosX]
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
            #-- Cal Side Buds Distance --
            # Not Supported
    #        if key == ord( 'c' ):
    #            disparity = GetDisparityMap()
    #
    #            capL_target = GetRedDetect( imgL )
    #            x, y, w, h = cv2.boundingRect( capL_target )
    #            tomatoPosX = x + (w / 2)
    #            tomatoPosY = y + (h / 2)
    #
    #            # from disparity to dist
    #            # Z = f x T / um / disparity
    #            tomatoDist = 6.6 * 62 / 0.003 / disparity[tomatoPosY][tomatoPosX]
    #            print tomatoDist
            #-- Cal weed Center --
            if key == ord( 'r' ):
                # Use Left Camera
                target = GetGreenDetect( imgL )

                # target Position
                x, y, w, h = cv2.boundingRect( target )

                if dispMode == 1:
                    cv2.rectangle( imgL, (x, y), (x + w, y + h), (0, 0, 255), 1 )
                    cv2.imshow( "red", imgL )

                weedPosX = x + (w / 2)
                weedPosY = y + (h / 2)
                print weedPosX, weedPosY
            #-- Look for SideBuds --
            if key == ord( 'l' ):
                # pattern read Side Buds Jpeg at GrayScale
                sbsJpg = cv2.imread( './SideBuds_1.JPG' , 0 )
                if dispMode == 1:
                    cv2.imshow( "SideBuds_1", sbsJpg )

                grayImg = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)

                result = cv2.matchTemplate( grayImg, sbsJpg, cv2.TM_CCOEFF_NORMED )
                unused_minVal, unused_maxVal, unused_minLoc, maxLoc = cv2.minMaxLoc( result )
                top_left     = maxLoc
                w, h         = sbsJpg.shape[::-1]
                bottom_right = ( top_left[0] + w, top_left[1] + h )
                if dispMode == 1:
                    cv2.rectangle( imgL, top_left, bottom_right, (0, 255, 0), 1 )
                    cv2.imshow( "SideBuds Result", imgL )

                sidebudsPosX = top_left[0] + (w / 2)
                sidebudsPosY = top_left[1] + (h / 2)
                print sidebudsPosX, sidebudsPosY
        #==================================================

        # メイン処理
        eyes.main()
        #
        r.sleep()
#=======================================================================================================================

if __name__ == '__main__':
    try:
        eyes_py(CAMERA.SUB)

    except rospy.ROSInterruptException:
        # Stop
        capL.release()
        capR.release()
        cv2.destroyAllWindows()
        print("[eyes2] end")
