#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import cv2
import numpy as np
from matplotlib import pyplot as plt

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

from params import MODE, TARGET, CAMERA
from eye_consts import PUB_RATE, EYE3_DISP
from eyes import Eyes

#=======================================================================================================================
# Display Mode(for Debug)
dispMode  = EYE3_DISP                   # display 1:ON 0:OFF


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

# Pole Position/Distance
polePosX = 0
polePosY = 0
poleDist = 0


#=======================================================================================================================
# Get Disparity Map
def GetDisparityMap():
    imgGrayL = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )
    imgGrayR = cv2.GaussianBlur( cv2.equalizeHist(cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)), (5, 5), 0 )

    stereo = cv2.StereoBM_create( numDisparities=dispar, blockSize=sadws )
    return stereo.compute( imgGrayL, imgGrayR )


#=======================================================================================================================
# Get Blue Detect
def GetBlueDetect( img ):
    # HSV space
    hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

    # thresh1
    hsv_min = np.array( [ 100, 150, 100] )
    hsv_max = np.array( [ 140, 255, 255] )
    mask1   = cv2.inRange( hsv, hsv_min, hsv_max )

    binary_img = mask1

    if dispMode == 1:    
        cv2.imshow( "Blue", binary_img )

    unused_image, contours, unused_hierarchy = cv2.findContours( binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE )
    max_index   = 0
    max_contour = cv2.contourArea( contours[max_index] )

    for i in range( 0, len( contours ) ):
        if len( contours[i] ) > 0:
            tmp = cv2.contourArea( contours[i] )

            # remove small objects
            if tmp < 10:
                continue

            # Max Blue Contours
            if tmp > max_contour:
                max_index    = i
                max_contour  = tmp

    return contours[max_index]


#=======================================================================================================================
def eyes_py(id):
    global polePosX, polePosY, dispar, sadws

    # 初期化宣言 : このソフトウェアは"eyes_py_node"という名前
    rospy.init_node('eyes_py_node', anonymous=True)
    # インスタンスの作成 
    eyes = Eyes(id)
    # 処理周期の設定
    r = rospy.Rate(PUB_RATE)

    print("[eyes3] start")
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
                print disparity[polePosY][polePosX]
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
            #-- Cal poleDistance --
            if key == ord( 'c' ):
                disparity = GetDisparityMap()

                capL_target = GetBlueDetect( imgL )
                Lx, Ly, Lw, Lh = cv2.boundingRect( capL_target )

                capR_target = GetBlueDetect( imgR )
                Rx, Ry, Rw, Rh = cv2.boundingRect( capR_target )

                polePosX = (Lx + Rx) / 2 + ((Lw + Rw) / 2) / 2
                polePosY = (Ly + Ry) / 2 + ((Lh + Rh) / 2) / 2

                # from disparity to dist
                # Z = f x T / um / disparity
                poleDist = 6.6 * 62 / 0.003 / disparity[polePosY][polePosX]
                print poleDist
            #-- Cal PoleCenter --
            if key == ord( 'r' ):
                capL_target = GetBlueDetect( imgL )
                Lx, Ly, Lw, Lh = cv2.boundingRect( capL_target )

                capR_target = GetBlueDetect( imgR )
                Rx, Ry, Rw, Rh = cv2.boundingRect( capR_target )

                if dispMode == 1:
                    cv2.rectangle( imgL, (Lx, Ly), (Lx + Lw, Ly + Lh), (255, 0, 0), 1 )
                    cv2.imshow( "Left Pole", imgL )
                    cv2.rectangle( imgR, (Rx, Ry), (Rx + Rw, Ry + Rh), (255, 0, 0), 1 )
                    cv2.imshow( "Right Pole", imgR )

                Lpos = Lx + (Lw / 2)
                Rpos = Rx + (Rw / 2)
                print Lpos, Rpos
        #==================================================

        # メイン処理
        eyes.main()
        #
        r.sleep()
#=======================================================================================================================

if __name__ == '__main__':
    try:
        eyes_py(CAMERA.POLL)

    except rospy.ROSInterruptException:
        # Stop
        capL.release()
        capR.release()
        cv2.destroyAllWindows()
        print("[eyes3] end")