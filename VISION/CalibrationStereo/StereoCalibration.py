import numpy as np
import cv2
import glob



################ ÉCHÉQUIER #############################

chessboardSize = (8,6)
frameSize = (720,540)




criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)



objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
objp = objp*26.2 #On multiplie par la largeur des carrés en mm


objpoints = [] 
imgpointsL = [] 
imgpointsR = [] 


imagesLeft = sorted(glob.glob('Images/Gauche/*.png'))
imagesRight = sorted(glob.glob('Images/Droite/*.png'))
counter = 0
for imgLeft, imgRight in zip(imagesLeft, imagesRight):

    imgL = cv2.imread(imgLeft)
    imgR = cv2.imread(imgRight)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)


    retL, cornersL = cv2.findChessboardCorners(grayL, chessboardSize, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, chessboardSize, None)


    if retL and retR == True:

        objpoints.append(objp)

        cornersL = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        print(counter)
        counter += 1

        cv2.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv2.imshow('img left', imgL)
        cv2.waitKey(300)
        


cv2.destroyAllWindows()




############## CALIBRATION #######################################################
print("Calibration des cameras")
retL, cameraMatrixL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpointsL, frameSize, None, None)
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv2.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpointsR, frameSize, None, None)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv2.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))



########## CALIBRATION STEREO #############################################
print("Stereo Calibration")
flags = cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_SAME_FOCAL_LENGTH

criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)



########## RECTIFICATION STEREO #################################################
print("Stereo Rectification")
rectifyScale= 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv2.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0))

stereoMapL = cv2.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv2.CV_16SC2)
stereoMapR = cv2.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv2.CV_16SC2)

print("Saving parameters!")
cv2_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_WRITE)

cv2_file.write('stereoMapL_x',stereoMapL[0])
cv2_file.write('stereoMapL_y',stereoMapL[1])
cv2_file.write('stereoMapR_x',stereoMapR[0])
cv2_file.write('stereoMapR_y',stereoMapR[1])
cv2_file.write('q',Q)
cv2_file.release()

