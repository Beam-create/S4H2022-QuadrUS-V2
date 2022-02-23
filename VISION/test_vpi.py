# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import cv2
import sys
import vpi
import numpy as np
from PIL import Image
from CamCSI import *


scale=1


backend = vpi.Backend.CUDA


left_camera = CSI_Camera(1)
right_camera = CSI_Camera(0)

def acquisition():
    left_camera.open()
    left_camera.start()

    right_camera.open()
    right_camera.start()

    if left_camera.video_capture.isOpened() and right_camera.video_capture.isOpened():

        try:
            while True:
                _, left_image = left_camera.read()
                _, right_image = right_camera.read()
                cv2.imshow('right_image', right_image)
                with vpi.Backend.CUDA:
                    left = vpi.asimage(np.asarray(Image.fromarray(cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)))).convert(vpi.Format.Y16_ER, scale=scale)
                    right = vpi.asimage(np.asarray(Image.fromarray(cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)))).convert(vpi.Format.Y16_ER, scale=scale)

               
                confidenceMap = vpi.Image(left.size, vpi.Format.U16)
              
                disparity = vpi.stereodisp(left, right, out_confmap=confidenceMap, backend=backend, window=5, maxdisp=64)

               
                with vpi.Backend.CUDA:
                    
                    disparity = disparity.convert(vpi.Format.U8, scale=255.0/(32*64))

                   
                    disparityColor = cv2.applyColorMap(disparity.cpu(), cv2.COLORMAP_JET)

                    disparityColor = cv2.cvtColor(disparityColor, cv2.COLOR_BGR2RGB)

                    if confidenceMap:
                        confidenceMap = confidenceMap.convert(vpi.Format.U8, scale=255.0/65535)

                       
                        mask = cv2.threshold(confidenceMap.cpu(), 1, 255, cv2.THRESH_BINARY)[1]
                        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                        disparityColor = cv2.bitwise_and(disparityColor, mask)

    
                disparity = Image.fromarray(disparityColor)
                cv2.imshow('disparity', disparityColor)
                
                if confidenceMap:
                    Image.fromarray(confidenceMap.cpu())
                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == 27:
                    break
        finally:
            left_camera.stop()
            left_camera.release()
            right_camera.stop()
            right_camera.release()
        cv2.destroyAllWindows()
    else:
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()

if __name__ == "__main__":
    acquisition()
