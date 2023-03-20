#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

class realSenseCamera:
    def __init__(self):
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        print("[+] realsense {} initialization \n".format(device_product_line))

    def __del__(self):
        self.pipeline.stop()
        print("[+] realsense terminated")


    def getAlignedFrames(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        aligned_color_frame = aligned_frames.get_color_frame()

        return aligned_depth_frame, aligned_color_frame
    
    def getColorImage(self):
        depth_frame = None
        color_frame = None

        while depth_frame is None or color_frame is None:
            depth_frame, color_frame = self.getAlignedFrames()

        return np.asanyarray(color_frame.get_data())
    
    def getDistance(self, x, y):
        depth_frame = None
        color_frame = None

        while depth_frame is None or color_frame is None:

            depth_frame, color_frame = self.getAlignedFrames()

        return depth_frame.get_distance(x, y)
    
    def getCoordinate(self, x, y):
        depth_frame = None
        color_frame = None

        while depth_frame is None or color_frame is None:
            depth_frame, color_frame = self.getAlignedFrames()

        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # get point depth
        depth = depth_frame.get_distance(x, y)

        return rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)  
    
    def stream(self):
        # Streaming loop        
        while True:                
                aligned_depth_frame, aligned_color_frame = self.getAlignedFrames()

                # Validate that both frames are valid
                if aligned_depth_frame is None or aligned_color_frame is None:
                    continue                

                depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
                
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(aligned_color_frame.get_data())

                # get center point depth and draw
                depth = aligned_depth_frame.get_distance(320, 240)
                cv2.putText(color_image, " {:.2f} meter".format(depth), (280, 220), cv2.FONT_HERSHEY_DUPLEX, 1 , (0,0,255), 2)
                cv2.circle(color_image, (320, 240), 3, (0,0,255), -1)

                center_xyz = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [320, 240], depth)
                print("[+] coordinate x: {:.2f} y:{:.2f}, depth{:.2f}".format(center_xyz[0], center_xyz[1], center_xyz[2]))                
                
                # Render images:
                #   depth align to color on left
                #   depth on right
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                images = np.hstack((color_image, depth_colormap))

                cv2.namedWindow('Align Images press "ESC" to leave', cv2.WINDOW_NORMAL)
                cv2.imshow('Align Images press "ESC" to leave', images)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break            


if __name__ == "__main__":
    depthCamera = realSenseCamera()
    depthCamera.stream()


