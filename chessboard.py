import cv2
import numpy as np
import pyrealsense2 as rs


class ChessBoard:

    def __init__(self, frame_width, frame_height, num_of_width, num_of_height, size_of_square):
        # camera setting
        self.frame_width = frame_width
        self.frame_height = frame_height

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.frame_width, self.frame_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.frame_width, self.frame_height, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)

        # Reference by align-depth2color.py
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        print('detect_soma')


    def start(self):
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays
            # img_depth = np.asanyarray(aligned_depth_frame.get_data())
            img_color = np.asanyarray(color_frame.get_data())

            cv2.waitKey(1)
            cv2.imshow('img_color', img_color)

