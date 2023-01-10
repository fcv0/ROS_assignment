#!/usr/bin/env python

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import rospy, numpy, image_geometry, tf, imutils, roslib
from cv2 import *
from cv_bridge import CvBridge

class Colour_mask:
    # Computer vision used multiple sources to obtain the mask and centroid, this is mentioned throughout the script.
    '''
    In a real world setting thing would be less robust therefore multiple evidence bases will be useful to use.
    I.E YOLOv3 training a model to detect grapes ensuring detection of grapes is more accurate.
    '''

    LOW_PURPLE, HIGH_PURPLE = numpy.array([75,0,25]), numpy.array([255,70,105])
    # https://www.rapidtables.com/web/color/purple-color.html Used as a ref then modified via trial and error for better mask resolution reducing noise.

    # Finding coordinates of camera mainly used the image_projection_3.py script provided
    cam_model, image_depth_ros = None, None
    color2depth_aspect = (84.1/1920) / (70/512*1.202)
    grape_count_global = []

    def __init__(self, camera_pos):
        self.camera_pos = camera_pos
        # Constructor for the CVbridge and its subscribing nodes.
        # I want to use this for the right and left camera and even perhaps the front camera if possible.
        self.cvb = CvBridge()
        rospy.Subscriber(f'/thorvald_001/kinect2_{self.camera_pos}_camera/hd/image_color_rect', Image, self.cam_colour)
        rospy.Subscriber(f'/thorvald_001/kinect2_{self.camera_pos}_sensor/sd/image_depth_rect', Image, self.cam_depth)
        self.cam_info = rospy.Subscriber(f'/thorvald_001/kinect2_{self.camera_pos}_camera/hd/camera_info', CameraInfo, self.cam_details)
        self.tf = tf.TransformListener()

    def cam_colour(self, data):
        '''
        Turn on the above for the report as it shows a good representation of the difference in BGR to HSV...
        Flicker due to ambient light therefore HSV is better the use due to less noise as ambient light affects it less...
        '''
        if self.cam_details is None:
            return
        if self.image_depth_ros is None:
            return
        try:
            img = self.cvb.imgmsg_to_cv2(data,'bgr8')
            imgdep = self.cvb.imgmsg_to_cv2(self.image_depth_ros, '32FC1')
        except:
            pass
        imgcolourdep = cvtColor(imgdep, COLOR_GRAY2BGR)
        #img = resize(img, None, fx=0.5, fy=0.5, interpolation = False)
        hsv = cvtColor(img, COLOR_BGR2HSV)
        mask = inRange(img, self.LOW_PURPLE, self.HIGH_PURPLE)

        hsvmask = inRange(hsv, numpy.array([90,50,40]), numpy.array([120,255,100]))
        contours = findContours(hsvmask, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        
        # Draw lines around masked object in question then increased the line width of the contours to make a large blob...
        # https://www.youtube.com/watch?v=xSzsD4kXhRw&ab_channel=ProgrammingKnowledge helpful video for extra operations to increase connection.

        hsvc = drawContours(hsvmask, contours, -1, (255,255,255), 8)
        #imshow('hsvgrape', hsvc)
        kernel = numpy.ones((6,6), numpy.uint8)
        ## Applies dilation then erosion.
        closer = morphologyEx(hsvc, MORPH_CLOSE, kernel)
        #imshow('closer', closer)
        #dilation = dilate(hsvc, kernel, iterations=1)
        #imshow('dil', dilation)

        ## https://learnopencv.com/blob-detection-using-opencv-python-c/ helpful to understand the parameters of choice.
        params = SimpleBlobDetector_Params()
        ## Colour filter 0 - 255 0 = black / darker --> 255 = white / brighter.
        params.filterByColor = True
        params.blobColor = 255
        ## min/max area = pixel coverage area.
        params.minArea = 125
        params.maxArea = 7000
        ## Elongation of the object where 1 = circle.
        params.minInertiaRatio = 0.03
        ## How convex a shape is -- closer to 1 = more circlular closer to 0 = more concaved.
        params.minConvexity = 0.17

        detector = SimpleBlobDetector_create(params)
        keypoints = detector.detect(closer)
        blob = drawKeypoints(closer, keypoints, numpy.array([]),(0,0,255), DRAW_MATCHES_FLAGS_DEFAULT)
        print('Counted grape bunches on image view: ', len(keypoints))

        ## I figured in this part that the keypoints give the central coordinates on the image for the blob(s) that meet the requirements.
        ## So instead of manual calculation of the moment I'll use opencv's implentation to keep it simple...
        local_grape_coords = []
        c=0
        for i, key in enumerate(keypoints):
            circle(img, (int(key.pt[0]), int(key.pt[1])), 1, (0,0,255),-1)
            putText(img, f'{i}', (int(key.pt[0] + 10), int(key.pt[1])), FONT_HERSHEY_PLAIN, 1, (255,255,255))
            xy = (int(key.pt[0]), int(key.pt[1]))
            blob_coordinates.append(xy)
            depth_coords = (imgdep.shape[0]/2 + (xy[0] - img.shape[0]/2)*self.color2depth_aspect,
            imgdep.shape[1]/2 + (xy[1] - img.shape[1]/2)*self.color2depth_aspect)
            circle(imgcolourdep, (int(depth_coords[0]), int(depth_coords[1])),3, (0,0,255),-1)
            try:
                depth_value = imgdep[int(depth_coords[0]), int(depth_coords[1])]
            except:
                continue
            if depth_coords[0] < 0 or depth_coords[1] < 0:
                continue

            if numpy.isnan(depth_value):
                continue
                for j in range(10):
                    try:
                        depth_value = imgdep[int(depth_coords[0]+j), int(depth_coords[1])]
                    except:
                        continue
                    # if numpy.isnan(imgdep[int(depth_coords[0]+j), int(depth_coords[1])]) != True:
                    #     depth_value = imgdep[int(depth_coords[0]+j), int(depth_coords[1])]
                    #     break
                    # elif numpy.isnan(imgdep[int(depth_coords[0]-j), int(depth_coords[1])]) != True:
                    #     depth_value = imgdep[int(depth_coords[0]-j), int(depth_coords[1])]
                    #     break
                    # elif numpy.isnan(imgdep[int(depth_coords[0]), int(depth_coords[1]+j)]) != True:
                    #     depth_value = imgdep[int(depth_coords[0]), int(depth_coords[1]+j)]
                    #     break
                    # elif numpy.isnan(imgdep[int(depth_coords[0]), int(depth_coords[1]-j)]) != True:
                    #     depth_value = numpy.isnan(imgdep[int(depth_coords[0]), int(depth_coords[1]-j)])
                    #     break
                    # elif numpy.isnan(imgdep[int(depth_coords[0]+j), int(depth_coords[1]+j)]) != True:
                    #     depth_value = imgdep[int(depth_coords[0]+j), int(depth_coords[1]+j)]
                    #     break
                    # elif numpy.isnan(imgdep[int(depth_coords[0]-j), int(depth_coords[1]+j)]) != True:
                    #     depth_value = imgdep[int(depth_coords[0]-j), int(depth_coords[1]+j)]
                    #     break
                    # elif numpy.isnan(imgdep[int(depth_coords[0]+j), int(depth_coords[1]-j)]) != True:
                    #     depth_value = imgdep[int(depth_coords[0]+j), int(depth_coords[1]-j)]
                    #     break
                    # elif numpy.isnan(imgdep[int(depth_coords[0]-j), int(depth_coords[1]-j)]) != True:
                    #     depth_value = imgdep[int(depth_coords[0]-j), int(depth_coords[1]-j)]
                    #     break

            camera_coords = self.cam_model.projectPixelTo3dRay((xy[1], xy[0]))
            camera_coords = [x/camera_coords[2] for x in camera_coords]
            camera_coords = [x*depth_value for x in camera_coords]

            for x in range(len(camera_coords)):
                if numpy.isnan(camera_coords[x]):
                    continue
                else:
                    c+=1

            if numpy.isnan(camera_coords[0]) != True or numpy.isnan(camera_coords[1]) != True or numpy.isnan(camera_coords[2]) != True:
                object_location = PoseStamped()
                object_location.header.frame_id = f"thorvald_001/kinect2_{self.camera_pos}_rgb_optical_frame"
                object_location.pose.orientation.w = 1.0
                object_location.pose.position.x = camera_coords[0]
                object_location.pose.position.y = camera_coords[1]
                object_location.pose.position.z = camera_coords[2]


                p_camera = self.tf.transformPose('map', object_location)
                x, y, z = p_camera.pose.position.x, p_camera.pose.position.y, p_camera.pose.position.z
                xyz = (x, y, z)

                tolerance = 0.05
                if len(self.gcdict) == 0:
                    self.gcdict[i] = xyz
                else:
                    for j in range(len(self.gcdict)):
                        if x in numpy.arange(self.grape_count_global[j][0]-tolerance, self.grape_count_global[j][0]+tolerance) and y in numpy.arange(self.grape_count_global[j][1]-tolerance, self.grape_count_global[j][1]+tolerance) and z in numpy.arange(self.grape_count_global[j][2]-tolerance, self.grape_count_global[j][2]+tolerance): continue
                        else:
                            self.gcdict[i] = xyz
                            print('x added one')
                            break
            
        #print('Grape count with depth values: ', int(c/3))
        imgcolourdep *= 1.0/10.0
        #imshow('dep', imgcolourdep)
        #imshow('hv', blob)
        imshow('img', img)
        print(self.grape_count_global)

        waitKey(1)

    def cam_depth(self, data):
        self.image_depth_ros = data

    def cam_details(self, data):
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(data)
        self.cam_info.unregister() 


if __name__ == '__main__':
    rospy.init_node('Colour_mask')
    Colour_mask('right')
    rospy.spin()