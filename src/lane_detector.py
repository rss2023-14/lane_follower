#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv

class LaneDetector:
    """
    TODO
    """

    def __init__(self):
        pass

    def f1(self):
        pass

    def f2(self):
        pass

    def detect_lane(self,image):
        img = cv.imread(image)

        # mask top half of image
        top_half_mask = np.zeros_like(img)
        height = img.shape[0]
        height_part = int(np.floor(height*0.5))
        top_half_mask[height_part:,:] = 255
        blacked_img = cv.bitwise_and(img,top_half_mask)

        # grayscale and blur image to get the edges 
        gray_img = cv.cvtColor(blacked_img,cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray_img,(5,5),0)
        edges = cv.Canny(blur,50, 150, apertureSize=3)

        # Get the lines from opencv
        lines = cv.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=50, minLineLength=150,maxLineGap=10)

        # filter lines based on their angle, aiming for almost vertical lines
        filtered_lines = []
        for  line in lines:
            x1,y1,x2,y2 = line[0]
            # angle = np.arctan2(y2-y1, x2-x1) * 180 / np.pi
            if abs(y1-y2) < 50:
                continue
            # if abs(angle) < 30: # vertical lines
            else:
                filtered_lines.append(list(line[0]))
                    
        #filter lines again, this time to try to get a single line per lane
        slope_tracker = {}
        copy_filt = filtered_lines.copy()
        for line in copy_filt:
            x1,y1,x2,y2 = line
            slope = (y2-y1)/(x2-x1)
            get_rid = False
            if slope_tracker != {}:
                copy_slopes = slope_tracker.copy()
                for oth_slope in copy_slopes.keys():
                    if abs(slope-oth_slope) < 0.15:
                        if slope < 0:
                            if x1 < slope_tracker[oth_slope][0]:
                                filtered_lines.remove(slope_tracker[oth_slope])
                                del slope_tracker[oth_slope]
                            else:
                                get_rid = True
                                continue
                        else:
                            if x1 > slope_tracker[oth_slope][0]:
                                filtered_lines.remove(slope_tracker[oth_slope])
                                del slope_tracker[oth_slope]
                            else:
                                get_rid = True
                                continue
            if get_rid:
                filtered_lines.remove(line)
                continue
            slope_tracker[slope] = line


                        
            
        i = 0
        # print(slope_tracker)
        y_return =  int(np.floor(height*0.9))
        if len(filtered_lines) == 2:
            x1,y1,x2,y2 = filtered_lines[0]
            m = (y2-y1)/(x2-x1)
            b = y1 - m*x1
            x_lane_1 = (y_return-b)/m
            x1,y1,x2,y2 = filtered_lines[1]
            m = (y2-y1)/(x2-x1)
            b = y1 - m*x1
            x_lane_2 = (y_return-b)/m
            x_return = int(np.floor((x_lane_1+x_lane_2)/2))
            cv.circle(img, (x_return,y_return), 5, (0, 0, 255), -1)

        
        for line in filtered_lines:
            x1,y1,x2,y2 = line
            
            cv.line(img, (x1,y1),(x2,y2),(i,255,255 - i),2)
            i += 100
        cv.imshow('Detected Lines', img)
        cv.waitKey(0)
        cv.destroyAllWindows()



    ##IDEAS
    # REturn line locations after first call back, use these to see where we atski

    # make the line, return it, next time if theres only one, we peep if that one is close by the previous

    # I think homography needs to do the mean of the lines, the pixel mean is probably not accurate for  shit



if __name__ == "__main__":
    rospy.init_node("lane_detector")
    ld = LaneDetector()
    rospy.spin()
