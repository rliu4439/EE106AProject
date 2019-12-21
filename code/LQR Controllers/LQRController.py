#See if we should even run the pilot module. 
#This is only needed because the part run_condition only accepts boolean

import time
import sys
import cv2
import serial
from controlpy.controlpy import synthesis
import scipy.linalg
import numpy as np
from math import sqrt, atan, pi, pow, cos, sin, asin, tan, atan2
import PIL
from PIL import Image
import os
import shapely
from shapely.geometry import LineString
import pyrealsense2 as rs
import time
#
from qpsolvers import solve_qp
import qpsolvers
from scipy.linalg import block_diag
from time import time
from numpy import hstack, inf, ndarray, ones

source_path = "/Users/luyu/Downloads/useful_red"
post_name = "_cam-image_array_a_.jpg"
#from controlpy.controlpy import 

class LQRController:


    def __init__(self):
        self.throttle = 0
        self.str = 0
        self.running = True
        self.ser=serial.Serial('/dev/ttyUSB0',9600, timeout=1)
        self.vels=np.array([0,0,0,0,0])
        self.count = 0
        self.old_p1 = np.array([0,0,100])
        self.old_p2 = np.array([0,0,200])
        self.old_p3 = np.array([0,0,300])

    def run_threaded(self,img1,img2,acl_x, acl_y,acl_z,gyr_x, gyr_y, gyr_z, intrinsics_object):
        #import time as time
        #start = time.time()

        self.img = img1
        self.depth = img2
        self.intrinsics = intrinsics_object
        self.w = 640
        self.h = 480
        self.v_ref = .4
        self.sx = 5 #moving average 
        self.movmean = np.zeros([2,self.sx])
        self.imgRate = 60
        self.dt = 1/self.imgRate
        self.stopmoving = 0
        self.count += 1

       
        
        

        '''
        process E-Stop state machine
        '''

        #### Simple KP Controller ####
        '''
        self.Kp = 1
        self.ControlPWMThresh = .12
        self.acl_desired = .1
        self.error = self.acl_desired - acl_z
        self.MotorPWM = self.Kp * self.error
        if self.MotorPWM > self.ControlPWMThresh:
          self.MotorPWM = self.ControlPWMThresh
        elif self.MotorPWM < -self.ControlPWMThresh:
          self.MotorPWM = -self.ControlPWMThresh
        '''
        
    
        
        '''
        ####################
        x_ref = np.array([.01,.05,.11])
        y_ref = np.array([.1,1,2])
        v_ref = .5
        ###############
        '''
        # p1,p2,p3 = self.getWayPoint()
        
       
        #time.sleep(self.ts)

        # return 0,0,'user',True
        self.MotorPWM = .132
        #import time

        #start = time.time()
        img = self.img
        img = img[...,::-1]
        #cv2.imwrite('/home/irg/projects/EE106aOld/image/color_img'+str(self.count)+".png",img)
        #np.save("/home/irg/projects/EE106aOld/depth/depth_array"+str(self.count)+".npy", self.depth)

        try:
            p1,p2,p3 = self.getWayPoint()
            #boxes = self.getObjectBoxes()
            #print(boxes)
            #self.getwheelvel()
            x_ref = np.array([p1[0]/1000,p2[0]/1000,p3[0]/1000]) 
            y_ref = np.array([(p1[2]/1000)+.125,(p2[2]/1000)+.125,(p3[2]/1000)+.125])
            #self.MotorPWM,self.Str = self.compute_uOpt(x_ref,y_ref,self.v_ref)
            self.mid_x = p2[0]/1000
            self.U_PD(x_ref,y_ref)
            self.old_p1 = p1
            self.old_p2 = p2
            self.old_p3 = p3
        except:
            p1, p2, p3 = self.old_p1, self.old_p2, self.old_p3
            #boxes = self.getObjectBoxes()
            #print(boxes)
            self.mid_x = p2[0]/1000
            #self.getwheelvel()
            x_ref = np.array([p1[0]/1000,p2[0]/1000,p3[0]/1000]) 
            y_ref = np.array([(p1[2]/1000)+.125,(p2[2]/1000)+.125,(p3[2]/1000)+.125])
            #self.MotorPWM,self.Str = self.compute_uOpt(x_ref,y_ref,self.v_ref)
            self.U_PD(x_ref,y_ref)

        #end = time.time()
        #print('vision time:',end - start)
        #print(p2)
       
        #print(self.vels)
        #self.InputThresh()
        #print('user angle',self.Str,"user_throttle",self.MotorPWM)
     
        #return self.Str,self.MotorPWM,'user',True
        #end=time.time()
        #print('Total time:',end - start)
        return self.Str,self.MotorPWM,'user',True


    def counter(self):
        self.count += 1
        return 0

    def InputThresh(self):
        if self.Str > 0:
            if self.Str > 15:
                self.Str = 14.9
        else:
            if self.Str < -15:
                self.Str = -14.9

    def compute_uOpt(self,x_ref,y_ref,v_ref):
        #computes car inputs
        if not(self.stopmoving):
            dt = self.dt
            lr = 0.13 #dist cg to rear
            lf = 0.13 #dist cg to front
            j = 0 #radius calc ofset
            interval = 1 #radius calc length
            x_ref_for_radius = [x_ref[j+interval],x_ref[j+interval*2]]
            y_ref_for_radius = [y_ref[j+interval],y_ref[j+interval*2]]
            x_ref_for_radius = np.append(x_ref[j],x_ref_for_radius)
            y_ref_for_radius = np.append(y_ref[j],y_ref_for_radius)

            x1 = x_ref_for_radius[0]
            x2 = x_ref_for_radius[1]
            x3 = x_ref_for_radius[2]
            y1 = y_ref_for_radius[0]
            y2 = y_ref_for_radius[1]
            y3 = y_ref_for_radius[2]
            ma = (y2-y1)/(x2-x1)
            mb = (y3-y2)/(x3-x2)

            if (abs(y2-y1<1e-5))or(abs(y3-y2<1e-5)):
                y2 = y2 + 1e-3
                ma = (y2-y1)/(x2-x1)
                mb = (y3-y2)/(x3-x2)
            x_c = (ma*mb*(y1-y3)+mb*(x1+x2)-ma*(x2+x3))/(2*(mb-ma))
            y_c = (-1/ma)*(x_c-(x1+x2)/2)+(y1+y2)/2
            Radius = sqrt(pow((x2-x_c),2)+pow((y2-y_c),2))
            psidot_des = v_ref/Radius
            if Radius<lr:
                Radius = lr*2
            if y_c>0:
                beta_des = abs(asin(lr/Radius))*-1
                psi_des = psidot_des*dt
            else:
                beta_des = abs(asin(lr/Radius))
                psi_des = psidot_des*dt*-1

            z = np.matrix([[0],[0],[0]])
            z_ref = [[x_ref[0]],[y_ref[0]],[psi_des]]
            u_bar = [[v_ref],[beta_des]]

            Ac = np.matrix([[0, 0, -v_ref*sin(psi_des+beta_des)],[0, 0, v_ref*cos(psi_des+beta_des)],[0, 0, 0]])
            Bc = np.matrix([[cos(psi_des+beta_des), -v_ref*sin(psi_des+beta_des)],[sin(psi_des+beta_des), v_ref*cos(psi_des+beta_des)],[sin(beta_des)/lr, v_ref*cos(beta_des)/lr]])

            Q = np.matrix([[50, 0, 0],[0, 50, 0],[0, 0, 1]])
            R = np.matrix([[25, 0 ],[0, 1]])

            # Compute the LQR controller
            K, X, closedLoopEigVals = synthesis.controller_lqr_discrete_from_continuous_time(Ac, Bc, Q, R, dt)

            u_Opt = -K*(z-z_ref)+u_bar
            vOpt = u_Opt[0,0]
            betaOpt = u_Opt[1,0]
            deltaOpt = atan2(((lf+lr)*tan(u_Opt[1,0])),lr)


            znext = z + self.dt*np.matrix([[vOpt*cos(z[2]+betaOpt)],[vOpt*sin(z[2]+betaOpt)],[vOpt*sin(betaOpt)/lr]])
            # Moving Average
            if (self.count>self.sx):
                self.movmean = np.delete(self.movmean,0,1)
                self.movmean = np.append(self.movmean,np.array([[vOpt],[deltaOpt]]),axis=1)
                #print(self.movmean)
                #print(np.mean(self.movmean,axis=1))
                vOpt = np.mean(self.movmean,axis=1)[0]
                deltaOpt = np.mean(self.movmean,axis=1)[1]

            #self.uOpt_pub.publish(vOpt,deltaOpt)

        return vOpt,deltaOpt

    def TractionControl(self):
        vels = self.vels
        fl = vels[0]
        fr = vels[1]
        rl = vels[2]
        rr = vels[3]
        bodyV = vels[4]
        FrontV = (fl +fr)/2
        RearV = (rr+rl)/2

        return 0

    def getwheelvel(self):

        vel_wheel=self.ser.readline()
        vel_wheel=str(vel_wheel)
        vel_wheel=vel_wheel[2:][:-5]
        vel_wheel=vel_wheel.split()
        #print(vel_wheel)
        try:
            self.vels= np.array(vel_wheel, dtype=np.float32)
        except:
            pass
        return 0
            

    ###################################################33
    def getWayPoint(self):

        # cv2.namedWindow('img')

        # hMin = sMin = vMin = hMax = sMax = vMax = 0
        # phMin = psMin = pvMin = phMax = psMax = pvMax = 0

        img = self.img
        img = img[...,::-1]
        # r = img[:, :, 0].copy()
        # g = img[:, :, 1].copy()
        # b = img[:, :, 2].copy()

        # img[:, :, 0], img[:, :, 1], img[:, :, 2] = b, g, r

        # img[:, :, 0], img[:, :, 1], img[:, :, 2] = img[:, :, 2], img[:, :, 1], img[:, :, 0]

        scale = 8

        # cv2.imwrite('/home/irg/vis/'+ str(self.count)+'img.jpg', img)

        h, w , _ = img.shape
        # img = cv2.resize(img, (int(w/2), int(h/2)))

        img = cv2.resize(img, (int(w/scale), int(h/scale)))

        # cv2.imwrite('/home/irg/vis/'+ str(self.count)+'img.jpg', img)


        use_mask = np.zeros(img.shape[:2], np.uint8)
        # print(use_mask.shape)
        use_mask[30: 60, :] = 255
        img = cv2.bitwise_and(img, img, mask=use_mask)

        #cv2.imwrite('/home/irg/vis/'+ str(self.count)+'img.jpg', img)


        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hMin, sMin,  vMin, hMax, sMax, vMax= 0, 100, 100, 10, 255, 255
        hMin2, sMin2,  vMin2, hMax2, sMax2, vMax2= 160, 100, 100, 180, 255, 255

        red_hsv_lower = (hMin, sMin, vMin)
        red_hsv_upper = (hMax, sMax, vMax)

        red_hsv_lower2 = (hMin2, sMin2, vMin2)
        red_hsv_upper2 = (hMax2, sMax2, vMax2)

        mask = cv2.inRange(hsv, red_hsv_lower, red_hsv_upper)
        mask2 = cv2.inRange(hsv, red_hsv_lower2, red_hsv_upper2)

        mask_combine = mask | mask2

        red = cv2.bitwise_and(img,img, mask=mask_combine)
        kernel = np.ones((3, 3), np.uint8)
        red = cv2.erode(red, kernel, iterations=1)
        red = cv2.dilate(red, kernel, iterations=1)
        # cv2.imwrite('/home/irg/vis/'+ str(self.count)+'red.jpg', red)
        h, s, white = cv2.split(red)
        ret, white = cv2.threshold(white, 127, 255, cv2.THRESH_BINARY)
        # cv2.imwrite('/home/irg/vis/'+ str(self.count)+'white.jpg', white)
        ret, labels = cv2.connectedComponents(white)
        largestCC = labels == np.argmax(np.bincount(labels.flat)[1:]) + 1
        component_img = self.imget_components(largestCC)

        #X and y coord of pixels in largest connected component
        y_set, x_set = np.nonzero(largestCC)



        minimal_y_line = []
        minimal_y_line_x_set = []

        middle_y_line = []
        middle_y_line_x_set = []

        max_y_line = []
        max_y_line_x_set = []


        for i in range(len(y_set)):
            if y_set[i] == (np.min(y_set) + 1):
                minimal_y_line.append((x_set[i], y_set[i]))
                minimal_y_line_x_set.append(x_set[i])
            elif y_set[i] == int((np.min(y_set) + 1 + np.max(y_set))/ 2):
                middle_y_line.append((x_set[i], y_set[i]))
                middle_y_line_x_set.append(x_set[i])
            elif y_set[i] == int(((np.min(y_set) + 1 + np.max(y_set))/ 2 + np.max(y_set)) / 2):
                max_y_line.append((x_set[i], y_set[i]))
                max_y_line_x_set.append(x_set[i])

        # minimal_y_position = np.where(y_set == (np.min(y_set) + 1))
        # # minimal_y_position = np.array(minimal_y_position)
        # minimal_y_line_x_set = x_set[minimal_y_position]

        # middle_y_position = np.where(y_set == int((np.min(y_set) + 1 + np.max(y_set))/ 2))
        # # middle_y_position = np.array(middle_y_position)
        # middle_y_line_x_set = x_set[middle_y_position]

        # max_y_position = np.where(y_set == int(((np.min(y_set) + 1 + np.max(y_set))/ 2 + np.max(y_set)) / 2))
        # # max_y_position = np.array(max_y_position)
        # max_y_line_x_set = x_set[max_y_position]






        farest_point_x = int(np.average(minimal_y_line_x_set))
        farest_point_y = np.min(y_set) + 1

        middle_point_x = int(np.average(middle_y_line_x_set))
        middle_point_y = int((np.min(y_set) + 1 + np.max(y_set))/ 2)

        max_point_x = int(np.average(max_y_line_x_set))
        max_point_y = int(((np.min(y_set) + 1 + np.max(y_set))/ 2 + np.max(y_set)) / 2)


        cv2.circle(component_img, (farest_point_x, farest_point_y), 2, (255, 255, 255), 3)
        cv2.circle(component_img, (middle_point_x, middle_point_y), 2, (255, 255, 255), 3)
        cv2.circle(component_img, (max_point_x, max_point_y), 2, (255, 255, 255), 3)
        
        #cv2.imwrite('/home/irg/vis/'+ str(self.count)+'img_component.jpg', component_img)
        # cv2.imshow("com", component_img)
        # cv2.imshow('img', white)
        # cv2.imshow('img_', img)
        # cv2.waitKey(1)
        # minimal_y_index = np.min(y_set)

        # cores_minimal_x = x_set[minimal_y_index]
        farest_point_x = scale * farest_point_x
        farest_point_y = scale * farest_point_y

        middle_point_x = scale * middle_point_x
        middle_point_y = scale * middle_point_y

        max_point_x = scale * max_point_x
        max_point_y = scale * max_point_y



        p1 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [farest_point_x, farest_point_y], self.depth[farest_point_y][farest_point_x])
        p2 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [middle_point_x, middle_point_y], self.depth[middle_point_y][middle_point_x])
        p3 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [max_point_x, max_point_y], self.depth[max_point_y][max_point_x])

        #print('p1', p1, 'p2', p2, 'p3', p3)
        # print(p1)

        return p1,p2,p3


    def getObjectBoxes(self):
        import pyrealsense2 as rs
        # Import Numpy for easy array manipulation
        import numpy as np
        # Import OpenCV for easy image rendering
        import cv2
        rs_intrinsics_obj, depth_image, depth_scale=self.intrinsics,self.depth,.001 #Need to change real sense code to get scale
        clipping_distance_in_meters = 4 #3 meter
        clipping_distance = clipping_distance_in_meters / depth_scale

        # pixels further than clipping_distance to grey

        #No scaling necessary because already scaled
        middle_distance_in_meters=depth_image[420][200] #Depth of white line, 480 x 640 (y,x)

        print("Depth of pixel near middle of image:", middle_distance_in_meters)
        # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image > clipping_distance) | (depth_image <=middle_distance_in_meters+.1/.001), 0, 255) #middle_depth/depth_scale, yield 255(white) if inside range, objects need to be white colored
        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


        array = np.array(bg_removed, dtype='uint8')

        img_cv = cv2.resize(array,(640,480))



        ret, thresh = cv2.threshold(img_cv, 200, 255, 0)

        # find contours
        coins_contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # make copy of image

        # find contours of large enough area
        min_coin_area = 1500
        large_contours = [cnt for cnt in coins_contours if cv2.contourArea(cnt) > min_coin_area]

        # print number of contours
        print('number of objects: %d' % len(large_contours))

        # bounding_img = np.copy(depth_colormap)
        bounding_boxes=[]
        

        # for each contour find bounding box and draw rectangle
        for contour in large_contours:
            x, y, w, h = cv2.boundingRect(contour)
            center=(x+w//2,y+h//2)
            x_range=(center[0]-w//4,center[0]+w//4)
            y_range=(center[1]-h//4,center[1]+h//4)
            d=np.average(depth_image[y_range[0]:y_range[1],x_range[0]:x_range[1]])
            middle_position = rs.rs2_deproject_pixel_to_point(rs_intrinsics_obj,[x+w//2,y+h//2],d)
            pos=[middle_position[0],middle_position[2]]
            bounding_boxes.append(pos)
        return bounding_boxes
        


    def imget_components(self,labels):
        label_hue = np.uint8(179*labels/np.max(labels))
        blank_ch = 255*np.ones_like(label_hue)
        labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])
        labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)
        labeled_img[label_hue==0] = 0
        return labeled_img

    def U_PD(self,x_ref,y_ref):
        j = 0 #radius calc ofset
        interval = 1 #radius calc length
        x_ref_for_radius = [x_ref[j+interval],x_ref[j+interval*2]]
        y_ref_for_radius = [y_ref[j+interval],y_ref[j+interval*2]]
        x_ref_for_radius = np.append(x_ref[j],x_ref_for_radius)
        y_ref_for_radius = np.append(y_ref[j],y_ref_for_radius)

        x1 = x_ref_for_radius[0]
        x2 = x_ref_for_radius[1]
        x3 = x_ref_for_radius[2]
        y1 = y_ref_for_radius[0]
        y2 = y_ref_for_radius[1]
        y3 = y_ref_for_radius[2]
        ma = (y2-y1)/(x2-x1)
        mb = (y3-y2)/(x3-x2)

        if (abs(y2-y1<1e-5))or(abs(y3-y2<1e-5)):
            y2 = y2 + 1e-3
            ma = (y2-y1)/(x2-x1)
            mb = (y3-y2)/(x3-x2)
        x_c = (ma*mb*(y1-y3)+mb*(x1+x2)-ma*(x2+x3))/(2*(mb-ma))
        y_c = (-1/ma)*(x_c-(x1+x2)/2)+(y1+y2)/2
        Radius = sqrt(pow((x2-x_c),2)+pow((y2-y_c),2))
        #print(Radius)

        

        self.Kp = 5
        self.StrPWMThresh = .75
        self.error = self.mid_x
        self.Str = self.Kp * self.error
        if self.Str > self.StrPWMThresh:
          self.Str = self.StrPWMThresh
        elif self.Str < -self.StrPWMThresh:
          self.Str = -self.StrPWMThresh

        if abs(self.Str) > .4:
            self.MotorPWM = self.MotorPWM*(2/5)*(1/self.Str)
            if self.MotorPWM < .117:
                self.MotorPWM = .117
            elif self.MotorPWM > .132:
                self.MotorPWM = .132
    
    def MPC(self,x_ref,y_ref,v_ref):
        ######initializing blocks for MPC######

        dt=self.dt
        lr = 0.13 #dist cg to rear
        lf = 0.13 #dist cg to front

        lam=1
        lfh=1
        theta=0
    
        x_0=np.array([0,0,0,v_ref]).transpose()
        h_val=np.array([0,0])
        dh_val=np.array([0,0])
        x_opt=np.zeros((4,4),dtype=float)
        u_opt=np.zeros((2,3),dtype=float)
    
        Q=np.array([[1000,0,0,0],[0,1000,0,0],[0,0,1,0],[0,0,0,.01]])
        R=np.array(([2., 0.],[0., 2.]),dtype=float)
        Pt=Q
    
        #gets dims of square varrays to populate constraint matrices
        dimsQ=Q.shape[0]  #retvurn dim of square q
        dimsR=R.shape[0]  #retvurn dim of square r
        dimsPt=Pt.shape[0] #return dim of square P for the terminal cost matrix
    
        G=np.zeros((30,22), dtype=float)
    
        h=np.zeros(30,dtype=float)
    
        H=np.zeros((16,22), dtype=float)
        b=np.zeros(16,dtype=float).reshape(16,)
    
        temp=np.identity(dimsQ)
        H0_1=np.zeros((4*dimsQ,4*dimsQ))
        H0_1[0:dimsQ*4,0:dimsQ*4]=block_diag(temp,temp,temp,temp)
    
        H0_2=np.zeros((4,6),dtype=float)

        P=block_diag(Q, Q, Q, Pt, R, R, R)

        q_temp=np.zeros(22,dtype=float).reshape(22,1)

        soln=np.zeros(22,dtype=float)

#############needs to happen in the loop#############################
        Ac = np.matrix([[0, 0, -v_ref*sin(theta),cos(theta)],[0, 0, v_ref*cos(theta),sin(theta)],[0, 0, 0,0],[0, 0, 0,0]])*dt+np.eye(4)
        Bc = np.matrix([[0,0],[0,0],[1,0],[0,1]])*dt    
        
        #bookkeeping and calculations for q array
        q_temp[dimsQ:2*dimsQ]=np.array([x_ref[0],y_ref[0],0,0]).reshape(4,1)
        q_temp[2*dimsQ:3*dimsQ]=np.array([x_ref[1],y_ref[1],0,0]).reshape(4,1)
        q_temp[3*dimsQ:4*dimsQ]=np.array([x_ref[2],y_ref[2],0,0]).reshape(4,1)
        #
        Q_affine=block_diag(Q,Q,Q,Pt,np.zeros((2,2)),np.zeros((2,2)),np.zeros((2,2)))
        
        #calculate final q array
        q=-2*np.matmul(q_temp.transpose(),Q_affine).reshape(22,)
        
        #setup equality constraint matrix, only accounts for dynamics atm
        H0_1[dimsQ:2*dimsQ,0:dimsQ]=-1*Ac
        H0_1[2*dimsQ:3*dimsQ,dimsQ:2*dimsQ]=-1*Ac
        H0_1[3*dimsQ:4*dimsQ,2*dimsQ:3*dimsQ]=-1*Ac 
        H0_3=block_diag(-1*Bc,-1*Bc,-1*Bc)
        H0_4=np.concatenate((H0_2,H0_3), axis=0)
        
        b[0:dimsQ]=x_0
        #calculate final H matrix
        H=np.concatenate((H0_1,H0_4), axis=1)
        #initialize equality constraint matrix
    
        #declare G matrix
        #figure out why this won't compile
                    #first block is theta constraints 
        G[0:30,:]=np.array([[0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0],
                    #acceleration constraints
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1],
                    #x,y state constraints
                    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0]])
        print(G)
        temp2=v_ref/lr*tan(0.52359877559)
        h[0:30]=np.array([temp2,temp2,temp2,temp2,temp2,temp2,temp2,temp2,0.01,0.01,0.01,0.01,0.01,0.01,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50])
        print(h)

        #add control barrier constrait
        #G[16:17]=level_sets
        #h=np.array([-lfh-lam*h])

        #control barrier constraints 
        #import level_sets.py
        #class 
        # print(P.shape)
        # print(q.shape)
        # print(H)
        #print(b)
        # print(H.shape)
        # print(h.shape)
        #dt1=time()

        #run QP
        soln=solve_qp(2*P,q,G,h,H,b,initvals=None,solver='quadprog')
        #dt2=time()
        #dt=dt2-dt1
        #print('quadprog',dt)
        
        #print(qpsolvers.available_solvers)
        inputs=soln[15:-1]
        # print(inputs)
        
        #QP is formuated in terms of omega, covert input omega to delta, steering angle
        inputs[0]=atan2(inputs[0]*lr,v_ref)
        inputs[2]=atan2(inputs[2]*lr,v_ref)
        inputs[4]=atan2(inputs[4]*lr,v_ref)
        #print(inputs)
        soln[15:-1]=inputs
        # print(soln)
        
        #split solution into arrays of optimal x states and inputs
        x_opt[0:4,0]=soln[0:4].transpose()
        x_opt[0:4,1]=soln[4:8].transpose()
        x_opt[0:4,2]=soln[8:12].transpose()
        x_opt[0:4,3]=soln[12:16].transpose()
        u_opt[0:2,0]=soln[16:18].transpose()
        u_opt[0:2,1]=soln[18:20].transpose()
        u_opt[0:2,2]=soln[20:22].transpose()
        
        #print(x_opt)
        #print(u_opt)
        #dt2=time()
        #dt=dt2-dt1
        #print(dt)
        #convert accelerations into velocity commands, rate of change of theta into steering angle command
        return x_opt, u_opt
    def nothing(self,x):
        pass

    def update(self):
        pass