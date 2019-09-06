#See if we should even run the pilot module. 
#This is only needed because the part run_condition only accepts boolean
class LQRController:


    def __init__(self):
        self.throttle = 0
        self.str = 0
        self.running = True

    def run_threaded(self,img1,img2,acl_x, acl_y,acl_z,gyr_x, gyr_y, gyr_z):
      #  self.img_arr_a = img_arr_a
   #     self.img_arr_b = img_arr_b

        '''
        process E-Stop state machine
        '''
      #  print(img_arr_a)
      #  print(acl_x)
        self.Kp = 1
        self.ControlPWMThresh = .12
        self.acl_desired = .1
        self.error = self.acl_desired - acl_z
        self.MotorPWM = self.Kp * self.error
        if self.MotorPWM > self.ControlPWMThresh:
          self.MotorPWM = self.ControlPWMThresh
        elif self.MotorPWM < -self.ControlPWMThresh:
          self.MotorPWM = -self.ControlPWMThresh


        # return 0,0,'user',True
        return 0,self.MotorPWM,'user',True


        
    def update(self):
        pass

