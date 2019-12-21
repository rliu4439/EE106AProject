# See if we should even run the pilot module.
# This is only needed because the part run_condition only accepts boolean

import time


class LQRController:

    def __init__(self):
        self.throttle = 0
        self.str = 0
        self.running = True
        self.start = int(round(time.time() * 1000))
        self.t1 = 3
        self.t2 = 5
        self.t3 = 8

    def trac(self, MotorPWM, vF, vR, vB):

        self.Ktp = 10
        self.slipF = (vF-vB)/vF
        self.slipR = (vR-vB)/vR
        if (self.slipF > 0.25):
            self.slipE = self.slipF - 0.1
            MotorPWM = MotorPWM - self.slipE*self.Ktp
        elif (self.slipR > 0.25):
            self.slipE = self.slipR - 0.1
            MotorPWM = MotorPWM - self.slipE*self.Ktp

        return MotorPWM


"""
    def run_threaded(self, img1, img2, acl_x, acl_y, acl_z, gyr_x, gyr_y, gyr_z):
      #  self.img_arr_a = img_arr_a
      # self.img_arr_b = img_arr_b
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
        return 0, self.MotorPWM, 'user', True
"""

   def run_threaded(self, vF, vR, vB):

        self.curr_time = int(round(time.time() * 1000))
        if (self.curr_time < self.start + self.t1*1000):
            self.MotorPWM = 0.5*(self.curr_time-self.start)
            self.SteeringPWM = 0
        # elif (curr_time > self.start + t1*1000 & & curr_time < self.start + t2*1000):
        #     self.MotorPWM = 0.5*t1
        #     self.SteeringPWM = 100
        # elif (curr_time > self.start + t2*1000):
        #     self.MotorPWM = 0.5*(curr_time - t3*1000)
        # else
        # self.MotorPWM = 0
        # self.steeringPWM = 0

        self.MotorPWM = trac(self, self.MotorPWM, vF, vR, vB)

        return self.SteeringPWM, self.MotorPWM, 'user', True

    def update(self):
        pass
