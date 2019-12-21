# See if we should even run the pilot module.
# This is only needed because the part run_condition only accepts boolean

import time
import serial
import numpy as np
import csv


class TracController:

    def __init__(self):
        self.throttle = 0
        self.str = 0
        self.running = True
        self.start = int(round(time.time() * 1000))
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.vels = np.array([0, 0, 0, 0, 0])
        self.t1 = 1
        self.t2 = 5
        self.t3 = 8
        self.Ktp = 2.5
        self.file = open('latestTrac.csv', "w")
        self.writer = csv.writer(self.file, dialect='excel')
        self.writer.writerow(["vB vF vR"])

    def getwheelvel(self):

        vel_wheel = self.ser.readline()
        vel_wheel = str(vel_wheel)
        vel_wheel = vel_wheel[2:][:-5]
        vel_wheel = vel_wheel.split()
        # print(vel_wheel)
        try:
            self.vels = np.array(vel_wheel, dtype=np.float32)
        except:
            pass
        return 0

    def trac(self):

        if (self.slipF > 0.15 or self.slipF < 0.05):
            self.slipE = self.slipF - 0.1
            self.MotorPWM = self.MotorPWM - self.slipE*self.Ktp
        elif (self.slipR > 0.15 or self.slipR < 0.05):
            self.slipE = self.slipR - 0.1
            self.MotorPWM = self.MotorPWM - self.slipE*self.Ktp

        if (self.MotorPWM > 0.4):
            self.MotorPWM = 0.4
        elif (self.MotorPWM < 0):
            self.MotorPWM = 0

        return 0

    def run_threaded(self):
        self.getwheelvel()
        self.curr_time = int(round(time.time() * 1000))
        if (self.curr_time < self.start + 5*1000):
            self.MotorPWM = 0.4
            # print(self.vels[0])
            self.SteeringPWM = 0
        else:
            self.MotorPWM = 0
            self.SteeringPWM = 0
            if (self.curr_time > self.start + 10*1000):
                self.start = int(round(time.time() * 1000))

        vels = self.vels
        fl = vels[0]
        fr = vels[1]
        rl = vels[2]
        rr = vels[3]
        vB = vels[4]
        vF = (fl + fr)/2
        vR = (rr+rl)/2

        self.slipF = (vF-vB)/vF
        self.slipR = (vR-vB)/vR

        data = str(vB) + " " + str(vF) + " " + str(vR)
        self.writer.writerow([data])

        if (self.MotorPWM != 0):
            self.trac()

        print(self.MotorPWM)

        return self.SteeringPWM, self.MotorPWM, 'user', True

    def update(self):
        pass
