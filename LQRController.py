#See if we should even run the pilot module. 
#This is only needed because the part run_condition only accepts boolean
class LQRController:


    def __init__(self):
        self.throttle = 0
        self.str = 0
        self.running = True

    def run_threaded(self,img1,img2):
      #  self.img_arr_a = img_arr_a
   #     self.img_arr_b = img_arr_b

        '''
        process E-Stop state machine
        '''
      #  print(img_arr_a)
        return 0.3,0.3,'user',True


        
    def update(self):
        pass

