from cruise_control import branching_off_ahead
import time
from vehicle_behavior import behavior

class PID_Controller():
    def __init__(self, Kp, Ki, Kd, I, rospy_rate):
        self.Kp = Kp
        print("Kp is", self.Kp)
        self.Ki = Ki
        self.Kd = Kd
        self.I = I
        self.delta_t = rospy_rate

    def apply_controller(self, car, err, last_error):
        if err == 0:
            car.forward()

        #elif delayed_turn == True:
         #   time.sleep(3)
          #  for i in range(20):
           #     time.sleep(0.1)
            #    self.speed_right_wheel = self.velocity * 0
             #   self.speed_left_wheel = self.velocity

        else:
            P = err
            self.I = (self.I + err) * self.delta_t
            self.I = max(min(self.I, 0.2), -0.2)  # 2, -2
            D = (err - last_error) / self.delta_t
            PID = (self.Kp * P) + (self.Ki * self.I) + (self.Kd * D)
            """print("PID:", PID)
            print("P * self.Kp is:", P * self.Kp)
            print("self.Ki * self.I * self.delta_t is:",
                  self.Ki * self.I * self.delta_t)
            #print("self.Kd * D / self.delta_t is:", self.Kd * D / self.delta_t)
            print("self.Kd * D / self.delta_t is:", self.Kd * D / self.delta_t)"""

            car.speed_right_wheel = car.velocity + (2.5 * PID) # PID oli ilma 1.5-ta
            car.speed_left_wheel = car.velocity - (2.5 * PID) # PID oli ilma 1.5-ta

            """  if car.velocity + PID >= 0:
                car.speed_right_wheel = car.velocity + PID
            else:
                car.speed_right_wheel = 0
            if car.velocity - PID >= 0:
                car.speed_left_wheel = car.velocity - PID
            else:
                car.speed_left_wheel = 0 """

# https://github.com/duckietown/mooc-exercises/blob/c476a67fea9836beab99eed23cfc1a83c77af7bb/modcon/solution/05-PID-Control/SOLUTION-PID_controller.ipynb
# plot drawing