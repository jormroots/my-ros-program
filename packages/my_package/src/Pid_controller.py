class PID_Controller():
    def __init__(self, Kp, Ki, Kd, I, rospy_rate):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.I = I
        self.delta_t = rospy_rate

    def apply_controller(self, car, err, last_error, ):        
        if err == 0:
            car.forward()
        else:
            P = err
            self.I = (self.I + err) * self.delta_t
            self.I = max(min(self.I, 0.2), -0.2)
            D = (err - last_error) / self.delta_t
            PID = (self.Kp * P) + (self.Ki * self.I) + (self.Kd * D)

            car.speed_right_wheel = car.velocity + (2.5 * PID)
            car.speed_left_wheel = car.velocity - (2.5 * PID)
