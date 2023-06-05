class Car():
    def __init__(self, velocity):
        self.velocity = velocity
        self.speed_right_wheel = velocity
        self.speed_left_wheel = velocity
        self.branching_off_first_detection = False
        self.branching_off_confirmed = False
        self.turn_at_next_left = True
        self.turn_at_next_right = True
        self.obstacle_ahead = False
        self.drive_around = False
        self.turn = False

    def forward(self):
        self.speed_left_wheel = self.velocity
        self.speed_right_wheel = self.speed_left_wheel
