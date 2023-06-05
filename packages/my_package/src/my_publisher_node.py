#!/usr/bin/env python3
import rospy
import time
from cruise_control import cruise_control
from Car import Car
from Pid_controller import PID_Controller
from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range

sparkfun_device_address = 62
sparkfun_registry_address = 17
target_sensor_position = 4.5
vehicle_speed = 0.2
rospy_rate = 60
Kp = 0.1
Ki = 0.004
Kd = 0.22
I = 0

emergentcy = False

speed = WheelsCmdStamped()
error = 0
last_error = 0
car = Car(vehicle_speed)
pid_controller = PID_Controller(Kp, Ki, Kd, I, rospy_rate)

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.sub = rospy.Subscriber(
            "/weirdbot/front_center_tof_driver_node/range", Range, self.obstacle_callback)
        #self.sub = rospy.Subscriber(
         #   "/weirdbot/left_wheel_encoder_node/tick", WheelEncoderStamped, self.tick_callback)

    def obstacle_callback(self, data):
        global tof_distance
        tof_distance = int(round(data.range * 100))
        print("TOF NOW:", tof_distance)
        print("car.obstacle:", car.obstacle_ahead)
        if tof_distance < 30:
            print('I am close: %s cm', tof_distance)
            car.obstacle_ahead = True
            #self.drive_around()
        else:
            car.obstacle_ahead = False

    #def tick_callback(self, data):
     #   print("TICKS", data.data)

    def on_shutdown(self):
        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def stopper(self, binary):
        v = 0
        while v < 2:
            time.sleep(0.17)
            v += 1
            if car.turn == True:
                if binary == '00000000':
                    car.speed_right_wheel = 0
                    car.speed_left_wheel = 0
                    self.pub.publish(speed)
                    print("Halt in the name of the bot")
                else:
                    print("Magnus did everything wrong")
            elif binary == '00000000':
                car.speed_right_wheel = 0.4
                car.speed_left_wheel = -0.3
                speed.vel_right = car.speed_right_wheel
                speed.vel_left = car.speed_left_wheel
                self.pub.publish(speed)
                time.sleep(0.1)
                print("Second life")
                car.turn = True
            else:
                print("Not today")

    def turn(self, binary):
        if binary == '11011000':
            car.speed_right_wheel = 0.2
            car.speed_left_wheel = 0.07
            speed.vel_right = car.speed_right_wheel
            speed.vel_left = car.speed_left_wheel
            self.pub.publish(speed)
            time.sleep(1.8)
            print("Ragnorok")
                #time.sleep(1)
        else:
            print("Terra")

    def forward(self):
        car.speed_right_wheel = 0.3
        car.speed_left_wheel = 0.3
        self.pub.publish(speed)
        time.sleep(2)

    def right(self):
        car.speed_right_wheel = 0.3
        car.speed_left_wheel = 0
        self.pub.publish(speed)
        time.sleep(0.5)

    def left(self):
        car.speed_right_wheel = 0
        car.speed_left_wheel = 0.3
        self.pub.publish(speed)
        time.sleep(0.5)


    def simple_track(self):
        global error
        global last_error
        rate = rospy.Rate(rospy_rate)
        while not rospy.is_shutdown():
            #self.check_obstacle_ahead() 
            bus = SMBus(1)
            read = bus.read_byte_data(
                sparkfun_device_address, sparkfun_registry_address)
            binary = bin(read)[2:].zfill(8)

            """ if binary == '00000000':
                self.stopper(binary) """
            """ elif binary == '11011000':
                    self.turn(binary) """
            if car.drive_around:
                print('passing')
                # Execute drive_around actions
                car.speed_right_wheel = 0
                car.speed_left_wheel = 0.5
                self.pub.publish(speed)
                rospy.sleep(0.5)

                car.speed_right_wheel = 0.25
                car.speed_left_wheel = 0.1
                self.pub.publish(speed)
                rospy.sleep(0.5)

                car.drive_around = False
                car.obstacle_ahead = False
            else:
                print("in cruise_control")
                cruise_control(error, last_error, read, target_sensor_position, pid_controller, car)

            if not car.drive_around:
                if car.obstacle_ahead:
                    print("Obstacle detected, driving around")
                    #self.drive_around()
                    continue
            
            speed.vel_right = car.speed_right_wheel
            speed.vel_left = car.speed_left_wheel
            self.pub.publish(speed)
            rate.sleep()
            bus.close()
    
    def check_obstacle_ahead(self):
        if car.obstacle_ahead:
            #time.sleep(0.5)
            if car.obstacle_ahead:
                print("Obstacle still ahead")
                car.drive_around = True
                return True
            return False
                
    """ def drive_around(self):
        print('in drive around')
        self.check_obstacle_ahead()
        x = 1

        if car.drive_around == True:
            
            while x < 2:
                print('in while')
                # drive around
                print('STARTING DRIVE AROUND')
                car.speed_right_wheel = 0
                car.speed_left_wheel = 0.5
                self.pub.publish(speed)
                #time.sleep(2)
                    
                car.speed_right_wheel = 0.25
                car.speed_left_wheel = 0.1
                self.pub.publish(speed)
                #time.sleep(0.5)
                car.drive_around = False
                car.obstacle_ahead = False
                x += 1
          """   
    def run(self):
        #print('obstacle ahead:', car.obstacle_ahead)
        self.simple_track()

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()