import re
import time
from helper_functions import int_to_bitblock
import statistics
import rospy

def cruise_control(error, last_error, read, target, pid_controller, car):
    bits_block, indices = int_to_bitblock(read)
    #print('bits_block is', bits_block)
    branching_off_ahead(bits_block, car)
    #stopper(bits_block, car)

    if len(indices) != 0:
        last_error = error
        error = target - statistics.mean(indices)
        pid_controller.apply_controller(car, error, last_error)


def branching_off_ahead(binary, car):  # '00110011
    print('binary is ', binary)
    m = re.search('^1+1?0+0?1+1?0+$', binary)
    turn_left = ['11011000', '01101100', '00110110', '00110100', '01100100',
                 '11101100', '11001100', '11000100', '11100110', '01100010']
    #if binary == '11011000' or binary == '01101100' or binary == '00110110' or binary == '00110100' or binary == '01100100' or binary == '11101100' 
    #or binary == '11001100' or binary == '11000100' or binary == '11100110' or binary == '01100010' or binary == '':
    if binary in turn_left:
        t = 0
        while t < 2:
            time.sleep(0.5)
            car.speed_right_wheel = 0.1
            car.speed_left_wheel = -0.1
            time.sleep(0.2)
            print("t = ", t)
            t += 1

    else:
        print("all is ok")

"""     if m:
        if car.branching_off_first_detection == False:
            car.branching_off_first_detection = False
            print('match:', m[0])
        else:
            car.branching_off_confirmed = True
            # if binary[]

            # '10011000'
            # '00010010'
             
            car.turn_
        print("--first detection:", car.branching_off_first_detection,
              "\n-- detection confirmed:", car.branching_off_confirmed)
    else:
        print('nope') """

    # reads two zeros between ones

    # if branching off ahead, remember what side the sign was on
    # to define, what side it was on, see which line continues and which doesn't
    # the one that doesn't is the direction of the turn ahead
    # remember the side to turn to - global variable like a switch True

    # need another function that determines if branching is detected

    # if branching is detected, turn to the according side (left/right)
    # after turning, switch the variable back to False