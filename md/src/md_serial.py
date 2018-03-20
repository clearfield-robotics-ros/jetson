#!/usr/bin/env python

import serial
import sys
import datetime
import time
import rospy
from std_msgs.msg import Int16

start = time.time()


def hex_to_signed(source):
    """Convert a string hex value to a signed hexidecimal value.

    This assumes that source is the proper length, and the sign bit
    is the first bit in the first byte of the correct length.

    hex_to_signed("F") should return -1.
    hex_to_signed("0F") should return 15.
    """
    if not isinstance(source, str):
        raise ValueError("string type required")
    if 0 == len(source):
        raise ValueError("string is empty")
    sign_bit_mask = 1 << (len(source)*4-1)
    other_bits_mask = sign_bit_mask - 1
    value = int(source, 16)
    return -(value & sign_bit_mask) | (value & other_bits_mask)


def log_data(max_val):
    # used on 3/2 for grid signal measurements
    # now = datetime.datetime.now()
    elapsed_time = str(time.time()-start)
    x = sys.argv[1]
    y = sys.argv[2]
    with open(str(x)+'_'+str(y)+'.txt', "a+") as data_file:
        data_file.write(elapsed_time+','+str(max_val)+'\n')
        data_file.close()


def main():
    rospy.init_node('metal_detector_interface')
    pub = rospy.Publisher("md_signal", Int16, queue_size=10)

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    one_off_commands = 'tone spkr off\r'
    sent_one_off = False
    regular_commands = ''
    r = rospy.Rate(200)  # 100 Hz
    while not rospy.is_shutdown():
        ser.write(regular_commands)
        potentials = []
        vals = ser.read(10)
        ords = [ord(val) for val in vals]
        for i, ord_val in enumerate(ords):
            if ord_val == 3:
                try:
                    if ords[i+5] == 3:
                        # print i, ords
                        potentials.append(ords[i+1:i+5])
                except IndexError:
                    pass
        if len(potentials) == 1:
            four_vals = potentials[0]
            v1 = four_vals[1] * 256 + four_vals[0] if four_vals[1] < 128 \
                else -((255 - four_vals[1]) * 256 + (255 - four_vals[0]) + 1)
            v2 = four_vals[3] * 256 + four_vals[2] if four_vals[3] < 128 \
                else -((255 - four_vals[3]) * 256 + (255 - four_vals[2]) + 1)
            max_val = max(abs(v1), abs(v2))
            print(max_val)
            pub.publish(Int16(max_val))
            if not sent_one_off:
                sent_one_off = True
                ser.write(one_off_commands)
            # log_data(max_val)
        '''
        threes = [(ind, val) for ind, val in enumerate(ords) if val == 3]
        if len(threes) == 2:
            if threes[1][0] - threes[0][0] == 5:
                four_vals = ords[threes[0][0]+1:threes[0][0]+5]
                v1 = four_vals[1] * 256 + four_vals[0] if four_vals[1] < 128 \
                    else -((255 - four_vals[1]) * 256 + (255 - four_vals[0]) + 1)
                v2 = four_vals[3] * 256 + four_vals[2] if four_vals[3] < 128 \
                    else -((255 - four_vals[3]) * 256 + (255 - four_vals[2]) + 1)
                max_val = max(abs(v1), abs(v2))
                # print(max_val)
                # log_data(max_val)
        '''
        r.sleep()


if __name__ == "__main__":
    main()
