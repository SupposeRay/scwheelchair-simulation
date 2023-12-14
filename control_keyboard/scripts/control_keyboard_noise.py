#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
from geometry_msgs.msg import Point
import sys, select, termios, tty

msg = """
Control mbot with Gaussian noise!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
r/v : increase/decrease standard deviation by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

noiseBindings={
        'r':(1.1),
        'v':(0.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 1.0
turn = 1.0
mean = 0
sigma = 0.1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def gnoise(mean,sigma):
    return "current noise:\tmean %s\tsigma %s " % (mean,sigma)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('control_keyboard')
    pub = rospy.Publisher('/keyboard', Point, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.2
    dec = 0.2
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        print gnoise(mean, sigma)
        while(1):
            key = getKey()

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  
                turn = turn * speedBindings[key][1]    
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            
            elif key in noiseBindings.keys():
                sigma = sigma * noiseBindings[key]   
                count = 0

                print gnoise(mean, sigma)

            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break


            target_speed = speed * (x + random.gauss(mean,sigma))
            target_turn = turn * (th + random.gauss(mean,sigma))
            # target_speed = speed * x
            # target_turn = turn * th


            if target_speed > control_speed:
                control_speed = min( min (target_speed, 1.0), control_speed + acc )
            elif target_speed < control_speed:
                control_speed = max( max (target_speed, -1.0), control_speed - dec )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( min (target_turn, 1.0), control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( max (target_turn, -1.0), control_turn - 0.1 )
            else:
                control_turn = target_turn


            Output = Point()
            Output.x = control_speed
            Output.y = control_turn
            Output.z = 0
            pub.publish(Output)

    except:
        print ("Error")

    finally:
        Output = Point()
        Output.x = 0; Output.y = 0; Output.z = 0
        pub.publish(Output)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
