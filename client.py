#! /usr/bin/env python

import roslib
roslib.load_manifest('actionlib_tutorials')
import rospy
import actionlib
import actionlib_tutorials.msg


def Turtle_client():
    client = actionlib.SimpleActionClient('turtle_dance', actionlib_tutorials.msg.OddPairAction)
    client.wait_for_server()
    goal = actionlib_tutorials.msg.OddPairGoal(n = 1)
    client.send_goal(goal)
    print client.get_state()
    while(client.get_state()!=10):
        print client.get_state()
        client.wait_for_result(rospy.Duration.from_sec(10))
        print "Esperando ..."
	return client.get_result()

if __name__ == '__main__':
    rospy.init_node('turtle_client')
    result = Turtle_client()
    print result.oddpair
    # if result.oddpair == 1:
    # 	print "el numero es par"
    # else:
    # 	print "el numero es impar"
    # Fill in the goal here
