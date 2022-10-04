#!/usr/bin/env python

import rospy
import mazeSolve
rospy.init_node('maze_module')
ms = mazeSolve.solve_start()

if __name__ == "__main__":
    ms.find_destination()
