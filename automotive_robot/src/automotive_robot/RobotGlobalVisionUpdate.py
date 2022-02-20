#!/usr/bin/env python
import os
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import rospy
import threading
from std_msgs.msg import String
from nav_msgs.msg import Odometry


from automotive_robot.Robot_lib import intersection
from automotive_robot.Robot_paths_lib import *
from automotive_robot.Robot_draw_lib import *
from automotive_robot.Robot_sight_lib import *
from automotive_robot.Robot_map_lib import *
from automotive_robot.Robot_world_lib import *
from automotive_robot.Robot_csv_lib import *
from automotive_robot.Robot_goal_lib import *
from automotive_robot.Program_config import *
from automotive_robot.Robot_control_panel import *

from automotive_robot.msg import Point
from automotive_robot.msg import Path

config = Config()

pub = rospy.Publisher('cmd_moving', Path, queue_size=1000)

def convert_position_from_slam_to_csv(position):
    position[0] = (position[0])*20 +  200
    position[1] = -position[1]*20 +  184
    return (position[0],position[1])

def convert_position_from_csv_to_slam(position):
    position[0] = (position[0] -  200)/20
    position[1] = -(position[1] -  184)/20
    return (position[0],position[1])

# set configuration of robot
config.robot_type = RobotType.circle
robot_vision = config.robot_vision

# set same window size to capture pictures
plt.figure(figsize=(6, 6))

# get user input
menu_result = menu()
goal =convert_position_from_slam_to_csv([menu_result.gx, menu_result.gy])

no_way_to_goal = False  # there is no way to reach goal

# traversal sights to draw visible visited places
traversal_sights = []

# global active open points
g_active_open_pts = []

# visibility Graph which contains information of visited places
visibility_graph = graph_intiailze() #

# visited path 
visited_path = []

def updateGlobalVisionCallBack(msg):
    print("message received: " + msg.data)
    cmd = 'rosrun map_server map_saver -f ./vision'
    os.system(cmd)

    world_name = "vision.pgm"
    # read world map then get obstacles information
    read_map_from_world(world_name)

    global ob

    ob = read_map_csv(world_name + ".csv")

    #tia duoi

    for obstacle in ob:
        i = 0
        obstacle.pop()
        while (i < len(obstacle)):
            if (obstacle[(i + 1)%len(obstacle)] == obstacle[i - 1]):
                obstacle.pop(i)
                if (i == 0):
                    obstacle.pop(-1)
                elif (i == len(obstacle)):
                    obstacle.pop(0)
                    i = i - 2
                else:
                    obstacle.pop(i)
                    i = i - 1
            else:
                i = i + 1

        j = 0
        while (j < len(obstacle)):
            if (belong_line(obstacle[j],[obstacle[j - 1], obstacle[(j + 1)%len(obstacle)]])):
                obstacle.pop(j)
            else:
                j = j + 1
        
        if (len(obstacle) > 0):
            obstacle.append(obstacle[0]) 

    i = 0
    while (i < len(ob)):
        if (len(ob[i]) < 1):
            ob.pop(i)
        else:
            i += 1   

    # global varibles
    global cpos
    global no_way_to_goal
    global g_active_open_pts
    global visibility_graph
    global flag_plot
    global lock
    global ranks_new 
    global skeleton_path
    ranks_new = []
    skeleton_path = []

    # find configure space: ob1 = find_configure_space(ob)
    center = (cpos[0], cpos[1])

    ob = find_configure_space_update_2(ob, config.robot_radius)
    
    #print(ob)

    #input('check update ob')

    # scan to get sights at local
    closed_sights, open_sights = scan_around(center, robot_vision, ob, goal)

    # check if the robot saw or reach the given goal
    r_goal, s_goal = check_goal(center, goal, config, robot_vision, closed_sights)

    # initial local open points as empty
    local_open_pts = []
    if not r_goal and not s_goal:
        # get local open points
        local_open_pts = get_local_open_points(open_sights)

        # check whether local open points are active
        l_active_open_pts = get_active_open_points(local_open_pts, traversal_sights,
                                                    robot_vision, center, goal)
        
        # Ranking new active openPts then stack to global set.
        if len(l_active_open_pts) > 0:
            ranks_new = np.array([ranking(center, pt, goal) for pt in l_active_open_pts])

        # stack local active open point to global set
        g_active_open_pts = store_global_active_points(g_active_open_pts, l_active_open_pts, ranks_new)

        # add new active open points to graph_insert
        graph_add_lOpenPts(visibility_graph, center, l_active_open_pts)

        # pick next point to make a move
        picked_idx, next_pt = pick_next(g_active_open_pts)

        if picked_idx != -1:
            # find the shortest skeleton path from current position (center) to next point
            skeleton_path = BFS_skeleton_path(visibility_graph, tuple(center), tuple(next_pt))

            # then remove picked point from active global open point
            g_active_open_pts = np.delete(g_active_open_pts, picked_idx, axis=0)
        else:
            print("No way to reach the goal!")
            no_way_to_goal = True

    else:
        next_pt = goal
        # find the shortest path from center to next point
        skeleton_path = [center, goal]

    if not r_goal:
        # record the path
        traversal_sights.append([center, closed_sights, open_sights])

        if print_traversalSights:
            print("traversal_sights:", traversal_sights)

        asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sights, robot_vision)
        message = []
        for i in range(len(asp)):
            if i==0 or i == len(asp) - 1:
                pass
            temp = Point()
            temp.x = (asp[i][0]-200)/20
            temp.y = -(asp[i][1]-184)/20
            message.append(temp)
        print("Current {0}, Next {1}".format(center, next_pt))
        print("Message[0] {0}, Message[1] {1}".format(message[0], message[1]))
        print("Done calculate path")
        pub.publish(message)

        # recode visited path
        visited_path.append(asp)

        # make a move from current position
        if not no_way_to_goal:
            cpos = motion(cpos, next_pt)  # simulate robot

        if show_animation:

            # clear plot
            plt.cla()

            ##############################################
            # for stopping simulation with the esc key.
            ##############################################
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            ##############################################
            # draw world and map
            ##############################################
            if show_world and world_name is not None:
                world_display(plt, mpimg, world_name)

            # draw map obstacles 
            if show_map:
                #print(ob)
                if world_name is not None:
                    map_display(plt, world_name + ".csv", ob)
                else:
                    map_display(plt, map_name, ob)

            # show_traversalSights
            if show_traversalSights:
                for local in traversal_sights:
                    lcenter = local[0]  # center of robot at local
                    lc_sight = local[1]  # closed sight at local
                    lo_sight = local[2]  # open sight at local
                    plot_vision(plt, lcenter[0], lcenter[1], robot_vision, lc_sight, lo_sight)

            if show_robot:
                plot_robot(plt, center[0], center[1], 0, config)

            if show_goal:
                plot_goal(plt, goal, r_goal, s_goal)

            # plot robot's vision at local (center)
            plot_vision(plt, center[0], center[1], robot_vision, closed_sights, open_sights)

            if show_local_openpt and len(local_open_pts) > 0:
                plot_points(plt, local_open_pts, ls_lopt)

            if show_active_openpt and len(g_active_open_pts) > 0:
                plot_points(plt, g_active_open_pts, ls_aopt)

            if show_visibilityGraph:
                plot_visibilityGraph(plt, visibility_graph, ls_vg)

            if show_visitedPath:
                plot_paths(plt, visited_path, ls_vp, ls_goingp)

            if show_sketelonPath:
                plot_lines(plt, skeleton_path, ls_sp)

            if show_approximately_shortest_path:
                plot_lines(plt, asp, ls_asp)

            if show_critical_line_segments:
                plot_critical_line_segments(plt, critical_ls, ls_cls)

                # display next point if existing
            if show_next_point:
                if len(next_pt) > 0:
                    plot_point(plt, next_pt, ls_nextpt)

            # to set equal make sure x y axises are same resolution 
            plt.axis("equal")
            plt.grid(True)

        
        cpos = next_pt
        with lock:
            flag_plot = True

    else:
        print("Goal!!")

def getStartPos(msg):
    global cpos
    # current position of robot
    
    global flag
    if not flag:
        cpos = convert_position_from_slam_to_csv([msg.pose.pose.position.x, msg.pose.pose.position.y])
        flag = True



def main():
    
    print(__file__ + " start!!")

    rospy.init_node('robot_global_vision_update', anonymous=True)
    
    rospy.Subscriber("update_vision", String, updateGlobalVisionCallBack)
    
    global flag_plot
    flag_plot = False

    global lock
    lock = threading.Lock()

    global flag
    flag = False

    global goal

    global ob

    sub = rospy.Subscriber("odom", Odometry, getStartPos)
    rospy.loginfo("Done")
    rospy.loginfo("Goal: (%f,%f)",goal[0],goal[1])
    while not rospy.is_shutdown():
        with lock:
            pass
        if flag_plot:
            # map_display(plt, "Binh_test_configure_space", ob)   
            # plt.grid(True)
            # plt.pause(1)
            # plt.show()
            plt.pause(1)
            plt.show()
            flag_plot = False
            
    #rospy.spin()

if __name__ == '__main__':
    main()