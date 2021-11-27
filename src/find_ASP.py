#!/usr/bin/env python

pub = rospy.Publisher('cmd_moving', String, queue_size=1000)
# visited path 
visited_path = []

def find_ASP_callBack(&msg):
    if not msg->s_goal:
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
    
    if print_traversalSights:
            print("traversal_sights:", traversal_sights)

    asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sights, robot_vision)
    # recode visited path
    visited_path.append(asp)

def main():
    print(__file__ + " start!!")

    rospy.init_node('find_ASP', anonymous=True)

    rospy.Subscriber("calculate_ASP", String, find_ASP_callBack)

    rospy.spin()

if __name__ == '__main__':
    main()