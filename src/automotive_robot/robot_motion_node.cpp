//#include "robot_motion_node.hpp"
#include "automotive_robot/Path.h"
#include "automotive_robot/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <math.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

//#define CLOSE_MV_SPEED 0.05                              // square_unit/s
#define STABLE_ANGULAR_SPEED ((double(20) / 180) * M_PI) // radian/s
#define CLOSE_ANGULAR_SPEED ((double(10) / 180) * M_PI)  // radian/s
#define MV_ANGULAR_SPEED ((double(7) / 180) * M_PI)      // radian/s

using namespace std;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

ros::Publisher velocity_publisher;
ros::Publisher moveFinished_publisher;
geometry_msgs::Twist vel_msg;
automotive_robot::Point cpos;
double odomAngleXaxis = 0;
// double c_pos[2] = {0, 0};

const double stable_rotate_tolerance = 0.2;
const int n = 6;

void move(double distance, automotive_robot::Point &nextPt);
void rotate(double relative_angle, double direction);
void moveCallback(const automotive_robot::Path::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

// bool volatile flag_check_angle = true;

int main(int argc, char **argv) {
  // Initiate new ROS node named "talker"
  ros::init(argc, argv, "robot_motion");
  ros::NodeHandle n;

  velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  moveFinished_publisher = n.advertise<std_msgs::String>("update_vision", 1000);

  //	/turtle1/cmd_vel is the Topic name
  //	/geometry_msgs::Twist is the msg type
  ROS_INFO("\n\n\n ******** START *********\n");

  ros::Subscriber sub = n.subscribe("cmd_moving", 1000, moveCallback);
  ros::Subscriber Odometry_sub = n.subscribe("odom", 1000, odomCallback);
  std_msgs::String triggered_msg;
  triggered_msg.data = "done";
  // for(int i = 0; i < 10; i++){
  ROS_INFO("\n\n\n Before sleep\n");
  sleep(20);
  ROS_INFO("\n\n\n Now Starting!\n");
  moveFinished_publisher.publish(triggered_msg);
  //}
  ros::MultiThreadedSpinner Spinner(2);

  Spinner.spin();
  ros::waitForShutdown();

  return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  odomAngleXaxis = yaw;
  cpos.x = msg->pose.pose.position.x;
  cpos.y = msg->pose.pose.position.y;
}

// angle of p1p2 against Ox   
double Direction(const volatile automotive_robot::Point &p1, const automotive_robot::Point &p2) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  return atan2(dy, dx);
}

const float MAX_MV_SPEED = 0.18;      // square_unit/s
const float deceleration_time = 3000; // miliseconds
const float acceleration_time = 3000; // miliseconds
const float stable_moving_tolerance =(MAX_MV_SPEED * deceleration_time / 2000) * (MAX_MV_SPEED * deceleration_time / 2000);
const int smooth_level = 20;
const int small_acceleration_duration = acceleration_time / smooth_level;
const int small_deceleration_duration = deceleration_time / smooth_level;
const float vel_change = MAX_MV_SPEED / smooth_level;
const float close_rotate_tolerance = ((double(0.5) / 180) * M_PI);
const int small_rotate_time = (close_rotate_tolerance / MV_ANGULAR_SPEED) * 1000;

void move(double distance, automotive_robot::Point &next_point) {

    ROS_INFO("Moving start!");
    vel_msg.linear.x = vel_change;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    velocity_publisher.publish(vel_msg);

    auto start =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch())
            .count();
    auto temp = start;
    auto now = start;
    auto temp_temp = start;
    int not_use = 0;
    bool is_rotating = false;
    int small_acceleration_duration_modification = small_acceleration_duration;
    string debug;
    start -= small_acceleration_duration; // bc we move the robot immadiately
    do {
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch())
                .count();
        double direction = Direction(cpos, next_point);
        double current = odomAngleXaxis - direction;
        if (now - temp >= small_acceleration_duration_modification) {
        vel_msg.linear.x += vel_change;
        velocity_publisher.publish(vel_msg);
        cout << "Initial: " << (current / M_PI * 180)
            << " Velocity: " << vel_msg.linear.x << endl;
        cout << "Time: " << now - temp << endl;
        small_acceleration_duration_modification =
            small_acceleration_duration -
            (now - temp - small_acceleration_duration_modification);
        cout << "Modification: " << small_acceleration_duration_modification
            << endl
            << endl;
        temp = now; // mark temp as soon as possible
        }
        if (abs(current) > close_rotate_tolerance) {
        vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
        velocity_publisher.publish(vel_msg);
        temp_temp = now; // mark the time the robot starts rotating
        // Opps code
        if (!is_rotating) {
            cout << "Opps!!!" << endl;
        }

        is_rotating = true;
        } else if (is_rotating && now - temp_temp >= small_rotate_time) {
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
        is_rotating = false;
        }
    } while (now - start <= acceleration_time &&
            (cpos.x - next_point.x) * (cpos.x - next_point.x) +
                    (cpos.y - next_point.y) * (cpos.y - next_point.y) >
                stable_moving_tolerance);

    vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum bc velocity
                                    // might not be maximum
    vel_msg.angular.z =
        0; // force stop rotating bc the robot might still be rotating
    velocity_publisher.publish(vel_msg);
    is_rotating = false;
    temp_temp =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch())
            .count(); // used for cout

    while ((cpos.x - next_point.x) * (cpos.x - next_point.x) +
                (cpos.y - next_point.y) * (cpos.y - next_point.y) >
            stable_moving_tolerance) {
        double direction = Direction(cpos, next_point);
        double current = odomAngleXaxis - direction;
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch())
                .count();
        if (abs(current) > close_rotate_tolerance) {
        vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
        velocity_publisher.publish(vel_msg);
        temp = now; // mark the time the robot starts rotating
        // Opps code
        if (!is_rotating) {
            cout << "Opps!!!" << endl;
        }

        is_rotating = true;
        } else if (is_rotating && now - temp >= small_rotate_time) {
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
        is_rotating = false;
        }
        if (now - temp_temp >=
            small_acceleration_duration) { // check angle and velocity while moving
                                        // stably
        cout << "Fast: " << (current / M_PI * 180)
            << " Velocity: " << vel_msg.linear.x << endl;
        temp_temp = now;
        }
    }

    start = duration_cast<milliseconds>(system_clock::now().time_since_epoch())
                .count();
    temp = start;
    now = start;
    start -= small_deceleration_duration;

    vel_msg.linear.x -= vel_change; // force velocity to decrease immediately
    vel_msg.angular.z =
        0; // force stop rotating bc the robot might still be rotating
    velocity_publisher.publish(vel_msg);
    is_rotating = false;
    int small_deceleration_duration_modification = small_deceleration_duration;
    do {
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch())
                .count();
        double direction = Direction(cpos, next_point);
        double current = odomAngleXaxis - direction;
        if (now - temp >= small_deceleration_duration_modification) {
        vel_msg.linear.x -= vel_change;
        velocity_publisher.publish(vel_msg);
        cout << "Decrease: " << (current / M_PI * 180)
            << " Velocity: " << vel_msg.linear.x << endl;
        cout << "Time: " << now - temp << endl;
        small_deceleration_duration_modification =
            small_deceleration_duration -
            (now - temp - small_deceleration_duration_modification);
        cout << "Modification: " << small_deceleration_duration_modification
            << endl
            << endl;
        temp = now; // mark temp as soon as possible
        }
        if (abs(current) > close_rotate_tolerance) {
        vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
        velocity_publisher.publish(vel_msg);
        temp_temp = now; // mark the time the robot starts rotating
        // Opps code
        if (!is_rotating) {
            cout << "Opps!!!" << endl;
        }

        is_rotating = true;
        } else if (is_rotating && now - temp_temp >= small_rotate_time) {
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
        is_rotating = false;
        }
    } while (now - start <= deceleration_time * 0.9);

    vel_msg.angular.z =
        0; // force stop rotating bc the robot might still be rotating
    velocity_publisher.publish(vel_msg);
    is_rotating = false;
    temp_temp =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch())
            .count(); // used for cout

    while ((cpos.x - next_point.x) * (cpos.x - next_point.x) +
                (cpos.y - next_point.y) * (cpos.y - next_point.y) >
            stable_moving_tolerance / 400) {
        double direction = Direction(cpos, next_point);
        double current = odomAngleXaxis - direction;
        if (abs(current) > 1.57) {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
        break;
        }
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch())
                .count();
        if (abs(current) > close_rotate_tolerance) {
        vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
        velocity_publisher.publish(vel_msg);
        temp = now; // mark the time the robot starts rotating
        // Opps code
        if (!is_rotating) {
            cout << "Opps!!!" << endl;
        }

        is_rotating = true;
        } else if (is_rotating && now - temp >= small_rotate_time) {
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
        is_rotating = false;
        }
        if (now - temp_temp >= small_deceleration_duration) {
        cout << "Slow: " << (current / M_PI * 180)
            << " Velocity: " << vel_msg.linear.x << endl;
        temp_temp = now;
        }
    }

    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    ROS_INFO("Moving done!");
}

// rotate clockwise
void rotate(double relative_angle, double direction) {

  const double close_rotate_tolerance = ((double(0.75) / 180) * M_PI);

  // set a random linear velocity in the x-axis
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  // set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = ((relative_angle < 0) ? -1 : 1) * STABLE_ANGULAR_SPEED;

  velocity_publisher.publish(vel_msg);
  while (abs(odomAngleXaxis - direction) > stable_rotate_tolerance) {
    continue;
  }

  vel_msg.angular.z = ((relative_angle < 0) ? -1 : 1) * CLOSE_ANGULAR_SPEED;
  velocity_publisher.publish(vel_msg);
  double before = abs(odomAngleXaxis - direction);
  while (before > close_rotate_tolerance) {
    double after = abs(direction - odomAngleXaxis);
    if (after > before) {
      vel_msg.angular.z = -vel_msg.angular.z;
      velocity_publisher.publish(vel_msg);
    }
    before = after;
    continue;
  }

  vel_msg.angular.z = 0;
  // ROS_INFO("Odom Angle Axis After Rotate: [%f]", odomAngleXaxis / M_PI *
  // 180);
  velocity_publisher.publish(vel_msg);
  usleep(500000);
}

void moveCallback(const automotive_robot::Path::ConstPtr &msg) 
{
    unsigned loops = msg->points.size();
    ROS_INFO("I heard: [%f,%f]", msg->points[loops - 1].x, msg->points[loops - 1].y);

    if(loops == 2){
        automotive_robot::Point nextPt = msg->points[i];
        ROS_INFO("CPOS: [%f : %f]", cpos.x, cpos.y);
        ROS_INFO("Next Point: [%f : %f]", nextPt.x, nextPt.y);
        
        double direction = Direction(cpos, nextPt);
        double angleDiff = direction - odomAngleXaxis;

        if (odomAngleXaxis > 0 && M_PI < (odomAngleXaxis - direction))
        angleDiff = 2 * M_PI - odomAngleXaxis + direction;
        else if (odomAngleXaxis < 0 && M_PI < (-odomAngleXaxis + direction))
        angleDiff = -2 * M_PI + direction + odomAngleXaxis;
        double distance = sqrt(pow(cpos.x - nextPt.x, 2) + pow(cpos.y - nextPt.y, 2) * 1.0);

        rotate(angleDiff, direction);
        move(distance, nextPt);
    }
    
    else {
        vel_msg.linear.x = vel_change;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        // set a random angular velocity in the y-axis
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 0;

        velocity_publisher.publish(vel_msg);

        auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        auto temp = start;
        auto now = start;
        auto temp_temp = start;
        int not_use = 0;
        bool is_rotating = false;
        int small_acceleration_duration_modification = small_acceleration_duration;
        string debug;
        start -= small_acceleration_duration; // bc we move the robot immadiately
        automotive_robot::Point next_point = msg->points[1];
        do {
            now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            double direction = Direction(cpos, next_point);
            double current = odomAngleXaxis - direction;
            if (now - temp >= small_acceleration_duration_modification) {
                vel_msg.linear.x += vel_change;
                velocity_publisher.publish(vel_msg);
                cout << "Initial: " << (current / M_PI * 180)<< " Velocity: " << vel_msg.linear.x << endl;
                cout << "Time: " << now - temp << endl;
                small_acceleration_duration_modification = small_acceleration_duration - (now - temp - small_acceleration_duration_modification);
                cout << "Modification: " << small_acceleration_duration_modification << endl << endl;
                temp = now; // mark temp as soon as possible
            }
            if (abs(current) > close_rotate_tolerance) {
                vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
                velocity_publisher.publish(vel_msg);
                temp_temp = now; // mark the time the robot starts rotating
                // Opps code
                if (!is_rotating) {
                    cout << "Opps!!!" << endl;
                }

                is_rotating = true;
            } 
            else if (is_rotating && now - temp_temp >= small_rotate_time) {
                vel_msg.angular.z = 0;
                velocity_publisher.publish(vel_msg);
                is_rotating = false;
            }
        } while (now - start <= acceleration_time && (cpos.x - next_point.x) * (cpos.x - next_point.x) + (cpos.y - next_point.y) * (cpos.y - next_point.y) > stable_moving_tolerance);

        vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum bc velocity
                                        // might not be maximum
        vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
        velocity_publisher.publish(vel_msg);
        is_rotating = false;


        // stable moving 
        loops--;    // pay attention please
        float start_bending_distance = 0.1; // tam thoi cho nhu vay
        for(unsigned i = 1 ; i < loops; i++){
            next_point = msg->points[i];
            automotive_robot::Point start_bending_point;
            automotive_robot::Point stop_bending_point;
            float vector_point_to_start_bending_point[2] = {msg->points[i - 1].x - msg->points[i].x, msg->points[i - 1].y - msg->points[i].y};
            float vector_point_to_stop_bending_point[2] = {msg->points[i + 1].x - msg->points[i].x, msg->points[i + 1].y - msg->points[i].y};
            
            float distance__start_point = sqrt(vector_point_to_start_bending_point[0]*vector_point_to_start_bending_point[0] 
                + vector_point_to_start_bending_point[1]*vector_point_to_start_bending_point[1]);
            float distance_end_point = sqrt(vector_point_to_stop_bending_point[0]*vector_point_to_stop_bending_point[0] 
                + vector_point_to_stop_bending_point[1]*vector_point_to_stop_bending_point[1]);
            
            vector_point_to_start_bending_point[0] = vector_point_to_start_bending_point[0]/distance__start_point*start_bending_distance;
            vector_point_to_start_bending_point[1] = vector_point_to_start_bending_point[1]/distance__start_point*start_bending_distance;

            vector_point_to_stop_bending_point[0] = vector_point_to_stop_bending_point[0]/distance_end_point*start_bending_distance;
            vector_point_to_stop_bending_point[1] = vector_point_to_stop_bending_point[1]/distance_end_point*start_bending_distance;

            start_bending_point.x = msg->points[i].x + vector_point_to_start_bending_point[0];
            start_bending_point.y = msg->points[i].y + vector_point_to_start_bending_point[1];

            stop_bending_point.x = msg->points[i].x + vector_point_to_stop_bending_point[0];
            stop_bending_point.y = msg->points[i].y + vector_point_to_stop_bending_point[1];

            vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum bc velocity
            // might not be maximum
            vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
            velocity_publisher.publish(vel_msg);
            is_rotating = false;
            temp_temp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); // used for cout
            
            // mai mot nap len robot chinh lai dieu kien while vi` ct khong co bi ngung xung quanh 50ms nua~
            while ((cpos.x - start_bending_point.x) * (cpos.x - start_bending_point.x) + (cpos.y - start_bending_point.y) * (cpos.y - start_bending_point.y) > stable_moving_tolerance/100) {
                double direction = Direction(cpos, next_point);
                double current = odomAngleXaxis - direction;
                now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
                if (abs(current) > close_rotate_tolerance) {
                    vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
                    velocity_publisher.publish(vel_msg);
                    temp = now; // mark the time the robot starts rotating
                    // Opps code
                    if (!is_rotating) {
                        cout << "Opps!!!" << endl;
                    }

                    is_rotating = true;
                } 
                else if (is_rotating && now - temp >= small_rotate_time) {
                    vel_msg.angular.z = 0;
                    velocity_publisher.publish(vel_msg);
                    is_rotating = false;
                }
                if (now - temp_temp >= small_acceleration_duration) { // check angle and velocity while moving// stably
                    cout << "Fast: " << (current / M_PI * 180)<< " Velocity: " << vel_msg.linear.x << endl;
                    temp_temp = now;
                }
            }

            // be huong voi ban kinh cong R
            float alpha = acos((vector_point_to_start_bending_point[0]*vector_point_to_stop_bending_point[0] 
            + vector_point_to_start_bending_point[1]*vector_point_to_stop_bending_point[1]) / (start_bending_distance*start_bending_distance));

            float R = tan(alpha/2)*start_bending_distance;

            vel_msg.angular.z = MAX_MV_SPEED/R;     // v = R*W;
            velocity_publisher.publish(vel_msg);    // start bending
            double direction = Direction(cpos, next_point);
            while(abs(odomAngleXaxis - direction) > close_rotate_tolerance){
                direction = Direction(cpos, next_point);
            }

            vel_msg.angular.z = 0;     
            velocity_publisher.publish(vel_msg);    // stop bending
        }

        is_rotating = false;
        temp_temp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); // used for cout

        while ((cpos.x - next_point.x) * (cpos.x - next_point.x) + (cpos.y - next_point.y) * (cpos.y - next_point.y) > stable_moving_tolerance) {
            double direction = Direction(cpos, next_point);
            double current = odomAngleXaxis - direction;
            now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            if (abs(current) > close_rotate_tolerance) {
            vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
            velocity_publisher.publish(vel_msg);
            temp = now; // mark the time the robot starts rotating
            // Opps code
            if (!is_rotating) {
                cout << "Opps!!!" << endl;
            }

            is_rotating = true;
            } 
            else if (is_rotating && now - temp >= small_rotate_time) {
                vel_msg.angular.z = 0;
                velocity_publisher.publish(vel_msg);
                is_rotating = false;
            }
            if (now - temp_temp >= small_acceleration_duration) { // check angle and velocity while moving
                                            // stably
                cout << "Fast: " << (current / M_PI * 180) << " Velocity: " << vel_msg.linear.x << endl;
                temp_temp = now;
            }
        }

        start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        temp = start;
        now = start;
        start -= small_deceleration_duration;

        vel_msg.linear.x -= vel_change; // force velocity to decrease immediately
        vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
        velocity_publisher.publish(vel_msg);
        is_rotating = false;
        int small_deceleration_duration_modification = small_deceleration_duration;
        do {
            now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            double direction = Direction(cpos, next_point);
            double current = odomAngleXaxis - direction;
            if (now - temp >= small_deceleration_duration_modification) {
                vel_msg.linear.x -= vel_change;
                velocity_publisher.publish(vel_msg);
                cout << "Decrease: " << (current / M_PI * 180)<< " Velocity: " << vel_msg.linear.x << endl;
                cout << "Time: " << now - temp << endl;
                small_deceleration_duration_modification = small_deceleration_duration - (now - temp - small_deceleration_duration_modification);
                cout << "Modification: " << small_deceleration_duration_modification << endl << endl;
                temp = now; // mark temp as soon as possible
            }
            if (abs(current) > close_rotate_tolerance) {
                vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
                velocity_publisher.publish(vel_msg);
                temp_temp = now; // mark the time the robot starts rotating
                // Opps code
                if (!is_rotating) {
                    cout << "Opps!!!" << endl;
                }

                is_rotating = true;
            } 
            else if (is_rotating && now - temp_temp >= small_rotate_time) {
                vel_msg.angular.z = 0;
                velocity_publisher.publish(vel_msg);
                is_rotating = false;
            }
        } while (now - start <= deceleration_time * 0.9);

        vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
        velocity_publisher.publish(vel_msg);
        is_rotating = false;
        temp_temp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); // used for cout

        while ((cpos.x - next_point.x) * (cpos.x - next_point.x) + (cpos.y - next_point.y) * (cpos.y - next_point.y) > stable_moving_tolerance / 400) {
            double direction = Direction(cpos, next_point);
            double current = odomAngleXaxis - direction;
            if (abs(current) > 1.57) {
                vel_msg.linear.x = 0;
                vel_msg.angular.z = 0;
                velocity_publisher.publish(vel_msg);
                break;
            }
            now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            if (abs(current) > close_rotate_tolerance) {
                vel_msg.angular.z = (current < 0) ? MV_ANGULAR_SPEED : -MV_ANGULAR_SPEED;
                velocity_publisher.publish(vel_msg);
                temp = now; // mark the time the robot starts rotating
                // Opps code
                if (!is_rotating) {
                    cout << "Opps!!!" << endl;
                }

                is_rotating = true;
            } 
            else if (is_rotating && now - temp >= small_rotate_time) {
                vel_msg.angular.z = 0;
                velocity_publisher.publish(vel_msg);
                is_rotating = false;
            }
            if (now - temp_temp >= small_deceleration_duration) {
                cout << "Slow: " << (current / M_PI * 180) << " Velocity: " << vel_msg.linear.x << endl;
                temp_temp = now;
            }
        }

        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
        ROS_INFO("Moving done!");

    }
    
    std_msgs::String done_msg;
    done_msg.data = "done";
    moveFinished_publisher.publish(done_msg);
}