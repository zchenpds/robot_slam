#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class BillController{
private:
    Client client;
    ros::NodeHandle nh;
    ros::Subscriber sub_cmd_vel;
    ros::Publisher pub_cmd_vel;
    tf::TransformListener listener;
    geometry_msgs::Pose target_pose;
    double target_distance;
    int seq;
    double dist_lower_limit, dist_upper_limit1, dist_upper_limit2;
    double robot_max_vel_x, gain;
    ros::Time last_update_time;

public:

    BillController()
        : client("/bill/move_base")
        , seq(0)
        , last_update_time(0)
        , sub_cmd_vel(nh.subscribe("/bill/cmd_vel", 1, &BillController::cmd_vel_callback, this))
        , pub_cmd_vel(nh.advertise<geometry_msgs::Twist>("/bill/cmd_vel_scaled", 1))
    {
        nh.param("/dist_lower_limit", dist_lower_limit, 1.5);
        nh.param("/dist_upper_limit1", dist_upper_limit1, 2.5);
        nh.param("/dist_upper_limit2", dist_upper_limit2, 3.0);
        bool ret;
        ret = nh.param("/move_base/TrajectoryPlannerROS/max_vel_x", robot_max_vel_x, 0.85);
        robot_max_vel_x *= 0.50;
        ROS_INFO("Found robot_max_vel_x to be %.3f", robot_max_vel_x);
        ROS_INFO("bill_controller is waiting for server......");
        client.waitForServer();
        ROS_INFO("bill_controller is connected to the server!");
    }

    void cmd_vel_callback(geometry_msgs::Twist msg) {
        msg.angular.z *= gain;
        msg.linear.x *= gain;
        pub_cmd_vel.publish(msg);
    }

    void update() {

        ros::Time now = ros::Time::now();
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.seq = seq++;
        goal.target_pose.header.stamp = now;
        goal.target_pose.pose = target_pose;
        
        // /bill/move_base/TrajectoryPlannerROS/max_vel_x
        // 
        static int bill_max_vel_x_choice_1;
        int bill_max_vel_x_choice;
        // Hysteresis
        if (target_distance < dist_lower_limit) {
            bill_max_vel_x_choice = 3; // cancel goal
        }
        else if (target_distance < dist_upper_limit1) {
            gain = (target_distance - dist_lower_limit) / (dist_upper_limit1 - dist_lower_limit);
            bill_max_vel_x_choice = 1; // send goal
        }
        else if (target_distance > dist_upper_limit2) {
            bill_max_vel_x_choice = 2; // send goal
        }
        else if (bill_max_vel_x_choice_1 == 0) {
            // Initilization
            bill_max_vel_x_choice = 1;
        }
        
        if (bill_max_vel_x_choice == 3) {
            // cancel goal
            if (bill_max_vel_x_choice_1 != 3 && bill_max_vel_x_choice_1 != 0) {
                client.cancelGoal();
                ROS_INFO("The goal of bill is canceled.");
            }
        }
        else {
            static int count;
            count++;
            if (count >= 5) {
                count -= 5;
                client.sendGoal(goal);
                ROS_INFO("The goal of bill is updated.");
                
                // If the choice changes, interpret it.
                if (bill_max_vel_x_choice != bill_max_vel_x_choice_1) {
                    double bill_max_vel;
                    if (bill_max_vel_x_choice == 1)
                        bill_max_vel = robot_max_vel_x;
                    else // (bill_max_vel_x_choice == 2)
                        bill_max_vel = robot_max_vel_x * 1.1;
                    std::string command_prefix = "rosrun dynamic_reconfigure dynparam set "
                        "/bill/move_base/TrajectoryPlannerROS max_vel_x ";
                    std::ostringstream ss;
                    ss << bill_max_vel;
                    system((command_prefix + ss.str()).c_str());
                    ROS_INFO("New max_vel_x has been set to %.3f for Bill.", bill_max_vel);
                }
            }
        }
        bill_max_vel_x_choice_1 = bill_max_vel_x_choice;
        
    }

    void setTarget() {

        tf::StampedTransform target_tf1, target_tf2;
        try {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), target_tf1);
            listener.lookupTransform("/base_link", "/bill_link", ros::Time(0), target_tf2);
        }
        catch (tf::TransformException ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::TransformStamped target_transformStamped;
        tf::transformStampedTFToMsg(target_tf1, target_transformStamped);
        target_pose.position.x = target_transformStamped.transform.translation.x;
        target_pose.position.y = target_transformStamped.transform.translation.y;
        target_pose.position.z = target_transformStamped.transform.translation.z;
        target_pose.orientation.w = target_transformStamped.transform.rotation.w;
        target_pose.orientation.x = target_transformStamped.transform.rotation.x;
        target_pose.orientation.y = target_transformStamped.transform.rotation.y;
        target_pose.orientation.z = target_transformStamped.transform.rotation.z;

        tf::transformStampedTFToMsg(target_tf2, target_transformStamped);
        double x = target_transformStamped.transform.translation.x;
        double y = target_transformStamped.transform.translation.y;
        target_distance = pow(pow(x, 2) + pow(y, 2), 0.5);

        ROS_DEBUG("\n\tPosition: x: %.3f, y: %.3f. \n\tOrientation: z: %.3f", 
            target_transformStamped.transform.translation.x, 
            target_transformStamped.transform.translation.y,
            target_transformStamped.transform.rotation.z);
    }

    void spin() {
        ros::Rate rate(5.0);
        while (nh.ok()) {
            setTarget();
            update();
            ros::spinOnce();
            rate.sleep();
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "bill_controller_client");
    BillController bc;    
    bc.spin();
    return 0;
}