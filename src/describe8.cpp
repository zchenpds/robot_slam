#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include "describe8.h"

namespace robot_slam {
    Describe8::Describe8()
        : costmap_(NULL), costmap_ros_(NULL), initialized_(false), allow_unknown_(true) {
        
    }

    Describe8::Describe8(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
        : costmap_(NULL), costmap_ros_(NULL), initialized_(false), allow_unknown_(true) {
        //initialize the planner
        initialize(name, costmap_ros);
    }

    Describe8::Describe8(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
        : costmap_(NULL), costmap_ros_(NULL), initialized_(false), allow_unknown_(true) {
        //initialize the planner
        initialize(name, costmap, global_frame);
    }
    Describe8::~Describe8() {
    }

    void Describe8::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){
        if (!initialized_) {
            ROS_INFO("Describe8 is being initialized!!!!");
            costmap_ = costmap;
            global_frame_ = global_frame;

            ros::NodeHandle private_nh("~/" + name);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            

            private_nh.param("allow_unknown", allow_unknown_, true);
            //private_nh.param("planner_window_x", planner_window_x_, 0.0);
            //private_nh.param("planner_window_y", planner_window_y_, 0.0);
            //private_nh.param("default_tolerance", default_tolerance_, 0.0);

            //get the tf prefix
            ros::NodeHandle prefix_nh;
            tf_prefix_ = tf::getPrefixParam(prefix_nh);

            //make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

            initialized_ = true;

            if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
                ros::console::notifyLoggerLevelsChanged();
            }
        }
        else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }

    void Describe8::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        costmap_ros_ = costmap_ros;
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    bool Describe8::makePlan(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        ROS_INFO("Describe8 is making a plan!!!!");
        if (!initialized_) {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        costmap_ = costmap_ros_->getCostmap();

        if (goal.header.frame_id != global_frame_) {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        tf::Stamped<tf::Pose> goal_tf;
        tf::Stamped<tf::Pose> start_tf;

        poseStampedMsgToTF(goal,goal_tf);
        poseStampedMsgToTF(start,start_tf);

        double useless_pitch, useless_roll, goal_yaw, start_yaw;
        start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
        goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

        //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;

        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;
        double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

        double target_x = goal_x;
        double target_y = goal_y;
        double target_yaw = goal_yaw;

        bool done = false;
        double scale = 1.0;
        double dScale = 0.01;
        
        plan.push_back(start);
        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

        new_goal.pose.position.x = target_x;
        new_goal.pose.position.y = target_y;

        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();

        plan.push_back(new_goal);

        int reps = 1;
        double half_dist_circle = 2.0;
        double radius = 1.0;
        // Check validity of the parameters;
        if (radius > half_dist_circle) {
            ROS_WARN("Parameter radius cannot exceed half_dist_circle! Setting radius to half_dist_circle.");
            radius = half_dist_circle;
        }
        double alpha = acos(radius / half_dist_circle);
        double straight_part_len = radius / tan(alpha);
        double turn_angle = 2 * (M_PI - alpha);
        double curved_part_len = radius * turn_angle;
        double delta_d = straight_part_len / 10; // Delta displacement on the straight parts of the path
        double delta_alpha = alpha / 20;
        int seq = new_goal.header.seq;
        for (int i=0; i<reps; i++) {
            double distance_traveled = 0;
            double angle_turned = 0;
            for (int part=0; part<5; seq++) {
                target_x += delta_d * cos(target_yaw);
                target_y += delta_d * sin(target_yaw);
                switch (part) {
                case 0: // First straight part
                    // target_yaw remains unchanged
                    distance_traveled += delta_d;
                    if (distance_traveled >= straight_part_len) {
                        part++;
                        distance_traveled = 0;
                    }
                    break;
                case 1: // First curved part
                    target_yaw -= delta_alpha;
                    angle_turned += delta_alpha;
                    if (angle_turned >= turn_angle) {
                        part++;
                        angle_turned = 0;
                    }
                    break;
                case 2: // Second straight part
                    // target_yaw remains unchanged
                    distance_traveled += delta_d;
                    if (distance_traveled >= 2 * straight_part_len) {
                        part++;
                        distance_traveled = 0;
                    }
                    break;
                case 3:
                    target_yaw += delta_alpha;
                    angle_turned += delta_alpha;
                    if (angle_turned >= turn_angle) {
                        part++;
                        angle_turned = 0;
                    }
                case 4:
                    // target_yaw remains unchanged
                    distance_traveled += delta_d;
                    if (distance_traveled >= straight_part_len) {
                        part++;
                        distance_traveled = 0;
                    }
                    break;
                }

                geometry_msgs::PoseStamped new_goal = goal;
                tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);
                ros::Duration duration(0.1*seq);
                new_goal.header.seq = seq;
                new_goal.header.stamp += duration; 
                new_goal.pose.position.x = target_x;
                new_goal.pose.position.y = target_y;

                new_goal.pose.orientation.x = goal_quat.x();
                new_goal.pose.orientation.y = goal_quat.y();
                new_goal.pose.orientation.z = goal_quat.z();
                new_goal.pose.orientation.w = goal_quat.w();

                plan.push_back(new_goal);
            }

        }

        



        done = true;
        return (done);
    }
}




//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(robot_slam::Describe8, nav_core::BaseGlobalPlanner)