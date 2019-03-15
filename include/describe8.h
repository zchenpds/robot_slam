#ifndef NAVFN_NAVFN_ROS_H_
#define NAVFN_NAVFN_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace robot_slam {
    class Describe8 : public nav_core::BaseGlobalPlanner  {
    public:
        /** default constructor
         * @brief  Default constructor for the Describe8 object
         */
        Describe8();

        /**
         * @brief  Constructor for the Describe8 object
         * @param  name The name of this planner
         * @param  costmap A pointer to the ROS wrapper of the costmap to use
         */
        Describe8(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief  Constructor for the Describe8 object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  global_frame The global frame of the costmap
         */
        Describe8(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

        /** default destructor
         * @brief  Default destructor for the Describe8 object
         */
        ~Describe8();

        /**
         * @brief  Initialization function for the Describe8 object
         * @param  name The name of this planner
         * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief  Initialization function for the Describe8 object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use for planning
         * @param  global_frame The global frame of the costmap
         */
        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose 
         * @param goal The goal pose 
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    protected:
        costmap_2d::Costmap2D* costmap_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        bool initialized_, allow_unknown_;
        ros::Publisher plan_pub_;
    private:
        std::string global_frame_;
        std::string tf_prefix_;
    };
}


#endif