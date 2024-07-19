#ifndef SPACIAL_HORIZON_INTER_H_
#define SPACIAL_HORIZON_INTER_H_

#include <ros/ros.h>
#include <mbf_costmap_core/costmap_inter.h>
#include <boost/thread/mutex.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/server.h>
#include <spacial_horizon_inter/SpacialHorizonInterConfig.h>

#define BASE_TOPIC_GOAL "move_base_simple/goal"
#define BASE_TOPIC_ODOM "odom"
#define BASE_TOPIC_SUBGOAL "current_subgoal"
#define BASE_TOPIC_GLOBAL_PLAN "global_plan"
#define SERVICE_GLOBAL_PLANNER "move_base_flex/NavfnROS/make_plan"

namespace spacial_horizon_inter
{

    class SpacialHorizonInter : public mbf_costmap_core::CostmapInter
    {

        using mbf_costmap_core::CostmapInter::CostmapInter;

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the inter
         * @param cost The cost for the the plan
         * @param message Optional more detailed outcome as a string
         * @return Result code as described on GetInterPath action result:
         *         SUCCESS         = 0
         *         1..9 are reserved as plugin specific non-error results
         *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
         *         CANCELED        = 51
         *         INVALID_START   = 52
         *         INVALID_GOAL    = 53
         *         BLOCKED_START   = 54
         *         BLOCKED_GOAL    = 55
         *         NO_PATH_FOUND   = 56
         *         PAT_EXCEEDED    = 57
         *         EMPTY_PATH      = 58
         *         TF_ERROR        = 59
         *         NOT_INITIALIZED = 60
         *         INVALID_PLUGIN  = 61
         *         INTERNAL_ERROR  = 62
         *         71..99 are reserved as plugin specific errors
         */
        uint32_t makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                          std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message);

        /**
         * @brief  Set the plan that the planner is following
         * @param plan The plan to pass to the inter
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief Requests the inter to cancel, e.g. if it takes too much time.
         * @remark New on MBF API
         * @return True if a cancel has been successfully requested, false if not implemented.
         */
        bool cancel() { return false; };

        /**
         * @brief Initialization function for the CostmapInter
         * @param name The name of this inter
         * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros);

    private:
        // storage for setPlan
        std::vector<geometry_msgs::PoseStamped> plan_;
        boost::mutex plan_mtx_;

        // could be used for nh
        ros::NodeHandle nh_; // ROS node handle
        boost::mutex cfg_mtx_;

        // publisher
        ros::Publisher pub_subgoal, pub_global_plan;
        // subscriber
        ros::Subscriber sub_goal, sub_odom;
        // service
        ros::ServiceClient global_planner_srv;
        // timer
        ros::Timer subgoal_timer, update_global_plan_timer;

        // dynamic reconfigure params
        double goal_tolerance = 0.3;
        double planning_horizon = 5.0;
        double subgoal_tolerance = 0.5;
        double subgoal_publish_rate = 5.0;

        // vector of goal and odom
        Eigen::Vector2d end_pos, odom_pos, odom_vel;

        // flags
        bool called_make_plan = false;
        bool has_goal = false;
        bool has_odom = false;

        // dynamic reconfigure
        boost::shared_ptr< dynamic_reconfigure::Server<SpacialHorizonInterConfig> > dynamic_recfg_;

        void initializeGlobalPlanningService();

        bool getSubgoal(Eigen::Vector2d &subgoal);
        void updateSubgoal();
        void updateSubgoalCallback(const ros::TimerEvent &e);

        /* get global plan from move base */
        void getGlobalPath();
        void getGlobalPath(const ros::TimerEvent &e);
        void fillPathRequest(nav_msgs::GetPlan::Request &request);
        void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv, std::vector<geometry_msgs::PoseStamped> &plan);

        void odomCallback(const nav_msgs::OdometryConstPtr &msg);
        void goalCallback(const geometry_msgs::PoseStampedPtr &msg);

        void reconfigure(spacial_horizon_inter::SpacialHorizonInterConfig &config, uint32_t level);
    };
}

#endif