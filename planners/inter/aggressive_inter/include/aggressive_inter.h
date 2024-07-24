#ifndef AGGRESSIVE_INTER_H_
#define AGGRESSIVE_INTER_H_

#include <ros/ros.h>
#include <mbf_costmap_core/costmap_inter.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <aggressive_inter/AggressiveInterConfig.h>
#include "../../inter_util/include/inter_util.h"
#include <pedsim_msgs/AgentStates.h>

#include <thread>

namespace aggressive_inter
{

    class AggressiveInter : public mbf_costmap_core::CostmapInter
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
        // mutexes
        boost::mutex plan_mtx_;
        boost::mutex speed_mtx_;

        // storage for setPlan
        std::vector<geometry_msgs::PoseStamped> plan_;

        // could be used for nh
        std::string name;
        std::string node_namespace_; 

        ros::NodeHandle nh_;  

        // default values 
        // change in AggressiveInter.cfg to your preference
        double slowdown_distance = 5.0;
        double max_speed_ = 2;
        double danger_threshold = 0.6;

        // variables to control the speed
        double speed_;
        double last_speed_;
        std::thread velocity_thread_;
        
        ros::Subscriber subscriber_;
        ros::Publisher dangerPublisher;  
        ros::ServiceClient setParametersClient_;

        double max_vel_x_param_;

        dynamic_reconfigure::Reconfigure reconfig_;
        dynamic_reconfigure::DoubleParameter double_param_;
        dynamic_reconfigure::Config conf_;
        std::vector<inter_util::SimAgentInfo> simAgentInfos;

        /**
         * @brief Sets new maximum velocity in x direction for the robot
         * @param new_max_vel_x new max velocity in x direction, has to be greater than penality_epsilon (defined in teb_local_planner_params)
        */
        void reconfigure(aggressive_inter::AggressiveInterConfig &config, uint32_t level);
        void semanticCallback(const pedsim_msgs::AgentStates::ConstPtr& message);
        void setMaxVelocityThread();
    };
}

#endif // aggressive_inter_H_