// polite_inter.cpp
#include "../include/polite_inter.h"
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/costmap_2d_publisher.h>

PLUGINLIB_EXPORT_CLASS(polite_inter::PoliteInter, mbf_costmap_core::CostmapInter)

namespace polite_inter
{
uint32_t PoliteInter::makePlan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan,
                               double &cost,
                               std::string &message)
{
    boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
    boost::unique_lock<boost::mutex> lock2(plan_mtx_);

    // Check for obstacles in the semantic layer within 2 meters of the robot
    for (const auto &semantic : semantic_info_)
    {
        double distance = std::hypot(start.pose.position.x - semantic.x, start.pose.position.y - semantic.y);

        if (distance <= 2.0)
        {
            // Obstacle within 2 meters, stop the robot
            plan.clear();
            return 0; // Success, but the plan is empty
        }
    }

    // If no obstacle within 2 meters, proceed with the original behavior
    size_t limit = std::floor(plan_.size() * vision_limit_);
    limit = std::max(limit, min_poses_);
    limit = std::min(limit, plan_.size());

    plan = std::vector<geometry_msgs::PoseStamped>(plan_.begin(), plan_.begin() + limit);

    return 0;
}

bool PoliteInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan,
                          const std::vector<costmap_2d::SemanticDump> &semantic_info)
{
    boost::unique_lock<boost::mutex> lock(plan_mtx_);
    plan_ = plan;
    semantic_info_ = semantic_info;
    return true;
}

void PoliteInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros,
                             costmap_2d::Costmap2DROS *local_costmap_ros)
{
    this->name = name;

    dynamic_reconfigure::Server<polite_inter::PoliteInterConfig> server;
    server.setCallback(boost::bind(&PoliteInter::reconfigure, this, _1, _2));

    // Add a service server for getting semantic information
    ros::NodeHandle nh("~");
    ros::ServiceServer service = nh.advertiseService("get_semantic_info", &PoliteInter::getSemanticInfo, this);
}

bool PoliteInter::getSemanticInfo(costmap_2d::GetSemanticInfo::Request &req,
                                  costmap_2d::GetSemanticInfo::Response &res)
{
    costmap_2d::GetDump srv;
    ros::NodeHandle nh("~");

    // Assuming semantic layer service is named "semantic_layer/get_dump"
    ros::ServiceClient client = nh.serviceClient<costmap_2d::GetDump>("semantic_layer/get_dump");

    if (client.call(srv))
    {
        res.semantic_info = srv.response.semantic_layers;

        // Additional processing if needed

        return true;
    }
    else
    {
        ROS_ERROR("Failed to call semantic layer service");
        return false;
    }
}
void PoliteInter::reconfigure(polite_inter::PoliteInterConfig &config, uint32_t level)
{
    boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
    vision_limit_ = config.vision_limit;
    min_poses_ = config.min_poses;
}

} // namespace polite_inter
