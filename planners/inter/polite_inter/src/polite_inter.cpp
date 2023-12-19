#include "../include/polite_inter.h"
#include <costmap_2d/semantic_layer.h>
#include <costmap_2d/GetDump.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polite_inter::PoliteInter, mbf_costmap_core::CostmapInter)

namespace polite_inter
{

    const uint32_t SUCCESS = 0;
    const uint32_t INTERNAL_ERROR = 1;

    uint32_t PoliteInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                  std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        // Call the GetDump service to obtain semantic layers
        costmap_2d::GetDump getDump_srv;

        // Call the GetDump service
        if (get_dump_client_.call(getDump_srv))
        {
            ROS_INFO("We called the service");

            // Access the semantic layers from the response
            auto semantic_layers = getDump_srv.response.semantic_layers;

            // Process the semantic layers
            for (const auto &semantic_layer : semantic_layers)
            {
                ROS_INFO("Semantic Layer Names:");

                // Iterate through the layers
                for (const auto &layer : semantic_layer.layers)
                {
                    ROS_INFO_STREAM("Layer Name: " << layer);

                    // Iterate through the points in each layer
                    for (const auto &point : layer.points)
                    {
                        ROS_INFO_STREAM("Location: " << "x: " << point.location.x << ", y: " << point.location.y << ", z: " << point.location.z);
                    }
                }
            }

            // Use semantic_layers data to modify the plan as needed
            // ...

            // Lock the mutexes for plan_ and vision_cfg_mtx_
            boost::unique_lock<boost::mutex> lock2(plan_mtx_);
            boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);

            // Modify the plan based on the semantic_layers data
            size_t limit = std::floor(plan_.size() * vision_limit_);
            limit = std::max(limit, min_poses_);
            limit = std::min(limit, plan_.size());

            plan = std::vector<geometry_msgs::PoseStamped>(plan_.begin(), plan_.begin() + limit);

            // Unlock the mutexes
            lock.unlock();
            lock2.unlock();

            // Return the result code
            return 0;
        }
        else
        {
            ROS_ERROR("Failed to call GetDump service");
            // Handle the error and return an appropriate result code
            return polite_inter::INTERNAL_ERROR;;
        }
    }

    bool PoliteInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        plan_ = plan;
        return true;
    }

    void PoliteInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;

        // Create a service client for the GetDump service
        get_dump_client_ = ros::NodeHandle("~").serviceClient<costmap_2d::GetDump>("get_dump");

        dynamic_reconfigure::Server<polite_inter::PoliteInterConfig> server;
        server.setCallback(boost::bind(&PoliteInter::reconfigure, this, _1, _2));
    }

    void PoliteInter::reconfigure(polite_inter::PoliteInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
        vision_limit_ = config.vision_limit;
        min_poses_ = config.min_poses;
    }

}