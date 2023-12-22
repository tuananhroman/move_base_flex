#include "../include/polite_inter.h"
#include <costmap_2d/semantic_layer.h>
#include <costmap_2d/GetDump.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polite_inter::PoliteInter, mbf_costmap_core::CostmapInter)

namespace polite_inter
{

    ros::ServiceClient get_dump_client_;
    const uint32_t SUCCESS = 0;
    const uint32_t INTERNAL_ERROR = 1;

    uint32_t PoliteInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        // Create a request message
        //costmap_2d::GetDump::Request request;
        // No need to set any specific fields in the request for this example

        // Create a response message
        //costmap_2d::GetDump::Response response;

        costmap_2d::GetDump srv;

        ROS_ERROR("Calling GetDump service...");

        // Call the GetDump service
        if (get_dump_client_.call(srv))
        {
            ROS_ERROR("GetDump service call successful");

            // Access the semantic layers from the response
            auto semantic_layers = srv.response.semantic_layers;

            // Process the semantic layers
            for (const auto &semantic_layer : semantic_layers)
            {
                ROS_ERROR("Semantic Layer Names:");

                // Iterate through the layers
                for (const auto &layer : semantic_layer.layers)
                {
                    ROS_ERROR("Layer Name: %s", layer.type.c_str());

                    // Iterate through the points in each layer
                    for (const auto &point : layer.points)
                    {
                        ROS_ERROR("Location: x: %f, y: %f, z: %f", point.location.x, point.location.y, point.location.z);
                    }
                }
            }

            // Use semantic_layers data to modify the plan as needed

            // Lock the mutexes for plan_ and vision_cfg_mtx_
            boost::unique_lock<boost::mutex> lock2(plan_mtx_);
            boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);

            // Modify the plan based on the semantic_layers data
            size_t limit = std::floor(plan_.size() * vision_limit_);
            limit = std::max(limit, min_poses_);
            limit = std::min(limit, plan_.size());

            plan = std::vector<geometry_msgs::PoseStamped>(plan_.begin(), plan_.begin() + limit);

            // Return the result code
            return 0;
        }
        else
        {
            ROS_ERROR("Failed to call GetDump service");
            // Handle the error and return an appropriate result code
            return polite_inter::INTERNAL_ERROR;
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
        get_dump_client_ = ros::NodeHandle("~").serviceClient<costmap_2d::GetDump>("global_costmap/get_dump");
    
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