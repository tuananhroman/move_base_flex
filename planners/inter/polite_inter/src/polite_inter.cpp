#include "../include/polite_inter.h"
#include <pluginlib/class_list_macros.hpp>
#include <mbf_costmap_core/mbf_costmap_core.h> // Include the mbf_costmap_core header
#include <ros/service.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>

namespace polite_inter {

uint32_t PoliteInter::makePlan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan,
                               double &cost, std::string &message) {
  // Extract information from the semantic layer service
  costmap_2d::GetDump srv;
  if (!ros::service::call("/semantic_layer_namespace/get_dump", srv)) {
    ROS_ERROR("Failed to call semantic layer service");
    return mbf_costmap_core::INTERNAL_ERROR;
  }

  // Process the semantic layer information
  const auto &semanticLayers = srv.response.semantic_layers;
  for (const auto &semanticLayer : semanticLayers.layers) {
    // Extract information relevant to your logic from semanticLayer
    // Implement your logic to determine if an object is 2 meters or nearer
    // You may need to consider the transformation between the semantic layer and the robot's frame.
    // For simplicity, let's assume there is a function getObjectDistance() that extracts the distance.
    double objectDistance = getObjectDistance(semanticLayer);

    if (objectDistance <= 2.0) {
      // Stop the robot
      plan.clear();
      cost = 0.0;
      message = "Obstacle detected within 2 meters. Stopping the robot.";
      return mbf_costmap_core::SUCCESS;
    }
  }

  // Your existing planning logic here...

  // Populate the plan with a simple straight line for demonstration purposes.
  plan.clear();
  plan.push_back(start);
  plan.push_back(goal);

  cost = 0.0; // Set the cost as needed

  return mbf_costmap_core::SUCCESS;
}

double PoliteInter::getObjectDistance(const pedsim_msgs::SemanticData &semanticLayer) {
  // Implement the logic to extract distance information from semanticLayer
  // Return the distance value
  // Example: return semanticLayer.distance;
  return 0.0; // Replace with actual logic
}

bool PoliteInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  boost::unique_lock<boost::mutex> lock(plan_mtx_);
  plan_ = plan;
  return true;
}

void PoliteInter::initialize(std::string name,
                              costmap_2d::Costmap2DROS *global_costmap_ros,
                              costmap_2d::Costmap2DROS *local_costmap_ros) {
  this->name = name;

  dynamic_reconfigure::Server<polite_inter::PoliteInterConfig> server;
  server.setCallback(boost::bind(&PoliteInter::reconfigure, this, _1, _2));
}

void PoliteInter::reconfigure(polite_inter::PoliteInterConfig &config,
                               uint32_t level) {
  boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
  vision_limit_ = config.vision_limit;
  min_poses_ = config.min_poses;
}

}  // namespace polite_inter

PLUGINLIB_EXPORT_CLASS(polite_inter::PoliteInter, mbf_costmap_core::CostmapInter)
