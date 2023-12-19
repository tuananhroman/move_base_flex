// polite_inter.h
#ifndef POLITE_INTER_H_
#define POLITE_INTER_H_

#include <ros/ros.h>
#include <mbf_costmap_core/costmap_inter.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <polite_inter/PoliteInterConfig.h>
#include <costmap_2d/SemanticDump.h>
#include <costmap_2d/GetSemanticInfo.h>
#include <costmap_2d/GetDump.h>  // Add this include for the service definition

namespace polite_inter
{

class PoliteInter : public mbf_costmap_core::CostmapInter
{

    using mbf_costmap_core::CostmapInter::CostmapInter;

public:
    uint32_t makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan,
                      double &cost,
                      std::string &message) override;

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan,
                 const std::vector<costmap_2d::SemanticDump> &semantic_info);

    void initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros,
                    costmap_2d::Costmap2DROS *local_costmap_ros) override;

    bool getSemanticInfo(costmap_2d::GetSemanticInfo::Request &req,
                         costmap_2d::GetSemanticInfo::Response &res);

private:
    dynamic_reconfigure::Server<polite_inter::PoliteInterConfig> server;
    std::vector<geometry_msgs::PoseStamped> plan_;
    std::vector<costmap_2d::SemanticDump> semantic_info_;
    boost::mutex plan_mtx_;
    boost::mutex vision_cfg_mtx_;
    double vision_limit_ = 0.0;
    size_t min_poses_ = 0;

    void reconfigure(polite_inter::PoliteInterConfig &config, uint32_t level);
};

} // namespace polite_inter

#endif // POLITE_INTER_H_
