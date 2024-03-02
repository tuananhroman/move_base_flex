#include "../include/meta_inter.h"
#include "../../inter_util/include/inter_util.h"

#include <thread>
#include <vector>
#include <std_msgs/Int32.h>
#include <pedsim_msgs/AgentStates.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h> // Added for different colors of intermediate Planners
#include <nav_msgs/Odometry.h>






PLUGINLIB_EXPORT_CLASS(meta_inter::MetaInter, mbf_costmap_core::CostmapInter)

namespace meta_inter
{
    // this current setup for the Meta-Planner only works for
    // the Inter-Planners: Sideways, Polite and Agressive

    uint32_t MetaInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                   std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {



        boost::unique_lock<boost::mutex> plan_lock(plan_mtx_);
        boost::unique_lock<boost::mutex> speed_lock(speed_mtx_);

        double robot_x = start.pose.position.x;
        double robot_y = start.pose.position.y;
        double robot_z = start.pose.position.z;
        double minDistance = INFINITY;
        bool caution = false;
        bool wall_near = false;
        ROS_WARN("%s is our current inter", current_inter_.c_str());

        std::vector<double> distances;
        current_inter_ = "aggressive";



   

        for (const auto &simAgentInfo : simAgentInfos)
        {
            geometry_msgs::Point32 point = simAgentInfo.point;
            std::string agentType = simAgentInfo.type.c_str();
            double distance = std::sqrt(std::pow(point.x - robot_x, 2) + std::pow(point.y - robot_y, 2)) + std::pow(point.z - robot_z, 2);
            minDistance = std::min(minDistance, distance);
            distances.push_back(distance);
            double theta = tf::getYaw(start.pose.orientation);
            //works for polite and sideways, if there are more options, it needs adjustment
            //padding is used on top of robot size to account for minor calculation errors to avoid static obstacles
            double default_padding = (current_inter_ == "polite") ? 0.075 : 0.135;
            //detects if there are walls in the sideways_range and add padding here too to account for error
            bool activateSideways = inter_util::InterUtil::checkStaticObjects(distance, theta, (sideways_range_+default_padding), temp_goal_distance_, robot_radius_, detectedRanges, detectedAngles);

            selectPlanner(distance, agentType, activateSideways);
            if(current_inter_ != "aggressive")
            {
                // calculates if ped is behind the robot to determine if he can continue to drive or set temp_goal
                double angle_to_point = atan2(point.x - robot_x, point.y - robot_y);
                double angle_diff = angles::shortest_angular_distance(theta, angle_to_point);

                wall_near = inter_util::InterUtil::checkStaticObjects(distance, theta, default_padding, temp_goal_distance_, robot_radius_, detectedRanges, detectedAngles);

                // check speed restriction
                caution |= (distance <= caution_detection_range_);

                // Check if the pedestrian is in range to set temporary goal and move back
                if ((!new_goal_set_) && (distance <= ped_minimum_distance_) && (2 * std::abs(angle_diff) <= fov_))
                {
                    temp_goal_ = inter_util::InterUtil::setTempGoal(start, theta, distance, temp_goal_distance_, current_inter_);
                    new_goal_set_ = true;
                }

                // nothing else to compute
                if (caution && new_goal_set_)
                    break;
            }
        }
        speed_ = inter_util::InterUtil::setSpeed(caution, minDistance, changed_max_vel_x_param_, max_vel_x_param_, current_inter_);
        inter_util::InterUtil::checkDanger(dangerPublisher, distances, 0.6);
        if (new_goal_set_)
        {
            if (wall_near)
            {
                ROS_ERROR("AVOIDED COLLISION WITH OBSTACLE. CONTINUE NORMAL PLANNING");
                new_goal_set_ = false;
                plan = plan_;
                return 0;
            }
            // calculate distance to temporary goal
            double distance_to_temp_goal = std::sqrt(std::pow(temp_goal_.pose.position.x - robot_x, 2) + std::pow(temp_goal_.pose.position.y - robot_y, 2));

            // Clear the existing plan and add temp_goal
            plan.clear();
            plan.push_back(temp_goal_);
            if (distance_to_temp_goal <= temp_goal_tolerance_)
            {
                ROS_INFO("Reached temp_goal. Resetting goal.");
                new_goal_set_ = false;
            }
        }
        else
            plan = plan_;


        return 0;
    }



    void MetaInter::selectPlanner(double distance, std::string type, bool activateSideways) 
    {
        //sorted by priority
        if(current_inter_ == "polite") {
            return;
        }
        if(distance <= polite_range_ && type == "human/elder") {
            current_inter_ = "polite";
            return;
        }
        if(activateSideways) {
            current_inter_ = "sideways";
            return;
        }

    }

    bool MetaInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        plan_ = plan;
        return true;
    }

    void MetaInter::semanticCallback(const pedsim_msgs::AgentStates::ConstPtr &message)
    // turns our semantic layer data into points we can use to calculate distance
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        inter_util::InterUtil::processAgentStates(message, simAgentInfos);
    }

    void MetaInter::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &message)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        // Set a maximum distance threshold for wall detection (adjust as needed)
        double max_detection_range = caution_detection_range_ + 0.5; // detect every obstacle in his caution_detection_range plus 0.5 metres

        detectedRanges.clear();
        // Accessing and printing range data
        for (size_t i = 0; i < message->ranges.size(); ++i)
        {
            double angle = message->angle_min + i * message->angle_increment;
            double range = message->ranges[i];

            // Check if the range is under the maximum detection range
            if (range < max_detection_range)
            {
                detectedRanges.push_back(range);
                detectedAngles.push_back(angle);
            }
        }
    }

    void MetaInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;
        std::string node_namespace_ = ros::this_node::getNamespace();
        std::string semantic_layer = "/pedsim_simulator/simulated_agents";
        nh_ = ros::NodeHandle("~");
        subscriber_ = nh_.subscribe(semantic_layer, 1, &MetaInter::semanticCallback, this);
        dangerPublisher = nh_.advertise<std_msgs::String>("Danger", 10);

        // get topic for our scan
        std::string scan_topic_name;
        std::string helios_points_topic_name;
        if (!nh_.getParam(node_namespace_ + "/move_base_flex/local_costmap/obstacles_layer/scan/topic", scan_topic_name))
        {
            ROS_ERROR("Failed to get parameter %s/move_base_flex/local_costmap/obstacles_layer/scan/topic", node_namespace_.c_str());
            if (!nh_.getParam(node_namespace_ + "/move_base_flex/local_costmap/obstacles_layer/helios_points/topic", helios_points_topic_name))
            {
                ROS_ERROR("Failed to get parameter %s/move_base_flex/local_costmap/obstacles_layer/helios_points/topic", node_namespace_.c_str());
            }
        }
        if (!nh_.getParam("/robot_radius", robot_radius_)){
            ROS_ERROR("Failed to get parameter %s/local_planner", node_namespace_.c_str());
        }
        if (!scan_topic_name.empty())
        {
            laser_scan_subscriber_ = nh_.subscribe(scan_topic_name, 1, &MetaInter::laserScanCallback, this);
        }
        // if(!helios_points_topic_name.empty()){
        //     helios_points_subscriber_ = nh_.subscribe(helios_points_topic_name, 1, &MetaInter::pointCloudCallback, this);
        // }

        // get our local planner name
        std::string planner_keyword;
        if (!nh_.getParam(node_namespace_ + "/local_planner", planner_keyword))
        {
            ROS_ERROR("Failed to get parameter %s/local_planner", node_namespace_.c_str());
        }
        std::string local_planner_name = inter_util::InterUtil::getLocalPlanner(planner_keyword);
        // get the starting parameter for max_vel_x from our planner
        if (!nh_.getParam(node_namespace_ + "/move_base_flex/" + local_planner_name + "/max_vel_x", max_vel_x_param_))
        {
            ROS_ERROR("Failed to get parameter %s/move_base_flex/%s/max_vel_x", node_namespace_.c_str(), local_planner_name.c_str());
            return;
        }
        // Create service client for the Reconfigure service
        setParametersClient_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(node_namespace_ + "/move_base_flex/" + local_planner_name + "/set_parameters");
        dynamic_reconfigure::Server<meta_inter::MetaInterConfig> server;
        server.setCallback(boost::bind(&MetaInter::reconfigure, this, _1, _2));

        // thread to control the velocity for robot
        velocity_thread_ = std::thread(&MetaInter::setMaxVelocityThread, this);

        // needs to be declared here because cautious_speed gets declared with reconfigure
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);

        // Subscribe to the global plan topic
        global_plan_sub_ = nh_.subscribe("/jackal/move_base_flex/TebLocalPlannerROS/global_plan", 1, &MetaInter::globalPlanCallback, this);
        odom_sub = nh_.subscribe("/jackal/odom", 10, &MetaInter::odomCallback,this);

        // Publisher for modified global plan with color changes
        global_plan_pub_ = nh_.advertise<visualization_msgs::Marker>("global_plan_color", 1);

        path_pub_ = nh_.advertise<visualization_msgs::Marker>("robot_path", 10);

        
    }

    void MetaInter::reconfigure(meta_inter::MetaInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        // updating values from config
        caution_detection_range_ = config.caution_detection_range;
        cautious_speed_ = config.cautious_speed;
        ped_minimum_distance_ = config.ped_minimum_distance;
        temp_goal_distance_ = config.temp_goal_distance;
        temp_goal_tolerance_ = config.temp_goal_tolerance;
        fov_ = config.fov;
        danger_threshold_ = config.danger_threshold;
        current_inter_ = config.current_inter;
        polite_range_ = config.polite_range;
        sideways_range_ = config.sideways_range;
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);

    }

    void MetaInter::setMaxVelocityThread()
    {
        ros::Rate rate(1); // Adjust the rate as needed
        while (ros::ok())
        {
            // Lock to access shared variables
            boost::unique_lock<boost::mutex> lock(speed_mtx_);

            // Check if the speed has changed
            if (speed_ != last_speed_)
            {
                // set max_vel_x parameter
                double_param_.name = "max_vel_x";
                double_param_.value = speed_;
                conf_.doubles.clear();
                conf_.doubles.push_back(double_param_);
                reconfig_.request.config = conf_;

                // Call setParametersClient_ to update parameters
                if (setParametersClient_.call(reconfig_))
                {
                    ROS_INFO_ONCE("Dynamic reconfigure request successful");
                }
                else
                {
                    ROS_ERROR_ONCE("Failed to call dynamic reconfigure service");
                }

                // Update last_speed_ to avoid unnecessary calls
                last_speed_ = speed_;
            }

            // Unlock and sleep
            lock.unlock();
            rate.sleep();
        }
    }


    void MetaInter::globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        // Modify the global plan and change the color based on intermediate planner
        visualization_msgs::Marker modified_plan;
        modified_plan.header = msg->header;
        modified_plan.ns = "global_plan";
        modified_plan.id = 0;
        modified_plan.type = visualization_msgs::Marker::LINE_STRIP;
        modified_plan.action = visualization_msgs::Marker::ADD;
        modified_plan.pose.orientation.w = 1.0;
        modified_plan.scale.x = 0.01; // Adjust thickness as needed

        // Set color based on intermediate planner
        if (current_inter_ == "aggressive")
        {
            modified_plan.color.r = 1.0;
            modified_plan.color.g = 0.0;
            modified_plan.color.b = 0.0;
            modified_plan.color.a = 1.0; // Fully opaque red
        }
        else if (current_inter_ == "sideways")
        {
            modified_plan.color.r = 0.0;
            modified_plan.color.g = 1.0;
            modified_plan.color.b = 0.0;
            modified_plan.color.a = 1.0; // Fully opaque green
        }
        else if (current_inter_ == "polite")
        {
            modified_plan.color.r = 0.0;
            modified_plan.color.g = 0.0;
            modified_plan.color.b = 1.0;
            modified_plan.color.a = 1.0; // Fully opaque blue
        }
        else
        {
            ROS_WARN("Unknown intermediate planner: %s", current_inter_.c_str());
            // Use default color if intermediate planner is unknown
            modified_plan.color.r = 1.0;
            modified_plan.color.g = 1.0;
            modified_plan.color.b = 1.0;
            modified_plan.color.a = 1.0; // Fully opaque white
        }

        // Convert PoseStamped messages to Point messages
        for (const auto& pose_stamped : msg->poses)
        {
            geometry_msgs::Point point;
            point.x = pose_stamped.pose.position.x;
            point.y = pose_stamped.pose.position.y;
            point.z = pose_stamped.pose.position.z;
            modified_plan.points.push_back(point);
        }

        // Publish the modified global plan
        global_plan_pub_.publish(modified_plan);
    }




    void MetaInter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {


         // Initialisierung der Marker-Nachricht für den Pfad
        path_marker_.header.frame_id = "map";
        path_marker_.ns = "robot_path";
        path_marker_.id = 0;
        path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker_.action = visualization_msgs::Marker::ADD;
        path_marker_.pose.orientation.w = 1.0;
        path_marker_.scale.x = 0.05; // Dicke der Linie
 
        
               
        // Extrahieren der Position des Roboters aus der Odometrie
        geometry_msgs::PointStamped robot_position;
        robot_position.header = msg->header;
        robot_position.point = msg->pose.pose.position;

        // Hinzufügen der Position des Roboters zum Pfad
        path_marker_.points.push_back(robot_position.point);




        // Set color based on intermediate planner
        if (current_inter_ == "aggressive")
        {
            path_marker_.color.r = 1.0;
            path_marker_.color.g = 0.0;
            path_marker_.color.b = 0.0;
            path_marker_.color.a = 1.0; // Fully opaque red
        }
        else if (current_inter_ == "sideways")
        {
            path_marker_.color.r = 0.0;
            path_marker_.color.g = 1.0;
            path_marker_.color.b = 0.0;
            path_marker_.color.a = 1.0; // Fully opaque green
        }
        else if (current_inter_ == "polite")
        {
            path_marker_.color.r = 0.0;
            path_marker_.color.g = 0.0;
            path_marker_.color.b = 1.0;
            path_marker_.color.a = 1.0; // Fully opaque blue
        }
        else
        {
            ROS_WARN("Unknown intermediate planner: %s", current_inter_.c_str());
            // Use default color if intermediate planner is unknown
            path_marker_.color.r = 1.0;
            path_marker_.color.g = 1.0;
            path_marker_.color.b = 1.0;
            path_marker_.color.a = 1.0; // Fully opaque white
        }

        // Veröffentlichen der aktualisierten Pfadnachricht
        path_pub_.publish(path_marker_);
    }




}