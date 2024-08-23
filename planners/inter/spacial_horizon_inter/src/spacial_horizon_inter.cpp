#include "../include/spacial_horizon_inter.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(spacial_horizon_inter::SpacialHorizonInter, mbf_costmap_core::CostmapInter)


uint32_t spacial_horizon_inter::SpacialHorizonInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
{
    ROS_INFO_STREAM("[SPACIAL HORIZON - makePlan] ENTERING ");
    boost::unique_lock<boost::mutex> plan_lock(plan_mtx_);
    plan = plan_;
    bool subgoal_success = updateSubgoal();

    if (!subgoal_success)
    {
        ROS_WARN("[SpacialHorizon - makePlan] No subgoal found. No global plan received or goal reached!");
    }

    geometry_msgs::PoseStamped subgoal;
    subgoal.header.frame_id = "map";
    subgoal.pose.position.x = subgoal_pos(0);
    subgoal.pose.position.y = subgoal_pos(1);
    subgoal.pose.position.z = 0.0;
    subgoal.pose.orientation.w = 1.0;

    plan.clear();
    plan.push_back(subgoal);

    ROS_INFO_STREAM("[SPACIAL HORIZON - makePlan] LEAVING");
    return 0;
}

bool spacial_horizon_inter::SpacialHorizonInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_INFO_STREAM("[SPACIAL HORIZON - setPlan] ENTERING");
    boost::unique_lock<boost::mutex> lock(plan_mtx_);
    plan_ = plan;

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    path.poses = plan_;
    pub_global_plan.publish(path);
    ROS_INFO_STREAM("[SPACIAL HORIZON - setPlan] LEAVING");
    return true;
}

void spacial_horizon_inter::SpacialHorizonInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros){
    ros::NodeHandle nh_("~/" + name);

    nh_.param("/train_mode", train_mode, false);
    // load params from node handle
    nh_.param("goal_tolerance", goal_tolerance, goal_tolerance);
    nh_.param("planning_horizon", planning_horizon, planning_horizon);
    nh_.param("subgoal_tolerance", subgoal_tolerance, subgoal_tolerance);
    nh_.param("update_subgoal_rate", update_subgoal_rate, update_subgoal_rate);
    nh_.param("update_globalplan_rate", update_globalplan_rate, update_globalplan_rate);


    dynamic_reconfigure::Server<SpacialHorizonInterConfig> server;
    server.setCallback(boost::bind(&SpacialHorizonInter::reconfigure, this, _1, _2));

    initializeGlobalPlanningService();

    std::string odom_topic = ros::this_node::getNamespace() + "/" + BASE_TOPIC_ODOM;
    std::string goal_topic = ros::this_node::getNamespace() + "/" + BASE_TOPIC_GOAL;
    std::string subgoal_topic = ros::this_node::getNamespace() + "/" + BASE_TOPIC_SUBGOAL;
    std::string global_plan_topic = ros::this_node::getNamespace() + "/" + BASE_TOPIC_GLOBAL_PLAN;

    ROS_INFO_STREAM("[Spacial Horizon] Subscribing to topics: \n"
                "  Odometry: " << odom_topic << "\n"
                "  Goal:     " << goal_topic);

    sub_odom = nh_.subscribe(odom_topic, 1, &SpacialHorizonInter::odomCallback, this);
    sub_goal = nh_.subscribe(goal_topic, 1, &SpacialHorizonInter::goalCallback, this);

    pub_subgoal = nh_.advertise<geometry_msgs::PoseStamped>(subgoal_topic, 1);
    pub_global_plan = nh_.advertise<nav_msgs::Path>(global_plan_topic, 1);

    // Create service clients for the GetDump and Reconfigure services
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<SpacialHorizonInterConfig> >(nh_);
    dynamic_reconfigure::Server<SpacialHorizonInterConfig>::CallbackType cb = boost::bind(&SpacialHorizonInter::reconfigure, this, _1, _2);
    dynamic_recfg_->setCallback(cb);

    // subgoal_timer = nh_.createTimer(
    //     ros::Duration(1/update_subgoal_rate), &SpacialHorizonInter::updateSubgoalCallback, this
    // );
    // global_plan_timer = nh_.createTimer(
    //     ros::Duration(1/update_globalplan_rate), &SpacialHorizonInter::getGlobalPath, this
    // );

    if (train_mode)
    {
        initializeStepWorldService();
    }

    ROS_INFO("[Spacial Horizon] Initialized Spacial Horizon Inter plugin");
}

void spacial_horizon_inter::SpacialHorizonInter::reconfigure(spacial_horizon_inter::SpacialHorizonInterConfig &config, uint32_t level)
{
    ROS_INFO_STREAM("[SPACIAL HORIZON] ENTERING RECONFIGURE");
    goal_tolerance = config.goal_tolerance;
    planning_horizon = config.planning_horizon;
    subgoal_tolerance = config.subgoal_tolerance;
    if (update_subgoal_rate != config.update_subgoal_rate) {
        update_subgoal_rate = config.update_subgoal_rate;
        subgoal_timer.setPeriod(ros::Duration(1/update_subgoal_rate));
    }
    ROS_INFO_STREAM("[SPACIAL HORIZON] LEAVING RECONFIGURE");
}

void spacial_horizon_inter::SpacialHorizonInter::initializeGlobalPlanningService()
{
    std::string service_name = ros::this_node::getNamespace() + "/" + SERVICE_GLOBAL_PLANNER;
    ROS_INFO_STREAM("[Spacial Horizon - INIT] Initializing MBF service client with service name: \n" 
                    "" << service_name.c_str());

    while (!ros::service::waitForService(service_name, ros::Duration(5.0)))
    {
        ROS_INFO("[SpacialHorizon - INIT] Waiting for service %s to become available",
                service_name.c_str());
    }
    global_planner_srv = nh_.serviceClient<nav_msgs::GetPlan>(service_name, false);
}

void spacial_horizon_inter::SpacialHorizonInter::publishSubgoal(Eigen::Vector2d &subgoal)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = subgoal(0);
    pose_stamped.pose.position.y = subgoal(1);
    pose_stamped.pose.position.z = 0.0;

    pub_subgoal.publish(pose_stamped);
}

bool spacial_horizon_inter::SpacialHorizonInter::updateSubgoal()
{
    ROS_INFO_STREAM("[Spacial Horizon] Updating subgoal");

    if (!has_goal) {
        ROS_WARN("[SpacialHorizon] No goal received yet");
        return false;
    }

    bool subgoal_success = getSubgoal(subgoal_pos);

    // if to far away from subgoal -> recompute global path and subgoal
    double dist_to_subgoal = (odom_pos - subgoal_pos).norm();
    if (dist_to_subgoal > planning_horizon + 1.0)
    {
        ROS_INFO_STREAM("[Spacial Horizon]: Too far away from subgoal! Recomputing global path: " 
                        << end_pos << " " << odom_pos);
        getGlobalPath();
        subgoal_success = getSubgoal(subgoal_pos);
    }

    if (!subgoal_success)
    {
        ROS_WARN_STREAM("[Spacial Horizon] No subgoal found. No global plan received or goal reached!");
        return false;
    }

    // publishSubgoal(subgoal_pos);
    return true;
}


void spacial_horizon_inter::SpacialHorizonInter::updateSubgoalCallback(const ros::TimerEvent &e)
{
    if (has_goal && has_odom && called_make_plan)
    {
        updateSubgoal();
        return;
    }
    ROS_WARN_STREAM("[SpacialHorizon - UPDATE_SUBGOAL] No goal or odom received yet");
}

/**
 * @brief Retrieves the subgoal for the SpacialHorizon object.
 *
 * This function retrieves the subgoal, which is a 2D vector, for the SpacialHorizon object.
 *
 * @param subgoal A reference to an Eigen::Vector2d object where the subgoal will be stored.
 * @return bool Returns true if the subgoal was successfully retrieved, false otherwise.
 */
bool spacial_horizon_inter::SpacialHorizonInter::getSubgoal(Eigen::Vector2d &subgoal)
{   
    double dist_to_goal = (odom_pos - end_pos).norm();

    if (dist_to_goal <= goal_tolerance)
    {
        return false;
    }

    if (dist_to_goal < planning_horizon)
    {
        subgoal = end_pos;

        return true;
    }

    for (size_t i = 0; i < plan_.size(); i++)
    {
        Eigen::Vector2d wp_pt =
            Eigen::Vector2d(plan_[i].pose.position.x,
                            plan_[i].pose.position.y);
        double dist_to_robot = (odom_pos - wp_pt).norm();

        // If dist to robot is somewhere in planning_horizon +- subgoal_tolerance

        if (abs(dist_to_robot - planning_horizon) < subgoal_tolerance)
        {
            subgoal = wp_pt;

            return true;
        }
    }

    return false;
}

void spacial_horizon_inter::SpacialHorizonInter::getGlobalPath(const ros::TimerEvent &e) {
    getGlobalPath();
}

/* Get global plan from move_base */
void spacial_horizon_inter::SpacialHorizonInter::getGlobalPath()
{
    /* get global path from move_base */
    if (!global_planner_srv)
    {
        ROS_FATAL("[SpacialHorizon - GET_PATH] Could not initialize get plan "
                  "service from %s",
                  global_planner_srv.getService().c_str());
        return;
    }
    nav_msgs::GetPlan get_plan_srv;
    fillPathRequest(get_plan_srv.request);
    callPlanningService(global_planner_srv, get_plan_srv, plan_);
    
}

void spacial_horizon_inter::SpacialHorizonInter::tryUpdateGlobalplanAndSubgoal(int try_count)
{
    if (try_count > 5)
    {
        ROS_WARN("[SpacialHorizon] Could not update global plan and subgoal!");
        return;
    }
    if (!has_goal)
    {
        ROS_WARN("[SpacialHorizon] No goal received yet!");
        return;
    }
    // getGlobalPath();
    bool subgoal_success = getSubgoal(subgoal_pos);
    if (!subgoal_success)
    {
        ROS_WARN_STREAM("[Spacial Horizon] Probably got no new goal. No subgoal found!");
        if (train_mode)
        {
            std_srvs::Empty srv;
            step_world_srv.call(srv);
        }
        tryUpdateGlobalplanAndSubgoal(try_count + 1);
    }
    else
    {
        publishSubgoal(subgoal_pos);
    }
}

void spacial_horizon_inter::SpacialHorizonInter::fillPathRequest(nav_msgs::GetPlan::Request &request)
{
    request.start.header.frame_id = "map";
    request.start.pose.position.x =
        odom_pos[0]; // x coordinate of the initial position
    request.start.pose.position.y =
        odom_pos[1];                        // y coordinate of the initial position
    request.start.pose.orientation.w = 1.0; // direction
    request.goal.header.frame_id = "map";
    request.goal.pose.position.x = end_pos[0]; // End point coordinates
    request.goal.pose.position.y = end_pos[1];
    request.goal.pose.orientation.w = 1.0;
    request.tolerance = goal_tolerance; // If the goal cannot be reached, the
                                        // most recent constraint
}

void spacial_horizon_inter::SpacialHorizonInter::callPlanningService(ros::ServiceClient &serviceClient,
                                         nav_msgs::GetPlan &srv, std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_INFO_STREAM("[SpacialHorizon - callPlanningService] Calling service " << serviceClient.getService().c_str());
    
    // Check if service is available
    if (serviceClient.waitForExistence(ros::Duration(5.0)))
    {
        ROS_INFO_STREAM("[SpacialHorizon - callPlanningService] Service " << serviceClient.getService().c_str() << " is available");
    }
    else
    {
        ROS_ERROR("[SpacialHorizon - callPlanningService] Service %s is not available",
                  serviceClient.getService().c_str());
        return;
    }

    // boost::unique_lock<boost::mutex> lock(plan_mtx_);
    if (serviceClient.call(srv))
    {
        if (srv.response.plan.poses.empty())
        {
            ROS_WARN("[SpacialHorizon - GET_PATH] Global plan was empty!");
            return;
        }
        ROS_INFO_STREAM("[SpacialHorizon - GET_PATH] Received global plan with " << srv.response.plan.poses.size() << " poses");
        setPlan(srv.response.plan.poses);
    }
    else
    {
        ROS_ERROR("[SpacialHorizon - GET_PATH] Failed to call service %s - is the "
                  "robot moving?",
                  serviceClient.getService().c_str());
    }
    ROS_INFO_STREAM("[SpacialHorizon - GET_PATH] Finished calling service " << serviceClient.getService().c_str());
}

void spacial_horizon_inter::SpacialHorizonInter::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_vel = Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    has_odom = true;

    // ROS_INFO_STREAM("[SpacialHorizon - odomCallback] Received odom: " << odom_pos(0) << " " << odom_pos(1));

    // check if subgoal is reached
    if (has_goal && subgoal_pos.norm() > 0)
    {
        if ((odom_pos - subgoal_pos).norm() <= subgoal_tolerance && subgoal_pos != end_pos)
        {
            ROS_WARN("[SpacialHorizon - odomCallback] Reached subgoal. Recomputing subgoal.");
            tryUpdateGlobalplanAndSubgoal();
        }
    }
}

void spacial_horizon_inter::SpacialHorizonInter::goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    ROS_INFO_STREAM("[SpacialHorizon - goalCallback] Received goal: " << msg->pose.position.x << " " << msg->pose.position.y);

    end_pos = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
    has_goal = true;

    if (called_make_plan && train_mode)
    {
        tryUpdateGlobalplanAndSubgoal();
    }
}

void spacial_horizon_inter::SpacialHorizonInter::initializeStepWorldService()
{
    ROS_INFO_STREAM("[Spacial Horizon - INIT] Initializing StepWorld service client");

    std::string ns = ros::this_node::getNamespace(); 
    ns = ns.substr(0, ns.find_last_of("/"));
    std::string service_name = ns + "/" + SERVICE_STEP_WORLD;

    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_INFO("[SpacialHorizon - INIT] Waiting for service %s to become available",
                service_name.c_str());
    }
    step_world_srv = nh_.serviceClient<std_srvs::Empty>(service_name, true);
}