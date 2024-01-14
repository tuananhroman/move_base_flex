// inter_util.cpp
#include "inter_util.h"

namespace inter_util
{
    std::string InterUtil::getLocalPlanner(const std::string &keyword)
    {
        std::string local_planner_name;

        if (keyword == "teb")
        {
            local_planner_name = "TebLocalPlannerROS";
        }
        else if (keyword == "mpc")
        {
            local_planner_name = "MpcLocalPlannerROS";
        }
        else if (keyword == "dwa")
        {
            local_planner_name = "DwaLocalPlannerROS";
        }
        else if (keyword == "cohan")
        {
            local_planner_name = "HATebLocalPlannerROS";
        }

        return local_planner_name;
    }
}