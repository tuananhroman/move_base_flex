// inter_util.h
#ifndef INTER_UTIL_H
#define INTER_UTIL_H

#include <string>
#include <vector>

namespace inter_util
{
    class InterUtil
    {
    public:
        static std::string getLocalPlanner(const std::string &keyword);
        static double getDangerLevel(const std::vector<double>& terms);
    };
}

#endif // INTER_UTIL_H
