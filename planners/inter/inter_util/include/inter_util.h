// inter_util.h
#ifndef INTER_UTIL_H
#define INTER_UTIL_H

#include <string>

namespace inter_util
{
    class InterUtil
    {
    public:
        static std::string getLocalPlanner(const std::string &keyword);
    };
}

#endif // INTER_UTIL_H
