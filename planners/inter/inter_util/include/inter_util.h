// inter_util.h
#ifndef INTER_UTIL_H
#define INTER_UTIL_H

#include <string>

namespace polite_inter
{
    class InterUtil
    {
    public:
        static std::string getLocalPlanner(const std::string &keyword);
    };
}

#endif // INTER_UTIL_H
