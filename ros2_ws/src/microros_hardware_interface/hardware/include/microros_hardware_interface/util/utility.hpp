#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <string>
#include <bits/stdc++.h>
#include <stdexcept>

namespace utility{

    bool string_to_bool(std::string& input){
        std::transform(input.begin(), input.end(), input.begin(), ::tolower);

        if(input == "true")
            return true;
        else if(input == "false")
            return false;
        else
            throw std::invalid_argument("String was not true/false!");
    }
};

#endif