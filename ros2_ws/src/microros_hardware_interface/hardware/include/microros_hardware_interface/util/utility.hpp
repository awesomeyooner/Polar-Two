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

    double input_modulus(double input, double min, double max){

        double output = input;
    
        double mod = max - min;
    
        int min_wraps = (input - max) / mod;
        output -= (double)min_wraps * mod;
    
        int max_wraps = (input - min) / mod;
        output -= (double)max_wraps * mod;
    
        return output;
    }
};

#endif