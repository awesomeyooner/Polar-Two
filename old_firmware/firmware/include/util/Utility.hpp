#ifndef UTILITY_HPP
#define UTILITY_HPP

namespace Utility{

    struct TimestampedNumber{
        double value;
        double timestamp;

        /**
         * Two number pair for storing timestamps, in SECONDS
         */
        TimestampedNumber(double value, double timestamp) : value(value), timestamp(timestamp){}
        TimestampedNumber() : value(0), timestamp(0){}

        double getRate(TimestampedNumber next){
            double dv = next.value - this->value;
            double dt = next.timestamp - this->timestamp;

            return dv / dt;
        }
    };
}

#endif