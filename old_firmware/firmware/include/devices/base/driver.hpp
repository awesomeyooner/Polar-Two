#ifndef DRIVER_HPP
#define DRIVER_HPP

namespace hardware_component{

    class Driver{

        public:

            virtual ~Driver(){}

            virtual void initialize(){}

            virtual void set_percent(double request){}

            virtual void stop(){}
        
        private:


    }; // class Driver
     
} // namespace hardware_component

#endif