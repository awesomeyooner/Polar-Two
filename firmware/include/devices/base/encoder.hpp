#ifndef ENCODER_HPP
#define ENCODER_HPP

namespace hardware_component{

    class Encoder{

        public:
            virtual ~Encoder(){}

            virtual void initialize(){}

            virtual void update(){}

            virtual double get_position(){return 0;}

            virtual double get_velocity(){return 0;}


    }; // class Encoder
} // namespace hardware_component

#endif // ENCODER_HPP