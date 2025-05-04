#ifndef TIMESTAMPED_VALUE_HPP
#define TIMESTAMPED_VALUE_HPP

class TimestampedValue{

    public:

        TimestampedValue(double _value, double _time){
            value = _value;
            time = _time;

            prev_value = value;
            prev_time = time;
        }

        void set_value(double _value){
            prev_value = value;

            value = _value;
        }

        void set_time(double _time){
            prev_time = time;

            time = _time;
        }

        double get_value(){
            return value;
        }

        double get_time(){
            return time;
        }

        double update(double _value, double _time){
            set_value(_value);
            set_time(_time);
        }

        double get_rate(){
            double dV = value - prev_value;
            double dT = time - prev_time;

            if(dT == 0)
                return 0;
            else
                return dV / dT;
        }

    private:

        double prev_value;
        double prev_time;

        double value;
        double time;
        

}; // class TimestampedValue

#endif // TIMESTAMPED_VALUE_HPP