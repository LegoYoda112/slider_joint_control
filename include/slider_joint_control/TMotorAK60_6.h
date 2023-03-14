// Header file with specific constants for the AK60_6 motor

#include <string>

#include "TMotor.h"

class TMotorAK60_6 : public TMotor
{
    protected:


    public:
        TMotorAK60_6(string joint_name, int can_id){
            
            // Can socket
            this->socket = socket;

            this->joint_name = joint_name;

            this->motor_type = "TMotorAK60_6";

            this->can_id = can_id;

            // Motor constants
            this->P_MIN = -12.5;
            this->P_MAX = 12.5;
            this->V_MIN = -41.87;
            this->V_MAX = 41.87;
            this->T_MIN = -9;
            this->T_MAX = 9;
            this->KP_MIN = 0;
            this->KP_MAX = 500;
            this->KD_MIN = 0;
            this->KD_MAX = 5;

            this->INTERNAL_GEAR_RATIO = 6;
        }
};