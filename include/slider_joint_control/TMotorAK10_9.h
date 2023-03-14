// Header file with specific constants for the AK60_6 motor

#include <string>

#include "TMotor.h"

class TMotorAK10_9 : public TMotor
{
    protected:

    public:
        TMotorAK10_9(string joint_name, int can_id){
            
            // Can socket
            this->socket = socket;

            this->joint_name = joint_name;

            this->motor_type = "TMotorAK10_9";

            this->can_id = can_id;

            this->P_MIN = -12.5;
            this->P_MAX = 12.5;
            this->V_MIN = -23.24;
            this->V_MAX = 23.24;
            this->T_MIN = -54;
            this->T_MAX = 54;
            this->KP_MIN = 0;
            this->KP_MAX = 500;
            this->KD_MIN = 0;
            this->KD_MAX = 5;

            this->INTERNAL_GEAR_RATIO = 9;
        }
};