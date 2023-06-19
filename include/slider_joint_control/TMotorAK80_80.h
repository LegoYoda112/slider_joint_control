// Header file with specific constants for the AK80_80 motor

#include <string>

#include "TMotor.h"

class TMotorAK80_80 : public TMotor
{
    protected:

    public:
        TMotorAK80_80(string joint_name, int can_id){
            
            // Can socket
            this->socket = socket;

            this->joint_name = joint_name;

            this->motor_type = "TMotorAK80_80";

            this->can_id = can_id;


            this->P_MIN = -12.5;
            this->P_MAX = 12.5;
            this->V_MIN = -2.93;
            this->V_MAX = 2.93;
            this->T_MIN = -144;
            this->T_MAX = 144;
            this->KP_MIN = 0;
            this->KP_MAX = 500;
            this->KD_MIN = 0;
            this->KD_MAX = 5;

            this->INTERNAL_GEAR_RATIO = 80;
        }
};