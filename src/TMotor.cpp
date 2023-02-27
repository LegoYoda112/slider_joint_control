#include "slider_joint_control/TMotor.h"

// BEGIN UTILS

unsigned int second = 1000000;

// Converts float to uint given min, max and bits (used to go from Program -> CAN)
unsigned int float_to_uint(float x, float x_min, float x_max, unsigned short int bits) {
	float span = x_max - x_min;

	if (x < x_min) x = x_min;
	else if (x > x_max) x = x_max;

	return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}


// Converts uint back to float given min, max and bits (used to go from CAN -> Program)
float uint_to_float(int x_int, float x_min, float x_max, unsigned short int bits) {
	float span = x_max - x_min;

	float offset = x_min;

	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


// Write one frame to the provided socket
void CAN_write_frame(int s, struct can_frame frame){
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write Error: ");
    }
}

// Write data to specified can_id
void CAN_write_data(int s, uint8_t can_id, uint8_t data[], int length = 8){

    // Make new can frame
    struct can_frame frame;

    // Set frame ID
    frame.can_id = can_id;

    //__u8 data [4] = {0x00, 0x00, 0x00, 0x00};
    int dataLength = length;

    // Set frame length to be data length
    frame.can_dlc = dataLength;

    // Copy data into frame data and pad out to 8 bytes
    for(int i = 0; i<=7; i++){
        if(i < dataLength){
            frame.data[i] = data[i];
        } else {
            frame.data[i] = 0x00;
        }
    }

    // Write frame
    CAN_write_frame(s, frame);
}

// END UTILS


// =========== PACK AND UNPACK FUNCTIONS

void TMotor::pack_motor_cmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff){

    // Constrain inputs
    p_des = fminf(fmaxf(this->P_MIN, p_des), this->P_MAX);
    v_des = fminf(fmaxf(this->V_MIN, v_des), this->V_MAX);
    kp = fminf(fmaxf(this->KP_MIN, kp), this->KP_MAX);
    kd = fminf(fmaxf(this->KD_MIN, kd), this->KD_MAX);
    t_ff = fminf(fmaxf(this->T_MIN, t_ff), this->T_MAX);

    // Convert float to uints that will be passed through CAN
    int p_int = float_to_uint(p_des, this->P_MIN, this->P_MAX, 16);
    int v_int = float_to_uint(v_des, this->V_MIN, this->V_MAX, 12);
    int kp_int = float_to_uint(kp, this->KP_MIN, this->KP_MAX, 12);
    int kd_int = float_to_uint(kd, this->KD_MIN, this->KD_MAX, 12);
    int t_int = float_to_uint(t_ff, this->T_MIN, this->T_MAX, 12);

    // Populate data
    data[0] = p_int >> 8;
    data[1] = p_int&0xFF;
    data[2] = v_int >> 4;
    data[3] = ((v_int&0xF) << 4)|(kp_int >> 8);
    data[4] = kp_int&0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int&0xF) << 4)|(t_int >> 8);
    data[7] = t_int&0xFF;
}

// Unpacks the motor response into position, velocity and current (?)
void TMotor::unpack_motor_response(uint8_t* data, float &p, float &v, float &i){
    // Unpacks data ints
    int id = data[0];
    int p_int = (data[1] << 8)|data[2];
    int v_int = (data[3] << 4)|(data[4]>>4);
    int i_int = ((data[4]&0xF) << 8)|data[5];

    // Converts to floats
    // I don't really like putting the zero offset at this level, but here we are
    p = uint_to_float(p_int, P_MIN, P_MAX, 16) - this->zero_offset;
    v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    i = uint_to_float(i_int, T_MIN, T_MAX, 12);
}

// =========== SET FUNCTIONS

// Sets the constants
void TMotor::set_constants(float kP, float kD){
    this->kP = kP;
    this->kD = kD;
}

// Sets CAN socket
void TMotor::set_socket(int socket){
    this->socket = socket;
}

// Sets the zero offset for the motor
void TMotor::set_zero_offset(float zero_offset){
    // cout << "Start zero" << endl;
    // Depending on the motor type, figure out if it has started within it's zero window
    this->zero_offset = zero_offset;

    // Apply offset, needed for encoder overrun offset
    this->position -= zero_offset;

    // The offset tolerance we allow the motor to be within before assuming
    // we've overrun the magnetic encoder
    float const OFFSET_TOLERANCE_RADS = (6.2831 / this->INTERNAL_GEAR_RATIO) * 0.5; 
 
    // cout << this->joint_name << endl;
    // cout << this->position << endl;

    // A single motor turn, measured at the gearbox output
    // TODO: this might cast to int?
    float const ONE_MOTOR_TURN_RADS = 6.2831 / this->INTERNAL_GEAR_RATIO;

    // cout << this->INTERNAL_GEAR_RATIO << endl;
    // cout << this->T_MIN << endl;

    // Check if position has been set and if so, account for encoder overrun
    // TODO: throw an error?
    // TODO: Check that these are applied the right way around
    if(!std::isnan(this->position)){
        if(this->position < -OFFSET_TOLERANCE_RADS) {
            this->zero_offset -= ONE_MOTOR_TURN_RADS;
        }

        if(this->position > OFFSET_TOLERANCE_RADS) {
            this->zero_offset += ONE_MOTOR_TURN_RADS;
        }
    }
}

// Gets the zero offset for the motor
float TMotor::get_zero_offset(){
    return this->zero_offset;
}

// Sets the transmission ratio (used in the ROS joint state publisher)
void TMotor::set_transmission_ratio(float transmission_ratio){
    this->transmission_ratio = transmission_ratio;
}

void TMotor::set_position_limits(float position_min, float position_max){
    this->MIN = position_min;
    this->MAX = position_max;
}

void TMotor::copy_constants(TMotor *motor){
    this->MIN = motor->MIN;
    this->MAX = motor->MAX;

    this->kP = motor->kP;
    this->kD = motor->kD;
}

void TMotor::invert(){
    this->MIN = -this->MAX;
    this->MAX = -this->MIN;

    this->transmission_ratio = -this->transmission_ratio;
}

// =========== GET FUNCTIONS

float TMotor::get_transmission_ratio(){
    return this->transmission_ratio;
}


// =========== SEND FUNCTIONS

// Send a can message to the motor
void TMotor::send_can_msg(uint8_t* data){
    CAN_write_data(this->socket, this->can_id, data);
}

// Send a structured motor control command
void TMotor::send_motor_cmd(float p_des, float v_des, float kp, float kd, float t_ff){
    uint8_t data[8];

    this->p_des = p_des;
    this->v_des = v_des;
    this->kP = kp;
    this->kD = kd;
    this->t_ff = t_ff;

    this->pack_motor_cmd(data, p_des, v_des, kp, kd, t_ff);

    this->send_can_msg(data);
}

// Send the enable command
void TMotor::send_enable(){


    // Create CAN 8 byte data
    uint8_t data[8];

    // Populate enable frame
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    // Send data
    this->send_can_msg(data);

    this->read_motor_response();

    // cout << "Sent enable: " << this->joint_name << endl;
    // cout << this->kP << endl;
}

// Send the disable command
void TMotor::send_disable(){

    // Create CAN 8 byte data
    uint8_t data[8];
    
    // Populate disable frame
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    // Send data
    this->send_can_msg(data);
}

// Send the zero encoder command
void TMotor::send_zero_encoder(){

    // Create CAN 8 byte data
    uint8_t data[8];
    
    // Populate disable frame
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;

    // Send data
    this->send_can_msg(data);
}

// Sends a position goal
void TMotor::send_position_goal(float position, float torque_feedforward){

    // Clamp position goal
    if ( position > this->MAX){
        position = this->MAX;
    }else if( position < this->MIN ){
        position = this->MIN;
    }

    // cout << "Sending position goal" << endl;
    // cout << this->kP << endl;

    this->send_motor_cmd(position + this->zero_offset, 0.0, this->kP, this->kD, torque_feedforward);

    // Read motor's position, velocity and torque response
    //this->read_motor_response();
}

// Sends a velocity goal
void TMotor::send_velocity_goal(float velocity, float torque_feedforward){
    this->send_motor_cmd(0.0, velocity, 0.0, this->kD, torque_feedforward);

    // Read motor's position, velocity and torque response
    // this->read_motor_response();
}

// Sends a torque goal
void TMotor::send_torque_goal(float torque){
    this->send_motor_cmd(0.0, 0.0, 0.0, 0.0, torque);

    // Read motor's position, velocity and torque response
    //this->read_motor_response();
}

// Reads a motors response
void TMotor::read_motor_response(){

    // Recive a CAN message
    int nbytes;
    struct can_frame frame;
    nbytes = read(this->socket, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("Read");
    }

    // cout << "read motor" << endl;

    // Unpack response and assign values once recived
    this->unpack_motor_response(frame.data, this->position, this->velocity, this->torque);
}

// Reads and sets motor response from a can frame
void TMotor::read_motor_response_from_frame(struct can_frame frame){
    this->unpack_motor_response(frame.data, this->position, this->velocity, this->torque);
}


// =========== ROUTINE FUNCTIONS 

// Move motor back to 0 in a nice and controlled way 
// in *roughly* the specified time

// (loop is not real time so it will take slightly longer)

// Assumes you have run an enable and a motor read to get initial position

void TMotor::run_to_home(float speed){

    cout << "Homing motor: " << this->joint_name << endl;

    float start_pos = this->position;
    float distance = start_pos - this->zero_offset;

    float seconds = abs(distance) / speed;

    int loop_Hz = 200;

    int steps = seconds * loop_Hz;

    for(int i = 0; i < steps; i++){

        float progress = (float) i / steps;
        float desired_position = start_pos - progress * (start_pos - this->zero_offset);

        // cout << progress << endl;
        // cout << desired_position << endl;

        this->send_motor_cmd(desired_position, 0.0, 200.0, 0.5, 0.0);
        this->read_motor_response();

        //cout << this->position << endl;
        // cout << endl;

        usleep(second / loop_Hz);
    }

    cout << "Done!" << endl;
}