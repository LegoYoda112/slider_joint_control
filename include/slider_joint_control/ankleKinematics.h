#ifndef ANKLE_KINEMATICS_H
#define ANKLE_KINEMATICS_H

namespace ankleKinematics
{

/**
* Performs ankle Inverse Kinematics going from joint space to motor space
* @param alpha The ankle's pitch
* @param beta The ankle's roll
* @param motor_1 Position of right motor
* @param motor_1 Position of left motor
*/
void ankleIK(float alpha, float beta, float &motor_1, float &motor_2);

/**
* Performs ankle Forward Kinematics going from motor space to joint space
* @param motor_1 Position of right motor
* @param motor_2 Position of left motor
* @param alpha Pointer to set resulting ankle pitch
* @param beta Pointer to set resulting ankle roll
*/
void ankleFK(float motor_1, float motor_2, float &alpha, float &beta);

/**
* Performs ankle Forward Kinematics to convert motor space velocity to joint space velocity 
* using a finite difference method between two individual FK solutions
* @param motor_1 Position of right motor
* @param motor_2 Position of left motor
* @param motor_1_v Velocity of right motor
* @param motor_2_v Velocity of left motor
* @param alpha_v Pointer to set resulting ankle pitch velocity
* @param beta_v Pointer to set resulting ankle roll velocity
* @param dt Time delta to perform difference over
*/
void ankleFKvel(float motor_1, float motor_2, float motor_1_v, float motor_2_v, float &alpha_v, float &beta_v, float dt);

}

#endif