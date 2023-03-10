#ifndef ANKLE_KINEMATICS_H
#define ANKLE_KINEMATICS_H

/**
* Performs ankle Inverse Kinematics going from joint space to motor space
* @param alpha The ankle's roll
* @param beta The ankle's pitch
* @param motor_1 Pointer to the motor 1 value to set
* @param motor_1 Pointer to the motor 2 value to set
*/
void ankleIK(float alpha, float beta, float &motor_1, float &motor_2);

/**
* Performs ankle Forward Kinematics going from motor space to joint space
* @param motor_1 Position of motor 1
* @param motor_2 Position of motor 2
* @param alpha Pointer to set resulting ankle roll
* @param beta Pointer to set resulting ankle pitch
*/
void ankleFK(float motor_1, float motor_2, float &alpha, float &beta);

/**
* Performs ankle Forward Kinematics to convert motor space velocity to joint space velocity 
* using a finite difference method between two individual FK solutions
* @param motor_1 Position of motor 1
* @param motor_2 Position of motor 2
* @param motor_1_v Velocity of motor 1
* @param motor_2_v Velocity of motor 2
* @param alpha_v Pointer to set resulting ankle roll velocity
* @param beta_v Pointer to set resulting ankle pitch velocity
* @param dt Time delta to perform difference over
*/
void ankleFKvel(float motor_1, float motor_2, float motor_1_v, float motor_2_v, float &alpha_v, float &beta_v, float dt);

#endif