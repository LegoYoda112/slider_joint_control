#ifndef ANKLE_KINEMATICS_H
#define ANKLE_KINEMATICS_H

void ankleIK(float alpha, float beta, float &motor_1, float &motor_2);
void ankleFK(float motor_1, float motor_2, float &alpha, float &beta);

#endif