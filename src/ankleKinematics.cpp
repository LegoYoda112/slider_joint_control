#include <math.h>
#include <iostream>

#include "ankleKinematics.h"

void ankleIK(float alpha, float beta, float &motor_1, float &motor_2)
{
    // Calculates the required position of the motor for the desired pitch (alpha) and roll (beta) using
    // the formula given in 'On the Comprehensive Kinematics Analysis of a Humanoid Parallel Ankle
    // Mechanism' by Chengxu Zhou and Nikos Tsagarakis in the Journal of Mechanisms and Robots, in
    // October 2018, Vol.10. DOI: 10.1115/1.4040886

    // Inputs:
    //            alpha = (float)  : Desired pitch angle in randians
    //            beta  = (float)  : Desired roll angle in randians
    //            Lx    = (float)  : x position where the rod connects to the foot
    //            Ly    = (float)  : y position where the rod connects to the foot
    //            Mx    = (float)  : x position of the motor
    //            My    = (float)  : y position of the motor
    //            Mz    = (float)  : z position of the motor

    //    Output:
    //            gamma = (float)  : The required position of the motor in radians

    float lBar = -40.0; // mm
    float lRod = 192.5; // mm

    // position of the motor
    float rA_y = -57.800;    
    float rA_x = -60.000;
    float rA_z = 186.000;

    // the connection of the rod to the foot
    float rCo_x = -48.94180;
    float rCo_y = -54.33013;
    float rCo_z = 0.0;

    // rC after the foot is rotated
    float rC_x = cos(beta) * rCo_x - sin(beta) * rCo_z;
    float rC_y = sin(alpha) * sin(beta) * rCo_x + cos(alpha) * rCo_y + cos(beta) * sin(alpha) * rCo_z;
    float rC_z = cos(alpha) * sin(beta) * rCo_x - sin(alpha) * rCo_y + cos(alpha) * cos(beta) * rCo_z;

    float a = rC_x - rA_x;
    float b = rA_z - rC_z;
    float c = ((lRod * lRod) - (lBar * lBar) - ((rC_x - rA_x) * (rC_x - rA_x) + (rC_y - rA_y) * (rC_y - rA_y) + (rC_z - rA_z) * (rC_z - rA_z)) ) / (2 * lBar);

    motor_1 = asin((b * c + sqrt(b * b * c * c - (a * a + b * b) * (c * c - a * a))) / (a * a + b * b));


    // ====== Motor 2
    // TODO: optimize!

    rA_y = -rA_y;
    rCo_y = -rCo_y;

    rC_x = cos(beta) * rCo_x - sin(beta) * rCo_z;
    rC_y = sin(alpha) * sin(beta) * rCo_x + cos(alpha) * rCo_y + cos(beta) * sin(alpha) * rCo_z;
    rC_z = cos(alpha) * sin(beta) * rCo_x - sin(alpha) * rCo_y + cos(alpha) * cos(beta) * rCo_z;

    a = rC_x - rA_x;
    b = rA_z - rC_z;
    c = ((lRod * lRod) - (lBar * lBar) - ((rC_x - rA_x) * (rC_x - rA_x) + (rC_y - rA_y) * (rC_y - rA_y) + (rC_z - rA_z) * (rC_z - rA_z)) ) / (2 * lBar);

    motor_2 = asin((b * c + sqrt(b * b * c * c - (a * a + b * b) * (c * c - a * a))) / (a * a + b * b));
}

void ankleFK(float motor_1, float motor_2, float &alpha, float &beta)
{   
    float roll = 0.0;
    float pitch = 0.0;

    float motor_1_measured;
    float motor_2_measured;

    for(int i = 0; i < 10; i++)
    {
        // std::cout << "Iteration number:" << i << std::endl;
        ankleIK(roll, pitch, motor_1_measured, motor_2_measured);

        // std::cout << motor_1_measured / (M_PI / 180.0) << std::endl;
        // std::cout << motor_2_measured / (M_PI / 180.0) << std::endl;

        float motor_1_error = motor_1 - motor_1_measured;
        float motor_2_error = motor_2 - motor_2_measured;

        // std::cout << motor_1_error / (M_PI / 180.0) << std::endl;
        // std::cout << motor_2_error / (M_PI / 180.0) << std::endl;

        pitch += 0.2 * motor_1_error + 0.2 * motor_2_error;
        roll += -0.4 * motor_1_error + 0.4 * motor_2_error;

        // std::cout << roll / (M_PI / 180.0) << std::endl;
        // std::cout << pitch / (M_PI / 180.0) << std::endl;
    }
}

int main()
{
    // // initialise alpha and beta
    // float alpha;
    // float beta;
    // // get the values for alpha and beta
    // std::cout << "Insert the roll angle: ";
    // std::cin >> alpha;
    // std::cout << "Insert the pitch angle: ";
    // std::cin >> beta;

    float motor_1;
    float motor_2;

    // ankleIK(alpha * M_PI / 180.0, beta * M_PI / 180.0, motor_1, motor_2);

    // ankleIK(10.0 * M_PI / 180.0, -10.0 * M_PI / 180.0, motor_1, motor_2);

    // std::cout << "motor 1 = " << (motor_1 / (M_PI / 180.0)) << "\n";
    // std::cout << "motor 2 = " << (motor_2 / (M_PI / 180.0)) << "\n";

    auto start = std::chrono::high_resolution_clock::now();
    ankleFK(-4.0 * (M_PI / 180.0), 7.0 * (M_PI / 180.0));
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << duration.count() << std::endl;

    return 0;
}