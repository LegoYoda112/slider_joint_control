from numpy import pi, arcsin, sqrt, cos , sin, arctan2, sqrt

# Given roll and pitch of the ankle, find desired motor positions
def ik(roll, pitch):
    motor_left = calcGamma(roll, pitch, motor = 'left')
    motor_right = calcGamma(roll, pitch, motor = 'right')
    return [motor_left, motor_right]


def calcGamma(alpha, beta, motor = 'right'):
    """
    Calculates the required position of the motor for the desired pitch (alpha) and roll (beta)
        Inputs: 
                alpha = (float)  : Desired pitch angle in randians
                beta  = (float)  : Desired roll angle in randians
                motor = (string) : Left or right motor, default is right 
        Output:
                gamma = (float)  : The required position of the motor in radians
    """
    # position of the connection with no pitch or roll
    Al0_x = 64.0
    Al0_y = 103.0
    Al0_z = 0.0
 
    # position of the motors
    Ml_x = 54.0
    Ml_y = 60.0
    Ml_z = 196.0

    if motor =='left':
        # poscition of the connection with the desired pitch and roll
        Al_x = cos(beta) * Al0_x - sin(beta)*Al0_z
        Al_y = sin(alpha) * sin(beta) * Al0_x + cos(alpha) * Al0_y + cos(beta) * sin(alpha) * Al0_z
        Al_z = cos(alpha) * sin(beta) * Al0_x - sin(alpha) * Al0_y + cos(alpha) * cos(beta) * Al0_z
        
    else:
        # poscition of the connection with the desired pitch and roll
        Al_x = cos(beta) * -Al0_x - sin(beta)*Al0_z
        Al_y = sin(alpha) * sin(beta) * -Al0_x + cos(alpha) * Al0_y + cos(beta) * sin(alpha) * Al0_z
        Al_z = cos(alpha) * sin(beta) * -Al0_x - sin(alpha) * Al0_y + cos(alpha) * cos(beta) * Al0_z
        Ml_x = - Ml_x
    bz = calcBz(Al_x, Al_y, Al_z, Ml_x, Ml_y, Ml_z)

    return arcsin((bz - Ml_z)/40.0)

def calcGammaPaper(alpha, beta , motor):
    """
    Calculates the required position of the motor for the desired pitch (alpha) and roll (beta) using
    the formula given in 'On the Comprehensive Kinematics Analysis of a Humanoid Parallel Ankle 
    Mechanism' by Chengxu Zhou and Nikos Tsagarakis in the Journal of Mechanisms and Robots, in
    October 2018, Vol.10. DOI: 10.1115/1.4040886 
        Inputs: 
                alpha = (float)  : Desired pitch angle in randians
                beta  = (float)  : Desired roll angle in randians
                motor = (string) : Left or right motor, default is right 
        Output:
                gamma = (float)  : The required position of the motor in radians
    """
    lBar = 40.0
    lRod = 195.0

    ## original positions 

    # the motor position
    rA_x = - 60.0
    rA_y = 60.0
    rA_z = 184.0

    # the connection of the rod to the foot
    rCo_x = - 103.0
    rCo_y = 65.0
    rCo_z = 0.0

    # positions after rotation

    # connection to the foot

    if motor =='left':
        rC_x = cos(-alpha) * rCo_x + rCo_y*sin(-alpha)*sin(beta) - sin(-alpha)*cos(beta)*rCo_z
        rC_y = cos(beta) * rCo_y + sin(beta) * rCo_z
        rC_z = sin(-alpha) * rCo_x - sin(beta) * cos(-alpha) * rCo_y + cos(-alpha) * cos(beta) * rCo_z
    else:
        rC_x = cos(-alpha) * rCo_x - rCo_y*sin(-alpha)*sin(beta) - sin(-alpha)*cos(beta)*rCo_z
        rC_y = cos(beta) * -rCo_y + sin(beta) * rCo_z
        rC_z = sin(-alpha) * rCo_x + sin(beta) * cos(-alpha) * rCo_y + cos(-alpha) * cos(beta) * rCo_z
        rA_y = - rA_y

    a = rC_x - rA_x
    b = rA_z - rC_z
    c = (lRod**2 - lBar**2 - ((rC_x - rA_x)**2 + (rC_y - rA_y)**2 + (rC_z - rA_z)**2) )/ (2*lBar)

    gamma = arcsin( (b*c + sqrt(b**2 * c**2 - (a**2+b**2)*(c**2 - a**2))) / (a**2 + b**2) )

    return gamma

def calcBz(A_x,A_y,A_z,H_x,M_y,M_z):

    L = 195.0
    R = 40.0

    t2 = A_x**2;
    t3 = A_x**3;
    t5 = A_y**2;
    t6 = A_y**3;
    t8 = A_z**2;
    t9 = A_z**3;
    t11 = H_x**2;
    t12 = H_x**3;
    t14 = L**2;
    t16 = M_y**2;
    t17 = M_y**3;
    t19 = M_z**2;
    t20 = M_z**3;
    t22 = R**2;
    t24 = A_y*M_y*2.0;
    t25 = A_z*M_z*2.0;
    t26 = A_x*A_z*H_x*2.0;
    t28 = A_x*H_x*M_z*2.0;
    t55 = A_y*A_z*M_y*-2.0;
    t56 = A_y*M_y*M_z*-2.0;
    t69 = A_x*A_y*H_x*M_y*8.0;
    t70 = A_x*A_z*H_x*M_z*8.0;
    t71 = A_y*A_z*M_y*M_z*8.0;
    t4 = t2**2;
    t7 = t5**2;
    t10 = t8**2;
    t13 = t11**2;
    t15 = t14**2;
    t18 = t16**2;
    t21 = t19**2;
    t23 = t22**2;
    # t27 = A_z*t24;
    # t29 = M_z*t24;
    t30 = A_z*t2;
    t31 = A_z*t5;
    t32 = A_z*t11;
    t33 = A_z*t14;
    t34 = M_z*t2;
    t35 = A_z*t16;
    t36 = M_z*t5;
    t37 = A_z*t19;
    t38 = M_z*t8;
    t39 = A_z*t22;
    t40 = M_z*t11;
    t41 = M_z*t14;
    t42 = M_z*t16;
    t43 = M_z*t22;
    t44 = -t24;
    t45 = -t25;
    t54 = -t26;
    t57 = A_x*t12*4.0;
    t58 = H_x*t3*4.0;
    t59 = A_y*t17*4.0;
    t60 = M_y*t6*4.0;
    t61 = A_z*t20*4.0;
    t62 = M_z*t9*4.0;
    t72 = A_x*H_x*t5*4.0;
    t73 = A_x*H_x*t8*4.0;
    t74 = A_y*M_y*t2*4.0;
    t75 = A_y*M_y*t8*4.0;
    t78 = A_x*H_x*t14*4.0;
    t79 = A_x*H_x*t16*4.0;
    t80 = A_x*H_x*t19*4.0;
    t81 = A_y*M_y*t11*4.0;
    t83 = A_y*M_y*t14*4.0;
    t85 = A_x*H_x*t22*4.0;
    t86 = A_y*M_y*t19*4.0;
    t88 = A_y*M_y*t22*4.0;
    t90 = -t69;
    t91 = -t70;
    t92 = -t71;
    t98 = t2*t5*2.0;
    t99 = t2*t8*2.0;
    t100 = t5*t8*2.0;
    t101 = t2*t11*6.0;
    t102 = t5*t11*2.0;
    t103 = t8*t11*2.0;
    t104 = t2*t14*2.0;
    t105 = t5*t14*2.0;
    t106 = t8*t14*2.0;
    t107 = t2*t16*2.0;
    t108 = t2*t19*2.0;
    t109 = t5*t16*6.0;
    t110 = t5*t19*2.0;
    t111 = t8*t16*2.0;
    t112 = t8*t19*6.0;
    t113 = t2*t22*2.0;
    t114 = t5*t22*2.0;
    t115 = t8*t22*2.0;
    t116 = t11*t14*2.0;
    t117 = t11*t16*2.0;
    t118 = t11*t19*2.0;
    t119 = t14*t16*2.0;
    t120 = t14*t19*2.0;
    t121 = t11*t22*2.0;
    t122 = t16*t19*2.0;
    t123 = t14*t22*2.0;
    t124 = t16*t22*2.0;
    t125 = t19*t22*2.0;
    t46 = -t4;
    t47 = -t7;
    t48 = -t10;
    t49 = -t13;
    t50 = -t15;
    t51 = -t18;
    t52 = -t21;
    t53 = -t23;
    t63 = -t33;
    t64 = -t34;
    t65 = -t37;
    t66 = -t38;
    t67 = -t40;
    t68 = -t43;
    t76 = M_z*t30*4.0;
    t77 = M_z*t31*4.0;
    t82 = M_z*t32*4.0;
    t84 = M_z*t33*4.0;
    t87 = M_z*t35*4.0;
    t89 = M_z*t39*4.0;
    t93 = -t78;
    t94 = -t83;
    t96 = -t88;
    t126 = -t98;
    t127 = -t99;
    t128 = -t100;
    t129 = -t101;
    t130 = -t102;
    t131 = -t103;
    t132 = -t107;
    t133 = -t108;
    t134 = -t109;
    t135 = -t110;
    t136 = -t111;
    t137 = -t112;
    t138 = -t113;
    t139 = -t117;
    t140 = -t118;
    t141 = -t121;
    t142 = -t122;
    t143 = t5+t8+t16+t19+t44+t45;
    t95 = -t84;
    t97 = -t89;
    t144 = 1.0/t143;
    t145 = t46+t47+t48+t49+t50+t51+t52+t53+t57+t58+t59+t60+t61+t62+t72+t73+t74+t75+t76+t77+t79+t80+t81+t82+t85+t86+t87+t90+t91+t92+t93+t94+t95+t96+t97+t104+t105+t106+t114+t115+t116+t119+t120+t123+t124+t125+t126+t127+t128+t129+t130+t131+t132+t133+t134+t135+t136+t137+t138+t139+t140+t141+t142;
    t146 = sqrt(t145);
    t147 = A_y*t146;
    t148 = M_y*t146;

    return (t144*(t9+t20+t28+t30+t31+t32+t35+t36+t39+t41+t42+t54+t55+t56+t63+t64+t65+t66+t67+t68+t147-t148))/2.0

def calcBy(A_x,A_y,A_z,H_x,M_y,M_z):

    L = 195.0
    R = 40.0

    t2 = A_x**2;
    t3 = A_x**3;
    t5 = A_y**2;
    t6 = A_y**3;
    t8 = A_z**2;
    t9 = A_z**3;
    t11 = H_x**2;
    t12 = H_x**3;
    t14 = L**2;
    t16 = M_y**2;
    t17 = M_y**3;
    t19 = M_z**2;
    t20 = M_z**3;
    t22 = R**2;
    t24 = A_y*M_y*2.0;
    t25 = A_z*M_z*2.0;
    t26 = A_x*A_y*H_x*2.0;
    t28 = A_x*H_x*M_y*2.0;
    t55 = A_y*A_z*M_z*-2.0;
    t56 = A_z*M_y*M_z*-2.0;
    t69 = A_x*A_y*H_x*M_y*8.0;
    t70 = A_x*A_z*H_x*M_z*8.0;
    t71 = A_y*A_z*M_y*M_z*8.0;
    t4 = t2**2;
    t7 = t5**2;
    t10 = t8**2;
    t13 = t11**2;
    t15 = t14**2;
    t18 = t16**2;
    t21 = t19**2;
    t23 = t22**2;
    # t27 = A_y*t25;
    # t29 = M_y*t25;
    t30 = A_y*t2;
    t31 = A_y*t8;
    t32 = A_y*t11;
    t33 = A_y*t14;
    t34 = M_y*t2;
    t35 = A_y*t16;
    t36 = M_y*t5;
    t37 = A_y*t19;
    t38 = M_y*t8;
    t39 = A_y*t22;
    t40 = M_y*t11;
    t41 = M_y*t14;
    t42 = M_y*t19;
    t43 = M_y*t22;
    t44 = -t24;
    t45 = -t25;
    t54 = -t26;
    t57 = A_x*t12*4.0;
    t58 = H_x*t3*4.0;
    t59 = A_y*t17*4.0;
    t60 = M_y*t6*4.0;
    t61 = A_z*t20*4.0;
    t62 = M_z*t9*4.0;
    t72 = A_x*H_x*t5*4.0;
    t73 = A_x*H_x*t8*4.0;
    t76 = A_z*M_z*t2*4.0;
    t77 = A_z*M_z*t5*4.0;
    t78 = A_x*H_x*t14*4.0;
    t79 = A_x*H_x*t16*4.0;
    t80 = A_x*H_x*t19*4.0;
    t82 = A_z*M_z*t11*4.0;
    t84 = A_z*M_z*t14*4.0;
    t85 = A_x*H_x*t22*4.0;
    t87 = A_z*M_z*t16*4.0;
    t89 = A_z*M_z*t22*4.0;
    t90 = -t69;
    t91 = -t70;
    t92 = -t71;
    t98 = t2*t5*2.0;
    t99 = t2*t8*2.0;
    t100 = t5*t8*2.0;
    t101 = t2*t11*6.0;
    t102 = t5*t11*2.0;
    t103 = t8*t11*2.0;
    t104 = t2*t14*2.0;
    t105 = t5*t14*2.0;
    t106 = t8*t14*2.0;
    t107 = t2*t16*2.0;
    t108 = t2*t19*2.0;
    t109 = t5*t16*6.0;
    t110 = t5*t19*2.0;
    t111 = t8*t16*2.0;
    t112 = t8*t19*6.0;
    t113 = t2*t22*2.0;
    t114 = t5*t22*2.0;
    t115 = t8*t22*2.0;
    t116 = t11*t14*2.0;
    t117 = t11*t16*2.0;
    t118 = t11*t19*2.0;
    t119 = t14*t16*2.0;
    t120 = t14*t19*2.0;
    t121 = t11*t22*2.0;
    t122 = t16*t19*2.0;
    t123 = t14*t22*2.0;
    t124 = t16*t22*2.0;
    t125 = t19*t22*2.0;
    t46 = -t4;
    t47 = -t7;
    t48 = -t10;
    t49 = -t13;
    t50 = -t15;
    t51 = -t18;
    t52 = -t21;
    t53 = -t23;
    t63 = -t33;
    t64 = -t34;
    t65 = -t35;
    t66 = -t36;
    t67 = -t40;
    t68 = -t43;
    t74 = M_y*t30*4.0;
    t75 = M_y*t31*4.0;
    t81 = M_y*t32*4.0;
    t83 = M_y*t33*4.0;
    t86 = M_y*t37*4.0;
    t88 = M_y*t39*4.0;
    t93 = -t78;
    t95 = -t84;
    t97 = -t89;
    t126 = -t98;
    t127 = -t99;
    t128 = -t100;
    t129 = -t101;
    t130 = -t102;
    t131 = -t103;
    t132 = -t107;
    t133 = -t108;
    t134 = -t109;
    t135 = -t110;
    t136 = -t111;
    t137 = -t112;
    t138 = -t113;
    t139 = -t117;
    t140 = -t118;
    t141 = -t121;
    t142 = -t122;
    t143 = t5+t8+t16+t19+t44+t45;
    t94 = -t83;
    t96 = -t88;
    t144 = 1.0/t143;
    t145 = t46+t47+t48+t49+t50+t51+t52+t53+t57+t58+t59+t60+t61+t62+t72+t73+t74+t75+t76+t77+t79+t80+t81+t82+t85+t86+t87+t90+t91+t92+t93+t94+t95+t96+t97+t104+t105+t106+t114+t115+t116+t119+t120+t123+t124+t125+t126+t127+t128+t129+t130+t131+t132+t133+t134+t135+t136+t137+t138+t139+t140+t141+t142;
    t146 = sqrt(t145);
    t147 = A_z*t146;
    t148 = M_z*t146;

    return (t144*(t6+t17+t28+t30+t31+t32+t37+t38+t39+t41+t42+t54+t55+t56+t63+t64+t65+t66+t67+t68-t147+t148))/2.0
