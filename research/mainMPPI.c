#include <stdio.h>

int main() {
    double Afull[12][12] = {{0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.9988, -0.0009, 0.0493, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 3.3325, 0.0000, -67.5899, 0.0000, -0.9998, -0.0175, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 0.0000, 67.5899, 0.0000, 0.0493, 0.0175, -0.9986, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, -0.0009, 0.0494},
    {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.9998, 0.0175},
    {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0176, 1.0011},
    {-0.0000, -0.0000, 0.0002, -1.7910, -28.7478, 0.0000, -0.0156, -0.0014, 0.0264, -1.0130, 1.1241, -0.4113},
    {0.0000, 0.0000, 0.0001, 33.1424, -0.3921, -0.0000, 0.0079, -0.1015, 0.0165, -1.5512, -1.5106, -65.9068},
    {-0.0000, -0.0000, -0.0033, -2.6711, 1.7629, -0.0000, -0.1352, 0.0043, -0.6177, 8.8359, 68.2588, 2.0707},
    {0.0000, 0.0000, 0.0002, -0.7252, -0.6595, -0.0000, 0.0058, -0.0292, 0.0424, -7.1172, -1.6572, 0.1983},
    {-0.0000, -0.0000, 0.0000, 0.0642, -0.7286, 0.0000, 0.0019, 0.0059, -0.0013, 0.0097, -1.5146, -0.0941},
    {-0.0000, -0.0000, 0.0000, -0.8489, 0.0071, 0.0000, -0.0036, 0.0167, -0.0027, -0.1995, -0.0263, -0.6115}};

    double Bfull[12][4] = {{0.0000, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 0.0000},
    {0.0000, 0.0000, 0.0000, 0.0000},
    {-0.1577, -0.0046, 0.0529, 0.0601},
    {0.0197, 0.0977, -0.0752, 0.0082},
    {-0.1559, 0.0106, 0.0865, -0.8894},
    {0.0307, 0.1185, -0.0356, 0.0067},
    {0.0339, 0.0007, -0.0022, 0.0024},
    {-0.0004, 0.0037, 0.0222, 0.0058}};

    double u0 = 67.5077;       //reference velocity along body X axis, ft/sec (positive forward)
    double v0 = -0.0585;       //reference velocity along body Y axis, ft/sec (positive right)
    double w0 = 3.3319;        //reference velocity along body Z axis, ft/sec (positive down)
    double phi0 = -0.0175;     //reference roll attitude, rad
    double theta0 = 0.0493;    //reference pitch attitude, rad
    double psi0 = 0.0;           //reference yaw attitude, rad
    double del_b0 = 39.9557;   //trim longitudinal cyclic, percent
    double del_a0 = 53.1125;   //trim lateral cyclic, percent
    double del_p0 = 48.6692;   //trim pedal, percent
    double del_c0 = 34.5226;   //trim collective, percent
    double trim_val[6];
    trim_val[0] = u0;
    trim_val[1] = v0;
    trim_val[2] = w0;
    trim_val[3] = phi0;
    trim_val[4] = theta0;
    trim_val[5] = psi0;

    int model_time=40;  //Enter time for vehicle to land in seconds (total simulation time)
    double time_step=0.1;  //Enter time step in seconds
    int tr=100;         //Enter number of random trajectories
    int band_z=5;       //final z-position error range, ft
    int band_y=5;       //final y-position error range, ft
    int band_x=5;       //final x-position error range, ft
    int band_w=3;     //final z-velocity error range, ft/sec
    int band_v=3;     //final y-velocity error range, ft/sec
    int band_u=3;     //final x-velocity error range, ft/sec
    double rb_inc=0.1;    //ddelb sensitivity increment, percentage of optimal trajectory
    double ra_inc=0.1;    //ddela sensitivity increment, percentage of optimal trajectory
    double rp_inc=0.1;    //ddelp sensitivity increment, percentage of optimal trajectory
    double rc_inc=0.1;     //ddelc sensitivity increment, percentage of optimal trajectory
    //us0=8.14;        //Ship's average forward velocity (ft/sec), LOW level sea state
    double us0=33.49;        //Ship's average forward velocity (ft/sec), MEDIUM level sea state
    //us0=49.98;        //Ship's average forward velocity (ft/sec), HIGH level sea state

    ////SEA STATE SELECTION
    //sdz=1.14;                //Standard deviation for ship's z-acceleration (ft/sec^2), LOW level sea state
    double sdz=2.43;                //Standard deviation for ship's z-acceleration (ft/sec^2), MEDIUM level sea state
    //sdz=5.13;                //Standard deviation for ship's z-acceleration (ft/sec^2), HIGH level sea state
    //sdy=0.736;                //Standard deviation for ship's y-velocity (ft/sec), MEDIUM level sea state
    double sdy=0.89;                //Standard deviation for ship's y-velocity (ft/sec), MEDIUM level sea state
    //sdy=1.76;                //Standard deviation for ship's y-velocity (ft/sec), MEDIUM level sea state
    //sdx=0.181;              //Standard deviation for ship's x-velocity (ft/sec^2), LOW level sea state
    double sdx=0.363;              //Standard deviation for ship's x-velocity (ft/sec^2), MEDIUM level sea state
    //sdx=0.726;              //Standard deviation for ship's x-velocity (ft/sec^2), HIGH level sea state

    //BVP Optimal Solution legend
    //xvec(1)=X(t)       Perturbation in Vehicle forward position (X-inertial, positive forward)
    //xvec(2)=Y(t)       Perturbation in Vehicle sideward position (Y-inertial, positive left)
    //xvec(3)=Z(t)       Perturbation in Vehicle vertical position (Z-inertial, positive up)
    //xvec(4)=phi(t)     Perturbation in Vehicle roll attitude
    //xvec(5)=theta(t)   Perturbation in Vehicle pitch attitude
    //xvec(6)=psi(t)     Perturbation in Vehicle yaw attitude
    //xvec(7)=u(t)       Perturbation in Vehicle longitudinal velocity along body X axis
    //xvec(8)=v(t)       Perturbation in Vehicle lateral velocity along body X axis
    //xvec(9)=w(t)       Perturbation in Vehicle vertical velocity along body Z axis
    //xvec(10)=p(t)      Perturbation in Vehicle roll rate
    //xvec(11)=q(t)      Perturbation in Vehicle pitch rate
    //xvec(12)=r(t)      Perturbation in Vehicle yaw rate

    //Outputs Solution legend
    //out_states(1)=U(t)  Vehicle forward velocity (X-inertial)
    //out_states(2)=V(t)  Vehicle lateral velocity (Y-inertial)
    //out_states(3)=W(t)  Vehicle vertical velocity (Z-inertial)
    //out_states(4)=X(t)  Vehicle forward position (X-inertial)
    //out_states(5)=Y(t)  Vehicle lateral position (Y-inertial)
    //out_states(6)=Z(t)  Vehicle vertical position (Z-inertial)

    // [xvec,del,tvec,Xeq,xseq,X_optdel,X_opt,Y_opt,Z_opt,U_opt,V_opt,W_opt,inert_val] = optimal_control_40_full_rel(
    //     model_time,time_step,Afull,Bfull,trim_val,us0);  %Optimal control and reference paths

    double xvec[12][400];
    double tvec[400];
    double del[4][400];
    double Xeq[400];
    double xseq[400];
    double U_opt[400];
    double V_opt[400];
    double W_opt[400];
    double X_opt[400];
    double X_optdel[400];
    double Y_opt[400];
    double Z_opt[400];
    double inert_val[6];
    int length = 400;

    int xvec_r = sizeof(xvec) / sizeof(xvec[0]);
    int xvec_c = sizeof(xvec[0]) / sizeof(xvec[0][0]);
    int del_r = sizeof(del) / sizeof(del[0]);
    int del_c = sizeof(del[0]) / sizeof(del[0][0]);
    int Afull_c = sizeof(Afull[0])/sizeof(Afull[0][0]);
    int Afull_r = sizeof(Afull) / sizeof(Afull[0]);
    int Bfull_c = sizeof(Bfull[0])/sizeof(Bfull[0][0]);
    int Bfull_r = sizeof(Bfull) / sizeof(Bfull[0]);

    optimal_control_40_full_rel(xvec_r, xvec_c, xvec, tvec,
        del_r, del_c,
        del, Xeq, xseq, U_opt, V_opt,
        W_opt, X_opt, X_optdel, Y_opt, Z_opt, inert_val,
        model_time, time_step, Afull_r, Afull_c, Afull, Bfull_r,
        Bfull_c, Bfull, trim_val, us0, length);

    double dt = tvec[length - 1] / length;    //time step (sec)
    double U0_opt = inert_val[0];
    double V0_opt = inert_val[1];
    double W0_opt = inert_val[2];
    double X0_opt = inert_val[3];
    double Y0_opt = inert_val[4];
    double Z0_opt = inert_val[5];

    //VARIABLE INITIALIZATION
    //del_con=repmat(del,[1 1 tr]);     %initial optimal command
    double del_con[del_r][del_c][tr];
    for (int k = 0; k < tr; k++) {
        for (int i = 0; i < del_r; i++) {
            for (int j = 0; j < del_c; j++) {
                del_con[i][j][k] = del[i][j];
            }
        }
    }
    // double **states = &xvec[0][0];
    // double states[xvec_r][xvec_c];
    // for (int i = 0; i < xvec_r; i++) {
    //     for (int j = 0; j < xvec_c; j++) {
    //         states[i][j] = xvec[i][j];
    //     }
    // }
    // states_p=repmat(xvec,[1 1 tr]);   %Predicted perturbation in states
    double states_p[xvec_r][xvec_c][tr];
    for (int k = 0; k < tr; k++) {
        for (int i = 0; i < xvec_r; i++) {
            for (int j = 0; j < xvec_c; j++) {
                states_p[i][j][k] = xvec[i][j];
            }
        }
    }

    // double pathcost[del_r][del_c][tr];
    // fill3D(del_r, del_c, tr, pathcost, 0.0);

    // double Ji[del_r][del_c][tr];
    // fill3D(del_r, del_c, tr, Ji, 1.0);

    // double Ji_mod[del_r][del_c][tr];
    // fill3D(del_r, del_c, tr, Ji_mod, 1.0);

    // double ddeli_e[del_r][del_c][tr];
    // fill3D(del_r, del_c, tr, ddeli_e, 0.0);

    // double out_states[6][xvec_c];
    // fill2D(6, xvec_c, out_states, 0.0);

    // double cycles[sizeof(tvec) / sizeof(tvec[0]);];
    // for (int i = 0; i < sizeof(tvec) / sizeof(tvec[0]; i++) {
    //     cycles[i] = 0.0;
    // }
}
