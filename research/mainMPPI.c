#include <stdio.h>
#include <time.h>
// #include "myLib.h"


int main() {
    clock_t begin = clock();
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
    //
    // double pathcost[del_r][del_c][tr];
    // fill3D(del_r, del_c, tr, pathcost, 0.0);
    //
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

    //Placeholders
    double azs[xvec_c];
    fill1D(xvec_c, azs, 0);

    double ws[xvec_c];
    fill1D(xvec_c, ws, 0);

    double zs[xvec_c];
    fill1D(xvec_c, zs, 0);

    double xs[xvec_c];
    fill1D(xvec_c, xs, 0);

    double ys[xvec_c];
    fill1D(xvec_c, ys, 0);

    double np[xvec_c];
    fill1D(xvec_c, np, 0);

    //Inertial calculations for initizlizations
    double out_states[6][400];
    out_states[0][0] = cos(theta0) * cos(psi0) * u0 + (sin(phi0) * sin(theta0) * cos(psi0) - cos(phi0) * sin(psi0)) * v0 + (cos(phi0) * sin(theta0) * cos(psi0) + sin(phi0) * sin(psi0)) * w0;
    out_states[1][0] = -(cos(theta0)*sin(psi0)*u0 + (sin(phi0)*sin(theta0)*sin(psi0) + cos(phi0)*cos(psi0))*v0 + (cos(phi0)*sin(theta0)*sin(psi0) - sin(phi0)*cos(psi0))*w0);
    out_states[2][0] = -(-sin(theta0)*u0 + sin(phi0)*cos(theta0)*v0 + cos(phi0)*cos(theta0)*w0);
    out_states[3][0] = xvec[0][0];
    out_states[4][0] = xvec[1][0];
    out_states[5][0] = xvec[2][0];

    // out_states_p=repmat(out_states,[1 1 tr]);  %Predicted output states
    int out_states_r = sizeof(out_states) / sizeof(out_states[0]);
    int out_states_c = sizeof(out_states[0]) / sizeof(out_states[0][0]);
    double out_states_p[out_states_r][out_states_c][tr];
    for (int k = 0; k < tr; k++) {
        for (int i = 0; i < out_states_r; i++) {
            for (int j = 0; j < out_states_c; j++) {
                out_states_p[i][j][k] = out_states[i][j];
            }
        }
    }

    double cost = 0;

    int s = 0;

    while (s < xvec_c) {
        srand(time(0));
        double A = sdz * rand();
        double f = 0.09722 * (double)rand() / 10000000000.0 + 0.125;
        int T_2 = round((1.0 / (dt * f)) / 2);
        int hp = (s - 1) + T_2;

        if (hp < xvec_c) {
            while (s <= hp) {
                azs[s] = A * sin(2 * 3.14 * f * tvec[s]); //Calculates peak
                s++;
            }
        } else {
            hp = xvec_c;
            while (s <= hp) {
                azs[s] = A * sin(2 * 3.14 * f * tvec[s]); //Calculates peak
                s++;
            }
        }
        srand(time(0));
        A = sdz * rand();
        int ep = s + T_2;

        if (ep < xvec_c) {
            s++;
            while (s <= ep) {
                azs[s] = A * sin(2 * 3.14 * f * tvec[s]); //Calculates valley
                s++;
            }
        } else {
            s++;
            ep = xvec_c;
            while (s <= hp) {
                azs[s] = A * sin(2 * 3.14 * f * tvec[s]); //Calculates valley
                s++;
            }
        }
        // break;


    }

    //Calculate ship's vertical velocty and Position
    s = 1;
    while (s <= xvec_c - 1) {
        ws[s] = azs[s] * dt;
        zs[s] = zs[s - 1] + ws[s] * dt;
        s++;
    }

    //SHIP'S HORIZONTAL MOTION MODEL
    double us[xvec_c];
    for (int i = 0; i < xvec_c; i++) {
        srand(time(0));
        us[i] = sdx * rand();
    }
    s = 1;
    while (s <= xvec_c - 1) {
        xs[s] = xs[s - 1] + us[s] * dt;
        s++;
    }

    // SHIP'S LATERAL MOTION MODEL
    double vs[xvec_c];
    s = 1;
    while (s <= xvec_c - 1) {
        ys[s] = ys[s - 1] + vs[s] * dt;
        s++;
    }

    //SENSITIVITY SELECTION FOR TRAJECTORY GENERATION
    double rb[xvec_c];
    double ra[xvec_c];
    double rp[xvec_c];
    double rc[xvec_c];
    fill1D(xvec_c, rb, 0.1);                //Sensitivity of differential commanded longitudinal cyclic, starts at 2% of initial optimal longitudinal cyclic
    fill1D(xvec_c, ra, 0.1);                //Sensitivity of differential commanded lateral cyclic, starts at 2% of initial optimal lateral cyclic
    fill1D(xvec_c, rp, 0.1);                //Sensitivity of differential commanded pedal, starts at 2% of initial optimal pedal
    fill1D(xvec_c, rc, 0.1);                //Sensitivity of differential commanded collective, starts at 10% of initial optimal collective
    double F[(int) (model_time/time_step)];
    logspace(-2, 0.5, (int) (model_time/time_step), F);    //weight used to trade between path cost and terminal cost

    //PATH INTEGRAL MODEL
    //R E M I N D E R index from MATLAB is 1
    //meaning everything will now be -1 the index from MATLAB

    for (int j = 2; j < xvec_c; j++) {
        int n = tr;
        int m = 1;
        // while (n > 90 && m < 10) {
        //     delbmax = rb
        // }

         double ** delbmax = (double **) malloc(del_c * sizeof(double *));
         double ** delamax = (double **) malloc(del_c * sizeof(double *));
         double ** delpmax = (double **) malloc(del_c * sizeof(double *));
         double ** delcmax = (double **) malloc(del_c * sizeof(double *));

         double ** ddelb = (double **) malloc(del_c * sizeof(double *));
         double ** ddela = (double **) malloc(del_c * sizeof(double *));
         double ** ddelp = (double **) malloc(del_c * sizeof(double *));
         double ** ddelc = (double **) malloc(del_c * sizeof(double *));

         double ** del_bi = (double **) malloc(del_c * sizeof(double *));
         double ** del_ai = (double **) malloc(del_c * sizeof(double *));
         double ** del_pi = (double **) malloc(del_c * sizeof(double *));
         double ** del_ci = (double **) malloc(del_c * sizeof(double *));


         for (int i = 0; i < del_c; i++) {
             delbmax[i] = (double *) malloc(tr * sizeof(double));
             delamax[i] = (double *) malloc(tr * sizeof(double));
             delpmax[i] = (double *) malloc(tr * sizeof(double));
             delcmax[i] = (double *) malloc(tr * sizeof(double));

             ddelb[i] = (double *) malloc(tr * sizeof(double));
             ddela[i] = (double *) malloc(tr * sizeof(double));
             ddelp[i] = (double *) malloc(tr * sizeof(double));
             ddelc[i] = (double *) malloc(tr * sizeof(double));

             del_bi[i] = (double *) malloc(tr * sizeof(double));
             del_ai[i] = (double *) malloc(tr * sizeof(double));
             del_pi[i] = (double *) malloc(tr * sizeof(double));
             del_ci[i] = (double *) malloc(tr * sizeof(double));
         }


    }

    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("Before Path Integral took : %f seconds \n", time_spent);





}
