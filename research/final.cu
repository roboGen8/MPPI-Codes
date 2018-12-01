
#include <string.h>
#include "myLib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
// #include "libxl.h"

// time=(xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','A2:A401')).';

// predicted_vehicle_state_trapz(int states_p_r, int states_p_c, double states_p[][states_p_c],
//     int out_states_p_r, int out_states_p_c, double out_states_p[][out_states_p_c],
//     int del_r, int del_c, double del_bi[][del_c], double del_ai[][del_c], double del_pi[][del_c],
//     double del_ci[][del_c], double dt, int j, int Afull_r, int Afull_c, double Afull[][Afull_c],
//     int Bfull_r, int Bfull_c, double Bfull[][Bfull_c], double trim_val[]) {
//
//
// }


extern "C" void optimal_control_40_full_rel(int xvec_r, int xvec_c, double xvec[][xvec_c], double tvec[],
    int del_r, int del_c,
    double del[][del_c], double Xeq[], double xseq[], double U_opt[], double V_opt[],
    double W_opt[], double X_opt[], double X_optdel[], double Y_opt[], double Z_opt[], double inert_val[],
    int model_time, double time_step, int Afull_r, int Afull_c, double Afull[][Afull_c], int Bfull_r,
    int Bfull_c, double Bfull[][Bfull_c], double trim_val[], double us0, int length) {


        double u0 = trim_val[0];
        double v0 = trim_val[1];
        double w0 = trim_val[2];
        double phi0 = trim_val[3];
        double theta0 = trim_val[4];
        double psi0 = trim_val[5];

        int tf = model_time;

        // int length = sizeof(time) / sizeof(time[0]);
        // double A_data[length];
        double X_opt_rel[length];
        double U_opt_rel[length];
        double Y_opt_rel[length];
        double V_opt_rel[length];
        double Z_opt_rel[length];
        double W_opt_rel[length];
        //Reading data

        char filename[] = "Medium_40s.csv";
        readCSV(filename, length, tvec, X_opt_rel, U_opt_rel, Y_opt_rel, V_opt_rel, Z_opt_rel, W_opt_rel);


        double U_max = 0.1;
        double V_max = 1.5;
        double W_max = 0.1;
        double X_max = 25000;
        double Y_max = 1000;
        double Z_max = 10;
        double delb_max = 1;
        double delc_max = 1;
        double dela_max = 1;
        double delp_max = 0.5;

        // Xeq=zeros(1,size(time,2));
        fill1D(sizeof(tvec) / sizeof(tvec[0]), Xeq, 0.0);
        // xseq=zeros(1,size(time,2));
        fill1D(sizeof(tvec) / sizeof(tvec[0]), xseq, 0.0);
        // Snew=zeros(12,size(time,2)*12);
        double Snew[12][12 * sizeof(tvec) / sizeof(tvec[0])];
        fill2D(12, 12 * sizeof(tvec) / sizeof(tvec[0]), Snew, 0.0);
        // g=zeros(12,size(time,2));
        double g[12][sizeof(tvec) / sizeof(tvec[0])];
        fill2D(12, sizeof(tvec) / sizeof(tvec[0]), Snew, 0.0);

        for (int i = 0; i < sizeof(tvec) / sizeof(tvec[0]); i++) {
            Xeq[i] = u0 * tvec[i];
            xseq[i] = us0 * tvec[i];
        }

        double X0_opt = X_opt_rel[0] + xseq[0];
        X_opt[0] = 0;
        X_optdel[0] = 0;
        X_opt[length - 1] = X_opt_rel[length - 1] + xseq[length - 1] - X0_opt;

        double U0_opt = U_opt_rel[0] + us0;
        U_opt[0] = 0;

        double Y0_opt = Y_opt_rel[0];
        Y_opt[0] = 0;

        double V0_opt = V_opt_rel[0];
        V_opt[0] = 0;

        double Z0_opt = Z_opt_rel[0];
        Z_opt[0] = 0;

        double W0_opt = W_opt_rel[0];
        W_opt[0] = 0;

        double scale = X_opt[length - 1] / (X_opt[length - 1] - Xeq[length - 1]);
        for (int i = 1; i < length; i++) {
            X_opt[i] = X_opt_rel[i] + xseq[i] - X0_opt;
            X_optdel[i] = X_opt[i] / scale;
            U_opt[i] = U_opt_rel[i] + us0 -  U0_opt;
            Y_opt[i] = Y_opt_rel[i] - Y0_opt;
            V_opt[i] = V_opt_rel[i] - V0_opt;
            Z_opt[i] = Z_opt_rel[i] - Z0_opt;
            W_opt[i] = W_opt_rel[i] - W0_opt;
        }

        inert_val[0] = U0_opt;
        inert_val[1] = V0_opt;
        inert_val[2] = W0_opt;
        inert_val[3] = X0_opt;
        inert_val[4] = Y0_opt;
        inert_val[5] = Z0_opt;

        double Q[6][6];
        //This is actually Q inverse
        eye(6, Q);
        Q[0][0] = (tf * U_max);
        Q[1][1] = (tf * U_max);
        Q[2][2] = (tf * U_max);
        Q[3][3] = (tf * U_max);
        Q[4][4] = (tf * U_max);
        Q[5][5] = (tf * U_max);

        double R[4][4];
        eye(4, R);
        //This is actually R inverse
        R[0][0] = (tf * delb_max);
        R[1][1] = (tf * dela_max);
        R[2][2] = (tf * delp_max);
        R[3][3] = (tf * delc_max);

        // printf("helloooooo\n%f", R[0][0]);

        double Cfull[6][12] = {{0,0,0,-9.1547e-06,0.0016,0.0002,0.9988,-0.0009,0.0493,0,0,0},
                {0,0,0,3.3324,0,-67.5899,0,-0.9998,-0.0175,0,0,0},
                {0,0,0,0.0002,67.5899,0,0.0493,0.0175,-0.9986,0,0,0},
                {1,0,0,0,0,0,0,0,0,0,0,0},
                {0,1,0,0,0,0,0,0,0,0,0,0},
                {0,0,1,0,0,0,0,0,0,0,0,0}};

        // printf("helloooooo\n%f", R[0][0]);

        //first make S matrix [12][4800]
        //make right most 12x12 a 0 matrix
        //iterate from right to left by Sleft = Sright - dt * Sdot
        //where Sdot is
        //Sdot = -S*Alon - Alon.'*S + S*Blon*(R\(Blon.'))*S - Clon.'*Q*Clon;

        double S[12][4800];

        for (int i = 0; i < 12; i++) {
            for (int j = 4788; j < 4800; j++) {
                S[i][j] = 0;
                // tempMatrix[i][j - 4788] = 0;
            }
        }

        int h = 4776;
        while (h >= 0) {
            // //-S*Alon - Alon.'*S
            // for (int i = 0; i < 12; i++) {
            //     for (int j = 0; j < 12; j++) {
            //         S[i][j + h] = 0;
            //         for (int k = 0; k < 12; k++) {
            //             S[i][j + h] += -1 * S[i][k + h + 12] * Afull[k][j];
            //             S[i][j + h] += -1 * Afull[k][i] * S[k][j + h + 12];
            //         }
            //     }
            // }
            //S*Blon*(R\(Blon.'))*S
            double temp[4][12];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 12; j++) {
                    temp[i][j] = 0;
                    for (int k = 0; k < 4; k++) {
                        temp[i][j] = R[i][k] * Bfull[j][k];
                    }
                }
            }
            double temp2[12][4];
            for (int i = 0; i < 12; i++) {
                for (int j = 0; j < 4; j++) {
                    temp2[i][j] = 0;
                    for (int k = 0; k < 12; k++) {
                        temp2[i][j] = S[i][k + h + 12] * Bfull[k][j];
                    }
                }
            }
            // printf("hellooooooooooo");
            double temp3[12][12];

            multiply(12, 4, temp2, 4, 12, temp, temp3);

            double temp4[12][12];
            for (int i = 0; i < 12; i++) {
                for (int j = 0; j < 12; j++) {
                    temp4[i][j] = 0;
                    for (int k = 0; k < 12; k++) {
                        temp4[i][j] = temp3[i][k] + S[k][j + h + 12];
                    }
                }
            }

            //Clon.'*Q*Clon;
            double temp5[12][6];
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 6; j++) {
                    temp5[i][j] = 0;
                    for (int k = 0; k < 12; k++) {
                        temp5[i][j] = Cfull[k][i] * Q[k][j];
                    }
                }
            }
            double temp6[12][12];
            multiply(12, 6, temp4, 6, 12, Cfull, temp6);

            //-S*Alon - Alon.'*S
            for (int i = 0; i < 12; i++) {
                for (int j = 0; j < 12; j++) {
                    S[i][j + h] = 0;
                    for (int k = 0; k < 12; k++) {
                        //Sdot
                        S[i][j + h] += -1 * S[i][k + h + 12] * Afull[k][j];
                        S[i][j + h] += -1 * Afull[k][i] * S[k][j + h + 12];
                        S[i][j + h] += temp4[i][j];
                        S[i][j + h] += temp6[i][j];
                        //Sleft = Sright - dt * Sdot
                        S[i][j + h] = S[i][j + h + 12] - time_step * S[i][j + h];
                    }
                }
            }
            //decerement, moving to left square
            h = h - 12;
        }

        double U[length];
        double V[length];
        double W[length];
        double x[12][length];
        U[0] = cos(theta0)*cos(psi0)*u0 + (sin(phi0)*sin(theta0)*cos(psi0) - cos(phi0)*sin(psi0))*v0 + (cos(phi0)*sin(theta0)*cos(psi0) + sin(phi0)*sin(psi0))*w0;
        V[0] = -(cos(theta0)*sin(psi0)*u0 + (sin(phi0)*sin(theta0)*sin(psi0) + cos(phi0)*cos(psi0))*v0 + (cos(phi0)*sin(theta0)*sin(psi0) - sin(phi0)*cos(psi0))*w0);
        W[0] = -(-sin(theta0)*u0 + sin(phi0)*cos(theta0)*v0 + cos(phi0)*cos(theta0)*w0);

        //Reuse matrices:
        //temp is 4 x 12
        //temp2 is 12 x 4
        //temp3 is 12 x 12
        //temp4 is 12 x 12
        //temp5 is 12 x 6
        //temp6 is 12 x 12
        //temp7 is
        int k = 0;
        for (int j = 0; j < length - 1; j++) {
            // del(1:4,j) = -(R\(Bfull.'))*Snew(1:12,k:k+11)*x(1:12,j) - (R\(Bfull.'))*g(1:12,j);
            double temp[4][12];
            for (int ii = 0; ii < 4; ii++) {
                for (int jj = 0; jj < 12; jj++) {
                    temp[ii][jj] = 0;
                    for (int kk = 0; kk < 4; kk++) {
                        temp[ii][jj] = R[ii][kk] * Bfull[jj][kk];
                    }
                }
            }
            double temp2[12][1];
            for (int ii = 0; ii < 12; ii++) {
                for (int jj = 0; jj < 1; jj++) {
                    temp2[ii][jj] = 0;
                    for (int kk = 0; kk < 12; kk++) {
                        temp2[ii][jj] = Snew[ii][kk + k] * x[kk][j];
                    }
                }
            }

            for (int ii = 0; ii < 4; ii++) {
                for (int jj = 0; jj < 1; jj++) {
                    del[ii][j] = 0;
                    for (int kk = 0; kk < 12; kk++) {
                        del[ii][j] = -1 * temp[ii][kk] * temp2[kk][jj] - temp[ii][kk] * g[kk][j];
                    }
                }
            }

            // x(1:12,j+1) = x(1:12,j) + (time(j+1)-time(j))*(Afull*x(1:12,j) + Bfull*del(1:4,j));
            for (int ii = 0; ii < 12; ii++) {
                for (int jj = 0; jj < 1; jj++) {
                    temp2[ii][jj] = 0;
                    for (int kk = 0; kk < 12; kk++) {
                        temp2[ii][jj] = Afull[ii][kk] * x[kk][jj];
                    }
                }
            }
            double temp3[12][1];
            for (int ii = 0; ii < 12; ii++) {
                for (int jj = 0; jj < 1; jj++) {
                    temp3[ii][jj] = 0;
                    for (int kk = 0; kk < 4; kk++) {
                        temp3[ii][jj] = Bfull[ii][kk] * del[kk][j];
                    }
                }
            }
            for (int ii = 0; ii < 12; ii++) {
                for (int jj = 0; jj < 1; jj++) {
                    x[ii][j] = 0;
                    for (int kk = 0; kk < 12; kk++) {
                        x[ii][j] =  x[ii][j] + (tvec[j] + tvec[j + 1]) * temp2[kk][jj] + temp3[kk][jj];
                    }
                }
            }

            U[j + 1] = cos(theta0 + x[4][j + 1]) * cos(psi0 + x[5][j + 1]) * (u0 + x[6][j + 1]) + (sin(phi0 + x[3][j + 1])
            * sin(theta0 + x[4][j + 1]) * cos(psi0 + x[5][j + 1]) - cos(phi0 + x[3][j + 1]) * sin(psi0 + x[5][j + 1]))
            * (v0 + x[7][j + 1]) + (cos(phi0 + x[3][j + 1]) * sin(theta0 + x[4][j + 1]) * cos(psi0 + x[5][j + 1])
            + sin(phi0 + x[3][j + 1]) * sin(psi0 + x[5][j + 1])) * (w0 + x[8][j + 1]);

            V[j + 1] = -(cos(theta0 + x[4][j + 1]) * sin(psi0 + x[5][j + 1]) * (u0 + x[6][j + 1]) + (sin(phi0 + x[3][j + 1]) * sin(theta0
                + x[4][j + 1]) * sin(psi0 + x[5][j + 1]) + cos(phi0 + x[3][j + 1]) * cos(psi0 + x[5][j + 1])) * (v0 + x[7][j + 1])
                + (cos(phi0 + x[3][j + 1]) * sin(theta0 + x[4][j + 1]) * sin(psi0 + x[5][j + 1]) - sin(phi0 + x[3][j + 1])
                * cos(psi0 + x[5][j + 1])) * (w0 + x[8][j + 1]));


            W[j + 1] = -(-sin(theta0 + x[4][j + 1]) * (u0 + x[6][j + 1]) + sin(phi0 + x[3][j + 1]) * cos(theta0 + x[4][j + 1]) * (v0 + x[7][j + 1])
            + cos(phi0 + x[3][j + 1]) * cos(theta0 + x[4][j + 1]) * (w0 + x[8][j + 1]));

            k += 12;
        }











}

extern "C" void indexnorm2(double Ji[], int p, int tr, int length, double Ji_mod[]) {
    struct sort_struct Ji_struct[length];

    for (int i = 0; i < length; i++) {
        Ji_struct[i].value = Ji[i];
        Ji_struct[i].index = i;
    }

    qsort(Ji_struct, length, sizeof(Ji_struct[0]), cmp);

    if (p < tr) {
        double logscale[tr];
        logspace(-3.0, 1.0, p, logscale + tr - p);
        for (int i = 0; i < tr - p; i++) {
            logscale[i] = 0.0;
        }
        for (int i = 0; i < length; i++) {
            int index = Ji_struct[i].index;
            Ji_mod[index] = Ji_struct[index].value * logscale[index];
        }
    } else {
        double logscale[p];
        logspace(-3.0, 1.0, p, logscale);
        for (int i = 0; i < length; i++) {
            int index = Ji_struct[i].index;
            Ji_mod[index] = Ji_struct[index].value * logscale[index];
        }
    }


}

extern "C" void vehicle_dynamics_trapz(double* cost, int states_r, int states_c,
    double states[][states_c], int out_states_r, int out_states_c,
    double out_states[][out_states_c], int Afull_r, int Afull_c,
    double Afull[][Afull_c], int Bfull_r, int Bfull_c,
    double Bfull[][Bfull_c], double trim_val[], double inert_val[], int j,
    int dt, int del_con_r, int del_con_c, int del_con_d,
    double del_con[][del_con_c][del_con_d], double U_opt[], double V_opt[],
    double W_opt[], double X_optdel[], double Y_opt[], double Z_opt[]) {

    double u0 = trim_val[0];
    double v0 = trim_val[1];
    double w0 = trim_val[2];
    double phi0 = trim_val[3];
    double theta0 = trim_val[4];
    double psi0 = trim_val[5];

    double U0_opt = inert_val[0];
    double V0_opt = inert_val[1];
    double W0_opt = inert_val[2];

    //Line 16
    double temp1[states_r][1];
    double temp2[del_con_r][1];
    for (int i = 0; i < states_r; i++) {
        temp1[i][0] = states[i][j - 2];
    }
    for (int i = 0; i < del_con_r; i++) {
        temp2[i][0] = del_con[i][j - 2][0] + del_con[i][j - 2][0];
    }
    double temp1x[states_r][1];
    double temp2x[states_r][1];
    multiply(Afull_r, Afull_c, Afull, states_r, 1, temp1, temp1x);
    multiply(Bfull_r, Bfull_c, Bfull, del_con_r, 1, temp2, temp2x);
    for (int i = 0; i < states_r; i++) {
        states[i][j - 1] = states[i][j - 2] + dt * temp1x[i][0] + 0.5 * temp2x[i][0];
    }

    //Line 17
    out_states[0][j - 1] = cos(theta0 + states[4][j - 1]) * cos(psi0 + states[5][j - 1]) * (u0 + states[6][j - 1]) + (sin(phi0 + states[3][j - 1]) * sin(theta0 + states[4][j - 1]) * cos(psi0 + states[5][j - 1])
            - cos(phi0 + states[3][j - 1]) * sin(psi0 + states[5][j - 1])) * (v0 + states[7][j - 1]) + (cos(phi0 + states[3][j - 1]) * sin(theta0 + states[4][j - 1]) * cos(psi0 + states[5][j - 1])
            + sin(phi0 + states[3][j - 1]) * sin(psi0 + states[5][j - 1])) * (w0 + states[8][j - 1]);

    //Line 20
    out_states[1][j - 1] = -(cos(theta0 + states[4][j - 1]) * sin(psi0 + states[5][j - 1]) * (u0 + states[6][j - 1]) + (sin(phi0 + states[3][j - 1]) * sin(theta0 + states[4][j - 1]) * sin(psi0 + states[5][j - 1])
            + cos(phi0 + states[3][j - 1]) * cos(psi0 + states[5][j - 1])) * (v0 + states[7][j - 1]) + (cos(phi0 + states[3][j - 1]) * sin(theta0 + states[4][j - 1]) * sin(psi0 + states[5][j - 1])
            - sin(phi0 + states[3][j - 1]) * cos(psi0 + states[5][j - 1])) * (w0 + states[8][j - 1]));

    //Line 23
    out_states[2][j - 1] = -(-sin(theta0 + states[4][j - 1]) * (u0 + states[6][j - 1]) + sin(phi0 + states[3][j - 1]) * cos(theta0 + states[4][j - 1]) * (v0 + states[7][j - 1])
            + cos(phi0 + states[3][j - 1]) * cos(theta0 + states[4][j - 1]) * (w0 + states[8][j - 1]));

    out_states[3][j - 1] = states[0][j - 1];
    out_states[4][j - 1] = states[1][j - 1];
    out_states[5][j - 1] = states[2][j - 1];

    //Line 28
    double temp_cost = 0.025 * pow(del_con[0][j - 2][0], 2) + 0.025 * pow(del_con[1][j - 2][0], 2) + 0.05 * pow(del_con[2][j-2][0], 2) + 0.025 * pow(del_con[3][j - 2][0],2);
    temp_cost += 0.25 * pow(out_states[0][j - 2] - (U0_opt + U_opt[j - 1]), 2) + 0.0167 * pow(out_states[1][j - 2] - (V0_opt + V_opt[j - 2]), 2);
    temp_cost += 0.25 * pow(out_states[2][j - 2] - (W0_opt + W_opt[j - 1]), 2) + .000001 * pow(out_states[3][j - 2] - X_optdel[j - 2],2);
    temp_cost += 2.5 * .00001 * pow(out_states[4][j - 2] - Y_opt[j - 1], 2) + 0.0025 * pow(out_states[5][j - 2] - Z_opt[j - 1], 2);
    *cost += 0.5 * dt * temp_cost;




}

extern "C" void multiply(int m1, int m2, double mat1[][m2],
              int n1, int n2, double mat2[][n2], double res[][n2])
{
    int x, i, j;
    for (i = 0; i < m1; i++) {
        for (j = 0; j < n2; j++) {
            res[i][j] = 0;
            for (x = 0; x < m2; x++) {
                *(*(res + i) + j) += *(*(mat1 + i) + x) *
                                     *(*(mat2 + x) + j);
            }
        }
    }


}

extern "C" void fill1D(int length, double matrix[], double num) {
    for (int i = 0; i < length; i++) {
        matrix[i] = num;
    }
}

extern "C" void fill2D(int row, int col, double matrix[][col], double num) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            matrix[i][j] = num;
        }
    }
}

extern "C" void fill3D(int row, int col, int depth, double matrix[][col][depth], double num) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            for (int k = 0; k < depth; k++) {
                matrix[i][j][k] = num;
            }

        }
    }
}

extern "C" void logspace(double start, double end, int num, double out[]) {
    if (num < 2 || out == 0) {
        return;
    }
    // int out_length = sizeof(out) / sizeof(out[0]);
    double step = (end - start) / (num - 1);
    for (int i = 0; i < num; i++) {
        out[i] = pow(10.0, start + i * step);
    }
}

extern "C" int cmp(const void *a,const void *b)
{
  struct sort_struct *a1 = (struct str *)a;
  struct sort_struct *a2 = (struct str*)b;
  if((*a1).value>(*a2).value)return 1;
  else if((*a1).value<(*a2).value)return -1;
  else return 0;
}

//Take note of buffer size
extern "C" void readCSV(char filename[], int length, double A_data[], double B_data[], double C_data[],
    double D_data[], double E_data[], double F_data[], double G_data[]) {

    int size = 10000;
	char buffer[size];
	FILE *ptr;
    ptr = fopen(filename,"r");
    if(ptr == NULL)
    {
    	printf("Unable to open file '%s'\n",filename);
    	exit(1);
    }
    char* field;
    double temp;

    double buff[size];

    //Can get the fieldnames here
    fgets(buffer, size, ptr);

    for (int i = 0; i < length ; i++) {
        fgets(buffer, size, ptr);
        //A
        field = strtok(buffer, ",");
        A_data[i] = atof(field);
        //B
        field = strtok(NULL, ",");
        B_data[i] = atof(field);
        //C
        field = strtok(NULL, ",");
        C_data[i] = atof(field);
        //D
        field = strtok(NULL, ",");
        D_data[i] = atof(field);
        //E
        field = strtok(NULL, ",");
        E_data[i] = atof(field);
        //F
        field = strtok(NULL, ",");
        F_data[i] = atof(field);
        //G
        field = strtok(NULL, ",");
        G_data[i] = atof(field);
    }
    fclose(ptr);

}

extern "C" void eye(int length, double matrix[][length]) {
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < length; j++) {
            if (i == j) {
                matrix[i][j] = 1.0;
            } else {
                matrix[i][j] = 0.0;
            }
        }
    }
}
//
// double randn (double mu, double sigma)
// {
//   double U1, U2, W, mult;
//   static double X1, X2;
//   static int call = 0;
//
//   if (call == 1)
//     {
//       call = !call;
//       return (mu + sigma * (double) X2);
//     }
//
//   do
//     {
//       U1 = -1 + ((double) rand () / RAND_MAX) * 2;
//       U2 = -1 + ((double) rand () / RAND_MAX) * 2;
//       W = pow (U1, 2) + pow (U2, 2);
//     }
//   while (W >= 1 || W == 0);
//
//   mult = sqrt ((-2 * log (W)) / W);
//   X1 = U1 * mult;
//   X2 = U2 * mult;
//
//   call = !call;
//
//   return (mu + sigma * (double) X1);
// }


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

    printf("%d", del_r);





}
