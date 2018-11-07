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


void optimal_control_40_full_rel(int xvec_r, int xvec_c, double xvec[][xvec_c], double tvec[],
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

void indexnorm2(double Ji[], int p, int tr, int length, double Ji_mod[]) {
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

void vehicle_dynamics_trapz(double* cost, int states_r, int states_c,
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

void multiply(int m1, int m2, double mat1[][m2],
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

void fill1D(int length, double matrix[], double num) {
    for (int i = 0; i < length; i++) {
        matrix[i] = num;
    }
}

void fill2D(int row, int col, double matrix[][col], double num) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            matrix[i][j] = num;
        }
    }
}

void fill3D(int row, int col, int depth, double matrix[][col][depth], double num) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            for (int k = 0; k < depth; k++) {
                matrix[i][j][k] = num;
            }

        }
    }
}

void logspace(double start, double end, int num, double out[]) {
    if (num < 2 || out == 0) {
        return;
    }
    // int out_length = sizeof(out) / sizeof(out[0]);
    double step = (end - start) / (num - 1);
    for (int i = 0; i < num; i++) {
        out[i] = pow(10.0, start + i * step);
    }
}

int cmp(const void *a,const void *b)
{
  struct sort_struct *a1 = (struct str *)a;
  struct sort_struct *a2 = (struct str*)b;
  if((*a1).value>(*a2).value)return 1;
  else if((*a1).value<(*a2).value)return -1;
  else return 0;
}

//Take note of buffer size
void readCSV(char filename[], int length, double A_data[], double B_data[], double C_data[],
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

void eye(int length, double matrix[][length]) {
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
