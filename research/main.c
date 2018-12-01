// #include <stdio.h>
//
// int main() {
//
//
//     //////////////////////////////////////////////////////////////
//     //testing indexnorm2
//     int p = 100;
//     int tr = 100;
//     double Ji[100];
//     double Ji_mod[100];
//     int length = sizeof(Ji) / sizeof(Ji[0]);
//     //making struct array of Ji
//
//
//     for (int i = 0; i < length; i++) {
//         Ji[i] = (100 - i) * 1.0;
//     }
//     for (int i = 0; i < length; i++) {
//         printf("%f\n", Ji[i]);
//     }
//     indexnorm2(Ji, p, tr, length, Ji_mod);
//
//     for (int i = 0; i < length; i++) {
//         printf("%f\n", Ji_mod[i]);
//     }
//
//
//
//     //////////////////////////////////////////////////////////////
//     //testing vehicle_dynamics_trapz
//     double trim_val[] = {1.5, 2.5, 3.5, 4.5, 5.5, 6.5};
//     double inert_val[] = {1.2, 2.2, 3.2, 4.2, 5.2, 6.2};
//     double cost;
//     // int states_r = 12;
//     // int states_c = 400;
//     double states[12][400];
//     // int out_states_r = 6;
//     // int out_states_c = 400;
//     double out_states[6][400];
//     int j = 2;
//     int dt = 0.1;
//     double Afull[12][12];
//     double Bfull[12][4];
//     double del_con[4][400][100];
//
//     // function [states,out_states,cost]=vehicle_dynamics_trapz(del_con,dt,states,out_states,j,Afull,Bfull,trim_val,U_opt,V_opt,W_opt,X_optdel,Y_opt,Z_opt,inert_val,cost)
//     double U_opt[400];
//     double V_opt[400];
//     double W_opt[400];
//     double X_optdel[400];
//     double Y_opt[400];
//     double Z_opt[400];
//
//     int states_r = sizeof(states) / sizeof(states[0]);
//     int states_c = sizeof(states[0]) / sizeof(states[0][0]);
//     int out_states_r = sizeof(out_states) / sizeof(out_states[0]);
//     int out_states_c = sizeof(out_states[0]) / sizeof(out_states[0][0]);
//     int Afull_c = sizeof(Afull[0])/sizeof(Afull[0][0]);
//     int Afull_r = sizeof(Afull) / sizeof(Afull[0]);
//     int Bfull_c = sizeof(Bfull[0])/sizeof(Bfull[0][0]);
//     int Bfull_r = sizeof(Bfull) / sizeof(Bfull[0]);
//     int del_con_d = sizeof(del_con[0][0])/sizeof(del_con[0][0][0]);
//     int del_con_c = sizeof(del_con[0])/sizeof(del_con[0][0]);
//     int del_con_r = sizeof(del_con) / sizeof(del_con[0]);
//
//     fill2D(states_r, states_c, states);
//     fill2D(out_states_r, out_states_c, out_states);
//     fill2D(Afull_r, Afull_c, Afull);
//     fill2D(Bfull_r, Bfull_c, Bfull);
//     fill3D(del_con_r, del_con_c, del_con_d, del_con);
//
//     vehicle_dynamics_trapz(&cost, states_r, states_c, states, out_states_r,
//         out_states_c, out_states, Afull_r, Afull_c, Afull, Bfull_r,
//         Bfull_c, Bfull, trim_val, inert_val, j, dt, del_con_r,
//         del_con_c, del_con_d, del_con, U_opt, V_opt, W_opt, X_optdel, Y_opt, Z_opt);
//
//     printf("%f\n", cost);
//     printf("states %f\n", states[0][0]);
//
//     //////////////////////////////////////////////////////////////
//     //testing optimal_control
//     //input
//     printf("optimal_controooooool");
//     int model_time = 40;
//     double time_step = 0.1;
//     double us0 = 33.49;
//     //output
//     double X_opt[400];
//     double x[12][400];
//     double del[4][400];
//     double time[400];
//     double Xeq[400];
//     double xseq[400];
//
//     int x_r = sizeof(x) / sizeof(x[0]);
//     int x_c = sizeof(x[0]) / sizeof(x[0][0]);
//     int del_r = sizeof(del) / sizeof(del[0]);
//     int del_c = sizeof(del[0]) / sizeof(del[0][0]);
//
//     int time_length = sizeof(time) / sizeof(time[0]);
//
//
//     optimal_control_40_full_rel(x_r, x_c, x, del_r, del_c,
//         del, time, Xeq, xseq, U_opt, V_opt,
//         W_opt, X_opt, X_optdel, Y_opt, Z_opt, inert_val,
//         model_time, time_step, Afull_r, Afull_c, Afull, Bfull_r,
//         Bfull_c, Bfull, trim_val, us0, time_length);
//
//
//     return 0;
// }
