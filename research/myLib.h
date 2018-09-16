typedef struct sort_struct {
    double value;
    int index;
};

void optimal_control_40_full_rel(int x_r, int x_c, double x[][x_c], int del_r, int del_c,
    double del[][del_c], double time[], double Xeq[], double xseq[], double U_opt[], double V_opt[],
    double W_opt[], double X_opt[], double X_optdel[], double Y_opt[], double Z_opt[], double inert_val[],
    int model_time, double time_step, int Afull_r, int Afull_c, double Afull[][Afull_c], int Bfull_r,
    int Bfull_c, double Bfull[][Bfull_c], double trim_val[], double us0, int length);


void indexnorm2(double Ji[], int p, int tr, int length, double Ji_mod[]);

void vehicle_dynamics_trapz(double* cost, int states_r, int states_c,
    double states[][states_c], int out_states_r, int out_states_c,
    double out_states[][out_states_c], int Afull_r, int Afull_c,
    double Afull[][Afull_c], int Bfull_r, int Bfull_c,
    double Bfull[][Bfull_c], double trim_val[], double inert_val[], int j,
    int dt, int del_con_r, int del_con_c, int del_con_d,
    double del_con[][del_con_c][del_con_d], double U_opt[], double V_opt[],
    double W_opt[], double X_optdel[], double Y_opt[], double Z_opt[]);

void multiply(int m1, int m2, double mat1[][m2],
                  int n1, int n2, double mat2[][n2], double res[][n2]);

void fill2D(int row, int col, double matrix[][col]);

void fill3D(int row, int col, int depth, double matrix[][col][depth]);

void logspace(double start, double end, int num, double out[]);

int cmp(const void *a,const void *b);

void readCSV(char filename[], int length, double A_data[], double B_data[], double C_data[],
    double D_data[], double E_data[], double F_data[], double G_data[]);
