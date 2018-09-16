#include <string.h>
#include "myLib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
// #include "libxl.h"

// time=(xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','A2:A401')).';


void optimal_control_40_full_rel(int x_r, int x_c, double x[][x_c], int del_r, int del_c,
    double del[][del_c], double time[], double Xeq[], double xseq[], double U_opt[], double V_opt[],
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
        double A_data[length];
        double B_data[length];
        double C_data[length];
        double D_data[length];
        double E_data[length];
        double F_data[length];
        double G_data[length];
        //Reading data

        char filename[] = "Medium_40s.csv";
        printf("length = %d", length);
        readCSV(filename, length, A_data, B_data, C_data, D_data, E_data, F_data, G_data);
        
}

void indexnorm2(double Ji[], int p, int tr, int length, double Ji_mod[]) {
    struct sort_struct Ji_struct[length];

    for (int i = 0; i < length; i++) {
        Ji_struct[i].value = Ji[i];
        Ji_struct[i].index = i;
    }

    qsort(Ji_struct, length, sizeof(Ji_struct[0]), cmp);
    // printf("hello");
    // for (int i = 0; i < length; i++) {
    //     printf("%d\n", Ji_struct[i].index);
    // }
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
        // printf("%f", states[i][j-1]);
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


void fill2D(int row, int col, double matrix[][col]) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            matrix[i][j] = 1.5;
        }
    }
}

void fill3D(int row, int col, int depth, double matrix[][col][depth]) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            for (int k = 0; k < depth; k++) {
                matrix[i][j][k] = 1.5;
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
    // printf("%f\n", step);
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
    // field = strtok(buffer,",");
    // for (int i = 0; i < 22; i++) {
    //     field = strtok(NULL,",");
    //     printf(field);
    //     printf("\n");
    // }
    // field = strtok(buffer,",");


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
    // printf("length = %d", length);
    // for (int i = 0; i < length; i++) {
    //     printf("\n%f", A_data[i]);
    // }
    fclose(ptr);


    // field = strtok(buffer,",");
    // double z = atof(field);
    // printf(field);
    // printf("\n%f", z);
    // temp = atof(field);
    // printf("%f", temp);

	// char *field;
	// int year,month,day;
	// float high,low;
	// char *months[] = { "January", "February", "March",
	// 	"April", "May", "June", "July", "August",
	// 	"September", "October", "November", "December" };
    //
	// /* open the CSV file */
	// f = fopen(filename,"r");
	// if( f == NULL)
	// {
	// 	printf("Unable to open file '%s'\n",filename);
	// 	exit(1);
	// }
    //
	// /* process the data */
	// /* the file contains 5 fields in a specific order:
	//    year,month,day,high,low
	//    separated by commas */
	// while(fgets(buffer,BSIZE,f))
	// {
	// 	/* get year */
	// 	field=strtok(buffer,",");
	// 	year=atoi(field);
	// 	/* get month */
	// 	field=strtok(NULL,",");
	// 	month=atoi(field);
	// 	month--;	/* for the months[] array */
	// 	/* get day */
	// 	field=strtok(NULL,",");
	// 	day=atoi(field);
	// 	/* get high */
	// 	field=strtok(NULL,",");
	// 	high=atof(field);
	// 	/* get low */
	// 	field=strtok(NULL,",");
	// 	low=atof(field);
	// 	/* display the result in the proper format */
	// 	printf("%10s %2d %d:\tHigh %.1f\tLow %.1f\n",
	// 			months[month],
	// 			day,
	// 			year,
	// 			high,
	// 			low);
	// }
    //
	// /* close file */
	// fclose(f);
    //
	// return(0);
}
