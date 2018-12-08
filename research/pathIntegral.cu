#include <iostream>
#include <math.h>

// Kernel function to add the elements of two arrays
//Good Reference: http://developer.download.nvidia.com/compute/cuda/3_2_prod/toolkit/docs/CUDA_C_Programming_Guide.pdf

//Resource for multiply: https://github.com/sashasyedin/matrix-multiplication-with-cuda
#include <cstdlib>
#include <iostream>
#include <time.h>
#include <cuda_runtime_api.h>
#include <stdio.h>

#include <stdlib.h>
#include <time.h>

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

__global__
void add(int n, float *x, float *y)
{
  for (int i = 0; i < n; i++)
    y[i] = x[i] + y[i];
}

__global__
void fill1D(int length, double matrix[], double num) {
    for (int i = 0; i < length; i++) {
        matrix[i] = num;
    }
}

__global__
void fill2D(double* devPtr, size_t pitch, int width, int height) {
    for (int r = 0; r < height; ++r) {
        double * row = (double*)((char*)devPtr + r * pitch);
        for (int c = 0; c < width; ++c) {
            double element = row[c];
            // printf("%f", row[c]);
        }
    }
}

__global__
void fill2D_specific(double* devPtr, size_t pitch, int width, int height) {
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

    if (width == 12) {
        for (int r = 0; r < height; ++r) {
            double * row = (double*)((char*)devPtr + r * pitch);
            for (int c = 0; c < width; ++c) {
                // double element = row[c];
                row[c] = Afull[r][c];
                // printf("%f", row[c]);
            }
        }
    } else {
        for (int r = 0; r < height; ++r) {
            double * row = (double*)((char*)devPtr + r * pitch);
            for (int c = 0; c < width; ++c) {
                // double element = row[c];
                row[c] = Bfull[r][c];
                // printf("%f", row[c]);
            }
        }
    }

}

__global__ void fill3D(cudaPitchedPtr devPitchedPtr, int width, int height, int depth) {
    char* devPtr = (char*)devPitchedPtr.ptr;
    size_t pitch = devPitchedPtr.pitch;
    size_t slicePitch = pitch * height;
    for (int z = 0; z < depth; ++z) {
        char* slice = devPtr + z * slicePitch;
        for (int y = 0; y < height; ++y) {
            double* row = (double*)(slice + y * pitch);
            for (int x = 0; x < width; ++x) {
                double element = row[x];
            }
        }
    }
}

__global__
void fill3Dto2D(cudaPitchedPtr devPitchedPtr, int width, int height, int depth, double* devPtr2, double factor) {
    char* devPtr = (char*)devPitchedPtr.ptr;
    size_t pitch = devPitchedPtr.pitch;
    size_t slicePitch = pitch * height;
    for (int z = 0; z < depth; ++z) {
        char* slice = devPtr + z * slicePitch;
        for (int y = 0; y < height; ++y) {
            double* row = (double*)(slice + y * pitch);
            double * row_devPtr2 = (double*)((char*)devPtr2 + y * pitch);
            for (int x = 0; x < width; ++x) {
                // double element = row[x];
                row_devPtr2[x] = factor * abs(row[x]);
            }
        }
    }
}

__global__
void func1(cudaPitchedPtr devPitchedPtr, int width, int height, int depth, double* delPtr, double* ddelPtr, double factor) {
    char* devPtr = (char*)devPitchedPtr.ptr;
    size_t pitch = devPitchedPtr.pitch;
    size_t slicePitch = pitch * height;
    for (int z = 0; z < depth; ++z) {
        char* slice = devPtr + z * slicePitch;
        for (int y = 0; y < height; ++y) {
            double* row = (double*)(slice + y * pitch);
            double * delPtr_row = (double*)((char*)delPtr + y * pitch);
            double * ddelPtr_row = (double*)((char*)ddelPtr + y * pitch);
            for (int x = 0; x < width; ++x) {
                // double element = row[x];
                // factor is the random number
                ddelPtr_row[x] = 2 * delPtr_row[x] * factor * row[x] - delPtr_row[x];
            }
        }
    }
}

__global__
void func2(cudaPitchedPtr devPitchedPtr, int width, int height, int depth, double* delPtr, double* ddelPtr, double factor) {
    char* devPtr = (char*)devPitchedPtr.ptr;
    size_t pitch = devPitchedPtr.pitch;
    size_t slicePitch = pitch * height;
    for (int z = 0; z < depth; ++z) {
        char* slice = devPtr + z * slicePitch;
        for (int y = 0; y < height; ++y) {
            double* row = (double*)(slice + y * pitch);
            double * delPtr_row = (double*)((char*)delPtr + y * pitch);
            double * ddelPtr_row = (double*)((char*)ddelPtr + y * pitch);
            for (int x = 0; x < width; ++x) {
                // double element = row[x];
                // factor is the random number
                delPtr_row[x] = row[x] + ddelPtr_row[x];
            }
        }
    }
}

// Matrix multiplication functions
// c = a * b
// a: m x n
// b: n x k
// c: m x k
__global__ void mmult_kernel(int m, int n, int k, const double * a, const double * b, double * c)
{
	int globx = blockIdx.x * blockDim.x + threadIdx.x;
	int globy = blockIdx.y * blockDim.y + threadIdx.y;

	__shared__ int l;

	for (l = 0; l < n; l++)
		c[globx * k + globy] += a[globx * n + l] * b[l * k + globy];
}

void mmult_gpu(int m, int n, int k, const double * a, const double * b, double * c)
{
	dim3 dim_Grid(m, k);
	// dim3 dim_Block(BLOCK_SIZE,BLOCK_SIZE);
	mmult_kernel<<<1, 13>>>(m, n, k, a, b, c);
}

// __device__
// void pvs_helper1(cudaPitchedPtr devPitchedPtr, int width, int height, int depth, double* Afull_ptr, double* Bfull_ptr, double* del_bi, double* del_pi, double* del_ci, int k, int l, int dt) {
//     char* devPtr = (char*)devPitchedPtr.ptr;
//     size_t pitch = devPitchedPtr.pitch;
//     size_t slicePitch = pitch * height;
//     for (int z = l; z < l + 1; ++z) {
//         char* slice = devPtr + z * slicePitch;
//         for (int y = 0; y < height; ++y) {
//             double* row = (double*)(slice + y * pitch);
//             for (int x = k; x < k + 1; ++x) {
//                 // double element = row[x];
//                 // states_p(:,k,l) = states_p(:,k-1,l) + dt*(Afull*states_p(:,k-1,l)+Bfull*0.5*([del_bi(1,k-1,l);del_ai(1,k-1,l);del_pi(1,k-1,l);del_ci(1,k-1,l)]+[del_bi(1,k,l);del_ai(1,k,l);del_pi(1,k,l);del_ci(1,k,l)]));
//
//                 // row[x] = row[x - 1] + dt *
//
//             }
//         }
//     }
// }

__global__
void predicted_vehicle_state(cudaPitchedPtr states_p, cudaPitchedPtr out_states_p, double* del_bi, double* del_ai, double* del_pi, double* del_ci, double dt, int j, double* Afull_ptr, double* Bfull_ptr, double trim_val[]) {
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

    double u0 = trim_val[0];
    double v0 = trim_val[1];
    double w0 = trim_val[2];
    double phi0 = trim_val[3];
    double theta0 = trim_val[4];
    double psi0 = trim_val[5];

    char* states_p_ptr = (char*) states_p.ptr;
    size_t states_p_pitch = states_p.pitch;
    size_t states_p_slicePitch = states_p_pitch * 12;

    char* out_states_p_ptr = (char*) out_states_p.ptr;
    size_t out_states_p_pitch = out_states_p.pitch;
    size_t out_states_p_slicePitch = out_states_p_pitch * 6;

    //depth
    for (int l = 0; l < 100; ++l) {
        char* states_p_slice = states_p_ptr + l * states_p_slicePitch;
        char* out_states_p_slice = out_states_p_ptr + l * out_states_p_slicePitch;

        double* row1 = (double*)(states_p_slice + 0 * states_p_pitch);
        double* row2 = (double*)(states_p_slice + 1 * states_p_pitch);
        double* row3 = (double*)(states_p_slice + 2 * states_p_pitch);
        double* row4 = (double*)(states_p_slice + 3 * states_p_pitch);
        double* row5 = (double*)(states_p_slice + 4 * states_p_pitch);
        double* row6 = (double*)(states_p_slice + 5 * states_p_pitch);
        double* row7 = (double*)(states_p_slice + 6 * states_p_pitch);
        double* row8 = (double*)(states_p_slice + 7 * states_p_pitch);
        double* row9 = (double*)(states_p_slice + 8 * states_p_pitch);
        double* row10 = (double*)(states_p_slice + 9 * states_p_pitch);
        double* row11 = (double*)(states_p_slice + 10 * states_p_pitch);
        double* row12 = (double*)(states_p_slice + 11 * states_p_pitch);

        double* row_out1 = (double*)(out_states_p_slice + 0 * out_states_p_pitch);
        double* row_out2 = (double*)(out_states_p_slice + 1 * out_states_p_pitch);
        double* row_out3 = (double*)(out_states_p_slice + 2 * out_states_p_pitch);
        double* row_out4 = (double*)(out_states_p_slice + 3 * out_states_p_pitch);
        double* row_out5 = (double*)(out_states_p_slice + 4 * out_states_p_pitch);
        double* row_out6 = (double*)(out_states_p_slice + 5 * out_states_p_pitch);
        //height or row
        for (int m = 0; m < 12; ++m) {
            double* row = (double*)(states_p_slice + m * states_p_pitch);
            // double* row_out = (double*)(out_states_p_slice + m * out_states_p_pitch);
            double * row_del_bi = (double*)((char*)del_bi + 0 * states_p_pitch);
            double * row_del_ai = (double*)((char*)del_ai + 0 * states_p_pitch);
            double * row_del_pi = (double*)((char*)del_pi + 0 * states_p_pitch);
            double * row_del_ci = (double*)((char*)del_ci + 0 * states_p_pitch);

            //width or column
            for (int k = j - 1; k < 400; ++k) {
                //Afull * states_p[k -1]
                double temp1 = 0;
                for(int indx1 = 0; indx1 < 12; indx1++) {
                    temp1 += Afull[m][indx1] * row[k - 1];
                }

                double temp2 = 0;
                temp2 = Bfull[m][0] * 0.5 * (row_del_bi[k - 1] + row_del_bi[k])
                        + Bfull[m][1] * 0.5 * (row_del_ai[k - 1] + row_del_ai[k])
                        + Bfull[m][2] * 0.5 * (row_del_pi[k - 1] + row_del_pi[k])
                        + Bfull[m][3] * 0.5 * (row_del_ci[k - 1] + row_del_ci[k]);

                row[k] = row[k - 1] + dt * (temp1 + temp2);


                row_out1[k] = cos(theta0+row5[k])*cos(psi0+row6[k])*(u0+row7[k]) + (sin(phi0+row4[k])*sin(theta0+row5[k])*cos(psi0+row6[k])
                            - cos(phi0+row4[k])*sin(psi0+row6[k]))*(v0+row8[k]) + (cos(phi0+row4[k])*sin(theta0+row5[k])*cos(psi0+row6[k])
                            + sin(phi0+row4[k])*sin(psi0+row6[k]))*(w0+row9[k]);

                row_out2[k] = -(cos(theta0+row5[k])*sin(psi0+row6[k])*(u0+row7[k]) + (sin(phi0+row4[k])*sin(theta0+row5[k])*sin(psi0+row6[k])
                        + cos(phi0+row4[k])*cos(psi0+row6[k]))*(v0+row8[k]) + (cos(phi0+row4[k])*sin(theta0+row5[k])*sin(psi0+row6[k])
                        - sin(phi0+row4[k])*cos(psi0+row6[k]))*(w0+row9[k]));

                row_out3[k] = -(-sin(theta0+row5[k])*(u0+row7[k]) + sin(phi0+row4[k])*cos(theta0+row5[k])*(v0+row8[k])
                    + cos(phi0+row4[k])*cos(theta0+row5[k])*(w0+row9[k]));

                row_out4[k] = row1[k];
                row_out5[k] = row2[k];
                row_out6[k] = row3[k];
                // break;
            }
        }
    }


}


__global__
void helper1(cudaPitchedPtr Ji, cudaPitchedPtr out_states_p, double dt, int j, double* del_bi, double* del_ai,
    double* del_pi, double* del_ci, double* X_optdel, double* U_opt, double* V_opt, double* Y_opt, double* W_opt, double* Z_opt,
    int* n) {
    // int n = 0;  //Starts counter for trajectories with final position outside of specified band
    double U0_opt = 67.5;
    double V0_opt = 0;
    double W0_opt = 0;
    double X0_opt = -1100;
    double Y0_opt = 550;
    double Z0_opt = 100;
    char* out_states_p_ptr = (char*) out_states_p.ptr;
    size_t out_states_p_pitch = out_states_p.pitch;
    size_t out_states_p_slicePitch = out_states_p_pitch * 6;

    char* Ji_ptr = (char*) Ji.ptr;
    size_t Ji_pitch = Ji.pitch;
    size_t Ji_slicePitch = Ji_pitch * 6;

    double band_u = 3;
    double band_v = 3;
    double band_w = 3;
    double band_x = 5;
    double band_y = 5;
    double band_z = 5;

    double us0 = 33.49;
    //Calculates the performance index for each trajectory
    //These should be fixed by Malloc
    double pathcost[4][400][100];
    double us[400];
    double xeq[400];
    double xseq[400];
    double xs[400];
    double vs[400];
    double ys[400];
    double ws[400];
    double zs[400];
    double F[400];

    for (int l = 1; l < 100; l++) {
        double pc[4] = {};
        char* out_states_p_slice = out_states_p_ptr + l * out_states_p_slicePitch;
        double* row_out1 = (double*)(out_states_p_slice + 0 * out_states_p_pitch);
        double* row_out2 = (double*)(out_states_p_slice + 1 * out_states_p_pitch);
        double* row_out3 = (double*)(out_states_p_slice + 2 * out_states_p_pitch);
        double* row_out4 = (double*)(out_states_p_slice + 3 * out_states_p_pitch);
        double* row_out5 = (double*)(out_states_p_slice + 4 * out_states_p_pitch);
        double* row_out6 = (double*)(out_states_p_slice + 5 * out_states_p_pitch);

        double * row_del_bi = (double*)((char*)del_bi + 0 * out_states_p_pitch);
        double * row_del_ai = (double*)((char*)del_ai + 0 * out_states_p_pitch);
        double * row_del_pi = (double*)((char*)del_pi + 0 * out_states_p_pitch);
        double * row_del_ci = (double*)((char*)del_ci + 0 * out_states_p_pitch);

        char* Ji_slice = Ji_ptr + l * Ji_slicePitch;


        for (int k = j - 1; k < 400; k++) {
            // pc(1)=pc(1)+0.5*dt*(0.025*del_bi(1,k,l)^2+0.25*((out_states_p(1,j,l)-(U0_opt+U_opt(j)))^2)+(1e-06)*((out_states_p(4,j,l)-X_optdel(j))^2));   %Calculates path cost for each trajectory
            pc[0] = pc[0] + 0.5 * dt * (0.025 * row_del_bi[k] * row_del_bi[k] + 0.25 * ((row_out1[j - 1]) - (U0_opt + U_opt[j - 1])) * ((row_out1[j - 1]) - (U0_opt + U_opt[j - 1])) + (0.000001) *(row_out4[j - 1] - X_optdel[j - 1]) * (row_out4[j - 1] - X_optdel[j - 1]));
            // pc(2)=pc(2)+0.5*dt*(0.025*del_ai(1,k,l)^2+0.0167*((out_states_p(2,j,l)-(V0_opt+V_opt(j)))^2)+(2.5e-05)*((out_states_p(5,j,l)-Y_opt(j))^2));   %Calculates path cost for each trajectory
            pc[1] = pc[1] + 0.5 * dt * (0.025 * row_del_ai[k] * row_del_bi[k] + 0.0167 * ((row_out2[j - 1]) - (V0_opt + V_opt[j - 1])) * ((row_out1[j - 1]) - (V0_opt + V_opt[j - 1])) + (0.000025) *(row_out5[j - 1] - Y_opt[j - 1]) * (row_out5[j - 1] - Y_opt[j - 1]));
            // pc(3)=pc(3)+0.5*dt*(0.05*del_pi(1,k,l)^2);   %Calculates path cost for each trajectory
            pc[2] = pc[2] + 0.5 * dt * (0.05 * row_del_pi[k] * row_del_pi[k]) * (0.05 * row_del_pi[k] * row_del_pi[k]);
            // pc(4)=pc(4)+0.5*dt*(0.025*del_ci(1,k,l)^2+0.25*((out_states_p(4,j,l)-(W0_opt+W_opt(j)))^2)+0.0025*((out_states_p(6,j,l)-Z_opt(j))^2));   %Calculates path cost for each trajectory
            pc[3] = pc[3] + 0.5 * dt * (0.025 * row_del_ci[k] * row_del_ci[k] + 0.25 * ((row_out4[j - 1]) - (W0_opt + W_opt[j - 1])) * ((row_out4[j - 1]) - (W0_opt + W_opt[j - 1])) + (0.00025) *(row_out6[j - 1] - Z_opt[j - 1]) * (row_out5[j - 1] - Z_opt[j - 1]));
        }
        pathcost[0][j - 1][l] = pc[0];
        pathcost[1][j - 1][l] = pc[1];
        pathcost[2][j - 1][l] = pc[2];
        pathcost[3][j - 1][l] = pc[3];

        if (abs(row_out1[399]-(us0+us[399]))>band_u && abs(row_out4[399] + xeq[399]+X0_opt-(xseq[399]+xs[399]))>band_x &&
                abs(row_out2[399]-vs[399])>band_v && abs(row_out5[399]+Y0_opt-ys[399])>band_y &&
                abs(row_out3[399]-ws[j - 1])>band_w && abs(row_out6[399]+Z0_opt-zs[j - 1])>band_z) {
            for (int indx = 0; indx < 4; indx++) {
                double* row_Ji = (double*)(Ji_slice + indx * Ji_pitch);
                row_Ji[j - 1] = 0;
            }
            *n = *n + 1;
        } else {
            double* row_Ji1 = (double*)(Ji_slice + 0 * Ji_pitch);
            double* row_Ji2 = (double*)(Ji_slice + 1 * Ji_pitch);
            double* row_Ji3 = (double*)(Ji_slice + 2 * Ji_pitch);
            double* row_Ji4 = (double*)(Ji_slice + 3 * Ji_pitch);

            row_Ji1[j - 1] = pathcost[0][j - 1][l] + pathcost[1][j - 1][l] + pathcost[2][j - 1][l] + pathcost[3][j - 1][l] + F[j - 1] *
            ((row_out1[399] - (us0 + us[399])) * (row_out1[399] - (us0 + us[399]))
                + (row_out4[399] + xeq[399] + X0_opt - xseq[399] - xs[399]) * (row_out4[399] + xeq[399] + X0_opt - xseq[399] - xs[399]));

            row_Ji2[j - 1] = pathcost[0][j - 1][l] + pathcost[1][j - 1][l] + pathcost[2][j - 1][l] + pathcost[3][j - 1][l] + F[j - 1] *
                (row_out2[399] - vs[399]) * (row_out2[399] - vs[399]) +
                (row_out5[399] + Y0_opt - ys[399]) * (row_out5[399] + Y0_opt - ys[399]);

            row_Ji3[j - 1] = pathcost[0][j - 1][l] + pathcost[1][j - 1][l] + pathcost[2][j - 1][l] + pathcost[3][j - 1][l];
            row_Ji4[j - 1] = pathcost[0][j - 1][l] + pathcost[1][j - 1][l] + pathcost[2][j - 1][l] + pathcost[3][j - 1][l] + F[j - 1] *
                (row_out3[399] - ws[j - 1]) * (row_out3[399] - ws[j - 1]) +
                (row_out6[399] + Z0_opt - zs[j - 1]) * (row_out6[399] + Z0_opt - zs[j - 1]);
        }


    }
}




int main(void)
{
    clock_t begin = clock();

    //variables
    double rb_inc = 0.1;
    double ra_inc = 0.1;
    double rp_inc = 0.1;
    double rc_inc = 0.1;

    //some variables to add values
    double *rb, *ra, *rp, *rc;
    cudaMallocManaged(&rb, 400 * sizeof(double));
    cudaMallocManaged(&ra, 400 * sizeof(double));
    cudaMallocManaged(&rp, 400 * sizeof(double));
    cudaMallocManaged(&rc, 400 * sizeof(double));


    fill1D<<<1, 256>>>(400, rb, 0.1);
    cudaDeviceSynchronize();
    fill1D<<<1, 256>>>(400, ra, 0.1);
    cudaDeviceSynchronize();
    fill1D<<<1, 256>>>(400, rp, 0.1);
    cudaDeviceSynchronize();
    fill1D<<<1, 256>>>(400, rc, 0.1);
    cudaDeviceSynchronize();

    // Host code
    //width is columns and height is rows
    int height = 400, width = 100, depth = 4;

    //1D stuff
    double* X_optdel;
    size_t optdel_size = 400 * sizeof(double);
    cudaMalloc(&X_optdel, optdel_size);
    double* U_opt;
    cudaMalloc(&U_opt, optdel_size);
    double* V_opt;
    cudaMalloc(&V_opt, optdel_size);
    double* Y_opt;
    cudaMalloc(&Y_opt, optdel_size);
    double* W_opt;
    cudaMalloc(&W_opt, optdel_size);
    double* Z_opt;
    cudaMalloc(&Z_opt, optdel_size);

    double* cycles;
    cudaMalloc(&cycles, optdel_size);
    double* np;
    cudaMalloc(&np, optdel_size);


    //2D stuff
    size_t pitch;

    double* delbmax;
    cudaMallocPitch(&delbmax, &pitch, width * sizeof(double), height);
    // fill2D<<<100, 400000>>>(delbmax, pitch, width, height);

    double* delamax;
    cudaMallocPitch(&delamax, &pitch, width * sizeof(double), height);

    double* delpmax;
    cudaMallocPitch(&delpmax, &pitch, width * sizeof(double), height);

    double* delcmax;
    cudaMallocPitch(&delcmax, &pitch, width * sizeof(double), height);

    double* ddelb;
    cudaMallocPitch(&ddelb, &pitch, width * sizeof(double), height);

    double* ddela;
    cudaMallocPitch(&ddela, &pitch, width * sizeof(double), height);

    double* ddelp;
    cudaMallocPitch(&ddelp, &pitch, width * sizeof(double), height);

    double* ddelc;
    cudaMallocPitch(&ddelc, &pitch, width * sizeof(double), height);

    double* del_bi;
    cudaMallocPitch(&del_bi, &pitch, width * sizeof(double), height);

    double* del_ai;
    cudaMallocPitch(&del_ai, &pitch, width * sizeof(double), height);

    double* del_pi;
    cudaMallocPitch(&del_pi, &pitch, width * sizeof(double), height);

    double* del_ci;
    cudaMallocPitch(&del_ci, &pitch, width * sizeof(double), height);



    double* Afull_ptr;
    cudaMallocPitch(&Afull_ptr, &pitch, 12 * sizeof(double), 12);
    fill2D_specific<<<100, 40000>>>(Afull_ptr, pitch, 12, 12);

    double* Bfull_ptr;
    cudaMallocPitch(&Bfull_ptr, &pitch, 4 * sizeof(double), 12);
    fill2D_specific<<<100, 40000>>>(Bfull_ptr, pitch, 4, 12);

    //3D stuff
    cudaExtent extent = make_cudaExtent(400 * sizeof(double), 4, 100);
    cudaPitchedPtr del_con;
    cudaMalloc3D(&del_con, extent);
    fill3D<<<100, 40000>>>(del_con, 400, 4, 100);

    cudaExtent extent2 = make_cudaExtent(400 * sizeof(double), 12, 100);
    cudaPitchedPtr states_p;
    cudaMalloc3D(&states_p, extent2);
    fill3D<<<100, 40000>>>(states_p, 400, 12, 100);

    cudaExtent extent3 = make_cudaExtent(400 * sizeof(double), 6, 100);
    cudaPitchedPtr out_states_p;
    cudaMalloc3D(&out_states_p, extent3);
    fill3D<<<100, 40000>>>(out_states_p, 400, 6, 100);

    cudaExtent extent4 = make_cudaExtent(400 * sizeof(double), 4, 100);
    cudaPitchedPtr Ji;
    cudaMalloc3D(&Ji, extent4);
    fill3D<<<100, 40000>>>(Ji, 400, 4, 100);


    //other stuff
    double trim_val[] = {67.5077, -0.0585, 3.3319, -0.0175, 0.0493, 0};





    for (int j = 2; j <= 400; j++) {
        int n = 100; //Counter of trajectories with final x-position outside of specified band in relation to ship's landing position
        int m = 1;    //Counter for number of cycles in while loop


        while (n > 90 && m < 10) {
            fill3Dto2D<<<100, 40000>>>(del_con, width, height, 1, delbmax, rb[j - 1]);
            fill3Dto2D<<<100, 40000>>>(del_con, width, height, 2, delamax, ra[j - 1]);
            fill3Dto2D<<<100, 40000>>>(del_con, width, height, 3, delpmax, rp[j - 1]);
            fill3Dto2D<<<100, 40000>>>(del_con, width, height, 4, delcmax, rc[j - 1]);

            srand(time(0));
            func1<<<100, 40000>>>(del_con, width, height, 1, delbmax, ddelb, (double)rand() / 32767.0);
            srand(time(0));
            func1<<<100, 40000>>>(del_con, width, height, 2, delamax, ddela, (double)rand() / 32767.0);
            srand(time(0));
            func1<<<100, 40000>>>(del_con, width, height, 3, delpmax, ddelp, (double)rand() / 32767.0);
            srand(time(0));
            func1<<<100, 40000>>>(del_con, width, height, 4, delcmax, ddelc, (double)rand() / 32767.0);


            func2<<<100, 40000>>>(del_con, width, height, 4, del_bi, ddelb, 1.0);
            func2<<<100, 40000>>>(del_con, width, height, 4, del_ai, ddela, 1.0);
            func2<<<100, 40000>>>(del_con, width, height, 4, del_pi, ddelp, 1.0);
            func2<<<100, 40000>>>(del_con, width, height, 4, del_ci, ddelc, 1.0);

            //Predicted vehicle state -------------------
            // void predicted_vehicle_state(double* states_p, cudaPitchedPtr out_states_p, double* del_bi, double* del_ai, double* del_pi, double* del_ci, double dt, int j, double* Afull_ptr, double* Bfull_ptr, double trim_val[])
            double dt = 0.1;
            predicted_vehicle_state<<<100, 40000>>>(states_p, out_states_p, del_bi, del_ai, del_pi, del_ci, dt, j, Afull_ptr, Bfull_ptr, trim_val);

            helper1<<<100, 40000>>>(Ji, out_states_p, dt, j, del_bi, del_ai,
                del_pi, del_ci, X_optdel, U_opt, V_opt, Y_opt, W_opt, Z_opt,
                &n);

            rb[j - 1] = rb[j - 1] + rb_inc;
            ra[j - 1] = ra[j - 1] + ra_inc;
            rp[j - 1] = rp[j - 1] + rp_inc;
            rc[j - 1] = rc[j - 1] + rc_inc;
            m++;
            // break;


        }

        //Calculation of Cost is done here
        // cycles[j - 1] = m;
        // np[j - 1] = 100 - n;


    }

    cudaFree(U_opt);
    cudaFree(V_opt);
    cudaFree(W_opt);
    cudaFree(X_optdel);
    cudaFree(Y_opt);
    cudaFree(Z_opt);

    cudaFree(rb);
    cudaFree(ra);
    cudaFree(rp);
    cudaFree(rc);

    cudaFree(delbmax);
    cudaFree(delamax);
    cudaFree(delpmax);
    cudaFree(delcmax);

    cudaFree(ddelb);
    cudaFree(ddela);
    cudaFree(ddelp);
    cudaFree(ddelc);

    cudaFree(del_bi);
    cudaFree(del_ai);
    cudaFree(del_pi);
    cudaFree(del_ci);

    // cudaFree(del_con);

    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("Path Integral took: %f seconds \n", time_spent);
    return 0;
}
