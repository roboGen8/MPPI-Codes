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

__device__
void pvs_helper1(cudaPitchedPtr devPitchedPtr, int width, int height, int depth, double* Afull_ptr, double* Bfull_ptr, double* del_bi, double* del_pi, double* del_ci, int k, int l, int dt) {
    char* devPtr = (char*)devPitchedPtr.ptr;
    size_t pitch = devPitchedPtr.pitch;
    size_t slicePitch = pitch * height;
    for (int z = l; z < l + 1; ++z) {
        char* slice = devPtr + z * slicePitch;
        for (int y = 0; y < height; ++y) {
            double* row = (double*)(slice + y * pitch);
            for (int x = k; x < k + 1; ++x) {
                // double element = row[x];
                // states_p(:,k,l) = states_p(:,k-1,l) + dt*(Afull*states_p(:,k-1,l)+Bfull*0.5*([del_bi(1,k-1,l);del_ai(1,k-1,l);del_pi(1,k-1,l);del_ci(1,k-1,l)]+[del_bi(1,k,l);del_ai(1,k,l);del_pi(1,k,l);del_ci(1,k,l)]));

                // row[x] = row[x - 1] + dt *

            }
        }
    }
}

__global__
void predicted_vehicle_state(cudaPitchedPtr states_p, cudaPitchedPtr out_states_p, double* del_bi, double* del_ai, double* del_pi, double* del_ci, double dt, int j, double* Afull_ptr, double* Bfull_ptr, double trim_val[]) {
    double u0 = trim_val[0];
    double v0 = trim_val[1];
    double w0 = trim_val[2];
    double phi0 = trim_val[3];
    double theta0 = trim_val[4];
    double psi0 = trim_val[5];

    //Calculates vehicle prdicted state based on vehicle dynamics
    for (int k = j; k <= 400; k++) {
        for (int l = 1; l <= 100; l++) {
            // states_p(:,k,l) = states_p(:,k-1,l) + dt*(Afull*states_p(:,k-1,l)+Bfull*0.5*([del_bi(1,k-1,l);del_ai(1,k-1,l);del_pi(1,k-1,l);del_ci(1,k-1,l)]+[del_bi(1,k,l);del_ai(1,k,l);del_pi(1,k,l);del_ci(1,k,l)]));

            pvs_helper1(states_p, 400, 12, 100, Afull_ptr, Bfull_ptr, del_bi, del_pi, del_ci, k, l, dt);
            void mmult_gpu(int m, int n, int k, const double * a, const double * b, double * c);


        }
    }
}
int main(void)
{

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


            // break;


        }
    }

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

    return 0;
}
