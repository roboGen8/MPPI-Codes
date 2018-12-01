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

    //3D stuff
    cudaExtent extent = make_cudaExtent(width * sizeof(double), height, depth);
    cudaPitchedPtr del_con;
    cudaMalloc3D(&del_con, extent);
    fill3D<<<100, 40000>>>(del_con, width, height, depth);




    for (int j = 2; j <= 400; j++) {
        int n = 100; //Counter of trajectories with final x-position outside of specified band in relation to ship's landing position
        int m = 1;    //Counter for number of cycles in while loop


        while (n > 90 && m < 10) {
            fill3Dto2D<<<100, 40000>>>(del_con, width, height, 1, delbmax, rb[j - 1]);
            break;

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
