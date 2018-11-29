#include <iostream>
#include <math.h>
// Kernel function to add the elements of two arrays

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
        n = 100; //Counter of trajectories with final x-position outside of specified band in relation to ship's landing position
        m=1;    //Counter for number of cycles in while loop


        while (n > 90 && m < 10) {
            delbmax = rb[j - 1] * abs(del_con)

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

    cudaFree(del_con)

    return 0;
}
