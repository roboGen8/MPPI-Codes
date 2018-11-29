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

    cudaMallocManaged(&);


    double ** delbmax = (double **) malloc(del_c * sizeof(double *));


    for (int j = 2; j <= 400; j++) {
        n = 100; //Counter of trajectories with final x-position outside of specified band in relation to ship's landing position
        m=1;    //Counter for number of cycles in while loop


        while (n > 90 && m < 10) {
            delbmax = rb[j -1] * abs(del_con)

        }
    }

    cudaFree(rb);
    cudaFree(ra);
    cudaFree(rp);
    cudaFree(rc);

  return 0;
}