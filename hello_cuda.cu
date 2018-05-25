#include <stdio.h>

__global__ void myKernel() {

}

int main() {
    myKernel<<<1, 1>>>();
    printf("Hello, World!\n");
    return 0;
}
