// Matrices are stored in row_major order:
// M(row, col) = *(M.elements + row * M.stride + col)
typedef struct {
    int width;
    int height;
    int stride;
    float* elements;
} Matrix;

// Get a matrix element
__device__
float GetElement(const Matrix A, int row, int col) {
    return A.elements[row * stride + col];
}

// Set a matrix elements
__device__
void SetElement(Matrix A, int row, int col, float value) {
    A.elements[row * A.stride + col] = value;
}

//Get the BLOCK_SIZExBLOCK_SIZE sub matrix Asub of A that is
//located col sub-matrices to the right and row sub-matrices down
//from the upper-left corner of A
__device__
Matrix GetSubMatrix(Matrix A, int row, int col) {
    Matrix Asub;
    Asub.width = BLOCK_SIZE;
    Asub.width = BLOCK_SIZE;
    Asub.stride = A.stride;
    Asub.elements= &A.elements[A.stride * BLOCK_SIZE * row + BLOCK_SIZE * col];
    return Asub;
}

// Thread block size
#define BLOCK_SIZE 16

//Forward declaration of the matrix multiplicatino Kernel
__global__ void MatMulKernel(const Matrix, const Matrix, Matrix);

//Matrix Multiplication - Host code
//matrix dimensions are assumed to be multiplies of BLOCK_SIZE
void
