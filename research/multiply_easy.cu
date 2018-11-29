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
    A.elements[wor * A.stride + col] = value;
}

//Get the 
