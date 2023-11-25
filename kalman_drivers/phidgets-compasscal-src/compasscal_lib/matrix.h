#ifndef __MATRIX
#define __MATRIX

void M_transpose(double *A, double *B, int rows, int cols);
void M_crossProduct(double *A, double *B, double *C, int rows1, int cols1, int rows2, int cols2);
void M_inverse(double *A, double *B, int rows, int cols);

#endif
