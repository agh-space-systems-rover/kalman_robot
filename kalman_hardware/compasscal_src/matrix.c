#include "stdafx.h"

void M_transpose(double *A, double *B, int rows, int cols) {
	int i, j;
	for (i=1; i <= rows; i++)
		for (j=1; j <= cols; j++)
			*(B + j*(rows + 1) + i) = *(A + i*(cols + 1) + j);
}

//C will be rows1 x cols2
void M_crossProduct(double *A, double *B, double *C, int rows1, int cols1, int rows2, int cols2) {
	int i, j, k;

	if (cols1 != rows2) {
		printf("Inner matrix dimensions must agree.");
		return;
	}

	for (i = 1; i <= rows1; i++) {
		for (j = 1; j <= cols2; j++) {
			*(C + i*(cols2 + 1) + j) = 0;
			for (k = 1; k <= cols1; k++) {
				*(C + i*(cols2 + 1) + j) += *(A + i*(cols1 + 1) + k) * *(B + k*(cols2 + 1) + j);
			}
		}
	}
}

static int M_IsUpperTrapeze(double *A, int rows, int cols) {
	int i, j;
	for (j = 1; j <= cols; j++)
		for (i = j + 1; i <= rows; i++)
			if (*(A + i*(cols + 1) + j) != 0) return 0;

	return 1;
}

static int M_IsLowerTrapeze(double *A, int rows, int cols) {
	int i, j;
	for (i = 1; i <= rows; i++)
		for (j = i + 1; j <= cols; j++)
			if (*(A + i*(cols + 1) + j) != 0) return 0;

	return 1;
}

static int M_IsTrapeze(double *A, int rows, int cols) {
	return (M_IsUpperTrapeze(A, rows, cols) | M_IsLowerTrapeze(A, rows, cols));
}

static double M_DiagProd(double *A, int rows, int cols) {
	double buf = 1;
	int dim = rows;
	int i;

	for (i = 1; i <= dim; i++) {
		buf *= *(A + i*(cols + 1) + i);
	}

	return buf;
}

static void M_SwapRows(double *A, int cols, int row1, int  row2) {
	int i;
	double d;
	for (i=1; i <= cols; i++) {
		d = *(A + row1*(cols + 1) + i);
		*(A + row1*(cols + 1) + i) = *(A + row2*(cols + 1) + i);
		*(A + row2*(cols + 1) + i) = d;
	}
}

static double M_Signum(double *A, int rows, int cols) {
	double buf = 1;
	int i, j;

	int n = rows;
	double fi, fj;

	for (i = 1; i < n; i++) {
		for (fi = 1; fi < n && *(A + i*(cols + 1) + (int)fi) != 1; fi++);

		for (j = i + 1; j <= n; j++) {
			for (fj = 1; fj <= n && *(A + j*(cols + 1) + (int)fj) != 1; fj++);

			buf *= (fi - fj) / (i - j);
		}
	}

	return buf;
}

static void M_LUSafe(double *A, double *B, double *P, int rows, int cols) {
	int n = cols;
	int m, i, j, k;

	//P = I
	for (i = 1; i <= rows; i++) {
		for (j = 1; j <= cols; j++) {
			if (i == j)
				*(P + i*(cols + 1) + j) = 1;
			else
				*(P + i*(cols + 1) + j)=0;
		}
	}

	//copy A
	memcpy(B, A, (cols + 1)*(rows + 1) * sizeof(double));
	for (i = 1; i <= rows; i++) {
		for (j = 1; j <= cols; j++) {
			*(B + i*(cols + 1) + j) = *(A + i*(cols + 1) + j);
		}
	}

	for (j = 1; j <= n; j++) {
		if (j < n) {
			m = j;
			for (i = j + 1; i <= n; i++)
				if (fabs(*(B + i*(cols + 1) + j)) > fabs(*(B + m*(cols + 1) + j)))
					m = i;

			if (m > j) // <=> j2 != j
			{
				M_SwapRows(P, cols, j, m);
				M_SwapRows(B, cols, j, m);
			}

			if (*(B + j*(cols + 1) + j) == 0) {
				printf("Warning: Matrix close to singular.\n");
				return;
			}
		}

		for (k = 1; k < j; k++) {
			for (i = k + 1; i <= n; i++) {
				*(B + i*(cols + 1) + j) = *(B + i*(cols + 1) + j) - *(B + i*(cols + 1) + k) * *(B + k*(cols + 1) + j);
			}
		}

		for (i = j + 1; i <= n; i++) {
			*(B + i*(cols + 1) + j) = *(B + i*(cols + 1) + j) / *(B + j*(cols + 1) + j);
		}
	}
}

static double M_determinant(double *A, int rows, int cols) {
	double *X = malloc((rows + 1)*(cols + 1) * sizeof(double));
	double *P = malloc((rows + 1)*(cols + 1) * sizeof(double));
	double a, b;

	M_LUSafe(A, X, P, rows, cols);
	a = M_Signum(P, rows, cols);
	b = M_DiagProd(X, rows, cols);

	free(X);
	free(P);

	return a*b;
}

static void M_minor(double *A, double *B, int rows, int cols, int delRow, int delCol) {
	int i, j, r=1, c=1;
	for (i = 1; i <= rows; i++) {
		if (i != delRow) {
			for (j = 1; j <= cols; j++) {
				if (j != delCol) {
					*(B + r*cols + c) = *(A + i*(cols + 1) + j);
					c++;
				}
			}

			c = 1;
			r++;
		}
	}
}

void M_inverse(double *A, double *B, int rows, int cols) {
	double det;
	int n = cols, i, j;

	det = M_determinant(A, rows, cols);

	if (det == 0) {
		printf("Cannot invert (nearly) singular matrix.\n");
		return;
	}

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			double *min = malloc(rows*cols * sizeof(double));
			double val;
			M_minor(A, min, rows, cols, j + 1, i + 1);
			val = pow(-1, i + j) * M_determinant(min, rows - 1, cols - 1);
			val /= det;
			*(B + (i + 1)*(cols + 1) + j + 1) = val;
			free(min);
		}
	}
}
