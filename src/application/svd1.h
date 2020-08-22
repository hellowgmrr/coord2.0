/*
 * File: svd1.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 11-Mar-2020 13:21:05
 */

#ifndef SVD1_H
#define SVD1_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations 1*/
extern void svd(const double A[36], double U[36], double S[36], double V[36]);
extern void xscal(double a, double x[36], int ix0);
extern void xswap(double x[36], int ix0, int iy0);
extern void xrotg(double *a, double *b, double *c, double *s);
extern void xrot(double x[36], int ix0, int iy0, double c, double s);
extern double b_xnrm2(int n, const double x[6], int ix0);
extern double xnrm2(int n, const double x[36], int ix0);
extern double xdotc(int n, const double x[36], int ix0, const double y[36], int
                    iy0);
extern void b_xaxpy(int n, double a, const double x[36], int ix0, double y[6],
                    int iy0);
extern void c_xaxpy(int n, double a, const double x[6], int ix0, double y[36],
                    int iy0);
extern void xaxpy(int n, double a, int ix0, double y[36], int iy0);

#endif


