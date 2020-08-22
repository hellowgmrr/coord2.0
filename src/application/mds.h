/*
 * File: mds.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 11-Mar-2020 13:21:05
 */

#ifndef MDS_H
#define MDS_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void mds(const double d[36], double m1[6][3]);
void mul(double m1[6][3],double anl[3][3],int a_m,int a_n,int b_m,int b_n);
void mul_3(double m4[3][3],double anl[3][3],int a_m,int a_n,int b_m,int b_n);
void coordSwitch(double m1[6][3],double m2[3][3], double ans[6][3]);
void main_mds(double arr[6][6],int len,double m1_t[6][3]);


#endif


