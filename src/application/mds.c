/*
 * File: mds.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 11-Mar-2020 13:21:05
 */

/* Include Files */
#include "mds.h"
#include "svd1.h"

void main_mds(double arr[6][6],int len,double m1_t[6][3])
{
    if(len == 4 || len == 5)
    {
        int i,j;
        for(i = len; i<=5; i++)
        {
            for(j = 0; j<=5; j++)
            {
                if((i == j)||(j == 0)||((i>=len)&&(j>=len)))
                {
                    arr[i][j] = 0;
                    arr[j][i] = 0;
                }
                else
                {
                    arr[i][j] = arr[0][j];
                    arr[j][i] = arr[0][j];
                }
            }
        }
    }
 /*   int i,j;
      for(i = 0; i<6; i++)
  {
      for(j = 0; j<6; j++)
      {
          printf("%f\t",arr[i][j]);
      }
      printf("\n");
  }*/
  mds(arr, m1_t);
  /*printf("根据距离矩阵求解坐标（将An0放在原点，Anc1放在x轴）：\n");
  for(i = 0; i<6; i++)
  {
      for(j = 0; j<3; j++)
      {
          printf("%f\t",m1[i][j]);
      }
      printf("\n");
  }*/
}

 void mul(double m1[6][3],double anl[3][3],int a_m,int a_n,int b_m,int b_n){
    double tmp[6][3] = {{0.0}};
    int i,j,k;
    for(i=0;i<a_m;i++){
        for(j=0;j<b_n;j++){
            for(k=0;k<a_n;k++){
                tmp[i][j]+= m1[i][k]*anl[k][j];
            }
        }
    }
    memcpy(m1,tmp,sizeof(tmp));
}

void mul_3(double m4[3][3],double anl[3][3],int a_m,int a_n,int b_m,int b_n){

    double tmp[3][3] = {{0}};
    int i,j,k;
    for(i=0;i<a_m;i++){
        for(j=0;j<b_n;j++){
            for(k=0;k<a_n;k++){
                tmp[i][j]+= m4[i][k]*anl[k][j];
            }
        }
    }
    memcpy(m4,tmp,sizeof(tmp));
}

void coordSwitch(double m1[6][3],double m2[3][3], double ans[6][3])
{
    double tmp_m2[3][3];
    memcpy(tmp_m2,m2,sizeof(tmp_m2));
    double tmp_m1[6][3];
    memcpy(tmp_m1,m1,sizeof(tmp_m1));
    int i,j;
    //先利用Anc0平移
    for(i = 0; i<3;i++){
        for(j = 0; j<3; j++){
            tmp_m2[i][j] -= m2[0][j];
        }
    }
    //通过An1在两个坐标系中的坐标计算旋转角度
    double CurrAng_z = -acos(tmp_m2[1][0]/sqrt(pow(tmp_m2[1][0],2)+pow(tmp_m2[1][1],2)));
    if(tmp_m2[1][1]<0) CurrAng_z = -CurrAng_z;
    double Anro_z[3][3] = {{cos(CurrAng_z),sin(CurrAng_z),0},
                          {-sin(CurrAng_z),cos(CurrAng_z),0},
                          {0,0,1}};
    mul_3(tmp_m2,Anro_z,3,3,3,3);
    double CurrAng_y = -acos(tmp_m2[1][0]/sqrt(pow(tmp_m2[1][0],2)+pow(tmp_m2[1][2],2)));
    if(tmp_m2[1][2]<0) CurrAng_y = -CurrAng_y;
    double Anro_y[3][3] = {{cos(CurrAng_y),0,sin(CurrAng_y)},
                          {0,1,0},
                          {-sin(CurrAng_y),0,cos(CurrAng_y)}};
    mul_3(tmp_m2,Anro_y,3,3,3,3);

    double CurrAng_x1 = -acos(tmp_m2[2][1]/sqrt(pow(tmp_m2[2][1],2)+pow(tmp_m2[2][2],2)));
    if(tmp_m2[2][2]<0) CurrAng_x1 = -CurrAng_x1;
    double CurrAng_x2 = -acos(tmp_m1[2][1]/sqrt(pow(tmp_m1[2][1],2)+pow(tmp_m1[2][2],2)));
    if(tmp_m1[2][2]<0) CurrAng_x2 = -CurrAng_x2;
    double CurrAng_x = CurrAng_x2 - CurrAng_x1;
    if(tmp_m2[2][2]<tmp_m1[2][2]) CurrAng_y = -CurrAng_y;

    double Anro_x2[3][3] = {{1,0,0},
                          {0,cos(-CurrAng_x),-sin(-CurrAng_x)},
                          {0,sin(-CurrAng_x),cos(-CurrAng_x)}};
    double Anro_z2[3][3] = {{cos(-CurrAng_z),sin(-CurrAng_z),0},
                          {-sin(-CurrAng_z),cos(-CurrAng_z),0},
                          {0,0,1}};
    double Anro_y2[3][3] = {{cos(-CurrAng_y),0,sin(-CurrAng_y)},
                          {0,1,0},
                          {-sin(-CurrAng_y),0,cos(-CurrAng_y)}};

    mul(tmp_m1,Anro_x2,6,3,3,3);
    mul(tmp_m1,Anro_z2,6,3,3,3);
    mul(tmp_m1,Anro_y2,6,3,3,3);
    for(i = 0; i<6;i++){
        for(j = 0; j<3; j++){
            tmp_m1[i][j] += m2[0][j];
        }
    }
    memcpy(ans,tmp_m1,sizeof(tmp_m1));

}

void mds(const double d[36], double m1[6][3])
{
  double a[36];
  double b_d[36];
  int i0;
  double b_a[36];
  int i1;
  double U[36];
  double S[36];
  double x;
  double b_x;
  double c_x;
  int i2;
  double B[6];
  double C[6];
  static const double c_a[36] = { -0.41666666666666669, 0.083333333333333329,
    0.083333333333333329, 0.083333333333333329, 0.083333333333333329,
    0.083333333333333329, 0.083333333333333329, -0.41666666666666669,
    0.083333333333333329, 0.083333333333333329, 0.083333333333333329,
    0.083333333333333329, 0.083333333333333329, 0.083333333333333329,
    -0.41666666666666669, 0.083333333333333329, 0.083333333333333329,
    0.083333333333333329, 0.083333333333333329, 0.083333333333333329,
    0.083333333333333329, -0.41666666666666669, 0.083333333333333329,
    0.083333333333333329, 0.083333333333333329, 0.083333333333333329,
    0.083333333333333329, 0.083333333333333329, -0.41666666666666669,
    0.083333333333333329, 0.083333333333333329, 0.083333333333333329,
    0.083333333333333329, 0.083333333333333329, 0.083333333333333329,
    -0.41666666666666669 };

  double D[6];
  static const double b[36] = { 0.83333333333333337, -0.16666666666666666,
    -0.16666666666666666, -0.16666666666666666, -0.16666666666666666,
    -0.16666666666666666, -0.16666666666666666, 0.83333333333333337,
    -0.16666666666666666, -0.16666666666666666, -0.16666666666666666,
    -0.16666666666666666, -0.16666666666666666, -0.16666666666666666,
    0.83333333333333337, -0.16666666666666666, -0.16666666666666666,
    -0.16666666666666666, -0.16666666666666666, -0.16666666666666666,
    -0.16666666666666666, 0.83333333333333337, -0.16666666666666666,
    -0.16666666666666666, -0.16666666666666666, -0.16666666666666666,
    -0.16666666666666666, -0.16666666666666666, 0.83333333333333337,
    -0.16666666666666666, -0.16666666666666666, -0.16666666666666666,
    -0.16666666666666666, -0.16666666666666666, -0.16666666666666666,
    0.83333333333333337 };

  for (i0 = 0; i0 < 6; i0++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_d[i1 + 6 * i0] = d[i1 + 6 * i0] * d[i1 + 6 * i0];
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_a[i0 + 6 * i1] = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        b_a[i0 + 6 * i1] += c_a[i0 + 6 * i2] * b_d[i2 + 6 * i1];
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      a[i0 + 6 * i1] = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        a[i0 + 6 * i1] += b_a[i0 + 6 * i2] * b[i2 + 6 * i1];
      }
    }
  }

  svd(a, U, S, b_d);
  x = sqrt(S[0]);
  b_x = sqrt(S[7]);
  c_x = sqrt(S[14]);
  for (i0 = 0; i0 < 6; i0++) {
    B[i0] = U[i0] * x;
    C[i0] = U[6 + i0] * b_x;
    D[i0] = U[12 + i0] * c_x;
  }

  memset(&m1[0], 0, 18U * sizeof(double));
  for (i0 = 0; i0 < 3; i0++) {
    m1[0][i0] = 0.0;
  }

  for(i0 = 1; i0<6; i0++)
  {
      m1[i0][0] = B[i0] - B[0];
      m1[i0][1] = C[i0] - C[0];
      m1[i0][2] = D[i0] - D[0];
  }

  double CurrAng_z = -acos(m1[1][0]/sqrt(pow(m1[1][0],2)+pow(m1[1][1],2)));
  //printf("m1[1][0]:%f\nsqrt(pow(m1[1][0],2)+pow(m1[1][1],2)):%f\n",ans[1][0],sqrt(pow(ans[1][0],2)+pow(ans[1][1],2)));
  if(m1[1][1]<0) CurrAng_z = -CurrAng_z;
  double Anro_z[3][3] = {{cos(CurrAng_z),sin(CurrAng_z),0},
                          {-sin(CurrAng_z),cos(CurrAng_z),0},
                          {0,0,1}};
  mul(m1,Anro_z,6,3,3,3);
  double CurrAng_y = -acos(m1[1][0]/sqrt(pow(m1[1][0],2)+pow(m1[1][2],2)));
  if(m1[1][2]<0) CurrAng_y = -CurrAng_y;
  double Anro_y[3][3] = {{cos(CurrAng_y),0,sin(CurrAng_y)},
         {0,1,0},
         {-sin(CurrAng_y),0,cos(CurrAng_y)}};
  mul(m1,Anro_y,6,3,3,3);
}

/*
 * File trailer for mds.c
 *
 * [EOF]
 */
