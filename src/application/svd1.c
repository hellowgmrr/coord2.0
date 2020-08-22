
#include "rtwtypes.h"
#include "mds.h"
#include "svd1.h"


void svd(const double A[36], double U[36], double S[36], double V[36])
{
  double b_A[36];
  double s[6];
  double e[6];
  double work[6];
  int kase;
  int q;
  int m;
  int iter;
  boolean_T apply_transform;
  double ztest0;
  int qp1jj;
  int qs;
  double snorm;
  double rt;
  double ztest;
  int exitg3;
  boolean_T exitg2;
  double f;
  double varargin_1[5];
  double mtmp;
  boolean_T exitg1;
  double sqds;
  memcpy(&b_A[0], &A[0], 36U * sizeof(double));
  for (kase = 0; kase < 6; kase++) {
    s[kase] = 0.0;
    e[kase] = 0.0;
    work[kase] = 0.0;
  }

  memset(&U[0], 0, 36U * sizeof(double));
  memset(&V[0], 0, 36U * sizeof(double));
  for (q = 0; q < 5; q++) {
    iter = q + 6 * q;
    apply_transform = false;
    ztest0 = xnrm2(6 - q, b_A, iter + 1);
    if (ztest0 > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabs(s[q]) >= 1.0020841800044864E-292) {
        ztest0 = 1.0 / s[q];
        kase = (iter - q) + 6;
        for (qp1jj = iter; qp1jj + 1 <= kase; qp1jj++) {
          b_A[qp1jj] *= ztest0;
        }
      } else {
        kase = (iter - q) + 6;
        for (qp1jj = iter; qp1jj + 1 <= kase; qp1jj++) {
          b_A[qp1jj] /= s[q];
        }
      }

      b_A[iter]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }

    for (qs = q + 1; qs + 1 < 7; qs++) {
      kase = q + 6 * qs;
      if (apply_transform) {
        xaxpy(6 - q, -(xdotc(6 - q, b_A, iter + 1, b_A, kase + 1) / b_A[q + 6 *
                       q]), iter + 1, b_A, kase + 1);
      }

      e[qs] = b_A[kase];
    }

    for (qp1jj = q; qp1jj + 1 < 7; qp1jj++) {
      U[qp1jj + 6 * q] = b_A[qp1jj + 6 * q];
    }

    if (q + 1 <= 4) {
      ztest0 = b_xnrm2(5 - q, e, q + 2);
      if (ztest0 == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -ztest0;
        } else {
          e[q] = ztest0;
        }

        ztest0 = e[q];
        if (fabs(e[q]) >= 1.0020841800044864E-292) {
          ztest0 = 1.0 / e[q];
          for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
            e[qp1jj] *= ztest0;
          }
        } else {
          for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
            e[qp1jj] /= ztest0;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
          work[qp1jj] = 0.0;
        }

        for (qs = q + 1; qs + 1 < 7; qs++) {
          b_xaxpy(5 - q, e[qs], b_A, (q + 6 * qs) + 2, work, q + 2);
        }

        for (qs = q + 1; qs + 1 < 7; qs++) {
          c_xaxpy(5 - q, -e[qs] / e[q + 1], work, q + 2, b_A, (q + 6 * qs) + 2);
        }
      }

      for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
        V[qp1jj + 6 * q] = e[qp1jj];
      }
    }
  }

  m = 4;
  s[5] = b_A[35];
  e[4] = b_A[34];
  e[5] = 0.0;
  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    U[30 + qp1jj] = 0.0;
  }

  U[35] = 1.0;
  for (q = 4; q >= 0; q += -1) {
    iter = q + 6 * q;
    if (s[q] != 0.0) {
      for (qs = q + 1; qs + 1 < 7; qs++) {
        kase = (q + 6 * qs) + 1;
        xaxpy(6 - q, -(xdotc(6 - q, U, iter + 1, U, kase) / U[iter]), iter + 1,
              U, kase);
      }

      for (qp1jj = q; qp1jj + 1 < 7; qp1jj++) {
        U[qp1jj + 6 * q] = -U[qp1jj + 6 * q];
      }

      U[iter]++;
      for (qp1jj = 1; qp1jj <= q; qp1jj++) {
        U[(qp1jj + 6 * q) - 1] = 0.0;
      }
    } else {
      for (qp1jj = 0; qp1jj < 6; qp1jj++) {
        U[qp1jj + 6 * q] = 0.0;
      }

      U[iter] = 1.0;
    }
  }

  for (q = 5; q >= 0; q += -1) {
    if ((q + 1 <= 4) && (e[q] != 0.0)) {
      kase = (q + 6 * q) + 2;
      for (qs = q + 1; qs + 1 < 7; qs++) {
        qp1jj = (q + 6 * qs) + 2;
        xaxpy(5 - q, -(xdotc(5 - q, V, kase, V, qp1jj) / V[kase - 1]), kase, V,
              qp1jj);
      }
    }

    for (qp1jj = 0; qp1jj < 6; qp1jj++) {
      V[qp1jj + 6 * q] = 0.0;
    }

    V[q + 6 * q] = 1.0;
  }

  for (q = 0; q < 6; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0) {
      rt = fabs(s[q]);
      ztest = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 6) {
        ztest0 = e[q] / ztest;
      }

      xscal(ztest, U, 1 + 6 * q);
    }

    if ((q + 1 < 6) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      s[q + 1] *= ztest;
      xscal(ztest, V, 1 + 6 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0;
  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    ztest0 = fabs(s[qp1jj]);
    ztest = fabs(e[qp1jj]);
    if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
    } else {
      ztest0 = ztest;
    }

    if ((snorm >= ztest0) || rtIsNaN(ztest0)) {
    } else {
      snorm = ztest0;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    qp1jj = m;
    do {
      exitg3 = 0;
      q = qp1jj + 1;
      if (qp1jj + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabs(e[qp1jj]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s[qp1jj]) + fabs(s[qp1jj +
               1]))) || (ztest0 <= 1.0020841800044864E-292) || ((iter > 20) &&
             (ztest0 <= 2.2204460492503131E-16 * snorm))) {
          e[qp1jj] = 0.0;
          exitg3 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg3 == 0);

    if (qp1jj + 1 == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      kase = m + 2;
      exitg2 = false;
      while ((!exitg2) && (kase >= qp1jj + 1)) {
        qs = kase;
        if (kase == qp1jj + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0;
          if (kase < m + 2) {
            ztest0 = fabs(e[kase - 1]);
          }

          if (kase > qp1jj + 2) {
            ztest0 += fabs(e[kase - 2]);
          }

          ztest = fabs(s[kase - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s[kase - 1] = 0.0;
            exitg2 = true;
          } else {
            kase--;
          }
        }
      }

      if (qs == qp1jj + 1) {
        kase = 3;
      } else if (qs == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0;
      for (qp1jj = m; qp1jj + 1 >= q + 1; qp1jj--) {
        xrotg(&s[qp1jj], &f, &ztest0, &ztest);
        if (qp1jj + 1 > q + 1) {
          f = -ztest * e[qp1jj - 1];
          e[qp1jj - 1] *= ztest0;
        }

        xrot(V, 1 + 6 * qp1jj, 1 + 6 * (m + 1), ztest0, ztest);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (qp1jj = q; qp1jj + 1 <= m + 2; qp1jj++) {
        xrotg(&s[qp1jj], &f, &ztest0, &ztest);
        f = -ztest * e[qp1jj];
        e[qp1jj] *= ztest0;
        xrot(U, 1 + 6 * qp1jj, 1 + 6 * (q - 1), ztest0, ztest);
      }
      break;

     case 3:
      varargin_1[0] = fabs(s[m + 1]);
      varargin_1[1] = fabs(s[m]);
      varargin_1[2] = fabs(e[m]);
      varargin_1[3] = fabs(s[q]);
      varargin_1[4] = fabs(e[q]);
      kase = 1;
      mtmp = varargin_1[0];
      if (rtIsNaN(varargin_1[0])) {
        qp1jj = 2;
        exitg1 = false;
        while ((!exitg1) && (qp1jj < 6)) {
          kase = qp1jj;
          if (!rtIsNaN(varargin_1[qp1jj - 1])) {
            mtmp = varargin_1[qp1jj - 1];
            exitg1 = true;
          } else {
            qp1jj++;
          }
        }
      }

      if (kase < 5) {
        while (kase + 1 < 6) {
          if (varargin_1[kase] > mtmp) {
            mtmp = varargin_1[kase];
          }

          kase++;
        }
      }

      f = s[m + 1] / mtmp;
      ztest0 = s[m] / mtmp;
      ztest = e[m] / mtmp;
      sqds = s[q] / mtmp;
      rt = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0;
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      if ((rt != 0.0) || (ztest0 != 0.0)) {
        ztest = sqrt(rt * rt + ztest0);
        if (rt < 0.0) {
          ztest = -ztest;
        }

        ztest = ztest0 / (rt + ztest);
      } else {
        ztest = 0.0;
      }

      f = (sqds + f) * (sqds - f) + ztest;
      rt = sqds * (e[q] / mtmp);
      for (qp1jj = q + 1; qp1jj <= m + 1; qp1jj++) {
        xrotg(&f, &rt, &ztest0, &ztest);
        if (qp1jj > q + 1) {
          e[qp1jj - 2] = f;
        }

        f = ztest0 * s[qp1jj - 1] + ztest * e[qp1jj - 1];
        e[qp1jj - 1] = ztest0 * e[qp1jj - 1] - ztest * s[qp1jj - 1];
        rt = ztest * s[qp1jj];
        s[qp1jj] *= ztest0;
        xrot(V, 1 + 6 * (qp1jj - 1), 1 + 6 * qp1jj, ztest0, ztest);
        s[qp1jj - 1] = f;
        xrotg(&s[qp1jj - 1], &rt, &ztest0, &ztest);
        f = ztest0 * e[qp1jj - 1] + ztest * s[qp1jj];
        s[qp1jj] = -ztest * e[qp1jj - 1] + ztest0 * s[qp1jj];
        rt = ztest * e[qp1jj];
        e[qp1jj] *= ztest0;
        xrot(U, 1 + 6 * (qp1jj - 1), 1 + 6 * qp1jj, ztest0, ztest);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
        xscal(-1.0, V, 1 + 6 * q);
      }

      kase = q + 1;
      while ((q + 1 < 6) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        xswap(V, 1 + 6 * q, 1 + 6 * (q + 1));
        xswap(U, 1 + 6 * q, 1 + 6 * (q + 1));
        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    e[qp1jj] = s[qp1jj];
  }

  memset(&S[0], 0, 36U * sizeof(double));
  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    S[qp1jj + 6 * qp1jj] = e[qp1jj];
  }
}

void xswap(double x[36], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 6; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

void xscal(double a, double x[36], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 5; k++) {
    x[k - 1] *= a;
  }
}

void xrotg(double *a, double *b, double *c, double *s)
{
  double roe;
  double absa;
  double absb;
  double scale;
  double ads;
  double bds;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    scale = 0.0;
    *b = 0.0;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }
  }

  *a = scale;
}

void xrot(double x[36], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 6; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

double b_xnrm2(int n, const double x[6], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

double xnrm2(int n, const double x[36], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

double xdotc(int n, const double x[36], int ix0, const double y[36], int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

void b_xaxpy(int n, double a, const double x[36], int ix0, double y[6], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                double a
 *                const double x[6]
 *                int ix0
 *                double y[36]
 *                int iy0
 * Return Type  : void
 */
void c_xaxpy(int n, double a, const double x[6], int ix0, double y[36], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                double a
 *                int ix0
 *                double y[36]
 *                int iy0
 * Return Type  : void
 */
void xaxpy(int n, double a, int ix0, double y[36], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}


/*
 * File trailer for svd1.c
 *
 * [EOF]
 */
