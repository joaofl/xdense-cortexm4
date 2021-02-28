/*
 * math_lib.h
 *
 * Created: 26/04/2016 18:51:55
 *  Author: ps
 */ 


#ifndef MATH_LIB_H_
#define MATH_LIB_H_

double prod_and_sum_double(double * value_x1, double * value_x2, int n);
void gauss(double **a, double *b, double *x, int n);
void polyfit2indepentvars(double * x1, double *x2, double *y, int n, double * returned_b, double * returned_sse);

//improved function
float prod_and_sum_float(float * value_x1, float * value_x2, int n);
void gauss_lm(float (*a) [6], float *b, float *x, int n);
void polyfit2indepentvars_lm(float * x1, float *x2, float *y, int n, float * returned_b, float * returned_sse);
void polyfit2indepentvars_lm_static(float * x1, float *x2, float *y, int n, float * returned_b, float * returned_sse);

float prod_and_sum_float_vlm(float * value_x1, float * value_x2, float * value_x3, float * value_x4, int n);
void polyfit2indepentvars_vlm(float * x1, float *x2, float *y, int n, float * returned_b, float * returned_sse);


#endif /* MATH_LIB_H_ */