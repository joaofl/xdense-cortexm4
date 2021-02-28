/*
 * math_lib.c
 *
 * Created: 26/04/2016 18:51:29
 *  Author: ps
 */ 

#include "math_lib.h"
#include <math.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
//#include "task.h"
#include "ioport.h"

double prod_and_sum_double(double * value_x1, double * value_x2, int n){
	int i;
	double result=0;
	for(i = 0; i < n; i++){
		result += (value_x1[i] * value_x2[i]) ;
	}
	return result;
}

float prod_and_sum_float(float * value_x1, float * value_x2, int n){
	int i;
	float result=0;
	for(i = 0; i < n; i++){
		if(value_x2 != NULL){
			result += (value_x1[i] * value_x2[i]);
		}
		else if(value_x1 != NULL){
			result += (value_x1[i]);
			} else {
			result += 1;
		}
	}
	return result;
}

float prod_and_sum_float_vlm(float * value_x1, float * value_x2, float * value_x3, float * value_x4, int n){
	int i;
	float result=0;
	for(i = 0; i < n; i++){
		if(value_x4 != NULL){
			result += (value_x1[i] * value_x2[i] * value_x3[i] * value_x4[i]);
		}
		else if(value_x3 != NULL){
			result += (value_x1[i] * value_x2[i] * value_x3[i]);
		}
		else if(value_x2 != NULL){
			result += (value_x1[i] * value_x2[i]);
		}
		else if(value_x1 != NULL){
			result += (value_x1[i]);
		} else {
			result += 1;
		}
	}
	return result;
}


void gauss(double **a, double *b, double *x, int n) {
	int   i,j,k,m,rowx;
	double xfac,temp,temp1,amax;


	/////////////////////////////////////////
	// Do the forward reduction step.
	/////////////////////////////////////////

	rowx = 0;   // Keep count of the row interchanges
	for (k=0; k<=n-2; ++k) {
		
		amax = (double) fabs(a[k][k]) ;
		m = k;
		for (i=k+1; i<=n-1; i++){   // Find the row with largest pivot
			xfac = (double) fabs(a[i][k]);
			if(xfac > amax) {amax = xfac; m=i;}
		}
		if(m != k) {  // Row interchanges
			rowx++;
			temp1 = b[k];
			b[k]  = b[m];
			b[m]  = temp1;
			for(j=k; j<=n-1; j++) {
				temp = a[k][j];
				a[k][j] = a[m][j];
				a[m][j] = temp;
			}
		}
		for (i=k+1; i<=n-1; ++i) {
			xfac = a[i][k]/a[k][k];

			for (j=k+1; j<=n-1; ++j) {
				a[i][j] = a[i][j]-xfac*a[k][j];
			}
			b[i] = b[i]-xfac*b[k];
		}
	}
	
	/////////////////////////////////////////
	// Do the back substitution step
	/////////////////////////////////////////

	for (j=0; j<=n-1; ++j) {
		k=(n-(j+1)+1)-1;
		x[k] = b[k];
		for(i=k+1; i<=n-1; ++i) {
			x[k] = x[k]-a[k][i]*x[i];
		}
		if(a[k][k]) {
			x[k] = x[k]/a[k][k];
			} else {
			x[k] = 0;
		}
	}
	
	return;
}

void gauss_lm(float (*a) [6], float *b, float *x, int n) {
	//gauss low memory use -> only float
	
	int   i,j,k,m,rowx;
	float xfac,temp,temp1,amax;


	/////////////////////////////////////////
	// Do the forward reduction step.
	/////////////////////////////////////////

	rowx = 0;   // Keep count of the row interchanges
	for (k=0; k<=n-2; ++k) {
		
		amax = (float) fabsf(a[k][k]) ;
		m = k;
		for (i=k+1; i<=n-1; i++){   // Find the row with largest pivot
			xfac = (float) fabsf(a[i][k]);
			if(xfac > amax) {amax = xfac; m=i;}
		}
		if(m != k) {  // Row interchanges
			rowx++;
			temp1 = b[k];
			b[k]  = b[m];
			b[m]  = temp1;
			for(j=k; j<=n-1; j++) {
				temp = a[k][j];
				a[k][j] = a[m][j];
				a[m][j] = temp;
			}
		}
		for (i=k+1; i<=n-1; ++i) {
			xfac = a[i][k]/a[k][k];

			for (j=k+1; j<=n-1; ++j) {
				a[i][j] = a[i][j]-xfac*a[k][j];
			}
			b[i] = b[i]-xfac*b[k];
		}
	}
	
	/////////////////////////////////////////
	// Do the back substitution step
	/////////////////////////////////////////

	for (j=0; j<=n-1; ++j) {
		k=(n-(j+1)+1)-1;
		x[k] = b[k];
		for(i=k+1; i<=n-1; ++i) {
			x[k] = x[k]-a[k][i]*x[i];
		}
		if(a[k][k]) {
			x[k] = x[k]/a[k][k];
			} else {
			x[k] = 0;
		}
	}
	
	return;
}


void polyfit2indepentvars(double * x1, double *x2, double *y, int n, double * returned_b, double * returned_sse){
	int i;
	int result = 0;
	
	int __n 		= n;
	double *__1 	= (double *) pvPortMalloc(sizeof(double) * n);
	double *__x1 	= (double *) pvPortMalloc(sizeof(double) * n);
	double *__x2 	= (double *) pvPortMalloc(sizeof(double) * n);
	double *__x1_2 	= (double *) pvPortMalloc(sizeof(double) * n);
	double *__x2_2 	= (double *) pvPortMalloc(sizeof(double) * n);
	double *__x1x2 	= (double *) pvPortMalloc(sizeof(double) * n);
	double *__y 	= (double *) pvPortMalloc(sizeof(double) * n);

	//[row][col]
	int __matrixA_rows = 6;
	int __matrixA_cols = 6;
	double ** __matrixA = (double **) pvPortMalloc(__matrixA_rows * sizeof(double*));
	for(i = 0; i < __matrixA_rows; i++) __matrixA[i] = (double *)pvPortMalloc(__matrixA_cols * sizeof(double));
	//double __matrixA[6][6]= {0};
	int __matrixB_rows = 6;
	double * __matrixB = (double *) pvPortMalloc(__matrixB_rows * sizeof(double));
	//double __matrixB[6][1]= {0};
	int __matrixC_rows = 6;
	double * __matrixC = (double *) pvPortMalloc(__matrixC_rows * sizeof(double));
	//double __matrixC[6][1]= {0};

	for ( i = 0; i < __n; i++){
		__1[i] = 1;
		__x1[i] = x1[i];
		__x2[i] = x2[i];
		__x1_2[i] = x1[i] * x1[i];
		__x2_2[i] = x2[i] * x2[i];
		__x1x2[i] = x1[i] * x2[i];
		__y[i] = y[i];
	}

	//Calc Matrix A
	for ( i = 0; i < 6; i++){
		switch(i){
			case 0:
					__matrixA[0][i] = __matrixA[i][0] = prod_and_sum_double(__1,    __1, __n);
					__matrixA[1][i] = __matrixA[i][1] = prod_and_sum_double(__x1,   __1, __n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_double(__x2,   __1, __n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_double(__x1_2, __1, __n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_double(__x2_2, __1, __n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_double(__x1x2, __1, __n);
			break;
			case 1: //*x1
					__matrixA[1][i] = __matrixA[i][1] = prod_and_sum_double(__x1,   __x1, __n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_double(__x2,   __x1, __n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_double(__x1_2, __x1, __n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_double(__x2_2, __x1, __n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_double(__x1x2, __x1, __n);
			break;
			case 2: //*x2
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_double(__x2,   __x2, __n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_double(__x1_2, __x2, __n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_double(__x2_2, __x2, __n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_double(__x1x2, __x2, __n);
			break;
			case 3: //*x1_2
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_double(__x1_2, __x1_2, __n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_double(__x2_2, __x1_2, __n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_double(__x1x2, __x1_2, __n);
			break;
			case 4: //*x2_2
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_double(__x2_2, __x2_2, __n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_double(__x1x2, __x2_2, __n);
			break;
			case 5: //*x1_x2
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_double(__x1x2, __x1x2, __n);
			break;
		}
	}

	//Calc Matrix C
	__matrixC[0] = prod_and_sum_double(__y, __1, __n);
	__matrixC[1] = prod_and_sum_double(__y, __x1, __n);
	__matrixC[2] = prod_and_sum_double(__y, __x2, __n);
	__matrixC[3] = prod_and_sum_double(__y, __x1_2, __n);
	__matrixC[4] = prod_and_sum_double(__y, __x2_2, __n);
	__matrixC[5] = prod_and_sum_double(__y, __x1x2, __n);

    //gauss elimination
    gauss(__matrixA, __matrixC, __matrixB, 6);
    
    //copy from local var to external var
    for (i=0; i<6; i++){
        returned_b[i]=__matrixB[i];
    }
           
    //Mean Squared Error Calc
    int __ms_error=0;
    for(i=0; i<n; i++){
        __ms_error += pow(__y[i] - (__matrixB[0] + __matrixB[1] * __x1[i] + __matrixB[2] * __x2[i] + __matrixB[3] * __x1_2[i] + __matrixB[4] * __x2_2[i] + __matrixB[5] * __x1x2[i]), 2);
    }
    __ms_error /= n;
    
    *returned_sse = __ms_error;
	
	vPortFree(__1);
    vPortFree(__x1);
	vPortFree(__x2);
	vPortFree(__x1_2);
	vPortFree(__x2_2);
	vPortFree(__x1x2);
    vPortFree(__y);
	

    for(i = 0; i < __matrixA_rows; i++) vPortFree(__matrixA[i]);
    vPortFree(__matrixA);
	vPortFree(__matrixB);
    vPortFree(__matrixC);
    

	return;
}

void polyfit2indepentvars_lm(float * x1, float *x2, float *y, int n, float * returned_b, float * returned_sse){
	//low memory use
	//Avoid copying to local variables to improve speed
	//Using only float to save memory and improve performance
	//faster
	
	int i;
	
	float *__x1_2 	= (float *) pvPortMalloc(sizeof(float) * n);
	float *__x2_2 	= (float *) pvPortMalloc(sizeof(float) * n);
	float *__x1x2 	= (float *) pvPortMalloc(sizeof(float) * n);
	
	//[row][col]
	float __matrixA[6][6];
	//float __matrixB[6] = {0}; 
	float __matrixC[6];

	for ( i = 0; i < n; i++){
		__x1_2[i] = x1[i] * x1[i];
		__x2_2[i] = x2[i] * x2[i];
		__x1x2[i] = x1[i] * x2[i];
	}

	//Calc Matrix A
	for ( i = 0; i < 6; i++){
		switch(i){
			case 0:
					__matrixA[0][i] = prod_and_sum_float(NULL, NULL, n);
					__matrixA[1][i] = __matrixA[i][1] = prod_and_sum_float(x1,   NULL, n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_float(x2,   NULL, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float(__x1_2, NULL, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, NULL, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, NULL, n);
			break;
			case 1: //*x1
					__matrixA[1][i] = prod_and_sum_float(x1,   x1, n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_float(x2,   x1, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float(__x1_2, x1, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, x1, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, x1, n);
			break;
			case 2: //*x2
					__matrixA[2][i] = prod_and_sum_float(x2,  x2, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float(__x1_2, x2, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, x2, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, x2, n);
			break;
			case 3: //*x1_2
					__matrixA[3][i] = prod_and_sum_float(__x1_2, __x1_2, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, __x1_2, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, __x1_2, n);
			break;
			case 4: //*x2_2
					__matrixA[4][i] = prod_and_sum_float(__x2_2, __x2_2, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, __x2_2, n);
			break;
			case 5: //*x1_x2
					__matrixA[5][i] = prod_and_sum_float(__x1x2, __x1x2, n);
			break;
		}
	}

	//Calc Matrix C
	__matrixC[0] = prod_and_sum_float(y, NULL, n);
	__matrixC[1] = prod_and_sum_float(y, x1, n);
	__matrixC[2] = prod_and_sum_float(y, x2, n);
	__matrixC[3] = prod_and_sum_float(y, __x1_2, n);
	__matrixC[4] = prod_and_sum_float(y, __x2_2, n);
	__matrixC[5] = prod_and_sum_float(y, __x1x2, n);

    //gauss elimination
    gauss_lm(__matrixA, __matrixC, returned_b, 6);
    
	*returned_sse = 0;
    //Mean Squared Error Calc
    for(i=0; i<n; i++){
         (*returned_sse) += pow(y[i] - (returned_b[0] + returned_b[1] * x1[i] + returned_b[2] * x2[i] + returned_b[3] * __x1_2[i] + returned_b[4] * __x2_2[i] + returned_b[5] * __x1x2[i]), 2);
    }
     (*returned_sse) /= n;
	
	vPortFree(__x1_2);
	vPortFree(__x2_2);
	vPortFree(__x1x2);

	return;
}

void polyfit2indepentvars_lm_static(float * x1, float *x2, float *y, int n, float * returned_b, float * returned_sse){
	//max nodes = 256
	//static memory only
	//low memory use
	//Avoid copying to local variables to improve speed
	//Using only float to save memory and improve performance
	
	#define polyfit2indepentvars_lm_static_MAX_NODES 256
	
	int i;
	int result = 0;
	
	float __x1_2[polyfit2indepentvars_lm_static_MAX_NODES];
	float __x2_2[polyfit2indepentvars_lm_static_MAX_NODES];
	float __x1x2[polyfit2indepentvars_lm_static_MAX_NODES];
	
	if (n > polyfit2indepentvars_lm_static_MAX_NODES) return; 
	

	//[row][col]
	float __matrixA[6][6] = {0};
	//float __matrixB[6] = {0}; //Using external variable (returned_b) ?
	float __matrixC[6] = {0};

	for ( i = 0; i < n; i++){
		__x1_2[i] = x1[i] * x1[i];
		__x2_2[i] = x2[i] * x2[i];
		__x1x2[i] = x1[i] * x2[i];
	}

	//Calc Matrix A
	for ( i = 0; i < 6; i++){
		switch(i){
			case 0:
					__matrixA[0][i] = prod_and_sum_float(NULL, NULL, n);
					__matrixA[1][i] = __matrixA[i][1] = prod_and_sum_float(x1,   NULL, n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_float(x2,   NULL, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float(__x1_2, NULL, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, NULL, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, NULL, n);
			break;
			case 1: //*x1
					__matrixA[1][i] = prod_and_sum_float(x1,   x1, n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_float(x2,   x1, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float(__x1_2, x1, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, x1, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, x1, n);
			break;
			case 2: //*x2
					__matrixA[2][i] = prod_and_sum_float(x2,  x2, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float(__x1_2, x2, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, x2, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, x2, n);
			break;
			case 3: //*x1_2
					__matrixA[3][i] = prod_and_sum_float(__x1_2, __x1_2, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float(__x2_2, __x1_2, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, __x1_2, n);
			break;
			case 4: //*x2_2
					__matrixA[4][i] = prod_and_sum_float(__x2_2, __x2_2, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float(__x1x2, __x2_2, n);
			break;
			case 5: //*x1_x2
					__matrixA[5][i] = prod_and_sum_float(__x1x2, __x1x2, n);
			break;
		}
	}

	//Calc Matrix C
	__matrixC[0] = prod_and_sum_float(y, NULL, n);
	__matrixC[1] = prod_and_sum_float(y, x1, n);
	__matrixC[2] = prod_and_sum_float(y, x2, n);
	__matrixC[3] = prod_and_sum_float(y, __x1_2, n);
	__matrixC[4] = prod_and_sum_float(y, __x2_2, n);
	__matrixC[5] = prod_and_sum_float(y, __x1x2, n);

    //gauss elimination
    gauss_lm(__matrixA, __matrixC, returned_b, 6);
    
    ////copy from local var to external var
    //for (i=0; i<6; i++){
        //returned_b[i]=__matrixB[i];
    //}
	
     *returned_sse = 0;
    //Mean Squared Error Calc
    for(i=0; i<n; i++){
         (*returned_sse) += pow(y[i] - (returned_b[0] + returned_b[1] * x1[i] + returned_b[2] * x2[i] + returned_b[3] * __x1_2[i] + returned_b[4] * __x2_2[i] + returned_b[5] * __x1x2[i]), 2);
    }
     (*returned_sse) /= n;
    
	return;
}

void polyfit2indepentvars_vlm(float * x1, float *x2, float *y, int n, float * returned_b, float * returned_sse){
	//very low memory use
	//Avoid copying to local variables to improve speed
	//Using only float to save memory and improve performance
	int i;
	//[row][col]
	float __matrixA[6][6];
	float __matrixC[6];

	//Calc Matrix A
	for ( i = 0; i < 6; i++){
		switch(i){
			case 0:
					__matrixA[0][i] = prod_and_sum_float_vlm(NULL, NULL, NULL, NULL, n);
					__matrixA[1][i] = __matrixA[i][1] = prod_and_sum_float_vlm(x1, NULL, NULL, NULL, n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_float_vlm(x2, NULL, NULL, NULL, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float_vlm(x1, x1, NULL, NULL, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float_vlm(x2, x2, NULL, NULL, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float_vlm(x1, x2, NULL, NULL, n);
			break;
			case 1: //*x1
					__matrixA[1][i] = prod_and_sum_float_vlm(x1, x1, NULL, NULL, n);
					__matrixA[2][i] = __matrixA[i][2] = prod_and_sum_float_vlm(x2, x1, NULL, NULL, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float_vlm(x1, x1, x1, NULL, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float_vlm(x2, x2, x1, NULL, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float_vlm(x1, x2, x1, NULL, n);
			break;
			case 2: //*x2
					__matrixA[2][i] = prod_and_sum_float_vlm(x2, x2, NULL, NULL, n);
					__matrixA[3][i] = __matrixA[i][3] = prod_and_sum_float_vlm(x1, x1, x2, NULL, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float_vlm(x2, x2, x2, NULL, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float_vlm(x1, x2, x2, NULL, n);
			break;
			case 3: //*x1_2
					__matrixA[3][i] = prod_and_sum_float_vlm(x1, x1, x1, x1, n);
					__matrixA[4][i] = __matrixA[i][4] = prod_and_sum_float_vlm(x2, x2, x1, x1, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float_vlm(x1, x2, x1, x1, n);
			break;
			case 4: //*x2_2
					__matrixA[4][i] = prod_and_sum_float_vlm(x2, x2, x2, x2, n);
					__matrixA[5][i] = __matrixA[i][5] = prod_and_sum_float_vlm(x1, x2, x2, x2, n);
			break;
			case 5: //*x1_x2
					__matrixA[5][i] = prod_and_sum_float_vlm(x1, x2, x1, x2, n);
			break;
		}
	}

	//Calc Matrix C
	__matrixC[0] = prod_and_sum_float_vlm(y, NULL, NULL, NULL, n);
	__matrixC[1] = prod_and_sum_float_vlm(y, x1, NULL, NULL, n);
	__matrixC[2] = prod_and_sum_float_vlm(y, x2, NULL, NULL, n);
	__matrixC[3] = prod_and_sum_float_vlm(y, x1, x1, NULL, n);
	__matrixC[4] = prod_and_sum_float_vlm(y, x2, x2, NULL, n);
	__matrixC[5] = prod_and_sum_float_vlm(y, x1, x2, NULL, n);

    //gauss elimination
    gauss_lm(__matrixA, __matrixC, returned_b, 6);
	
	*returned_sse = 0;
    //Mean Squared Error Calc
    for(i=0; i<n; i++){
         (*returned_sse) += pow(y[i] - (returned_b[0] + returned_b[1] * x1[i] + returned_b[2] * x2[i] + returned_b[3] * x1[i] * x1[i] + returned_b[4] * x2[i] * x2[i] + returned_b[5] * x1[i] * x2[i]), 2);
    }
     (*returned_sse) /= n;
	

	return;
}