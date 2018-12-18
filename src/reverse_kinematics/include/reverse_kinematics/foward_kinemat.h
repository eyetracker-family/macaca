#ifndef __FOWARD_KINEMAT_
#define __FOEARD_KINEMAT_
#include <cmath>


#define DSE 329.2f //定义浮点数型
#define DEW 255.5f //

/*1y?é???ó*/
extern float T_temp1[4][4];
extern float T_temp2[4][4];


/*T01???ó*/
extern float T01_x0_90[4][4];//è?x0?áDy×a90?è

							 /*T12???ó*/
extern float T12_x1_90[4][4];//è?x1?áDy×a90?è
extern float T12_z2_90[4][4];//è?z2?áDy×a90?è
							 /*T23???ó*/
extern float T23_x2_90[4][4];//è?x2?áDy×a-90?è
extern float T23_z3_90[4][4];//è?z3?áDy×a90?è
							 /*T34???ó*/
extern float T34_z3_L1_x3_90[4][4];//??z3?áò??ˉL1μ??àà?￡?è?x3Dy×a90?è
extern float T34_z4_180[4][4];//è?z4?áDy×a180?è
							  /*T45???ó*/
extern float T45_y4_L2_x4_90[4][4];//??y4?áò??ˉL2μ??àà?￡?è?x4Dy×a90?è
extern float T45_z5_90[4][4];//è?z5?áDy×a90?è
							 /*T56???ó*/
extern float T56_x5_90[4][4];//è?x5?áDy×a-90?è
extern float T56_z6_90[4][4];//è?z6?áDy×a90?è
							 /*T67???ó*/
extern float T67_x6_90[4][4];//è?x6?áDy×a90?è
extern float T67_z7_90[4][4];//è?z7?áDy×a0?è??????????????

const float  T87[4][4] = {
						{1,0,0,0},
						{0, 1, 0, 0},
						{0, 0, 1, -70},
						{0,0,0,1}
						};

void matrix_multiply(float *a, float *b, float *c, int x, int y, int z);
void matrix_inv_3_3(float(*m1)[3], float(*m2)[3]);





#endif // !__FOWARD_KINEMAT_
