#include "matrix_inverse.h"
#include <cmath>

/*****************************************************************************
@funcname: RPY_matrix
@brief   : Euler angle   -> Rotate the matrix
@param   : float *m1          m1=1x6   =Euler angle
@param   : float (*m2)[4]   m2=4x4   =Rotate the matrix
@return  :
*****************************************************************************/
void RPY_matrix(float *Euler, float(*Rotate)[4])
{

	Rotate[0][0] = cos(Euler[0]) * cos(Euler[1]);
	Rotate[0][1] = cos(Euler[0]) * sin(Euler[1]) * sin(Euler[2]) - sin(Euler[0])*cos(Euler[2]);
	Rotate[0][2] = cos(Euler[0]) * sin(Euler[1]) * cos(Euler[2]) + sin(Euler[0])*sin(Euler[2]);
	Rotate[0][3] = Euler[3];

	Rotate[1][0] = sin(Euler[0]) * cos(Euler[1]);
	Rotate[1][1] = sin(Euler[0]) * sin(Euler[1]) * sin(Euler[2]) + cos(Euler[0])*cos(Euler[2]);
	Rotate[1][2] = sin(Euler[0]) * sin(Euler[1]) * cos(Euler[2]) - cos(Euler[0])*sin(Euler[2]);
	Rotate[1][3] = Euler[4];

	Rotate[2][0] = -sin(Euler[1]);
	Rotate[2][1] = cos(Euler[1]) * sin(Euler[2]);
	Rotate[2][2] = cos(Euler[1]) * cos(Euler[2]);
	Rotate[2][3] = Euler[5];

	Rotate[3][0] = 0;
	Rotate[3][1] = 0;
	Rotate[3][2] = 0;
	Rotate[3][3] = 1;

}