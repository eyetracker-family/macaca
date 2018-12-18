#pragma once
#ifndef __MATRIX__MULTIPLY__
#define __MATRIX__MULTIPLY__

#include <cmath>

#include <iostream>
#include<cstring>
#include <string>
#include "foward_kinemat.h"
#include "matrix_inverse.h"
#include <fstream>

#define PI 3.14159265358979f

using namespace std;

typedef struct _joint_angle
{
	float theta1;
	float theta2;
	float theta3;
	float theta4;
	float theta5;
	float theta6;
	float theta7;
} joint_angle_t;

typedef struct _kinemat_envar {
	joint_angle_t foward_angle;
	joint_angle_t inverse_angle;
	float oldBallPos[3];//
	float ballPos[3];//
	float oldPos[3];
	float newPos[3];
	float PosOri[4][4];
	float PosOin[4][4];


}kinemat_envar_t;

typedef struct _Optimum_PSI {
	float psi;
	float min_float;
	joint_angle_t angle;
} Optimum_PSI_t;

bool Optimal_algorithm(void * pvParameters);




#endif // !MATRIX_MULTIPLY
