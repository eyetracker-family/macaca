#include "matrix_multiply.h"

void record_foward_angle(kinemat_envar_t *p_knmt)
{
	uint32_t angle_num;
	joint_angle_t *angle;
	angle = &p_knmt->foward_angle;

	angle_num = sizeof(joint_angle_t) / sizeof(float);
	
	cout << "please input seven foward angle"  << endl;
	cin >> angle->theta1 >> angle->theta2 >> angle->theta3 >> angle->theta4>> angle->theta5 >> angle->theta6>> angle->theta7 ;
	
}

void forward_kinemat(kinemat_envar_t * p_knmt)
{
	float T01_z1_Theta1[4][4] = {
		{ 0.0,0.0,0,0 },//T01_z1_Theta1[0][0]=cos(Theta[0]),T01_z1_Theta1[0][1]=-sin(Theta[0])
	{ 0.0,0.0,0,0 },//T01_z1_Theta1[1][0]=sin(Theta[0]),T01_z1_Theta1[1][1]=cos(Theta[0])
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};
	float T12_z2_Theta2[4][4] = {
		{ 0.0,0.0,0 ,0 },//T12_z2_Theta2[0][0]=cos(Theta[1]),T12_z2_Theta2[0][1]=-sin(Theta[1])
	{ 0.0,0.0,0 ,0 },//T12_z2_Theta2[1][0]=sin(Theta[1]),T12_z2_Theta2[1][1]=cos(Theta[1])
	{ 0  ,0  ,1 ,0 },
	{ 0  ,0  ,0 ,1 }
	};
	float T23_z3_Theta3[4][4] = {
		{ 0.0,0.0,0,0 },//T23_z3_Theta3[0][0]=cos(Theta[2]),T23_z3_Theta3[0][1]=-sin(Theta[2])
	{ 0.0,0.0,0,0 },//T23_z3_Theta3[1][0]=sin(Theta[2]),T23_z3_Theta3[1][1]=
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};
	float T34_z4_Theta4[4][4] = {
		{ 0.0,0.0,0,0 },//T34_z4_Theta4[0][0],T34_z4_Theta4[0][1]
	{ 0.0,0.0,0,0 },//T34_z4_Theta4[1][0],T34_z4_Theta4[1][1]
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};

	float T45_z5_Theta5[4][4] = {
		{ 0.0,0.0,0,0 },//T45_z5_Theta5[0][0],T45_z5_Theta5[0][1]
	{ 0.0,0.0,0,0 },//T45_z5_Theta5[1][0],T45_z5_Theta5[1][1]
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};

	float T56_z6_Theta6[4][4] = {
		{ 0.0,0.0,0,0 },//T56_z6_Theta6[0][0],T56_z6_Theta6[0][1]
	{ 0.0,0.0,0,0 },//T56_z6_Theta6[1][0],T56_z6_Theta6[1][1]
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};
	float T67_z7_Theta7[4][4] = {
		{ 0.0,0.0,0,0 },//T67_z7_Theta7[0][0],T67_z7_Theta7[0][1]
	{ 0.0,0.0,0,0 },//T67_z7_Theta7[1][0],T67_z7_Theta7[1][1]
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};

	float  T78[4][4] = {
						{ 1,0,0,70 },
						{ 0, -1, 0, 0 },
						{ 0, 0, -1, 0 },
						{ 0,0,0,1 }
						};

	joint_angle_t *angle;
	angle = &p_knmt->foward_angle;

	T01_z1_Theta1[0][0] = cos(angle->theta1);
	T01_z1_Theta1[0][1] = -sin(angle->theta1);
	T01_z1_Theta1[1][0] = sin(angle->theta1);
	T01_z1_Theta1[1][1] = cos(angle->theta1);

	T12_z2_Theta2[0][0] = cos(angle->theta2);
	T12_z2_Theta2[0][1] = -sin(angle->theta2);
	T12_z2_Theta2[1][0] = sin(angle->theta2);
	T12_z2_Theta2[1][1] = cos(angle->theta2);

	T23_z3_Theta3[0][0] = cos(angle->theta3);
	T23_z3_Theta3[0][1] = -sin(angle->theta3);
	T23_z3_Theta3[1][0] = sin(angle->theta3);
	T23_z3_Theta3[1][1] = cos(angle->theta3);

	T34_z4_Theta4[0][0] = cos(angle->theta4);
	T34_z4_Theta4[0][1] = -sin(angle->theta4);
	T34_z4_Theta4[1][0] = sin(angle->theta4);
	T34_z4_Theta4[1][1] = cos(angle->theta4);

	T45_z5_Theta5[0][0] = cos(angle->theta5);
	T45_z5_Theta5[0][1] = -sin(angle->theta5);
	T45_z5_Theta5[1][0] = sin(angle->theta5);
	T45_z5_Theta5[1][1] = cos(angle->theta5);

	T56_z6_Theta6[0][0] = cos(angle->theta6);
	T56_z6_Theta6[0][1] = -sin(angle->theta6);
	T56_z6_Theta6[1][0] = sin(angle->theta6);
	T56_z6_Theta6[1][1] = cos(angle->theta6);

	T67_z7_Theta7[0][0] = cos(angle->theta7);
	T67_z7_Theta7[0][1] = -sin(angle->theta7);
	T67_z7_Theta7[1][0] = sin(angle->theta7);
	T67_z7_Theta7[1][1] = cos(angle->theta7);

	matrix_multiply(*T01_x0_90, *T01_z1_Theta1, *T_temp1, 4, 4, 4);//T01_x0_90[4][4]*T01_z1_Theta1[4][4]
	matrix_multiply(*T_temp1, *T12_x1_90, *T_temp2, 4, 4, 4);//*T12_x1_90[4][4]
	matrix_multiply(*T_temp2, *T12_z2_90, *T_temp1, 4, 4, 4);//*T12_z2_90[4][4]
	matrix_multiply(*T_temp1, *T12_z2_Theta2, *T_temp2, 4, 4, 4);//*T12_z2_Theta2[4][4]
	matrix_multiply(*T_temp2, *T23_x2_90, *T_temp1, 4, 4, 4);//*T23_x2_90[4][4]
	matrix_multiply(*T_temp1, *T23_z3_90, *T_temp2, 4, 4, 4);//*T23_z3_90[4][4]
	matrix_multiply(*T_temp2, *T23_z3_Theta3, *T_temp1, 4, 4, 4);//*T23_z3_Theta3[4][4]
	matrix_multiply(*T_temp1, *T34_z3_L1_x3_90, *T_temp2, 4, 4, 4);//*T34_z3_L1_x3_90[4][4]

	matrix_multiply(*T_temp2, *T34_z4_180, *T_temp1, 4, 4, 4);//*T34_z4_180[4][4]
	matrix_multiply(*T_temp1, *T34_z4_Theta4, *T_temp2, 4, 4, 4);//*T34_z4_Theta4[4][4]
	matrix_multiply(*T_temp2, *T45_y4_L2_x4_90, *T_temp1, 4, 4, 4);//*T45_y4_L2_x4_90[4][4]
	matrix_multiply(*T_temp1, *T45_z5_90, *T_temp2, 4, 4, 4);//*T45_z5_90[4][4]
	matrix_multiply(*T_temp2, *T45_z5_Theta5, *T_temp1, 4, 4, 4);//*T45_z5_Theta5[4][4]
	matrix_multiply(*T_temp1, *T56_x5_90, *T_temp2, 4, 4, 4);//*T56_x5_90[4][4]
	matrix_multiply(*T_temp2, *T56_z6_90, *T_temp1, 4, 4, 4);//*T56_z6_90[4][4]
	matrix_multiply(*T_temp1, *T56_z6_Theta6, *T_temp2, 4, 4, 4);//*T56_z6_Theta6[4][4]
	matrix_multiply(*T_temp2, *T67_x6_90, *T_temp1, 4, 4, 4);//*T67_x6_90[4][4]
	matrix_multiply(*T_temp1, *T67_z7_90, *T_temp2, 4, 4, 4);//*T67_z7_90[4][4]
	matrix_multiply(*T_temp2, *T67_z7_Theta7, *T_temp1, 4, 4, 4);//*T67_z7_Theta7[4][4]	

	matrix_multiply(*T_temp1, *T78, *(p_knmt->PosOri), 4, 4, 4);//*T78[4][4]	

}





void inverse_kinemat(kinemat_envar_t * p_knmt, float Psi)
{
	float(*posin)[4];
	joint_angle_t *angle;
	posin = p_knmt->PosOin; //逆解矩阵
	float temp_posin[4][4] = {0.0};
	float  T87[4][4] = {
						{ 1,0,0,-70 },
						{ 0, -1, 0, 0 },
						{ 0, 0, -1, 0 },
						{ 0,0,0,1 }
						};

	matrix_multiply(*posin,*T87,*temp_posin,4,4,4);



	angle = &p_knmt->inverse_angle;
	float R07[3][3] = { 0.0 };
	float cosTheta4 = 0.0;
	float cosThetaESW = 0.0;
	float ThetaESW = 0.0;
	float Oe[3][1] = { 0.0 };
	float r_Oe = 0.0;
	float a[3] = { 0,0,0 };
	float b[3] = { 0,0,0 };
	float a_norm = 0;
	float b_norm = 0;
	float Ex = 0.0;
	float Ey = 0.0;
	float Ez = 0.0;
	float Zs_norm = 0;
	float Zs[3] = { 0,0,0 };
	float Ys_norm = 0;
	float Ys[3] = { 0,0,0 };
	float Xs[3] = { 0,0,0 };
	float Rac[3][3] = { 0.0 };
	float R_ac[3][3] = { 0.0 };
	float T04[4][4] = { 0.0 };//T04=T01*T12*T23*T34
	float T05[4][4] = { 0.0 };//T05=T01*T12*T23*T34*T45
	float R04[3][3] = { 0.0 };
	float R04_[3][3] = { 0.0 };
	float P05[3][1] = { 0.0 };
	float R47[3][3] = { 0.0 };
	float xsw[3][1] = {{ 1.5 },
		             { 2.5 },
					 { 3.5 } };
	float vi[3] = { 1 ,0 ,0 };
	float v[3][1] = { 0.0 };

	float T01_z1_Theta1[4][4] = {
		{ 0.0,0.0,0,0 },//T01_z1_Theta1[0][0]=cos(Theta[0]),T01_z1_Theta1[0][1]=-sin(Theta[0])
	{ 0.0,0.0,0,0 },//T01_z1_Theta1[1][0]=sin(Theta[0]),T01_z1_Theta1[1][1]=cos(Theta[0])
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};
	float T12_z2_Theta2[4][4] = {
		{ 0.0,0.0,0 ,0 },//T12_z2_Theta2[0][0]=cos(Theta[1]),T12_z2_Theta2[0][1]=-sin(Theta[1])
	{ 0.0,0.0,0 ,0 },//T12_z2_Theta2[1][0]=sin(Theta[1]),T12_z2_Theta2[1][1]=cos(Theta[1])
	{ 0  ,0  ,1 ,0 },
	{ 0  ,0  ,0 ,1 }
	};
	float T23_z3_Theta3[4][4] = {
		{ 0.0,0.0,0,0 },//T23_z3_Theta3[0][0]=cos(Theta[2]),T23_z3_Theta3[0][1]=-sin(Theta[2])
	{ 0.0,0.0,0,0 },//T23_z3_Theta3[1][0]=sin(Theta[2]),T23_z3_Theta3[1][1]=
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};
	float T34_z4_Theta4[4][4] = {
		{ 0.0,0.0,0,0 },//T34_z4_Theta4[0][0],T34_z4_Theta4[0][1]
	{ 0.0,0.0,0,0 },//T34_z4_Theta4[1][0],T34_z4_Theta4[1][1]
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};

	float T45_z5_Theta5[4][4] = {
		{ 0.0,0.0,0,0 },//T45_z5_Theta5[0][0],T45_z5_Theta5[0][1]
	{ 0.0,0.0,0,0 },//T45_z5_Theta5[1][0],T45_z5_Theta5[1][1]
	{ 0  ,0  ,1,0 },
	{ 0  ,0  ,0,1 }
	};

	xsw[0][0] = temp_posin[0][3];
	xsw[1][0] = temp_posin[1][3]  ;
	xsw[2][0] = temp_posin[2][3];

	R07[0][0] = temp_posin[0][0];
	R07[0][1] = temp_posin[0][1];
	R07[0][2] = temp_posin[0][2];

	R07[1][0] = temp_posin[1][0];
	R07[1][1] = temp_posin[1][1];
	R07[1][2] = temp_posin[1][2];

	R07[2][0] = temp_posin[2][0];
	R07[2][1] = temp_posin[2][1];
	R07[2][2] = temp_posin[2][2];
	//以下是求theta4
	cosTheta4 = (pow(xsw[0][0], 2) + pow(xsw[1][0], 2) + pow(xsw[2][0], 2) - pow(DSE, 2) - pow(DEW, 2)) / (2 * DSE*DEW);
	if (cosTheta4 > 1)
		cosTheta4 = 1;
	else if (cosTheta4<-1)
		cosTheta4 = -1;
	angle->theta4 = acos(cosTheta4);
	//求elbow组成的圆的方程
	cosThetaESW = (pow(DSE, 2) + pow(xsw[0][0], 2) + pow(xsw[1][0], 2) + pow(xsw[2][0], 2) - pow(DEW, 2)) / (2 * DSE*sqrt(pow(xsw[0][0], 2) + pow(xsw[1][0], 2) + pow(xsw[2][0], 2)));
	ThetaESW = acos(cosThetaESW);
	//求圆心的位置
	Oe[0][0] = xsw[0][0] * (DSE*cosThetaESW / sqrt(pow(xsw[0][0], 2) + pow(xsw[1][0], 2) + pow(xsw[2][0], 2)));
	Oe[1][0] = xsw[1][0] * (DSE*cosThetaESW / sqrt(pow(xsw[0][0], 2) + pow(xsw[1][0], 2) + pow(xsw[2][0], 2)));
	Oe[2][0] = xsw[2][0] * (DSE*cosThetaESW / sqrt(pow(xsw[0][0], 2) + pow(xsw[1][0], 2) + pow(xsw[2][0], 2)));
	//圆的半径
	r_Oe = DSE * sin(ThetaESW);
	//求法向量
	v[0][0] = xsw[0][0];
	v[1][0] = xsw[1][0];
	v[2][0] = xsw[2][0];
	//a=cross(v,vi);获得单位向量
	a[0] = v[1][0] * vi[2] - v[2][0] * vi[1];
	a[1] = v[2][0] * vi[0] - v[0][0] * vi[2];
	a[2] = v[0][0] * vi[1] - v[1][0] * vi[0];

	if ((a[0] == 0) && (a[1] == 0) && (a[2] == 0))
	{

		vi[0] = 0;
		vi[1] = 1;
		vi[2] = 0;
		//a=cross(v,vi);
		a[0] = v[1][0] * vi[2] - v[2][0] * vi[1];
		a[1] = v[2][0] * vi[0] - v[0][0] * vi[2];
		a[2] = v[0][0] * vi[1] - v[1][0] * vi[0];
		vi[0] = 1;
		vi[1] = 0;
		vi[2] = 0;
	}
	//b=cross(v,a);
	b[0] = v[1][0] * a[2] - v[2][0] * a[1];
	b[1] = v[2][0] * a[0] - v[0][0] * a[2];
	b[2] = v[0][0] * a[1] - v[1][0] * a[0];

	a_norm = sqrt(pow(a[0], 2) + pow(a[1], 2) + pow(a[2], 2));
	a[0] = a[0] / a_norm;
	a[1] = a[1] / a_norm;
	a[2] = a[2] / a_norm;
	
	b_norm = sqrt(pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2));
	b[0] = b[0] / b_norm;
	b[1] = b[1] / b_norm;
	b[2] = b[2] / b_norm;
	//求得E点的位置
	Ex = Oe[0][0] + r_Oe * cos(Psi - PI / 2)*a[0] + r_Oe * sin(Psi - PI / 2)*b[0];
	Ey = Oe[1][0] + r_Oe * cos(Psi - PI / 2)*a[1] + r_Oe * sin(Psi - PI / 2)*b[1];
	Ez = Oe[2][0] + r_Oe * cos(Psi - PI / 2)*a[2] + r_Oe * sin(Psi - PI / 2)*b[2];
	//利用构造法 得到旋转矩阵
	Zs[0] = -Ex;
	Zs[1] = -Ey;
	Zs[2] = -Ez;
	Zs_norm = sqrt(pow(Zs[0], 2) + pow(Zs[1], 2) + pow(Zs[2], 2));
	Zs[0] = Zs[0] / Zs_norm;
	Zs[1] = Zs[1] / Zs_norm;
	Zs[2] = Zs[2] / Zs_norm;

	Ys[0] = -(xsw[1][0] * Zs[2] - xsw[2][0] * Zs[1]);
	Ys[1] = -(xsw[2][0] * Zs[0] - xsw[0][0] * Zs[2]);
	Ys[2] = -(xsw[0][0] * Zs[1] - xsw[1][0] * Zs[0]);

	Ys_norm = sqrt(pow(Ys[0], 2) + pow(Ys[1], 2) + pow(Ys[2], 2));
	Ys[0] = Ys[0] / Ys_norm;
	Ys[1] = Ys[1] / Ys_norm;
	Ys[2] = Ys[2] / Ys_norm;
	
	Xs[0] = Ys[1] * Zs[2] - Ys[2] * Zs[1];
	Xs[1] = Ys[2] * Zs[0] - Ys[0] * Zs[2];
	Xs[2] = Ys[0] * Zs[1] - Ys[1] * Zs[0];

	Rac[0][0] = Xs[0];
	Rac[0][1] = Xs[1];
	Rac[0][2] = Xs[2];

	Rac[1][0] = Ys[0];
	Rac[1][1] = Ys[1];
	Rac[1][2] = Ys[2];

	Rac[2][0] = Zs[0];
	Rac[2][1] = Zs[1];
	Rac[2][2] = Zs[2];

	R_ac[0][0] = Rac[0][0];
	R_ac[0][1] = Rac[1][0];
	R_ac[0][2] = Rac[2][0];

	R_ac[1][0] = Rac[0][1];
	R_ac[1][1] = Rac[1][1];
	R_ac[1][2] = Rac[2][1];

	R_ac[2][0] = Rac[0][2];
	R_ac[2][1] = Rac[1][2];
	R_ac[2][2] = Rac[2][2];

	angle->theta2 = asin(R_ac[1][2]);
	angle->theta1 = atan(R_ac[2][2] / R_ac[0][2]);
	angle->theta3 = atan(R_ac[1][0] / R_ac[1][1]);


	T01_z1_Theta1[0][0] = cos(angle->theta1);
	T01_z1_Theta1[0][1] = -sin(angle->theta1);
	T01_z1_Theta1[1][0] = sin(angle->theta1);
	T01_z1_Theta1[1][1] = cos(angle->theta1);


	T12_z2_Theta2[0][0] = cos(angle->theta2);
	T12_z2_Theta2[0][1] = -sin(angle->theta2);
	T12_z2_Theta2[1][0] = sin(angle->theta2);
	T12_z2_Theta2[1][1] = cos(angle->theta2);


	T23_z3_Theta3[0][0] = cos(angle->theta3);
	T23_z3_Theta3[0][1] = -sin(angle->theta3);
	T23_z3_Theta3[1][0] = sin(angle->theta3);
	T23_z3_Theta3[1][1] = cos(angle->theta3);


	T34_z4_Theta4[0][0] = cos(angle->theta4);
	T34_z4_Theta4[0][1] = -sin(angle->theta4);
	T34_z4_Theta4[1][0] = sin(angle->theta4);
	T34_z4_Theta4[1][1] = cos(angle->theta4);
	matrix_multiply(*T01_x0_90, *T01_z1_Theta1, *T_temp1, 4, 4, 4);//T01_x0_90[4][4]*T01_z1_Theta1[4][4]
	matrix_multiply(*T_temp1, *T12_x1_90, *T_temp2, 4, 4, 4);//*T12_x1_90[4][4]
	matrix_multiply(*T_temp2, *T12_z2_90, *T_temp1, 4, 4, 4);//*T12_z2_90[4][4]
	matrix_multiply(*T_temp1, *T12_z2_Theta2, *T_temp2, 4, 4, 4);//*T12_z2_Theta2[4][4]
	matrix_multiply(*T_temp2, *T23_x2_90, *T_temp1, 4, 4, 4);//*T23_x2_90[4][4]
	matrix_multiply(*T_temp1, *T23_z3_90, *T_temp2, 4, 4, 4);//*T23_z3_90[4][4]
	matrix_multiply(*T_temp2, *T23_z3_Theta3, *T_temp1, 4, 4, 4);//*T23_z3_Theta3[4][4]
	matrix_multiply(*T_temp1, *T34_z3_L1_x3_90, *T_temp2, 4, 4, 4);//*T34_z3_L1_x3_90[4][4]
	matrix_multiply(*T_temp2, *T34_z4_180, *T_temp1, 4, 4, 4);//*T34_z4_180[4][4]
	matrix_multiply(*T_temp1, *T34_z4_Theta4, *T04, 4, 4, 4);//*T34_z4_Theta4[4][4]

	matrix_multiply(*T04, *T45_y4_L2_x4_90, *T_temp1, 4, 4, 4);//*T45_y4_L2_x4_90[4][4]
	matrix_multiply(*T_temp1, *T45_z5_90, *T_temp2, 4, 4, 4);//*T45_z5_90[4][4]
	matrix_multiply(*T_temp2, *T45_z5_Theta5, *T05, 4, 4, 4);//*T45_z5_Theta5[4][4]

	R04[0][0] = T04[0][0];
	R04[0][1] = T04[0][1];
	R04[0][2] = T04[0][2];

	R04[1][0] = T04[1][0];
	R04[1][1] = T04[1][1];
	R04[1][2] = T04[1][2];

	R04[2][0] = T04[2][0];
	R04[2][1] = T04[2][1];
	R04[2][2] = T04[2][2];

	P05[0][0] = T05[0][3];
	P05[1][0] = T05[1][3];
	P05[2][0] = T05[2][3];

	if (sqrt(pow(P05[0][0] - xsw[0][0], 2) + pow(P05[1][0] - xsw[1][0], 2) + pow(P05[2][0] - xsw[2][0], 2))>0.001)
	{
		angle->theta4 = -angle->theta4;

		T01_z1_Theta1[0][0] = cos(angle->theta1);
		T01_z1_Theta1[0][1] = -sin(angle->theta1);
		T01_z1_Theta1[1][0] = sin(angle->theta1);
		T01_z1_Theta1[1][1] = cos(angle->theta1);


		T12_z2_Theta2[0][0] = cos(angle->theta2);
		T12_z2_Theta2[0][1] = -sin(angle->theta2);
		T12_z2_Theta2[1][0] = sin(angle->theta2);
		T12_z2_Theta2[1][1] = cos(angle->theta2);


		T23_z3_Theta3[0][0] = cos(angle->theta3);
		T23_z3_Theta3[0][1] = -sin(angle->theta3);
		T23_z3_Theta3[1][0] = sin(angle->theta3);
		T23_z3_Theta3[1][1] = cos(angle->theta3);


		T34_z4_Theta4[0][0] = cos(angle->theta4);
		T34_z4_Theta4[0][1] = -sin(angle->theta4);
		T34_z4_Theta4[1][0] = sin(angle->theta4);
		T34_z4_Theta4[1][1] = cos(angle->theta4);

		matrix_multiply(*T01_x0_90, *T01_z1_Theta1, *T_temp1, 4, 4, 4);//T01_x0_90[4][4]*T01_z1_Theta1[4][4]
		matrix_multiply(*T_temp1, *T12_x1_90, *T_temp2, 4, 4, 4);//*T12_x1_90[4][4]
		matrix_multiply(*T_temp2, *T12_z2_90, *T_temp1, 4, 4, 4);//*T12_z2_90[4][4]
		matrix_multiply(*T_temp1, *T12_z2_Theta2, *T_temp2, 4, 4, 4);//*T12_z2_Theta2[4][4]
		matrix_multiply(*T_temp2, *T23_x2_90, *T_temp1, 4, 4, 4);//*T23_x2_90[4][4]
		matrix_multiply(*T_temp1, *T23_z3_90, *T_temp2, 4, 4, 4);//*T23_z3_90[4][4]
		matrix_multiply(*T_temp2, *T23_z3_Theta3, *T_temp1, 4, 4, 4);//*T23_z3_Theta3[4][4]
		matrix_multiply(*T_temp1, *T34_z3_L1_x3_90, *T_temp2, 4, 4, 4);//*T34_z3_L1_x3_90[4][4]
		matrix_multiply(*T_temp2, *T34_z4_180, *T_temp1, 4, 4, 4);//*T34_z4_180[4][4]
		matrix_multiply(*T_temp1, *T34_z4_Theta4, *T04, 4, 4, 4);//*T34_z4_Theta4[4][4]

		matrix_multiply(*T04, *T45_y4_L2_x4_90, *T_temp1, 4, 4, 4);//*T45_y4_L2_x4_90[4][4]
		matrix_multiply(*T_temp1, *T45_z5_90, *T_temp2, 4, 4, 4);//*T45_z5_90[4][4]
		matrix_multiply(*T_temp2, *T45_z5_Theta5, *T05, 4, 4, 4);//*T45_z5_Theta5[4][4]

		R04[0][0] = T04[0][0];
		R04[0][1] = T04[0][1];
		R04[0][2] = T04[0][2];

		R04[1][0] = T04[1][0];
		R04[1][1] = T04[1][1];
		R04[1][2] = T04[1][2];

		R04[2][0] = T04[2][0];
		R04[2][1] = T04[2][1];
		R04[2][2] = T04[2][2];

		P05[0][0] = T05[0][3];
		P05[1][0] = T05[1][3];
		P05[2][0] = T05[2][3];
	}
	matrix_inv_3_3(R04, R04_);
	matrix_multiply(*R04_, *R07, *R47, 3, 3, 3);

	angle->theta6 = asin(R47[1][2]);
	angle->theta5 = -atan(R47[0][2] / R47[2][2]);
	angle->theta7 = -atan(R47[1][1] / R47[1][0]);
	if (R47[0][2] * cos(angle->theta6)<0)
	{

		if (angle->theta5<0)
		{
			angle->theta5 = angle->theta5 + PI;
		}
	}
	else
	{
		angle->theta5 = angle->theta5 - PI;
	}
	if (R47[1][0] * cos(angle->theta6)<0)
	{

		angle->theta7 = angle->theta7 + PI;
	}
}

void vectorAdd(float *A, float *B,float *C ,int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		C[i] = A[i] + B[i];
	}
} 

void vectorMinus(float *A, float *B, float *C, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		C[i] = A[i] - B[i];
	}
}

float getVectorLength(float *vector, int n)
{
	float sum=0;
	int i;
	for (i = 0; i < n; i++)
	{
		sum += vector[i] * vector[i];

	}
	return sqrt(sum);
}

void getTargetMatrix(void * pvParameters,float Theta_z)
{
	kinemat_envar_t *p_knmt = (kinemat_envar_t*)pvParameters;

	float radius = 40.0;
	float distance, distance_ball, normalVectorLength;
	float temp[3];
	int i,j;
	float rotation[3][3],temp2[3][3],temp3[4][4];
	float theta,sin_theta,cos_theta;
	float vector[3];
	float normalVector[3];

	for (i = 0; i < 3; i++)
	{
		vector[i] = p_knmt->newPos[i] - p_knmt->oldPos[i];
	}

	distance = getVectorLength(vector, 3);
	if (distance != 0)
	{
		vectorMinus(p_knmt->newPos, p_knmt->oldPos, temp, 3);
		distance_ball = distance - radius;
		for (i = 0; i < 3; i++)
		{
			p_knmt->ballPos[i] = p_knmt->oldPos[i] + temp[i] * distance_ball / distance;
		}

	}
	else
	{
		for (i = 0; i < 3; i++)
		{
			p_knmt->ballPos[i] = p_knmt->oldBallPos[i];
		}
	}

	float Z[3] = {0, 0, 1};
	
	//求法向量
	normalVector[0] = Z[1] * vector[2] - vector[1]* Z[2];
	normalVector[1] = Z[2] * vector[0] - vector[2] * Z[0];
	normalVector[2] = Z[0] * vector[1] - vector[0] * Z[1];



	normalVectorLength = getVectorLength(normalVector,3);
	if (normalVectorLength != 0)
	{
		normalVector[0] = normalVector[0] / normalVectorLength;
		normalVector[1] = normalVector[1] / normalVectorLength;
		normalVector[2] = normalVector[2] / normalVectorLength;
		//theta = 1*acos(dot(z,AB)/(norm(z)*norm(AB)));
		theta = acos(vector[2] / distance);
	}
	else
	{
		normalVector[0] = 0;
		normalVector[1] = 0;
		normalVector[2] = 0;
		theta = 0;
	}

	//cout << "normalVector" << endl;
	//for (int i = 0; i < 3; i++)
	//{
	//	cout << normalVector[i] << "  ";
	//}
	//cout << "theta " << theta << endl;

	float K[3][3] =  { { 0,             -normalVector[2],    normalVector[1] },
					    {normalVector[2],         0,          -normalVector[0]},
						{-normalVector[1],     normalVector[0],              0}
					};
	float I[3][3] = { {1,0,0},
						{0,1,0},
						{0,0,1} };
	matrix_multiply(*K, *K, *temp2, 3, 3, 3);
	sin_theta = sin(theta);
	cos_theta = cos(theta);

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			rotation[i][j] = I[i][j]+(1 - cos_theta)*temp2[i][j] + sin_theta * K[i][j];

		}
	}
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			p_knmt->PosOin[i][j] = rotation[i][j];

		}
	}



	p_knmt->PosOin[0][3] = p_knmt->ballPos[0];
	p_knmt->PosOin[1][3] = p_knmt->ballPos[1];
	p_knmt->PosOin[2][3] = p_knmt->ballPos[2];

	p_knmt->PosOin[3][0] = 0;
	p_knmt->PosOin[3][1] = 0;
	p_knmt->PosOin[3][2] = 0;
	p_knmt->PosOin[3][3] = 1;


	float R_z[4][4] = { {cos(Theta_z), -sin(Theta_z), 0, 0},
						{sin(Theta_z), cos(Theta_z), 0, 0},
						{	0,0,1,0},
						{	0,0,0,1}};
	matrix_multiply(*p_knmt->PosOin, *R_z,*temp3,4, 4, 4);
	memcpy(p_knmt->PosOin, temp3, sizeof(temp3));

	
	

}
/*
第二种方法

*/

void getTargetMatrix_two(void * pvParameters, float Theta_z)
{
	kinemat_envar_t *p_knmt = (kinemat_envar_t*)pvParameters;

	float radius = 40.0;
	float distance, distance_ball, normalVectorLength;
	float temp[3];
	int i, j;
	float rotation[3][3] = {0.0},  temp3[4][4];
	float theta, sin_theta, cos_theta;
	float vector[3];
	float normalVector[3];


	p_knmt->ballPos[0] = p_knmt->newPos[0];
	p_knmt->ballPos[1] = p_knmt->newPos[1] - radius	;
	p_knmt->ballPos[2] = p_knmt->newPos[2];

	rotation[0][0] = 1.0;
	rotation[0][1] = 0.0;
	rotation[0][2] = 0.0;

	rotation[1][0] = 0.0;
	rotation[1][1] = 0.0;
	rotation[1][2] = 1.0;

	rotation[2][0] = 0.0;
	rotation[2][1] = -1.0;
	rotation[2][2] = 0.0;


	/*
	Rotation =[  1.0000         0         0
	0         0    1.0000
	0   -1.0000         0];
	*/

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			p_knmt->PosOin[i][j] = rotation[i][j];

		}
	}



	p_knmt->PosOin[0][3] = p_knmt->ballPos[0];
	p_knmt->PosOin[1][3] = p_knmt->ballPos[1];
	p_knmt->PosOin[2][3] = p_knmt->ballPos[2];

	p_knmt->PosOin[3][0] = 0;
	p_knmt->PosOin[3][1] = 0;
	p_knmt->PosOin[3][2] = 0;
	p_knmt->PosOin[3][3] = 1;


	float R_z[4][4] = { { cos(Theta_z), -sin(Theta_z), 0, 0 },
	{ sin(Theta_z), cos(Theta_z), 0, 0 },
	{ 0,0,1,0 },
	{ 0,0,0,1 } };
	matrix_multiply(*p_knmt->PosOin, *R_z, *temp3, 4, 4, 4);
	memcpy(p_knmt->PosOin, temp3, sizeof(temp3));

}
bool Optimal_algorithm(void * pvParameters)
{
	uint8_t i,j;
	kinemat_envar_t *p_knmt = (kinemat_envar_t*)pvParameters;
	joint_angle_t *joint = &p_knmt->inverse_angle;
	float psi,theta8;
	static Optimum_PSI_t last_value;
	Optimum_PSI_t new_value;
	bool flag = false ;
	last_value.min_float = 20.0f;
	for (j = 0; j<48; j++) {
		theta8 = j * 0.1308f;
		getTargetMatrix_two(p_knmt, theta8);
		for (i = 0; i<48; i++) {
			//if (last_value.psi)
			//	psi = (last_value.psi - 1.042f) + 0.0873*i; //-60° 每次+5°
			//else
				psi = i * 0.1308f; //15°
			if (psi<0)
				psi += 6.28f; // 如果为负 +360°
			if (psi>6.28f)  //如果大于360  -360
				psi -= 6.28f;

			new_value.psi = psi; 

			inverse_kinemat(p_knmt, psi);
			new_value.min_float = (joint->theta6)*(joint->theta6) + (joint->theta7)*(joint->theta7);

			
			memcpy(&new_value.angle, joint, sizeof(joint_angle_t)); 

			/***********************
			theta1 -90°~ 90°
			theta2 -10°~ 150°
			theta3 -90°~ 90°
			theta4 -10°~ 150°
			theta5 -90°~ 90°
			theta6 -90°~ 90°
			theta7 -90°~ 90°
			***************************/

			if (joint->theta1<1.57f && joint->theta1 >-1.57f && joint->theta2 < 2.6f && joint->theta2 >-0.17f && joint->theta3 <1.57f && joint->theta3>-1.57f \
				&& joint->theta4 <2.6f && joint->theta4 >-0.17f && joint->theta5 <1.57f && joint->theta5 >-1.57f && joint->theta6 < 3.0f && joint->theta6 >0 && joint->theta7 <1.57f && joint->theta7 >-1.57f)
				
			{
				
				flag = true;
				
				if (last_value.min_float > new_value.min_float)
				{
					// cout << "Psi = " << last_value.psi << endl;
					// cout << "last_value.min: " << last_value.min_float << "new_value.min: " << new_value.min_float << endl;
					memcpy(&last_value, &new_value, sizeof(Optimum_PSI_t));
				
				}
			}
		}
	}

	memcpy(joint, &last_value.angle, sizeof(joint_angle_t));
	memcpy(p_knmt->oldPos, p_knmt->newPos, sizeof(p_knmt->oldPos));
	memcpy(p_knmt->oldBallPos, p_knmt->ballPos, sizeof(p_knmt->oldPos));
	// cout << "Psi = " << last_value.psi << endl;
	return flag;
}

/*kinemat_envar_t knmt;

bool isArrive(float *euler)
{
	kinemat_envar_t *p_knmt;
	p_knmt = &knmt;
	bool flag;

	RPY_matrix(euler, p_knmt->PosOin);
	flag  = Optimal_algorithm(p_knmt);
	return flag;
}

int main()
{
	kinemat_envar_t *p_knmt;
    int row, column;
	p_knmt = &knmt;
	memset(p_knmt, 0, sizeof(kinemat_envar_t));

	float euler[6] = { 0.0 };

	while (1)
	{
		// 测试 利用键盘输入能否满足要求
		//RPY_matrix(euler, p_knmt->PosOin);
		cout << "请输入新的球心位置" << endl;
		for (int i = 0; i < 3; i++)
		{
			cin >> p_knmt->newPos[i];
		}

		Optimal_algorithm(p_knmt);

		//inverse_kinemat(p_knmt, 0);
		cout << "inverse angle = "<< endl \
			<< p_knmt->inverse_angle.theta1 << " " << p_knmt->inverse_angle.theta2 << " "\
			<< p_knmt->inverse_angle.theta3 << " " << p_knmt->inverse_angle.theta4 << " "\
			<< p_knmt->inverse_angle.theta5 << " " << p_knmt->inverse_angle.theta5 << " "\
			<< p_knmt->inverse_angle.theta7 << endl;
	}
		
	return 0;
}*/
