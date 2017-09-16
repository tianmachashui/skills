/************************************************************************/
/* Navigation subfunctions
	Designed and copyright by Xueyun Wang, Nov. 2, 2015
	Version Number:	1
	0315
	*/
/************************************************************************/
#include "include.h"

//#include "NavFun.h"
//#include "math.h"
//#include "MatrixFun.h"
//#include "stdio.h"
//#include "user_data.h"
////#include "malloc.h"
//#include "string.h"

#define Len_DWT 64//小波去噪信号长度,2的正整数次幂
#define dcp_class 4//小波分解级数
#define	LEN_F 8//滤波器长度，对于db4小波保持8不变
//======================== 组合导航参数 ============================
int KF_ini_flag=0;

g_kf_para g_kf_para1={0,0.0,0.0,0.0,0.0};
g_dzpg_kf g_dzpg_kf_matr={0.0};

double G_A_ibb_m[3]={0.0};
double G_W_ibb_m[3]={0.0};
double G_M_ebb_m[3]={0.0};

double Gyro_Bias[3]={0.0};

double g_Rota_FF[15*15]={0.0},g_Q0_Rot_FF[15*15]={0.0},g_FF_Q0[15*15]={0.0},DisF2[15*15]={0.0},DisF_temp[15*15]={0.0};

double g_RotDisFF[15*15]={0.0},g_DisFF_Pk[15*15]={0.0},g_temp_Pkk1[15*15]={0.0};//DisFF[15*15]={0.0},
double g_RotHk[15*7]={0.0},g_Hk_Xexp[7]={0.0},Kk[15*7]={0.0},g_DisFF_PK_RotDisFF[15*15]={0.0};

double g_temp_Mat_HkPk_k1[7*15]={0.0},g_temp_Mat_HkPk_k1_RotHk[7*7]={0.0},g_temp_sum_HkS_Rk[7*7]={0.0};
double g_temp_Pkk1_RotHk[15*7]={0.0},g_Rot_Kk[7*15]={0.0},g_temp_Kk_Hk[15*15]={0.0},g_temp_Kk_Hk1[15*7]={0.0};
double g_temp_sub_I_KH[15*15]={0.0},g_temp_KH[15*15]={0.0},g_Rot_temp_sub_I_KH[15*15]={0.0},g_temp_Pkk2[15*15]={0.0};
double g_temp1_Qk[15*6]={0.0},g_temp2_Qk[6*15]={0.0},G_Gk[15*6]={0.0};
//double g_temp1_Qk[15*15]={0.0},g_temp2_Qk[15*15]={0.0},G_Gk[15*15]={0.0};

double DT_VA = 0.01;
double G_Cnb[9] = {0.0},Q_err[4] = {0.0};

//extern FILE *fp_datain,*fp_dataout,*fp_Nav_out;
extern double G_W_ibb[3],G_A_ibb[3],G_M_ebb[3],G_A_nbn[3],GyroBias[3],AccBias[3],G_Mag_Heading,G_GPS_Longitude,G_GPS_Latitude,G_GPS_Height,G_GPS_Ve,G_GPS_Vn,G_GPS_Vu;
extern int FastAlign_Flag;
extern int FastCali_Flag;

double G_m[3] = {20,510,30}; //自定义磁场强度，待改
//小波变量
static double GyroX_DWT_Tmp[Len_DWT]={0.0},GyroX_DWT_Org[Len_DWT]={0.0},GyroX_DWT_Decomposition[Len_DWT]={0.0};
static double GyroX_DWT_Reconstruction[Len_DWT]={0.0},GyroX_DWT_RecTmp[Len_DWT]={0.0};
static double GyroY_DWT_Tmp[Len_DWT],GyroY_DWT_Org[Len_DWT]={0.0},GyroY_DWT_Decomposition[Len_DWT]={0.0};
static double GyroY_DWT_Reconstruction[Len_DWT]={0.0},GyroY_DWT_RecTmp[Len_DWT]={0.0};
static double GyroZ_DWT_Tmp[Len_DWT],GyroZ_DWT_Org[Len_DWT]={0.0},GyroZ_DWT_Decomposition[Len_DWT]={0.0};
static double GyroZ_DWT_Reconstruction[Len_DWT]={0.0},GyroZ_DWT_RecTmp[Len_DWT]={0.0};

static double AccX_DWT_Tmp[Len_DWT]={0.0},AccX_DWT_Org[Len_DWT]={0.0},AccX_DWT_Decomposition[Len_DWT]={0.0};
static double AccX_DWT_Reconstruction[Len_DWT]={0.0},AccX_DWT_RecTmp[Len_DWT]={0.0};
static double AccY_DWT_Tmp[Len_DWT]={0.0},AccY_DWT_Org[Len_DWT]={0.0},AccY_DWT_Decomposition[Len_DWT]={0.0};
static double AccY_DWT_Reconstruction[Len_DWT]={0.0},AccY_DWT_RecTmp[Len_DWT]={0.0};
static double AccZ_DWT_Tmp[Len_DWT]={0.0},AccZ_DWT_Org[Len_DWT]={0.0},AccZ_DWT_Decomposition[Len_DWT]={0.0};
static double AccZ_DWT_Reconstruction[Len_DWT]={0.0},AccZ_DWT_RecTmp[Len_DWT]={0.0};

static double MagX_DWT_Tmp[Len_DWT]={0.0},MagX_DWT_Org[Len_DWT]={0.0},MagX_DWT_Decomposition[Len_DWT]={0.0};
static double MagX_DWT_Reconstruction[Len_DWT]={0.0},MagX_DWT_RecTmp[Len_DWT]={0.0};
static double MagY_DWT_Tmp[Len_DWT]={0.0},MagY_DWT_Org[Len_DWT]={0.0},MagY_DWT_Decomposition[Len_DWT]={0.0};
static double MagY_DWT_Reconstruction[Len_DWT]={0.0},MagY_DWT_RecTmp[Len_DWT]={0.0};
static double MagZ_DWT_Tmp[Len_DWT]={0.0},MagZ_DWT_Org[Len_DWT]={0.0},MagZ_DWT_Decomposition[Len_DWT]={0.0};
static double MagZ_DWT_Reconstruction[Len_DWT]={0.0},MagZ_DWT_RecTmp[Len_DWT]={0.0};
int Len_DWT_dcp,Len_DWT_Denoi,Len_DWT_Recon,dcp_number;
double lemda; //小波降噪阈值
extern FILE *fp_DWT_out;

//int LEN_K0=(int)(Len_DWT+LEN_F-1)/2*2+1;
extern double h0[8],h1[8],g0[8],g1[8];//滤波器系数
double *recon_signal;//重建信号
double *decom_low,		*decom_hig;
double *decom_low_down,	*decom_hig_down;
double *recon_low_up,	*recon_hig_up;
double *recon_low,		*recon_hig;
double *recon_low_fix,*recon_hig_fix;
double *Median_temp;

/************************************************************************/
/* Function:
	导航数据初始化子程序                                          */
/************************************************************************/
void Nav_data_init(void)   //保留待用，修正主程序中加计、陀螺、磁强计参数
{
	int i=0;

	for(i=1;i<4;i++)	
		G_Q[i] = 0.0;

	G_Q[0] = 1.0;
	
	for(i=0;i<9;i++)		
	{
		G_Cbn[i] = 0.0;
	}

	G_Cbn[0] = 1.0;	G_Cbn[4] = 1.0;	G_Cbn[8] = 1.0;

}

/************************************************************************/
/* Function:
	温度补偿系数准备子程序                                          */
/************************************************************************/
void data_compensation_Coffi(void)
{

}

/************************************************************************/
/* Function:
	导航子程序                                                          */
/************************************************************************/
void Navigation()
{
	Gravity();
	UpdateVP_Nav();
	CalKalman_acc();
}

/************************************************************************/
/* Function:
	加速度、速度、位置更新                                              */
/************************************************************************/
void UpdateVP_Nav()
{
	int i=0;
	double temp_1[3],temp_2[3];
//	double k1=9.0,k2=81.0; 
	static int L_cnt_VP=0;
	double E_e;
	double Rxt,Ryt;
//	static double Vel_0[3]={0.0,0.0,0.0},Vel_1[3]={0.0,0.0,0.0},Vel_2[3]={0.0,0.0,0.0},Vel_3[3]={0.0,0.0,0.0};
	L_cnt_VP++;

	//计算delta_v_ibn
	G_delt_vn_p0[0] = (G_Cbn[0*3+0]*(G_A_ibb[0]) + G_Cbn[0*3+1]*(G_A_ibb[1]) + G_Cbn[0*3+2]*(G_A_ibb[2])) * DT_VA;
	G_delt_vn_p0[1] = (G_Cbn[1*3+0]*(G_A_ibb[0]) + G_Cbn[1*3+1]*(G_A_ibb[1]) + G_Cbn[1*3+2]*(G_A_ibb[2])) * DT_VA;
	G_delt_vn_p0[2] = (G_Cbn[2*3+0]*(G_A_ibb[0]) + G_Cbn[2*3+1]*(G_A_ibb[1]) + G_Cbn[2*3+2]*(G_A_ibb[2])) * DT_VA;
	for (i=0;i<3;i++)	temp_1[i] = G_delt_vn_p0[i];
	clear_vec(G_delt_vn_p0,3);

	//计算有害加速度
	temp_2[0] = ( (2*G_Wien[2]+G_Wenn[2])*G_Vel_I[1] - (2*G_Wien[1]+G_Wenn[1])*G_Vel_I[2] ) * DT_VA;
	temp_2[1] = (-(2*G_Wien[2]+G_Wenn[2])*G_Vel_I[0] + (2*G_Wien[0]+G_Wenn[0])*G_Vel_I[2] ) * DT_VA;
	temp_2[2] = ( (2*G_Wien[1]+G_Wenn[1])*G_Vel_I[0] - (2*G_Wien[0]+G_Wenn[0])*G_Vel_I[1] -G_g) * DT_VA;
	
	//速度更新
	for (i=0;i<3;i++)	G_Vel_I[i] += temp_1[i] + temp_2[i];
	
	//位置更新
	G_Pos_I[0] += G_Vel_I[0]*DT_VA*G_Rec_RE/cos(G_Pos_I[1]);
	G_Pos_I[1] += G_Vel_I[1]*DT_VA*G_Rec_RN;
	G_Pos_I[2] += G_Vel_I[2]*DT_VA;
	
	E_e = 1/298.3;
	Rxt = G_RE/(1-E_e*sin(G_GPS_Latitude)*sin(G_GPS_Latitude))+G_Pos_I[2];
	Ryt = G_RE/(1+2*E_e-3*E_e*sin(G_GPS_Latitude)*sin(G_GPS_Latitude))+G_Pos_I[2];
	
	G_P_I[0] = (G_Pos_I[0]-G_GPS_Longitude)*Rxt*cos(G_GPS_Latitude);
	G_P_I[1] = (G_Pos_I[1]-G_GPS_Latitude) *Ryt;
	G_P_I[2] =  G_Pos_I[2];
	
	
}

/************************************************************************/
/* Function:
	四元素及姿态矩阵更新子程序                                          */
/************************************************************************/
void UpdateQCbn()
{
	int i;
	double sum,sita0,shs0,chs0;
	double winb[3]={0.0};
	double Q0[4];
//	double l_temp_si2=0.0;
	double l_Q00,l_Q11,l_Q22,l_Q33,l_Q01,l_Q02,l_Q03,l_Q12,l_Q13,l_Q23;

	for(i=0;i<4;i++)
		Q0[i]=G_Q[i];

	UpdateWinn(); //更新地理系相对惯性性的角速度

	winb[0] = G_Cbn[0*3+0]*(G_Winn[0]) + G_Cbn[0*3+1]*(G_Winn[1]) + G_Cbn[0*3+0]*(G_Winn[2]);
	winb[1] = G_Cbn[1*3+0]*(G_Winn[0]) + G_Cbn[1*3+1]*(G_Winn[1]) + G_Cbn[1*3+1]*(G_Winn[2]);
	winb[2] = G_Cbn[2*3+0]*(G_Winn[0]) + G_Cbn[2*3+1]*(G_Winn[1]) + G_Cbn[2*3+2]*(G_Winn[2]);

	G_delt_thita_nbb[0] = (G_W_ibb[0] - winb[0])*DT_VA;
	G_delt_thita_nbb[1] = (G_W_ibb[1] - winb[1])*DT_VA;
	G_delt_thita_nbb[2] = (G_W_ibb[2] - winb[2])*DT_VA;

	sita0=(G_delt_thita_nbb[0]*G_delt_thita_nbb[0]+G_delt_thita_nbb[1]*G_delt_thita_nbb[1]+G_delt_thita_nbb[2]*G_delt_thita_nbb[2]);
	shs0=0.5-sita0/48.0 + sita0*sita0/3840.0;   
	chs0=1.0-sita0/8.0+sita0*sita0/384.0; 

	G_Q[0]=        			   chs0*Q0[0]-shs0*G_delt_thita_nbb[0]*Q0[1]-shs0*G_delt_thita_nbb[1]*Q0[2]-shs0*G_delt_thita_nbb[2]*Q0[3];
	G_Q[1]=shs0*G_delt_thita_nbb[0]*Q0[0]    				 +chs0*Q0[1]+shs0*G_delt_thita_nbb[2]*Q0[2]-shs0*G_delt_thita_nbb[1]*Q0[3];
	G_Q[2]=shs0*G_delt_thita_nbb[1]*Q0[0]-shs0*G_delt_thita_nbb[2]*Q0[1]        			+chs0*Q0[2]+shs0*G_delt_thita_nbb[0]*Q0[3];
	G_Q[3]=shs0*G_delt_thita_nbb[2]*Q0[0]+shs0*G_delt_thita_nbb[1]*Q0[1]-shs0*G_delt_thita_nbb[0]*Q0[2]        			   +chs0*Q0[3];

	sum=sqrt(G_Q[0]*G_Q[0]+G_Q[1]*G_Q[1]+G_Q[2]*G_Q[2]+G_Q[3]*G_Q[3]);
	for(i=0;i<4;i++)	
		G_Q[i]=G_Q[i]/sum;


	l_Q00 = G_Q[0]*G_Q[0];	l_Q11 = G_Q[1]*G_Q[1];	l_Q22 = G_Q[2]*G_Q[2];	l_Q33 = G_Q[3]*G_Q[3];
	l_Q12 = 2.0*G_Q[1]*G_Q[2];	l_Q03 = 2.0*G_Q[0]*G_Q[3];
	l_Q13 = 2.0*G_Q[1]*G_Q[3];	l_Q02 = 2.0*G_Q[0]*G_Q[2];
	l_Q23 = 2.0*G_Q[2]*G_Q[3];	l_Q01 = 2.0*G_Q[0]*G_Q[1];
	
	G_Cbn[0*3+0] = l_Q00 + l_Q11 - l_Q22 - l_Q33;  
	G_Cbn[0*3+1] = l_Q12 - l_Q03;
	G_Cbn[0*3+2] = l_Q13 + l_Q02;
	G_Cbn[1*3+0] = l_Q12 + l_Q03;
	G_Cbn[1*3+1] = l_Q00 - l_Q11 + l_Q22 - l_Q33;
	G_Cbn[1*3+2] = l_Q23 - l_Q01;
	G_Cbn[2*3+0] = l_Q13 - l_Q02;
	G_Cbn[2*3+1] = l_Q23 + l_Q01;
	G_Cbn[2*3+2] = l_Q00 - l_Q11 - l_Q22 + l_Q33;
	gram_sch(G_Cbn,3);

  G_Cnb[0]=G_Cbn[0]; G_Cnb[1]=G_Cbn[3]; G_Cnb[2]=G_Cbn[6];
	G_Cnb[3]=G_Cbn[1]; G_Cnb[4]=G_Cbn[4]; G_Cnb[5]=G_Cbn[7];
	G_Cnb[6]=G_Cbn[2]; G_Cnb[7]=G_Cbn[5]; G_Cnb[8]=G_Cbn[8];

	UpdateAttitude_Cbn();  //根据更新的Q和Cbn计算姿态角

}

/************************************************************************/
/* Function:
	姿态更新子程序                                                      */
/************************************************************************/
void UpdateAttitude_Cbn()   //姿态角定义待加
{

	G_Att[0] = asin(G_Cbn[2*3+1]);
	G_Att[1] = atan2(-G_Cbn[2*3+0],G_Cbn[2*3+2]);
	G_Att[2] = atan2(G_Cbn[0*3+1],G_Cbn[1*3+1]);

}											
/************************************************************************/
/* Function:
	更新相对运动角速度子程序	                                        */
/************************************************************************/
void UpdateWenn()
{
	double temp=0;
	temp = sqrt(1-e2*sin(G_Pos_I[1])*sin(G_Pos_I[1]));
	G_Rec_RN = pow(temp,3)/(R1_e2+G_Pos_I[2]);
	G_RN = 1.0/G_Rec_RN;
	G_Rec_RE = temp/(Re+G_Pos_I[2]);
	G_Wenn[0] = -G_Vel_I[1]*G_Rec_RN;
	G_Wenn[1] = G_Vel_I[0]*G_Rec_RE;
	G_Wenn[2] = G_Vel_I[0]*tan(G_Pos_I[1])*G_Rec_RE;

}
/************************************************************************/
/* Function:
	更新地球自转角速度子程序                                            */
/************************************************************************/
void UpdateWien()
{
	G_Wien[0] = 0;
	G_Wien[1] = wie*cos(G_Pos_I[1]);
	G_Wien[2] = wie*sin(G_Pos_I[1]);
}
/************************************************************************/
/* Function:
	更新指令角速度子函数                                                */
/************************************************************************/
void UpdateWinn()
{
	int i=0;
	UpdateWien();
	if (!FastAlign_Flag) //对准未完成
	{
		for(i=0;i<3;i++) G_Winn[i] = G_Wien[i];
	}
	else
	{
		UpdateWenn();
		for (i=0;i<3;i++)	G_Winn[i] = G_Wien[i] + G_Wenn[i];
	}
}
/************************************************************************/
/* Function:
	更新重力场子程序                                                    */
/************************************************************************/
void Gravity()
{
	double sin_lat=0.0;
	sin_lat = sin(G_Pos_I[1]);

	 //G_g = 9.780318 * (1 + (5.3024e-3)*sin_lat*sin_lat)*G_RN*G_RN/(G_RN + G_Pos_I[2])/(G_RN + G_Pos_I[2]);
	
	G_g = 9.780318 * (1 + (5.3024e-3)*pow(sin(G_Pos_I[1]),2) - (5.9e-6)*pow(sin(2*G_Pos_I[1]),2));
	//G_g = 9.78049 * (1 + (0.0052884)*pow(sin(G_Pos_I[1]),2) - (0.0000059)*pow(sin(2*G_Pos_I[1]),2));
}

/************************************************************************/
/* Function:
	OffLine Navigation Program.
	IMUGPS卡尔曼滤波子程序														*/
/************************************************************************/
void IMUGPS_Kalman(void)
{
	
	IMUGPS_Kalman_FirstSection();
	IMUGPS_Kalman_SecondSection();
	IMUGPS_Kalman_ThirdSection();
	IMUGPS_Kalman_FourthSection();
	IMUGPS_Kalman_FifthSection();

}
void IMUGPS_Kalman_FirstSection(void)
{
	double AlignTemp=0.0,AlignTemp2=0.0;

	if (KF_ini_flag == 0)
	{
		InitialKalman();//初始化				
		KF_ini_flag = 1;
	}

	CalculateF();//计算F阵
	GetDisF();
	CalculateQk();//计算Q阵
	
	//量测计算
	g_dzpg_kf_matr.Zk[0] = G_Pos_I[0] - G_GPS_Longitude;
	g_dzpg_kf_matr.Zk[1] = G_Pos_I[1] - G_GPS_Latitude;
	g_dzpg_kf_matr.Zk[2] = G_Pos_I[2] - G_GPS_Height;

	g_dzpg_kf_matr.Zk[3] = G_Vel_I[0] - G_GPS_Ve;
	g_dzpg_kf_matr.Zk[4] = G_Vel_I[1] - G_GPS_Vn;
	g_dzpg_kf_matr.Zk[5] = G_Vel_I[2] - G_GPS_Vu;

	AlignTemp = G_M_ebb[0]*cos(G_Att[0]) + G_M_ebb[1]*sin(G_Att[0])*sin(G_Att[1]) - G_M_ebb[2]*sin(G_Att[0])*cos(G_Att[1]);
    AlignTemp2 = G_M_ebb[1]*cos(G_Att[1]) - G_M_ebb[2]*sin(G_Att[1]);
    Heading_Mag = atan2(AlignTemp2,AlignTemp);

	//g_dzpg_kf_matr.Zk[6] = G_Att[2] - Heading_Mag;//使用自己计算的磁航向
	//g_dzpg_kf_matr.Zk[6] = (G_Att[2] - G_Mag_Heading);//使用数据中的磁航向
						
	//H阵的计算			
	g_dzpg_kf_matr.Hk[0*15+0] = 1.0; 
	g_dzpg_kf_matr.Hk[1*15+1] = 1.0; 
	g_dzpg_kf_matr.Hk[2*15+2] = 1.0; 
		
	g_dzpg_kf_matr.Hk[3*15+3] = 1.0; 
	g_dzpg_kf_matr.Hk[4*15+4] = 1.0; 
	g_dzpg_kf_matr.Hk[5*15+5] = 1.0; 

	g_dzpg_kf_matr.Hk[6*15+8] = 1.0;

}
void IMUGPS_Kalman_SecondSection(void)
{
		//卡尔曼滤波的5个方程
		computeXkexp();//Xexp = DisFF * X0;  X1 = DisFF * X0;	
		computePkexp();//P1 = DisFF * P0 * DisFF'+ Qk;
}
void IMUGPS_Kalman_ThirdSection(void)
{
		//卡尔曼滤波的5个方程
		computeKk();//Kk = P1 * Hk' * inv(Hk * P1 * Hk' + Rk);	

}
void IMUGPS_Kalman_FourthSection(void)
{
		computePk();//P0 = (eye(15) - Kk*Hk) * P1 * (eye(15) - Kk*Hk)' + Kk * Rk * Kk';	
}

void IMUGPS_Kalman_FifthSection(void)
{
		int i=0,j=0;			
		computeXk();
		AdjustNav();   //校正
		for(i=0;i<15;i++)
		{
			for(j=0;j<15;j++)
			{
				//g_dzpg_kf_matr.FF[i*15+j]=0.0; 
				g_dzpg_kf_matr.DisFF[i*15+j]=0.0; 
			}
		}	
}


/************************************************************************/
/* Function:
	OffLine Navigation Program.
	卡尔曼滤波初始化子程序                                                      */
/************************************************************************/
void InitialKalman()
{
    /*卡尔曼滤波初始化*/
	int i=0,j=0;
    KF_ini_flag=0;

	for (i=0;i<7;i++)
	{
		g_Hk_Xexp[i]=0.0;
	}
	for (i=0;i<7;i++)
	{
		g_dzpg_kf_matr.Zk[i]=0.0;//测量阵
	}

	for (i=0;i<15;i++)
	{
		g_dzpg_kf_matr.X0[i] = 0.0;
		g_dzpg_kf_matr.X1[i] = 0.0;
		g_dzpg_kf_matr.Xexp[i] = 0.0;
		
		for (j=0;j<15;j++)
		{
			
			g_dzpg_kf_matr.P1[i*15+j] = 0.0;			
			g_dzpg_kf_matr.Qk[i*15+j] = 0.0;
			g_dzpg_kf_matr.FF[i*15+j] = 0.0;
			g_dzpg_kf_matr.DisFF[i*15+j] = 0.0;

			g_Rota_FF[i*15+j]=0.0;
			//g_Q0_Rot_FF[i*15+j]=0.0;
			//g_FF_Q0[i*15+j]=0.0;
			DisF2[i*15+j]=0.0;
			DisF_temp[i*15+j]=0.0;
			g_RotDisFF[i*15+j]=0.0;
			g_DisFF_Pk[i*15+j]=0.0;
			g_temp_Pkk1[i*15+j]=0.0;
			g_DisFF_PK_RotDisFF[i*15+j]=0.0;
			g_temp_Kk_Hk[i*15+j]=0.0;
			g_temp_sub_I_KH[i*15+j]=0.0;
			g_temp_KH[i*15+j]=0.0;
			g_Rot_temp_sub_I_KH[i*15+j]=0.0;
			g_temp_Pkk2[i*15+j]=0.0;  
			//g_dzpg_kf_matr.Q0[i*15+j] = 0.0;
		}
	}

	for (i=0;i<105;i++)
	{

		g_dzpg_kf_matr.Hk[i] = 0.0;
		g_RotHk[i]= 0.0;
		g_temp_Mat_HkPk_k1[i]= 0.0;
		g_Rot_Kk[i]= 0.0;
		Kk[i]= 0.0;
		g_temp_Pkk1_RotHk[i]= 0.0;
		g_temp_Kk_Hk1[i]= 0.0;	
        //g_dzpg_kf_matr.Q0[i] = 0.0;
	}


	for(i=0;i<49;i++)
	{
		//g_dzpg_kf_matr.Q0[i] = 0.0;
		g_dzpg_kf_matr.Rk[i] = 0.0;
		g_temp_Mat_HkPk_k1_RotHk[i]= 0.0;
		g_temp_sum_HkS_Rk[i]= 0.0;
	}
	for(i=0;i<36;i++)
	{
		g_dzpg_kf_matr.Q0[i] = 0.0;
	}

/*P、Q、R阵初始化*/
		
	g_dzpg_kf_matr.P1[0*15+0] = pow(3.0/6371245,2);//位置误差
	g_dzpg_kf_matr.P1[1*15+1] = pow(3.0/6371245,2);
	g_dzpg_kf_matr.P1[2*15+2] = pow(3,2);

	g_dzpg_kf_matr.P1[3*15+3] = pow(1.0,2); //速度误差
	g_dzpg_kf_matr.P1[4*15+4] = pow(1.0,2);
	g_dzpg_kf_matr.P1[5*15+5] = pow(1.0,2);

	g_dzpg_kf_matr.P1[6*15+6] = pow(5*Deg,2);//姿态误差
	g_dzpg_kf_matr.P1[7*15+7] = pow(5*Deg,2);
	g_dzpg_kf_matr.P1[8*15+8] = pow(10*Deg,2);
/*
	g_dzpg_kf_matr.P1[9*15+9] = pow(0.5*Deg,2);//陀螺零偏重复性，说明书中未给出。按陀螺零偏值1deg/s设定
	g_dzpg_kf_matr.P1[10*15+10] = pow(0.5*Deg,2);
	g_dzpg_kf_matr.P1[11*15+11] = pow(0.5*Deg,2);

	g_dzpg_kf_matr.P1[12*15+12] = pow(9,2);//加计零偏重复性，说明书中未给出。按加计零偏值80mg设定
	g_dzpg_kf_matr.P1[13*15+13] = pow(9,2);
	g_dzpg_kf_matr.P1[14*15+14] = pow(9,2);
*/
	
	g_dzpg_kf_matr.Q0[0*6+0] = pow(0.005,2)*100;//实测陀螺噪声RMS=0.005rad/s
	g_dzpg_kf_matr.Q0[1*6+1] = pow(0.005,2)*100;
	g_dzpg_kf_matr.Q0[2*6+2] = pow(0.005,2)*100;
	g_dzpg_kf_matr.Q0[3*6+3] = pow(0.015,2)*100;//实测加速度计噪声
	g_dzpg_kf_matr.Q0[4*6+4] = pow(0.015,2)*100;
	g_dzpg_kf_matr.Q0[5*6+5] = pow(0.05,2)*100;

	g_dzpg_kf_matr.Rk[0*7+0] = pow(1e-7,2);//GPS位置噪声，2m/6371245m/cosd(40)	
	g_dzpg_kf_matr.Rk[1*7+1] = pow(8e-8,2);
	g_dzpg_kf_matr.Rk[2*7+2] = pow(0.2,2);//气压高度计位置误差0.2m
	
	g_dzpg_kf_matr.Rk[3*7+3] = pow(0.2,2);//GPS速度噪声，0.1m/s，注意R值取值！！
	g_dzpg_kf_matr.Rk[4*7+4] = pow(0.2,2);//0517
													//	g_dzpg_kf_matr.Rk[3*7+3] = pow(0.05,2);//GPS速度噪声，0.1m/s，注意R值取值！！
													//	g_dzpg_kf_matr.Rk[4*7+4] = pow(0.05,2);
	
	g_dzpg_kf_matr.Rk[5*7+5] = pow(0.1,2);
	g_dzpg_kf_matr.Rk[6*7+6] = pow(2*Deg,2);//磁航向仪等效航向噪声，未用


}
void CalKalman_acc()//100Hz计算
{
	/*加速度计算*/
	/*g_kf_para1.acc_e = (G_Vel_I[0] - g_kf_para1.vel_e_p)/DT_VA;
	g_kf_para1.acc_n = (G_Vel_I[1] - g_kf_para1.vel_n_p)/DT_VA;
	g_kf_para1.acc_u = (G_Vel_I[2] - g_kf_para1.vel_u_p)/DT_VA;	
	g_kf_para1.vel_e_p = G_Vel_I[0];
	g_kf_para1.vel_n_p = G_Vel_I[1];
	g_kf_para1.vel_u_p = G_Vel_I[2];*///此方法不对，天向比力不应为0

	g_kf_para1.acc_e = G_Cbn[0*3+0]*(G_A_ibb[0]) + G_Cbn[1*3+0]*(G_A_ibb[1]) + G_Cbn[2*3+0]*(G_A_ibb[2]);
	g_kf_para1.acc_n = G_Cbn[0*3+1]*(G_A_ibb[0]) + G_Cbn[1*3+1]*(G_A_ibb[1]) + G_Cbn[2*3+1]*(G_A_ibb[2]);
	g_kf_para1.acc_u = G_Cbn[0*3+2]*(G_A_ibb[0]) + G_Cbn[1*3+2]*(G_A_ibb[1]) + G_Cbn[2*3+2]*(G_A_ibb[2]);
	
	G_A_nbn[0] = g_kf_para1.acc_e;
	G_A_nbn[1] = g_kf_para1.acc_n;
	G_A_nbn[2] = g_kf_para1.acc_u - G_g;
}
/************************************************************************/
/* Function:
	OffLine Navigation Program.
	卡尔曼滤波计算F阵子程序                                             */
/************************************************************************/	
void CalculateF()
{
	/*计算F阵*/
//	int i=0;

    /*计算F阵*/
	double l_tan_lat=0.0,l_sec_lat=0.0,l_sec_lat2=0.0;
		
	l_tan_lat=tan(G_Pos_I[1]);
	l_sec_lat=1.0/cos(G_Pos_I[1]);
	l_sec_lat2 = l_sec_lat*l_sec_lat;
	//位置误差方程
	g_dzpg_kf_matr.FF[0*15+2] = -G_Vel_I[1]*G_Rec_RE*G_Rec_RE;
	g_dzpg_kf_matr.FF[0*15+4] = G_Rec_RE;
	
	g_dzpg_kf_matr.FF[1*15+0] = G_Vel_I[0]*l_tan_lat*l_sec_lat*G_Rec_RE;
	g_dzpg_kf_matr.FF[1*15+2] =-G_Vel_I[0]*G_Rec_RE*G_Rec_RE*l_sec_lat;
	g_dzpg_kf_matr.FF[1*15+3] = G_Rec_RE*l_sec_lat;
	
	g_dzpg_kf_matr.FF[2*15+5] = 1.0;

	//速度误差方程
	g_dzpg_kf_matr.FF[3*15+0] = G_Vel_I[1]*(G_Vel_I[0]*G_Rec_RE*l_sec_lat*l_sec_lat+2*G_Wien[1])+2*G_Wien[2]*G_Vel_I[2];
	g_dzpg_kf_matr.FF[3*15+2] = G_Vel_I[0]*G_Vel_I[2]* G_Rec_RE* G_Rec_RE-G_Vel_I[0]*G_Vel_I[1]*l_tan_lat*G_Rec_RE*G_Rec_RE;
	g_dzpg_kf_matr.FF[3*15+3] = G_Vel_I[1]*l_tan_lat*G_Rec_RE-G_Vel_I[2]*G_Rec_RE;
    g_dzpg_kf_matr.FF[3*15+4] = G_Vel_I[0]*l_tan_lat*G_Rec_RE+2*G_Wien[2];
	g_dzpg_kf_matr.FF[3*15+5] = -G_Vel_I[0]*G_Rec_RE-2*G_Wien[1];
    g_dzpg_kf_matr.FF[3*15+7] = -g_kf_para1.acc_u;
	g_dzpg_kf_matr.FF[3*15+8] = g_kf_para1.acc_n;
    g_dzpg_kf_matr.FF[3*15+12]= G_Cbn[0];
    g_dzpg_kf_matr.FF[3*15+13]= G_Cbn[1];
    g_dzpg_kf_matr.FF[3*15+14]= G_Cbn[2];

	g_dzpg_kf_matr.FF[4*15+0] = -(G_Vel_I[0]*l_sec_lat)*(G_Vel_I[0]*l_sec_lat)*G_Rec_RE-G_Vel_I[0]*2*G_Wien[1];
	g_dzpg_kf_matr.FF[4*15+2] = G_Vel_I[0]*G_Vel_I[0]*l_tan_lat*G_Rec_RE*G_Rec_RE+G_Vel_I[1]*G_Vel_I[2]*G_Rec_RN*G_Rec_RN;
	g_dzpg_kf_matr.FF[4*15+3] = -2*G_Vel_I[0]*l_tan_lat*G_Rec_RE-2*G_Wien[2];
	g_dzpg_kf_matr.FF[4*15+4] = -G_Vel_I[2]*G_Rec_RN;
	g_dzpg_kf_matr.FF[4*15+5] = -G_Vel_I[1]*G_Rec_RN; 
	g_dzpg_kf_matr.FF[4*15+6] = g_kf_para1.acc_u;
	g_dzpg_kf_matr.FF[4*15+8] = -g_kf_para1.acc_e;
    g_dzpg_kf_matr.FF[4*15+12]= G_Cbn[3];
    g_dzpg_kf_matr.FF[4*15+13]= G_Cbn[4];
    g_dzpg_kf_matr.FF[4*15+14]= G_Cbn[5];

	g_dzpg_kf_matr.FF[5*15+0] = -G_Vel_I[0]*2*G_Wien[2];
	g_dzpg_kf_matr.FF[5*15+2] = (G_Vel_I[1]*G_Rec_RN)*(G_Vel_I[1]*G_Rec_RN)+(G_Vel_I[0]*G_Rec_RE)*(G_Vel_I[0]*G_Rec_RE); 
	g_dzpg_kf_matr.FF[5*15+3] = 2*G_Vel_I[0]*G_Rec_RE+2*G_Wien[1];
	g_dzpg_kf_matr.FF[5*15+4] = 2*G_Vel_I[1]*G_Rec_RN;
	g_dzpg_kf_matr.FF[5*15+6] = -g_kf_para1.acc_n;
	g_dzpg_kf_matr.FF[5*15+7] = g_kf_para1.acc_e;
    g_dzpg_kf_matr.FF[5*15+12]= G_Cbn[6];
    g_dzpg_kf_matr.FF[5*15+13]= G_Cbn[7];
    g_dzpg_kf_matr.FF[5*15+14]= G_Cbn[8];

	//平台角误差方程
	g_dzpg_kf_matr.FF[6*15+2] = G_Vel_I[1]*G_Rec_RN*G_Rec_RN;
	g_dzpg_kf_matr.FF[6*15+4] = -G_Rec_RN; 
	g_dzpg_kf_matr.FF[6*15+7] = G_Wien[2] + G_Vel_I[0]*l_tan_lat*G_Rec_RE;
	g_dzpg_kf_matr.FF[6*15+8] = -G_Wien[1] - G_Vel_I[0]*G_Rec_RE;
    g_dzpg_kf_matr.FF[6*15+9] = G_Cbn[0];
	g_dzpg_kf_matr.FF[6*15+10] = G_Cbn[1];
	g_dzpg_kf_matr.FF[6*15+11] = G_Cbn[2];

	g_dzpg_kf_matr.FF[7*15+0] = -G_Wien[2];
	g_dzpg_kf_matr.FF[7*15+2] = -G_Vel_I[0]*G_Rec_RE*G_Rec_RE; 
	g_dzpg_kf_matr.FF[7*15+3] = G_Rec_RE;
	g_dzpg_kf_matr.FF[7*15+6] = -G_Wien[2] - G_Vel_I[0]*G_Rec_RE*l_tan_lat;
    g_dzpg_kf_matr.FF[7*15+8] = -G_Vel_I[1]*G_Rec_RN;
	g_dzpg_kf_matr.FF[7*15+9] = G_Cbn[3];
	g_dzpg_kf_matr.FF[7*15+10] = G_Cbn[4];
	g_dzpg_kf_matr.FF[7*15+11] = G_Cbn[5];

	g_dzpg_kf_matr.FF[8*15+0] = G_Wien[1]+G_Vel_I[0]*G_Rec_RE*l_sec_lat2;
	g_dzpg_kf_matr.FF[8*15+2] = -G_Vel_I[0]*l_tan_lat*G_Rec_RE*G_Rec_RE; 
	g_dzpg_kf_matr.FF[8*15+3] = l_tan_lat*G_Rec_RE;
	g_dzpg_kf_matr.FF[8*15+6] = G_Wien[1] + G_Vel_I[0]*G_Rec_RE;
    g_dzpg_kf_matr.FF[8*15+7] = G_Vel_I[1]*G_Rec_RN;
	g_dzpg_kf_matr.FF[8*15+9] = G_Cbn[6];
	g_dzpg_kf_matr.FF[8*15+10] = G_Cbn[7];
	g_dzpg_kf_matr.FF[8*15+11] = G_Cbn[8];

} 
/************************************************************************/
/* Function:
	OffLine Navigation Program.
	卡尔曼滤波计算Q阵子程序                                             */
/************************************************************************/
void CalculateQk()
{

//	int i=0,j=0;

	//Qk = Gk*Q0*Gk';

	G_Gk[3*6+3] = G_Cbn[0]; G_Gk[3*6+4] = G_Cbn[1]; G_Gk[3*6+5] = G_Cbn[2];
    G_Gk[4*6+3] = G_Cbn[3]; G_Gk[4*6+4] = G_Cbn[4]; G_Gk[4*6+5] = G_Cbn[5];
	G_Gk[5*6+3] = G_Cbn[6]; G_Gk[5*6+4] = G_Cbn[7]; G_Gk[5*6+5] = G_Cbn[8];
    
	G_Gk[6*6+0] = G_Cbn[0]; G_Gk[6*6+1] = G_Cbn[1]; G_Gk[6*6+2] = G_Cbn[2];
    G_Gk[7*6+0] = G_Cbn[3]; G_Gk[7*6+1] = G_Cbn[4]; G_Gk[7*6+2] = G_Cbn[5];
	G_Gk[8*6+0] = G_Cbn[6]; G_Gk[8*6+1] = G_Cbn[7]; G_Gk[8*6+2] = G_Cbn[8]; 

	/*G_Gk[9*6+0] = 1; G_Gk[9*6+1] = 0; G_Gk[9*6+2] = 0;
    G_Gk[10*6+0] = 0; G_Gk[10*6+1] = 1; G_Gk[10*6+2] = 0;
	G_Gk[11*6+0] = 0; G_Gk[11*6+1] = 0; G_Gk[11*6+2] = 1;

	G_Gk[12*6+3] = 1; G_Gk[12*6+4] = 0; G_Gk[12*6+5] = 0;
    G_Gk[13*6+3] = 0; G_Gk[13*6+4] = 1; G_Gk[13*6+5] = 0;
	G_Gk[14*6+3] = 0; G_Gk[14*6+4] = 0; G_Gk[14*6+5] = 1;*/
	
	mulmats(G_Gk,g_dzpg_kf_matr.Q0,g_temp1_Qk,15,6,6);
	RotateMat(G_Gk,g_temp2_Qk,15,6);
	mulmats(g_temp1_Qk,g_temp2_Qk,g_dzpg_kf_matr.Qk,15,6,15);
	
/*
	G_Gk[3*15+12] = G_Cbn[0]; G_Gk[3*15+13] = G_Cbn[1]; G_Gk[3*15+14] = G_Cbn[2];
    G_Gk[4*15+12] = G_Cbn[3]; G_Gk[4*15+13] = G_Cbn[4]; G_Gk[4*15+14] = G_Cbn[5];
	G_Gk[5*15+12] = G_Cbn[15]; G_Gk[5*15+13] = G_Cbn[7]; G_Gk[5*15+14] = G_Cbn[8];
    
	G_Gk[6*15+9] = G_Cbn[0]; G_Gk[6*15+10] = G_Cbn[1]; G_Gk[6*15+11] = G_Cbn[2];
    G_Gk[7*15+9] = G_Cbn[3]; G_Gk[7*15+10] = G_Cbn[4]; G_Gk[7*15+11] = G_Cbn[5];
	G_Gk[8*15+9] = G_Cbn[6]; G_Gk[8*15+10] = G_Cbn[7]; G_Gk[8*15+11] = G_Cbn[8]; 

	mulmats(G_Gk,g_dzpg_kf_matr.Q0,g_temp1_Qk,15,15,15);
	RotateMat(G_Gk,g_temp2_Qk,15,15);
	mulmats(g_temp1_Qk,g_temp2_Qk,g_dzpg_kf_matr.Qk,15,15,15);
*/
	/*
	for (i=0;i<15;i++)
	{
		for(j=0;j<15;j++)
		{
			g_dzpg_kf_matr.Qk[i*15+j] = (g_dzpg_kf_matr.Q0[i*15+j] + 0.5*(g_Q0_Rot_FF[i*15+j] + g_FF_Q0[i*15+j]) );
		}
	}
	*/
	
}
/************************************************************************/
/* Function:
	OffLine Navigation Program.
	卡尔曼滤波计算FAI阵子程序                                           */
/************************************************************************/
void GetDisF() //DisFF = eye(15) + F*t;
{
	//离散化F阵，滤波时间间隔为1s
	int i=0,j=0;

	for(i=0;i<225;i++)
	{
		g_dzpg_kf_matr.DisFF[i] = g_dzpg_kf_matr.FF[i];
	}

	for(i=0;i<15;i++)
	{
		for(j=0;j<15;j++)
		{
			if(i==j)
				g_dzpg_kf_matr.DisFF[i*15+j]+=1.0;
		}
	}

}

/*以下五个为卡尔曼滤波的5个方程*/
void computeXkexp(void)
{
	//Xexp = DisFF * X0;  X1 = DisFF * X0;
	mulmats(g_dzpg_kf_matr.DisFF,g_dzpg_kf_matr.X0,g_dzpg_kf_matr.Xexp,15,15,1);
}	
	
void computePkexp(void)
{
	int i=0,j=0;
	//P1 = DisFF * P0 * DisFF'+ Qk;
	RotateMat(g_dzpg_kf_matr.DisFF,g_RotDisFF,15,15);		
	mulmats(g_dzpg_kf_matr.DisFF,g_dzpg_kf_matr.P1,g_DisFF_Pk,15,15,15);	
	mulmats(g_DisFF_Pk,g_RotDisFF,g_DisFF_PK_RotDisFF,15,15,15);	
	for(i=0;i<15;i++)
	{
		for(j=0;j<15;j++)
			g_temp_Pkk1[i*15+j]=g_DisFF_PK_RotDisFF[i*15+j]+g_dzpg_kf_matr.Qk[i*15+j];
	}	
}


void computeKk(void)
{
	//Kk = P1 * Hk' * inv(Hk * P1 * Hk' + Rk);
	int i=0;
	mulmats(g_dzpg_kf_matr.Hk,g_temp_Pkk1,g_temp_Mat_HkPk_k1,7,15,15);
	RotateMat(g_dzpg_kf_matr.Hk,g_RotHk,7,15);
	mulmats(g_temp_Mat_HkPk_k1,g_RotHk,g_temp_Mat_HkPk_k1_RotHk,7,15,7);

	for(i=0;i<49;i++)
		g_temp_sum_HkS_Rk[i] = g_temp_Mat_HkPk_k1_RotHk[i] + g_dzpg_kf_matr.Rk[i];
	
	brinv2(g_temp_sum_HkS_Rk,7);
	//brinv(g_temp_sum_HkS_Rk,7);
	mulmats(g_temp_Pkk1,g_RotHk,g_temp_Pkk1_RotHk,15,15,7);
	mulmats(g_temp_Pkk1_RotHk,g_temp_sum_HkS_Rk,Kk,15,7,7);
	
}
			
void computePk(void)
{
	int i=0,j=0;
	//P0 = (eye(15) - Kk*Hk) * P1 * (eye(15) - Kk*Hk)' + Kk * Rk * Kk';
	mulmats(Kk,g_dzpg_kf_matr.Hk,g_temp_KH,15,7,15);
	for (i=0;i<15;i++)
	{
		for (j=0;j<15;j++)
		{
			g_temp_sub_I_KH[i*15+j] = -g_temp_KH[i*15+j];
			if (i==j)
			{
				g_temp_sub_I_KH[i*15+j] += 1;
			}
		}
	}
	RotateMat(g_temp_sub_I_KH,g_Rot_temp_sub_I_KH,15,15);
	mulmats(g_temp_sub_I_KH,g_temp_Pkk1,g_temp_Pkk2,15,15,15);
	mulmats(g_temp_Pkk2,g_Rot_temp_sub_I_KH,g_temp_Pkk1,15,15,15);
	RotateMat(Kk,g_Rot_Kk,15,7);
	mulmats(Kk,g_dzpg_kf_matr.Rk,g_temp_Kk_Hk1,15,7,7);
	mulmats(g_temp_Kk_Hk1,g_Rot_Kk,g_temp_Kk_Hk,15,7,15);
	for (i=0;i<15*15;i++)
	g_dzpg_kf_matr.P1[i] = g_temp_Pkk1[i] + g_temp_Kk_Hk[i];
	
}
void computeXk(void)
{
	int i=0;
	//X1 = Xexp + Kk * (Zk-Hk*Xexp);
	mulmats(g_dzpg_kf_matr.Hk,g_dzpg_kf_matr.Xexp,g_Hk_Xexp,7,15,1);//Hk*Xexp	
	for(i=0;i<7;i++)
		g_dzpg_kf_matr.Zk[i]-=g_Hk_Xexp[i];
	mulmats(Kk,g_dzpg_kf_matr.Zk,g_dzpg_kf_matr.X1,15,7,1);	
	for(i=0;i<15;i++)
		g_dzpg_kf_matr.X1[i]+=g_dzpg_kf_matr.Xexp[i];	
	for (i=0;i<15;i++)
	{	
	    g_dzpg_kf_matr.X0[i]=g_dzpg_kf_matr.X1[i];
	}

}	


/************************************************************************/
/* Function:
	OffLine Navigation Program.
	卡尔曼滤波后的校准                                                      */
/************************************************************************/
void AdjustNav(void)
{
	int i=0;

	Modify_Cbn(g_dzpg_kf_matr.X1[6],g_dzpg_kf_matr.X1[7],g_dzpg_kf_matr.X1[8]);	//

	
	G_Pos_I[0]=G_Pos_I[0]-g_dzpg_kf_matr.X1[0];//经度
	G_Pos_I[1]=G_Pos_I[1]-g_dzpg_kf_matr.X1[1];//纬度
  G_Pos_I[2]=G_Pos_I[2]-g_dzpg_kf_matr.X1[2];//高度

	G_Vel_I[0]=G_Vel_I[0]-g_dzpg_kf_matr.X1[3];//速度
	G_Vel_I[1]=G_Vel_I[1]-g_dzpg_kf_matr.X1[4];
	G_Vel_I[2]=G_Vel_I[2]-g_dzpg_kf_matr.X1[5];
	
/*	for(i=0;i<3;i++)                                //修正陀螺零偏和加速度计零偏
	{
		GyroBias[i] = g_dzpg_kf_matr.X1[i+9];
		AccBias[i] = g_dzpg_kf_matr.X1[i+12];
	}*/

	//ComputeRg();待考虑是否需要更新R

    for(i=0;i<9;i++)  
		g_dzpg_kf_matr.X0[i]=0.0;//g_dzpg_kf_matr.X0[i]/4.0;// 修正过的状态变量置0,未修正的不置零
}
/************************************************************************/
/* Function:
	姿态误差修正子程序                                         */
/************************************************************************/

void Modify_Cbn(double l_faie,double l_fain,double l_faiu)
{
	int i=0,j=0;
	double l_mat[3*3],l_temp[3*3];
	double l_cos0=0.0,l_cos1=0,l_cos2=0.0;
	double l_sin0=0.0,l_sin1=0,l_sin2=0.0;

	l_faie = l_faie; //注意此处符号
	l_fain = l_fain; 
	l_faiu = l_faiu;

	l_mat[0*3+0]=1.0;		l_mat[0*3+1]=-l_faiu; 	l_mat[0*3+2]=l_fain;
	l_mat[1*3+0]=l_faiu;	l_mat[1*3+1]=1.0;	    l_mat[1*3+2]=-l_faie;
	l_mat[2*3+0]=-l_fain;	l_mat[2*3+1]=l_faie;    l_mat[2*3+2]=1.0;

	mulmats(l_mat,G_Cbn,l_temp,3,3,3);
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			G_Cbn[i*3+j] = l_temp[i*3+j];
		}

	//gram_sch(G_Cbn,3);
		
	UpdateAttitude_Cbn();
	
	l_cos0=cos(G_Att[0]*0.5);	l_cos1=cos(G_Att[1]*0.5);	l_cos2=cos(G_Att[2]*0.5);
	l_sin0=sin(G_Att[0]*0.5);	l_sin1=sin(G_Att[1]*0.5);	l_sin2=sin(G_Att[2]*0.5);

	G_Q[0] = l_cos2*l_cos0*l_cos1 + l_sin2*l_sin0*l_sin1;  
	G_Q[1] = l_cos2*l_sin0*l_cos1 + l_sin2*l_cos0*l_sin1;
	G_Q[2] = l_cos2*l_cos0*l_sin1 - l_sin2*l_sin0*l_cos1;
	G_Q[3] = l_cos2*l_sin0*l_sin1 - l_sin2*l_cos0*l_cos1;
}
/************************************************************************/
/* Function:
	四元数相乘子程序                                         */
/************************************************************************/
/*void Q_Multiply(void)
{
	int i=0;
	double G_Q_New[4] = {0.0};

	G_Q_New[0] = G_Q[0]*Q_err[0] -G_Q[1]*Q_err[1]-G_Q[2]*Q_err[2]-G_Q[3]*Q_err[3];
    G_Q_New[1] = G_Q[0]*Q_err[1]+G_Q[1]*Q_err[0]+G_Q[2]*Q_err[3]-G_Q[3]*Q_err[2];
    G_Q_New[2] = G_Q[0]*Q_err[2]-G_Q[1]*Q_err[3]+G_Q[2]*Q_err[0]+G_Q[3]*Q_err[1];
	G_Q_New[3] = G_Q[0]*Q_err[3]+G_Q[3]*Q_err[0]-G_Q[2]*Q_err[1]+G_Q[1]*Q_err[2];

	for (i=0;i<4;i++)	
		G_Q[i] = G_Q_New[i];
}*/

void FastCali(int Calibration_Count)
{
	int i=0;
	
	Count_Cali++;
    
	if (Count_Cali<=Calibration_Count)
	{
		G_W_ibb_m[0] += G_W_ibb[0];
		G_W_ibb_m[1] += G_W_ibb[1];
		G_W_ibb_m[2] += G_W_ibb[2];
	}
	
	if (Count_Cali>=Calibration_Count)
	{
	
		for(i=0;i<3;i++)
		{
			Gyro_Bias[i] = G_W_ibb_m[i]/(Calibration_Count);
		}
		FastCali_Flag = 1;
	}
}
/************************************************************************/
/* Function:
	快速对准子程序                                         */
/************************************************************************/
void FastAlign(int Align_Count)
{
	double AlignTemp = 0.0;
  double AlignTemp2 = 0.0;
	double l_Q00,l_Q11,l_Q22,l_Q33,l_Q01,l_Q02,l_Q03,l_Q12,l_Q13,l_Q23;
	double sum = 0.0;
	
	int i=0;
	
	
	Count_CA++;
    
	//初始位置
  G_Pos_I[0] = G_GPS_Longitude;
	G_Pos_I[1] = G_GPS_Latitude;
  G_Pos_I[2] = G_GPS_Height;

	
	//初始速度
  G_Vel_I[0] = G_GPS_Ve;
	G_Vel_I[1] = G_GPS_Vn;
  G_Vel_I[2] = G_GPS_Vu;
	
	if (Count_CA<=Align_Count)
	{
		G_A_ibb_m[0] += G_A_ibb[0];
		G_A_ibb_m[1] += G_A_ibb[1];
		G_A_ibb_m[2] += G_A_ibb[2];
		
		G_M_ebb_m[0] += G_M_ebb[0];
		G_M_ebb_m[1] += G_M_ebb[1];
		G_M_ebb_m[2] += G_M_ebb[2];
	}
	
	if (Count_CA>=Align_Count)
	{
		for(i=0;i<3;i++)
		{
			G_A_ibb_m[i] = G_A_ibb_m[i]/(Count_CA);
			G_M_ebb_m[i] = G_M_ebb_m[i]/(Count_CA);
		}
		
		//初始俯仰角
		G_Att[0] = asin(G_A_ibb_m[1]/G_g);                               //G_Att[0] = asin(-G_A_ibb[0]/G_g);//0517
		//初始横滚角
		G_Att[1] = -asin(G_A_ibb_m[0]/G_g/cos(G_Att[0]));               // G_Att[1] = -asin(-G_A_ibb[1]/G_g/cos(G_Att[0]));
		
		//初始航向角,可由磁强计计算得到，目前采用磁航向进行初始化
		AlignTemp = G_M_ebb_m[0]*cos(G_Att[0]) + G_M_ebb_m[1]*sin(G_Att[0])*sin(G_Att[1]) - G_M_ebb_m[2]*sin(G_Att[0])*cos(G_Att[1]);
		AlignTemp2 = G_M_ebb_m[1]*cos(G_Att[1]) - G_M_ebb_m[2]*sin(G_Att[1]);
		G_Att[2] = atan2(AlignTemp2,AlignTemp);
		

			//根据初始姿态确定初始四元数及Cbn
		G_Q[0] = cos(G_Att[2]/2)*cos(G_Att[0]/2)*cos(G_Att[1]/2) + sin(G_Att[2]/2)*sin(G_Att[0]/2)*sin(G_Att[1]/2);
		G_Q[1] = cos(G_Att[2]/2)*sin(G_Att[0]/2)*cos(G_Att[1]/2) + sin(G_Att[2]/2)*cos(G_Att[0]/2)*sin(G_Att[1]/2);
		G_Q[2] = cos(G_Att[2]/2)*cos(G_Att[0]/2)*sin(G_Att[1]/2) - sin(G_Att[2]/2)*sin(G_Att[0]/2)*cos(G_Att[1]/2);
		G_Q[3] = cos(G_Att[2]/2)*sin(G_Att[0]/2)*sin(G_Att[1]/2) - sin(G_Att[2]/2)*cos(G_Att[0]/2)*cos(G_Att[1]/2);
		
		sum=sqrt(G_Q[0]*G_Q[0]+G_Q[1]*G_Q[1]+G_Q[2]*G_Q[2]+G_Q[3]*G_Q[3]);
	  for(i=0;i<4;i++)	
		G_Q[i]=G_Q[i]/sum;

		l_Q00 = G_Q[0]*G_Q[0];	l_Q11 = G_Q[1]*G_Q[1];	l_Q22 = G_Q[2]*G_Q[2];	l_Q33 = G_Q[3]*G_Q[3];
		l_Q12 = 2.0*G_Q[1]*G_Q[2];	l_Q03 = 2.0*G_Q[0]*G_Q[3];
		l_Q13 = 2.0*G_Q[1]*G_Q[3];	l_Q02 = 2.0*G_Q[0]*G_Q[2];
		l_Q23 = 2.0*G_Q[2]*G_Q[3];	l_Q01 = 2.0*G_Q[0]*G_Q[1];
				
		G_Cbn[0*3+0] = l_Q00 + l_Q11 - l_Q22 - l_Q33;  
		G_Cbn[0*3+1] = l_Q12 - l_Q03;
		G_Cbn[0*3+2] = l_Q13 + l_Q02;
		G_Cbn[1*3+0] = l_Q12 + l_Q03;
		G_Cbn[1*3+1] = l_Q00 - l_Q11 + l_Q22 - l_Q33;
		G_Cbn[1*3+2] = l_Q23 - l_Q01;
		G_Cbn[2*3+0] = l_Q13 - l_Q02;
		G_Cbn[2*3+1] = l_Q23 + l_Q01;
		G_Cbn[2*3+2] = l_Q00 - l_Q11 - l_Q22 + l_Q33;
		//gram_sch(G_Cbn,3);
		
		FastAlign_Flag = 1;
		
		Nav_cnt = 0;
	}
}

void WaveletDenoi_GX(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		GyroX_DWT_Org[i] = GyroX_DWT_Org[i+1];

	GyroX_DWT_Org[Len_DWT-1] = G_W_ibb[0];

	for(j=0;j<Len_DWT;j++)
			GyroX_DWT_Decomposition[j]=GyroX_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(GyroX_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=GyroX_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			GyroX_DWT_Decomposition[i]=shrinkage_soft(GyroX_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		GyroX_DWT_Reconstruction[i] = GyroX_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			GyroX_DWT_RecTmp[i] = GyroX_DWT_Reconstruction[i];

		recs(GyroX_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    GyroX_DWT_Reconstruction[i] = GyroX_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}

	free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\n",GyroX_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_GY(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		GyroY_DWT_Org[i] = GyroY_DWT_Org[i+1];

	GyroY_DWT_Org[Len_DWT-1] = G_W_ibb[1];

	for(j=0;j<Len_DWT;j++)
			GyroY_DWT_Decomposition[j]=GyroY_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(GyroY_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=GyroY_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			GyroY_DWT_Decomposition[i]=shrinkage_soft(GyroY_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		GyroY_DWT_Reconstruction[i] = GyroY_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			GyroY_DWT_RecTmp[i] = GyroY_DWT_Reconstruction[i];

		recs(GyroY_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    GyroY_DWT_Reconstruction[i] = GyroY_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}
    
	free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\n",GyroX_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_GZ(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		GyroZ_DWT_Org[i] = GyroZ_DWT_Org[i+1];

	GyroZ_DWT_Org[Len_DWT-1] = G_W_ibb[2];

	for(j=0;j<Len_DWT;j++)
			GyroZ_DWT_Decomposition[j]=GyroZ_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(GyroZ_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=GyroZ_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			GyroZ_DWT_Decomposition[i]=shrinkage_soft(GyroZ_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		GyroZ_DWT_Reconstruction[i] = GyroZ_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			GyroZ_DWT_RecTmp[i] = GyroZ_DWT_Reconstruction[i];

		recs(GyroZ_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    GyroZ_DWT_Reconstruction[i] = GyroZ_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}

    free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\t%10e\t%10e\t\n",GyroX_DWT_Reconstruction[Len_DWT-1],
	//	GyroY_DWT_Reconstruction[Len_DWT-1],GyroZ_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_AX(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		AccX_DWT_Org[i] = AccX_DWT_Org[i+1];

	AccX_DWT_Org[Len_DWT-1] = G_A_ibb[0];

	for(j=0;j<Len_DWT;j++)
			AccX_DWT_Decomposition[j]=AccX_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(AccX_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=AccX_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			AccX_DWT_Decomposition[i]=shrinkage_soft(AccX_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		AccX_DWT_Reconstruction[i] = AccX_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			AccX_DWT_RecTmp[i] = AccX_DWT_Reconstruction[i];

		recs(AccX_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    AccX_DWT_Reconstruction[i] = AccX_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}

	free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\n",AccX_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_AY(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		AccY_DWT_Org[i] = AccY_DWT_Org[i+1];

	AccY_DWT_Org[Len_DWT-1] = G_A_ibb[1];

	for(j=0;j<Len_DWT;j++)
			AccY_DWT_Decomposition[j]=AccY_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(AccY_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=AccY_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			AccY_DWT_Decomposition[i]=shrinkage_soft(AccY_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		AccY_DWT_Reconstruction[i] = AccY_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			AccY_DWT_RecTmp[i] = AccY_DWT_Reconstruction[i];

		recs(AccY_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    AccY_DWT_Reconstruction[i] = AccY_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}
    
	free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\n",AccX_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_AZ(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		AccZ_DWT_Org[i] = AccZ_DWT_Org[i+1];

	AccZ_DWT_Org[Len_DWT-1] = G_A_ibb[2];

	for(j=0;j<Len_DWT;j++)
			AccZ_DWT_Decomposition[j]=AccZ_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(AccZ_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=AccZ_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			AccZ_DWT_Decomposition[i]=shrinkage_soft(AccZ_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		AccZ_DWT_Reconstruction[i] = AccZ_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			AccZ_DWT_RecTmp[i] = AccZ_DWT_Reconstruction[i];

		recs(AccZ_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    AccZ_DWT_Reconstruction[i] = AccZ_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}

    free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\t%10e\t%10e\t\n",AccX_DWT_Reconstruction[Len_DWT-1],
	//	AccY_DWT_Reconstruction[Len_DWT-1],AccZ_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_MX(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		MagX_DWT_Org[i] = MagX_DWT_Org[i+1];

	MagX_DWT_Org[Len_DWT-1] = G_M_ebb[0];

	for(j=0;j<Len_DWT;j++)
			MagX_DWT_Decomposition[j]=MagX_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(MagX_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=MagX_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			MagX_DWT_Decomposition[i]=shrinkage_soft(MagX_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		MagX_DWT_Reconstruction[i] = MagX_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			MagX_DWT_RecTmp[i] = MagX_DWT_Reconstruction[i];

		recs(MagX_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    MagX_DWT_Reconstruction[i] = MagX_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}

	free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\n",MagX_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_MY(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		MagY_DWT_Org[i] = MagY_DWT_Org[i+1];

	MagY_DWT_Org[Len_DWT-1] = G_M_ebb[1];

	for(j=0;j<Len_DWT;j++)
			MagY_DWT_Decomposition[j]=MagY_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(MagY_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=MagY_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			MagY_DWT_Decomposition[i]=shrinkage_soft(MagY_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		MagY_DWT_Reconstruction[i] = MagY_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			MagY_DWT_RecTmp[i] = MagY_DWT_Reconstruction[i];

		recs(MagY_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    MagY_DWT_Reconstruction[i] = MagY_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}
    
	free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	//fprintf(fp_DWT_out,"%10e\n",MagX_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void WaveletDenoi_MZ(void)
{
	int i,j;

	recon_signal = (double *) calloc(Len_DWT,sizeof(double));
	decom_low= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_hig= (double *) calloc(Len_DWT+LEN_F-1,sizeof(double));
	decom_low_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	decom_hig_down= (double *) calloc((Len_DWT+LEN_F-1)/2,sizeof(double));
	recon_low_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_hig_up= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1,sizeof(double));
	recon_low= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_hig= (double *) calloc((Len_DWT+LEN_F-1)/2*2+1+LEN_F-1,sizeof(double));
	recon_low_fix= (double *) calloc(Len_DWT,sizeof(double));
	recon_hig_fix= (double *) calloc(Len_DWT,sizeof(double));
    Median_temp = (double *) calloc(Len_DWT/2,sizeof(double));

	for(i=0;i<Len_DWT-1;i++) //更新数据
		MagZ_DWT_Org[i] = MagZ_DWT_Org[i+1];

	MagZ_DWT_Org[Len_DWT-1] = G_M_ebb[2];

	for(j=0;j<Len_DWT;j++)
			MagZ_DWT_Decomposition[j]=MagZ_DWT_Org[j];

	//分解
	Len_DWT_dcp=Len_DWT;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)//逐级分解信号
	{
		dcp(MagZ_DWT_Decomposition,Len_DWT_dcp);//N个数据的小波分解
		Len_DWT_dcp/=2;//每级分解完成后数据循环长度减半
	}
   
	//去噪
	Len_DWT_Denoi = Len_DWT/2;

	for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{	
		double sigma;

	    if (dcp_number==1)
		{
			for(i=Len_DWT_Denoi;i<Len_DWT;i++)
			{
				Median_temp[i-Len_DWT_Denoi]=MagZ_DWT_Decomposition[i];
				if(Median_temp[i-Len_DWT_Denoi]<0)
					Median_temp[i-Len_DWT_Denoi] = -Median_temp[i-Len_DWT_Denoi];
			//将小波分解后的一级高频系数绝对值放到Median_temp数组中，待排序
			}
			bubblesort(Median_temp,Len_DWT_Denoi);
		    sigma = Median_temp[Len_DWT_Denoi/2]/0.6745;
			if(sigma<0.01 || sigma>10) //给sigma限幅
				lemda = 2.884;
			else
				lemda = sigma*sqrt(2*log(Len_DWT));//阈值，lemda=sigma*sqrt(2*log(Len_DWT)),Len_DWT为信号长度	
			
            //lemda = 2.884;
		}
		for(i=Len_DWT_Denoi;i<(2*Len_DWT_Denoi);i++)
			MagZ_DWT_Decomposition[i]=shrinkage_soft(MagZ_DWT_Decomposition[i],lemda);
		//对小波分解后的数据进行“软阈值”处理，只对高频系数进行阈值处理，不对低频系数进行处理

		Len_DWT_Denoi/=2;
	}

	//重建
    Len_DWT_Recon = Len_DWT_dcp*2;

    for(i=0;i<Len_DWT;i++)
		MagZ_DWT_Reconstruction[i] = MagZ_DWT_Decomposition[i];

    for(dcp_number=1;dcp_number<=dcp_class;dcp_number++)
	{
		for(i=0;i<Len_DWT_Recon;i++)
			MagZ_DWT_RecTmp[i] = MagZ_DWT_Reconstruction[i];

		recs(MagZ_DWT_RecTmp,Len_DWT_Recon);

		for(i=0;i<Len_DWT_Recon;i++)
		    MagZ_DWT_Reconstruction[i] = MagZ_DWT_RecTmp[i];

		Len_DWT_Recon*=2;
	}

    free(recon_signal);
	free(decom_low);	free(decom_hig);
	free(decom_low_down);free(decom_hig_down);
	free(recon_low_up);	free(recon_hig_up);
	free(recon_low);	free(recon_hig);
	free(recon_low_fix);free(recon_hig_fix);
    free(Median_temp);
	fprintf(fp_DWT_out,"%10e\t%10e\t%10e\t%10e\t%10e\t%10e\t%10e\t%10e\t%10e\t\n",GyroX_DWT_Reconstruction[Len_DWT-1],
		GyroY_DWT_Reconstruction[Len_DWT-1],GyroZ_DWT_Reconstruction[Len_DWT-1],AccX_DWT_Reconstruction[Len_DWT-1],
		AccY_DWT_Reconstruction[Len_DWT-1],AccZ_DWT_Reconstruction[Len_DWT-1],MagX_DWT_Reconstruction[Len_DWT-1],
		MagY_DWT_Reconstruction[Len_DWT-1],MagZ_DWT_Reconstruction[Len_DWT-1]);//输出重构数据
}

void dcp(double *start_position_dcp,int data_lenth_dcp)
{

	int dcp_i = 0;
	conv(h0,LEN_F,start_position_dcp,data_lenth_dcp,decom_low);//卷积
	conv(h1,LEN_F,start_position_dcp,data_lenth_dcp,decom_hig);
	dyaddown(decom_low,data_lenth_dcp,decom_low_down);//下抽样只保留不重复部分
	dyaddown(decom_hig,data_lenth_dcp,decom_hig_down);
	for(dcp_i=0;dcp_i<data_lenth_dcp/2;dcp_i++)
	{
		*(start_position_dcp+dcp_i)=decom_low_down[dcp_i];
		*(start_position_dcp+dcp_i+data_lenth_dcp/2)=decom_hig_down[dcp_i];
	}
}
void recs(double *start_position_recs,int data_lenth_recs)
{
    int recs_i = 0,LEN_K = 0;
    LEN_K=(data_lenth_recs+LEN_F-1)/2*2+1;

	for(recs_i=0;recs_i<data_lenth_recs/2;recs_i++)
	{
	    decom_low_down[recs_i] = *(start_position_recs+recs_i);
		decom_hig_down[recs_i] = *(start_position_recs+recs_i+data_lenth_recs/2);
	}

	dyadup(decom_low_down,data_lenth_recs/2,recon_low_up);
	dyadup(decom_hig_down,data_lenth_recs/2,recon_hig_up);
	conv(g0,LEN_F,recon_low_up,data_lenth_recs,recon_low);
	conv(g1,LEN_F,recon_hig_up,data_lenth_recs,recon_hig);
	wkeep(recon_low,LEN_K+LEN_F-1,data_lenth_recs,recon_low_fix); 
	wkeep(recon_hig,LEN_K+LEN_F-1,data_lenth_recs,recon_hig_fix); 
	vectoradd(recon_low_fix,recon_hig_fix,data_lenth_recs,recon_signal);
	for(recs_i=0;recs_i<data_lenth_recs;recs_i++)
		(*(start_position_recs+recs_i))=recon_signal[recs_i];

}

double shrinkage_soft(double input_shrinkage_soft,double input_shrinkage_soft_lemda)
{
	double sgn_shrinkage_soft,output_shrinkage_soft;
	if(input_shrinkage_soft>0)
		sgn_shrinkage_soft=1;
	else sgn_shrinkage_soft=-1;
	if(fabs(input_shrinkage_soft)>input_shrinkage_soft_lemda)
		output_shrinkage_soft=sgn_shrinkage_soft*(fabs(input_shrinkage_soft)-input_shrinkage_soft_lemda);
	else
		output_shrinkage_soft=0;
	return output_shrinkage_soft;
}

void db4(double *h0,double *h1,double *g0,double *g1)
/* Daubechies 4 wavelet */
{
int k,sign;
h0[0]= 0.2303778133088964;
h0[1]= 0.7148465705529154;
h0[2]= 0.6308807679398597;
h0[3]=-0.0279837694168599;
h0[4]=-0.1870348117190931;
h0[5]= 0.0308413818355607;
h0[6]= 0.0328830116668852;
h0[7]=-0.0105974017850890;
sign=1;
for(k=0;k<8;k++) {
	h1[k]=sign*h0[7-k];
	g0[k]=h0[7-k];
	g1[k]=-sign*h0[k];
	sign=-sign;
}
return;
}

void conv(double *a,int len_a,double *b,int len_b,double *result){
	int i,j,len,lenbb;
	double v;
	double *bb;

	len=len_b+len_a-1;
	lenbb = len_b+2*(len_a-1);
	bb=(double *) calloc(lenbb,sizeof(double));
	//bb=(double *) calloc(len,sizeof(double));
	//for(i=0; i<len_a-1; i++) bb[i]=0;
	//for(i=0; i<len_b; i++)   bb[i+len_a-1]=b[i];//补零延拓
	for(i=0; i<len_a-1; i++) bb[i]=b[len_b-len_a+1+i];
	for(i=0; i<len_b; i++)   bb[i+len_a-1]=b[i];
	for(i=0; i<len_a-1; i++) bb[i+len_b+len_a-1]=b[i];//周期延拓

	for(i=0; i<len; i++){
		v=0;
		for(j=0; j<len_a; j++)
			//v=v+bb[(i+j)%len]*a[len_a-1-j];//补零延拓
			v=v+bb[i+j]*a[len_a-1-j];//周期延拓
		result[i]=v;
	}
	free(bb);
	return; 
}

void wkeep(double *a,int len_a,int len,double *result){
	int i,i0;
	i0=(len_a-len)/2;
	for(i=0; i<len; i++) result[i]=a[i+i0];
	return;
}

void dyaddown(double *a,int len_a,double *result){
	int i;
	for(i=1; i<len_a; i=i+2) result[i/2]=a[i];
	return;	
}

void dyadup(double *a,int len_a,double *result){
	int i;
	for(i=0; i<len_a; i++){
		result[2*i]=0;
		result[2*i+1]=a[i];
	}
	result[2*len_a]=0;
	return;	
}

void vectoradd(double *a,double *b,int len_a,double *result){
	int i;
	for(i=0; i<len_a; i++){
		result[i]=a[i]+b[i];
	}
	return;	
}

void vectordetract(double *a,double *b,int len_a,double *result){
	int i;
	for(i=0; i<len_a; i++){
		result[i]=a[i]-b[i];
	}
	return;	
}

void bubblesort(double *list,int len) //冒泡排序
{
    int i,j;
    double temp;
    for(i=0;i<len - 1;i++)
        for(j=0;j<len-i - 1;j++) 
        {
            if(list[j+1]<list[j])
            {
                temp=list[j+1];
                list[j+1]=list[j];
                list[j]=temp;
            }
        }
}
