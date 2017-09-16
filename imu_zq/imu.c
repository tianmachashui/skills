#include "include.h"
#include <math.h>
#include "imu.h"


u8 imu_flag=1;

//extern nmea_msg gpsx;//1130�Ż��������

FILE *fp_datain,*fp_dataout,*fp_Nav_out,*fp_DWT_out;
extern int FastAlign_Flag;
extern int FastCali_Flag;
extern unsigned int G_cnt,G_nst,Nav_cnt;


extern double G_GPS_Longitude;
extern double G_GPS_Latitude;
extern double G_GPS_Height ;
extern double G_GPS_Ve,G_GPS_Vn,G_GPS_Vu;
extern double G_Mag_Heading;
extern double Heading_Mag;

extern double h0[8],h1[8],g0[8],g1[8];//С���˲���ϵ��
//extern usart_RX usart_rx;
extern int time_rtc;
extern 	char lati_long[20];

int G_Pos_I_DD[2] = {0};
double G_Pos_I_MM[2] = {0.0};
double G_Pos_DDMM[2] = {0.0};


//               ((double)bno_gyro[0],(double)bno_gyro[1],(double)bno_gyro[2],(double)bno_accel[0]...)
//void MahonyAHRSupdateIMU(double wwx, double wwy, double wwz, double aax, double aay, double aaz)
//0515�滻																																															
void MahonyAHRSupdateIMU(int16_t gyro_x,int16_t gyro_y,int16_t gyro_z,int16_t acc_x,int16_t acc_y,int16_t acc_z,int16_t magn_x,int16_t magn_y,int16_t magn_z,double longitude,double latitude,double gps_height,double gps_speed,double gps_speed_1,double gps_speed_2, int gpsx_utc_year, int gpsx_utc_month, int gpsx_utc_date, int gpsx_utc_hour, int gpsx_utc_min, int gpsx_utc_sec)
{ 	
	
  int i=0,int_decimal_lati=0,int_decimal_long=0,Int_Lati=0,Int_Long=0;//strleN;   //γ�Ⱦ���С�����֣�γ�Ⱦ�����������,���ݳ���
	int ascLEN=0,asdLEN=0,asdlen=0;
	int GPS_Availability = 0;

	char asc[10]={'0'},asc_long[10]={'0'},asc_Lati[10]={'0'},asc_Long[10]={'0'};
	char asd[10]={'0'},asd_long[10]={'0'},asd_Lati[10]={'0'},asd_Long[10]={'0'};

//************************************AHRS��������************************************************
	//db4(h0,h1,g0,g1);//�����˲����飨�ֽ�ĸ�ͨ��ͨ���ع��ĸ�ͨ��ͨ��
	
		G_cnt++;

		//0109
		G_W_ibb[0] = gyro_x/GYRO_Factor*deg_to_rad - Gyro_Bias[0];  //���ݼӼƴ�ǿ�����ݣ�δ�������Ƴ�����ƫ  Rad/s
		G_W_ibb[1] = gyro_y/GYRO_Factor*deg_to_rad - Gyro_Bias[1];
		G_W_ibb[2] = gyro_z/GYRO_Factor*deg_to_rad - Gyro_Bias[2]; 
		G_A_ibb[0] = acc_x /ACCEL_Factor*G_g;	    //�Ӽ����ݣ�δ�������Ƴ�����ƫ    m/s2
		G_A_ibb[1] = acc_y /ACCEL_Factor*G_g;
		G_A_ibb[2] = acc_z /ACCEL_Factor*G_g;
		G_M_ebb[0] = magn_x;						//��ǿ�����ݣ�δ�������Ƴ�����ƫ
		G_M_ebb[1] = magn_y;
		G_M_ebb[2] = magn_z;
		
	/*	
		G_GPS_Longitude  = 116.350125303669/180*pi;
		G_GPS_Latitude   = 39.98161143598447/180*pi;
		G_GPS_Height     = 80;
		G_GPS_Ve         = 0*1852/3600*sin(gpsx.Course_Over_Ground/180*pi);   //�ٶ�
		G_GPS_Vn         = 0*1852/3600*cos(gpsx.Course_Over_Ground/180*pi);
		G_GPS_Vu         = 0;
	*/
		G_Mag_Heading    = gpsx.Course_Over_Ground/180*pi;
		if(G_Mag_Heading>pi)
			G_Mag_Heading = G_Mag_Heading - 2*pi;
			
		if(!FastCali_Flag)
		{
      FastCali(Cali_time);
		}
		
		if(!FastAlign_Flag && FastCali_Flag==1 )
		{
      FastAlign(Align_time);
		}

    if (FastAlign_Flag)
		{
			
			Nav_cnt++;
			UpdateQCbn();   //��̬���£�Q��Cbn��  10ms����
			Navigation(); 	//�ٶ�λ�ø���
			
		}
		
		if (Nav_cnt >=100 && FastAlign_Flag==1)
		{			
			IMUGPS_Kalman(); //��ϵ���
			Nav_cnt = 0;
		}
				
	
}
	






void imu_process(void)//��������
{
	
//	double aaa=0;
//	double bbb=0;
	//1228����ע��1�� -------> MahonyAHRSupdateIMU();
//	MahonyAHRSupdateIMU((double)bno_gyro[0],(double)bno_gyro[1],(double)bno_gyro[2],(double)bno_accel[0],(double)bno_accel[1],(double)bno_accel[2]);

	//��Ҫ��14����������  MahonyAHRSupdateIMU()  ����
//	MahonyAHRSupdateIMU(bno_gyro[0],bno_gyro[1],bno_gyro[2],bno_accel[0],bno_accel[1],bno_accel[2],bno_mag[0],bno_mag[1],bno_mag[2],p_gps_location->longitude,p_gps_location->latitude,aaa,bbb,p_gps_location->speed);
//	MahonyAHRSupdateIMU(bno_gyro[0],bno_gyro[1],bno_gyro[2],bno_accel[0],bno_accel[1],bno_accel[2],bno_mag[0],bno_mag[1],bno_mag[2],0.0,0.0,aaa,bbb,0.0);

	
//	count++;
//	printf("%0.6f\r\n",ten_us_tim/100000.0);//���뿪��ʱ��
	

}

