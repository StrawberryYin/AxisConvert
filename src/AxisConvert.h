#pragma once
#pragma once

#include<stdlib.h>
#include<stdio.h>
#include<cmath>
#include <limits>
#include <vector>
#include <time.h>
#include <chrono>
//#include "Geometry2D.h"

#define M_PI (3.1415926)
#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)
#define CONSTANTS_RADIUS_OF_EARTH 6371000 // meters (m)
const static float PI = 3.1415926;
static const float epsilon = std::numeric_limits<float>::epsilon();


/*------------ʱ��ͳһ------------*/

static std::vector <int> timeUnitCountVec_;			//ʱͳ�������洢ÿ������Ӧ��ʱ��ͳһ��������

static int timeUnitCount_;								//ʱͳ����������վִ�е�ʱ��ͳһ��������

static std::vector <uint64_t> UTCTimeVec_;				//Э������ʱ���洢ÿ���ɿص�UTCʱ��

static std::vector <uint64_t> UTCTimeDiffVec_;			//Э������ʱ���洢ÿ���ɿ�UTCʱ�������վ�Ĳ�ֵ

static double stationUTCTime_;							//����վUTCʱ��


int calculateUTCTimeDiff(int size, uint64_t* UTCTime);

uint64_t  getCurrentUTCTime_ms();

//��������ϵ��WGS84 datum
struct GeoCoord
{
	double latitude;
	double longitude;
	float altitude;
	float timeStamp;
	float phi;			//20240223yym����ӳ������Ϣ��NED����ϵ�µĳ���нǣ�
};

//����������ϵ:�쵯�ķ����Ϊ���
struct NedCoord
{
	float x;
	float y;
	float z;
	float V;			//ˮƽ�ٶȴ�С
	float phi;			//ˮƽ�ٶȷ���
	float lambda;		//��ֱ�ٶȴ�С
	float nz;			//�������
	float timeStamp;
	GeoCoord centreGeo;	//ԭ��ľ�γ��
};

//��������ϵ���쵰�ķ����Ϊԭ�㣬���䷽��Ϊx��������y�ᳯ�ң�z�ᳯ�£��뱱��������ϵ��Ƚ���תһ���Ƕ�
struct ShootCoord
{
	float x;
	float y;
	float z;
	float V;			//ˮƽ�ٶȴ�С
	float phi;			//ˮƽ�ٶȷ���
	float lambda;		//��ֱ�ٶȴ�С
	float timeStamp;
	float theta;		//��ʼ���򣺷��䷽����NEDx��ļн�
};

/*
�������ϵ��
	ԭ��λ���쵯�����ģ������˶���
	��ǰΪx�ᣬ����Ϊy�ᣬ������z�ᣬ
	thetaΪ�쵥�ĺ���ǣ����쵥�����뱱��ļнǣ�ʵʱ�仯��
	�뱱��������ϵ��ȣ�����һ��ԭ���ʵʱλ�ƺ�x��ĳ���Ǳ仯
*/
struct RelatCoord
{
	float x;
	float y;
	float z;
	float V;			//ˮƽ�ٶȴ�С
	float phi;			//ˮƽ�ٶȷ���
	float lambda;		//��ֱ�ٶȴ�С
	float theta;		//�쵥�����뱱��ļнǣ�ʵʱ�仯��
	float timeStamp;
};


class AxisConvert
{
public:

	void convertGeoToNed(GeoCoord coord, GeoCoord origin, float* x, float* y, float* z);							//����ת������
	void convertNedToGeo(float x, float y, float z, GeoCoord origin, GeoCoord* coord);							//������ת����

	void convertNedToRel(NedCoord nedCoord , NedCoord nedOrigin, RelatCoord* relatCoord , float theta);			//������ת���
	void convertRelToNed(NedCoord nedOrigin, RelatCoord relatCoord, NedCoord* nedCoord);							//���ת������

	void convertGeoToRel(GeoCoord coord , GeoCoord origin , GeoCoord coordLead, RelatCoord* relatCoord, float theta);		//����ת���
	void convertRelToGeo(GeoCoord* coord, GeoCoord origin, NedCoord coordLead, RelatCoord relatCoord);			//���ת����

	void convertNedToSho(NedCoord nedCoord, ShootCoord* shootCoord, float theta);				//������ת����
	void convertShoToNed(ShootCoord shootCoord, NedCoord* nedCoord);							//����ת������

	void convertGeoToSho(GeoCoord coord, GeoCoord origin, ShootCoord* shootCoord, float theta);		//����ת����

	bool isGeoCoordValid(GeoCoord coord);

	void testAxisConvert01();

private:

};

