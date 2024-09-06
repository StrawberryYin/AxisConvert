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


/*------------时间统一------------*/

static std::vector <int> timeUnitCountVec_;			//时统计数：存储每个蛋响应的时间统一操作次数

static int timeUnitCount_;								//时统计数：地面站执行的时间统一操作次数

static std::vector <uint64_t> UTCTimeVec_;				//协调世界时：存储每个飞控的UTC时间

static std::vector <uint64_t> UTCTimeDiffVec_;			//协调世界时：存储每个飞控UTC时间与地面站的差值

static double stationUTCTime_;							//地面站UTC时间


int calculateUTCTimeDiff(int size, uint64_t* UTCTime);

uint64_t  getCurrentUTCTime_ms();

//地理坐标系：WGS84 datum
struct GeoCoord
{
	double latitude;
	double longitude;
	float altitude;
	float timeStamp;
	float phi;			//20240223yym：添加朝向角信息（NED坐标系下的朝向叫角）
};

//北东地坐标系:领弹的发射点为起点
struct NedCoord
{
	float x;
	float y;
	float z;
	float V;			//水平速度大小
	float phi;			//水平速度方向
	float lambda;		//垂直速度大小
	float nz;			//法向过载
	float timeStamp;
	GeoCoord centreGeo;	//原点的经纬高
};

//发射坐标系：领蛋的发射点为原点，发射方向为x轴正方向，y轴朝右，z轴朝下，与北东地坐标系相比仅旋转一个角度
struct ShootCoord
{
	float x;
	float y;
	float z;
	float V;			//水平速度大小
	float phi;			//水平速度方向
	float lambda;		//垂直速度大小
	float timeStamp;
	float theta;		//初始航向：发射方向与NEDx轴的夹角
};

/*
相对坐标系：
	原点位于领弹的质心，不断运动着
	向前为x轴，向右为y轴，向下是z轴，
	theta为领单的航向角，即领单航向与北向的夹角，实时变化的
	与北东地坐标系相比，仅有一个原点的实时位移和x轴的朝向角变化
*/
struct RelatCoord
{
	float x;
	float y;
	float z;
	float V;			//水平速度大小
	float phi;			//水平速度方向
	float lambda;		//垂直速度大小
	float theta;		//领单航向与北向的夹角，实时变化的
	float timeStamp;
};


class AxisConvert
{
public:

	void convertGeoToNed(GeoCoord coord, GeoCoord origin, float* x, float* y, float* z);							//地理转北东地
	void convertNedToGeo(float x, float y, float z, GeoCoord origin, GeoCoord* coord);							//北东地转地理

	void convertNedToRel(NedCoord nedCoord , NedCoord nedOrigin, RelatCoord* relatCoord , float theta);			//北东地转相对
	void convertRelToNed(NedCoord nedOrigin, RelatCoord relatCoord, NedCoord* nedCoord);							//相对转北东地

	void convertGeoToRel(GeoCoord coord , GeoCoord origin , GeoCoord coordLead, RelatCoord* relatCoord, float theta);		//地理转相对
	void convertRelToGeo(GeoCoord* coord, GeoCoord origin, NedCoord coordLead, RelatCoord relatCoord);			//相对转地理

	void convertNedToSho(NedCoord nedCoord, ShootCoord* shootCoord, float theta);				//北东地转发射
	void convertShoToNed(ShootCoord shootCoord, NedCoord* nedCoord);							//发射转北东地

	void convertGeoToSho(GeoCoord coord, GeoCoord origin, ShootCoord* shootCoord, float theta);		//地理转发射

	bool isGeoCoordValid(GeoCoord coord);

	void testAxisConvert01();

private:

};

