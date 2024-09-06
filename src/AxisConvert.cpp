#include "AxisConvert.h"
#include "pch.h"


int calculateUTCTimeDiff(int size, uint64_t* UTCTime)
{
    UTCTimeVec_.clear();
    UTCTimeDiffVec_.clear();

    uint64_t currentTime = getCurrentUTCTime_ms();
    for (int i = 0; i < size; i++) {

        UTCTimeVec_.push_back(UTCTime[i]);
        uint16_t timeDiff = currentTime - UTCTime[i];
        UTCTimeDiffVec_.push_back(timeDiff);

    }

    return 0;
}


uint64_t  getCurrentUTCTime_ms()
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();

    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);

    return now_ms.time_since_epoch().count();

    //std::printf("now_ms:%lld\n ", now_ms.time_since_epoch().count() );

    //uint64_t  millis = (uint64_t)(now_ms.time_since_epoch().count());

    //std::printf("millis:%lld\n ", millis);


}







void AxisConvert::convertGeoToNed(GeoCoord coord, GeoCoord origin, float* x, float* y, float* z)
{  
    if ((coord.altitude == origin.altitude) &&
        (coord.latitude == origin.latitude) &&
        (coord.longitude == origin.longitude))
    {
        // Short circuit to prevent NaNs in calculation
        *x = *y = *z = 0;
        return;
    }

    float lat_rad = coord.latitude * M_DEG_TO_RAD;
    float lon_rad = coord.longitude * M_DEG_TO_RAD;

    float ref_lon_rad = origin.longitude * M_DEG_TO_RAD;
    float ref_lat_rad = origin.latitude * M_DEG_TO_RAD;

    float sin_lat = sin(lat_rad);
    float cos_lat = cos(lat_rad);
    float cos_d_lon = cos(lon_rad - ref_lon_rad);

    float ref_sin_lat = sin(ref_lat_rad);
    float ref_cos_lat = cos(ref_lat_rad);

    float c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    float k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    *x = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    *y = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

    *z = -(coord.altitude - origin.altitude);
}

void AxisConvert::convertNedToGeo(float x, float y, float z, GeoCoord origin, GeoCoord* coord) 
{
    float x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    float y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    float c = sqrt(x_rad * x_rad + y_rad * y_rad);
    float sin_c = sin(c);
    float cos_c = cos(c);

    float ref_lon_rad = origin.longitude * M_DEG_TO_RAD;
    float ref_lat_rad = origin.latitude * M_DEG_TO_RAD;

    float ref_sin_lat = sin(ref_lat_rad);
    float ref_cos_lat = cos(ref_lat_rad);

    float lat_rad;
    float lon_rad;

    if (fabs(c) > epsilon) {
        lat_rad = asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c);
        lon_rad = (ref_lon_rad + atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c));

    }
    else {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    coord->altitude = -z + origin.altitude;
    coord->latitude = lat_rad * M_RAD_TO_DEG;
    coord->longitude = lon_rad * M_RAD_TO_DEG;
}


void AxisConvert::convertNedToRel(NedCoord nedCoord, NedCoord nedLead, RelatCoord* relatCoord, float theta)
{
    float x = nedCoord.x - nedLead.x;
    float y = nedCoord.y - nedLead.y;
    float z = nedCoord.z - nedLead.z;


    relatCoord->x = x * cos(theta) + y * sin(theta);
    relatCoord->y = -x * sin(theta) + y * cos(theta);
    relatCoord->z = z;
    relatCoord->phi = nedCoord.phi - nedLead.phi;
    relatCoord->V = nedCoord.V;
    relatCoord->lambda = nedCoord.lambda;
    relatCoord->theta = theta;

    //20230912yym:将phi限制再[-pi,pi]之间
    if (relatCoord->phi > M_PI) {
        relatCoord->phi -= 2 * M_PI;
    }
    if (relatCoord->phi < -M_PI) {
        relatCoord->phi += 2 * M_PI;
    }

}

void AxisConvert::convertRelToNed(NedCoord nedLead, RelatCoord relatCoord, NedCoord* nedCoord)
{
    float theta = nedLead.phi;
    float xr = relatCoord.x;
    float yr = relatCoord.y;
    float zr = relatCoord.z;

    nedCoord->x = xr * cos(theta) - yr * sin(theta) + nedLead.x;
    nedCoord->y = xr * sin(theta) + yr * cos(theta) + nedLead.y;
    nedCoord->z = zr + nedLead.z;
}


void AxisConvert::convertNedToSho(NedCoord nedCoord, ShootCoord* shootCoord, float theta)
{
    float x = nedCoord.x;
    float y = nedCoord.y;
    float z = nedCoord.z;

    shootCoord->x = x * cos(theta) + y * sin(theta);
    shootCoord->y = -x * sin(theta) + y * cos(theta);
    shootCoord->z = z;
    shootCoord->theta = theta;
}

void AxisConvert::convertShoToNed(ShootCoord shootCoord, NedCoord* nedCoord)
{
    float theta = shootCoord.theta;
    float xr = shootCoord.x;
    float yr = shootCoord.y;
    float zr = shootCoord.z;

    nedCoord->x = xr * cos(theta) - yr * sin(theta) ;
    nedCoord->y = xr * sin(theta) + yr * cos(theta) ;
    nedCoord->z = zr ;
}


void AxisConvert::convertGeoToRel(GeoCoord coord, GeoCoord origin, GeoCoord coordLead, RelatCoord* relatCoord, float theta)
{
    float xnf, ynf, znf , xnL, ynL, znL;
    NedCoord nedCoord,nedLead;
    convertGeoToNed(coord, origin, &xnf, &ynf, &znf);                   //从弹的经纬高转NED
    convertGeoToNed(coordLead, origin, &xnL, &ynL, &znL);               //领弹的经纬高转NED
    nedCoord.x = xnf;
    nedCoord.y = ynf;
    nedCoord.z = znf;
    nedLead.x = xnL;
    nedLead.y = ynL;
    nedLead.z = znL;
    convertNedToRel(nedCoord , nedLead , relatCoord , theta);           //从弹的NED转相对坐标
}


void AxisConvert::convertRelToGeo(GeoCoord* coord, GeoCoord origin, NedCoord nedLead, RelatCoord relatCoord)			//相对转地理
{
    NedCoord nedCoord;
    convertRelToNed(nedLead, relatCoord, &nedCoord);        //nedLead里要包含领单的速度方向

    convertNedToGeo(nedCoord.x, nedCoord.y, nedCoord.z, origin, coord);
}

void AxisConvert::convertGeoToSho(GeoCoord coord, GeoCoord origin, ShootCoord* shootCoord, float theta)
{
    float xnf, ynf, znf;
    NedCoord nedCoord;
    convertGeoToNed(coord, origin, &xnf, &ynf, &znf);                   //经纬高转NED
    nedCoord.x = xnf;
    nedCoord.y = ynf;
    nedCoord.z = znf;
    convertNedToSho(nedCoord, shootCoord,theta);                        //NED转发射
}

bool AxisConvert::isGeoCoordValid(GeoCoord coord)
{
    if (coord.altitude < 0 ||
        coord.latitude > 90 || coord.latitude < -90 ||
        coord.longitude >180 || coord.longitude < -180) {
        return false;
    }
    else
    {
        return true;
    }
}


void AxisConvert::testAxisConvert01()
{
    GeoCoord originGeo,geo1,geo2;
    originGeo.altitude = 100;
    originGeo.longitude = 110;
    originGeo.latitude = 23;

    geo1.altitude = 120;
    geo1.longitude = 110.0001;
    geo1.latitude = 23.0001;

    geo2 = geo1;

    float x, y, z;
    convertGeoToNed(geo1, originGeo, &x, &y, &z);
    convertNedToGeo(x, y, z, originGeo, &geo1);

    printf("geo1xyz = %.6f\t%.6f\t%.6f\n", x, y, z);
    printf("geo1 = %.6f\t%.6f\t%.6f\n", geo1.altitude, geo1.latitude, geo1.longitude);

    ShootCoord shoCoord;
    convertGeoToSho(geo2, originGeo, &shoCoord, 0);
    printf("geo2shoCoord = %.6f\t%.6f\t%.6f\n", shoCoord.x , shoCoord.y , shoCoord.z);

}