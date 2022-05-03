#include "data_generator.h"

#define WITH_NOISE 1
#define SEED 1
#define Y_COS 2
#define Z_COS 2 * 2

DataGenerator::DataGenerator()
{
    srand(SEED);
    t = 0;
    current_id = 0;
    t_x = 0.0;
    t_y = 0.0;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        //取值范围(-30,30)
        pts[i * 3 + 0] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 1] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 2] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
    }
    /*
    ap[0] = Vector3d( MAX_BOX, -MAX_BOX,  MAX_BOX);
    ap[1] = Vector3d(-MAX_BOX,  MAX_BOX,  MAX_BOX);
    ap[2] = Vector3d(-MAX_BOX, -MAX_BOX, -MAX_BOX);
    ap[3] = Vector3d( MAX_BOX,  MAX_BOX, -MAX_BOX);
    */
    Ric[0] <<
           0, 0, -1,
           -1, 0, 0,
           0, 1, 0;
    Tic[0] << 0.0, 0.5, 0.0;
    if (NUMBER_OF_CAMERA >= 2)
    {
        Ric[1] <<
               0, 0, -1,
               -1, 0, 0,
               0, 1, 0;
        Tic[1] << 0.0, -0.5, 0.0;
    }

    acc_cov = 0.01 * 0.01 * Matrix3d::Identity();
    gyr_cov = 0.001 * 0.001 * Matrix3d::Identity();
    pts_cov = (1.0 / 365) * (1.0 / 365) * Matrix2d::Identity();

    generator = default_random_engine(SEED);
    distribution = normal_distribution<double>(0.0, 1);
}



void DataGenerator::update()
{
    t += 1.0 / FREQ;
    // t = (int)t%30;
}

double DataGenerator::getTime()
{
    return t;
}

Vector3d DataGenerator::getPoint(int i)
{
    return Vector3d(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]);
}
/*
Vector3d DataGenerator::getAP(int i)
{
    return ap[i];
}
*/
Vector3d DataGenerator::getPosition()
{
    double x, y, z, r;
    // if (t < MAX_TIME)
    // {
    //     x = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI);
    //     y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * Y_COS);
    //     z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * Z_COS);
    // }
    // else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    // {
    //     x = MAX_BOX / 2.0 - MAX_BOX / 2.0;
    //     y = MAX_BOX / 2.0 + MAX_BOX / 2.0;
    //     z = MAX_BOX / 2.0 + MAX_BOX / 2.0;
    // }
    // else
    // {
    //     double tt = t - 2 * MAX_TIME;
    //     x = -MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI);
    //     y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * Y_COS);
    //     z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * Z_COS);
    // }

    // if (t < 7.5)
    // {
    //     x = r * (1 + cos(t / 15 * M_PI));
    //     y = -sqrt(pow(r,2) - pow(x-r,2));
    // }
    // else if (t >= 7.5 && t <= 15)
    // {
    //     x = r*(1 + cos(t / 15 * M_PI));
    //     y = -sqrt(pow(r,2) - pow(x-r,2));
    // }
    // else if (t >= 15 && t <= 22.5)
    // {
    //     x = r*(1 + cos(t / 15 * M_PI));
    //     y = sqrt(pow(r,2) - pow(x-r,2));
    // }
    // else
    // {
    //     x = r * (1 + cos(t / 15 * M_PI));
    //     y = sqrt(pow(r,2) - pow(x-r,2));
    // }
    
    //圆轨迹
    x = 0.0;
    y = 0.0;
    z = 0.0;
    r = 2.0;
    if(t < 15)
    {
        x = r * (1 + cos(t / 15 * M_PI));
        y = sqrt(pow(r,2) - pow(x-r,2));
    }
    else if(15<=t && t < 30)
    {
        x = r * (1 + cos(t / 15 * M_PI));
        y = -sqrt(pow(r,2) - pow(x-r,2));
        t_x = x;
        t_y = y;

        // y = y;
        // x = x +t;
        // t_x = x;
        // t_y = y;
    }
    else
    {
        x = t_x;
        y = t_y;
    }
    x-=2*r;
    // x = -x;
    // y = y/1.5;
    return Vector3d(y, x, z);

    //直线
    // if(t<6)
    // {
    //     x = t;
    //     y = 0;
    // }
    // if(t>=6 && t<12)
    // {
    //     x = 6;
    //     y = t - 6;
    // }
    // z = 0;
    // return Vector3d(x,y,z);
    
    // 半椭圆轨迹
    // x = 0.0;
    // y = 0.0;
    // z = 0.0;
    // r = 1.5;
    // if(t < 15)
    // {
    //     x = r * (1 + cos(t / 15 * M_PI));
    //     y = sqrt(pow(r,2) - pow(x-r,2));
    //     t_x = x;
    //     t_y = y;
    // }
    // else
    // {
    //     x = t_x;
    //     y = t_y;
    // }
    // x-=2*r;
    // x = -x;
    // // x = x/1.5;
    // return Vector3d(y, x, z);

    //方形轨迹
    // if(t<5)
    // {
    //     x = t;
    //     y = 0.0;
    //     t_x = x;
    //     t_y = y;
    // }
    // else if (t>=5 && t<10)
    // {
    //     x=5.0;
    //     y= t - 5.0;
    // }
    // else if(t>=10 && t<15)
    // {
    //     x = 15.0 - t;
    //     y = 5.0;
    // }
    // else if(t <= 20)
    // {
    //     x = 0.0;
    //     y = 20 - t;
    // }
    // else{
    //     x = 0.0;
    //     y = 0.0;
    // }
    // return Vector3d(x, y, z);

}

Matrix3d DataGenerator::getRotation()
{
    return (AngleAxisd(30.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitX())
            * AngleAxisd(40.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitY())
            * AngleAxisd(0, Vector3d::UnitZ())).toRotationMatrix();
}

//生成欧拉角，用来转换成四元数
Quaterniond DataGenerator::_getRotation()
{
    double roll,pitch,yaw;
    roll = 0;
    pitch = 0;
    if(t<=30)
    {
        yaw = -M_PI*t/15 - M_PI/2;
    }
    else
    {
        yaw = -M_PI/2;
    }
    AngleAxisd roolAngle(roll , Vector3d::UnitX());
    AngleAxisd pitchAngle(pitch , Vector3d::UnitY());
    AngleAxisd yawAngle(yaw , Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * roolAngle;
    return q;
}


Vector3d DataGenerator::getAngularVelocity()
{
    const double delta_t = 0.00001;

    // Matrix3d rot = getRotation();
    Matrix3d rot = _getRotation().toRotationMatrix();
    
    t += delta_t;
    // Matrix3d drot = (getRotation() - rot) / delta_t;
    Matrix3d drot = (_getRotation().toRotationMatrix() - rot) / delta_t;

    t -= delta_t;
    Matrix3d skew = rot.inverse() * drot;



#if WITH_NOISE
    Vector3d disturb = Vector3d(distribution(generator) * sqrt(gyr_cov(0, 0)),
                                distribution(generator) * sqrt(gyr_cov(1, 1)),
                                distribution(generator) * sqrt(gyr_cov(2, 2))
                               );
    return disturb + Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#else
    return Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#endif
}

Vector3d DataGenerator::getVelocity()
{
    double dx, dy, dz;
    if (t < MAX_TIME)
    {
        dx = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        dz = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        dx = 0.0;
        dy = 0.0;
        dz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        dx = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        dz = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }

    return getRotation().inverse() * Vector3d(dx, dy, dz);
}

Vector3d DataGenerator::getLinearAcceleration()
{
    double ddx, ddy, ddz;
    if (t < MAX_TIME)
    {
        ddx = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        ddz = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        ddx = 0.0;
        ddy = 0.0;
        ddz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        ddx = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        ddz = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }
#if WITH_NOISE
    Vector3d disturb = Vector3d(distribution(generator) * sqrt(acc_cov(0, 0)),
                                distribution(generator) * sqrt(acc_cov(1, 1)),
                                distribution(generator) * sqrt(acc_cov(2, 2))
                               );
    return getRotation().inverse() * (disturb + Vector3d(ddx, ddy, ddz - 9.8));
#else
    return getRotation().inverse() * Vector3d(ddx, ddy, ddz - 9.8);
#endif
}


vector<pair<int, Vector3d>> DataGenerator::getImage()
{
    vector<pair<int, Vector3d>> image;
    Vector3d position = getPosition();
    Matrix3d quat = getRotation();
    printf("max: %d\n", current_id);

    vector<int> ids[NUMBER_OF_CAMERA], gr_ids[NUMBER_OF_CAMERA];
    vector<Vector3d> pro_pts[NUMBER_OF_CAMERA];
    for (int k = 0; k < NUMBER_OF_CAMERA; k++)
    {
        for (int i = 0; i < NUM_POINTS; i++)
        {
            double xx = pts[i * 3 + 0] - position(0);
            double yy = pts[i * 3 + 1] - position(1);
            double zz = pts[i * 3 + 2] - position(2);
            Vector3d local_point = Ric[k].inverse() * (quat.inverse() * Vector3d(xx, yy, zz) - Tic[k]);
            xx = local_point(0);
            yy = local_point(1);
            zz = local_point(2);

            if (zz > 0.0 && std::fabs(atan2(xx, zz)) <= M_PI * FOV / 2 / 180
                    && std::fabs(atan2(yy, zz)) <= M_PI * FOV / 2 / 180)
            {
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
//条件编译，如果满足if后面的条件，就编译它们之间的代码，否则不编译。
#if WITH_NOISE
                xx += distribution(generator) * sqrt(pts_cov(0, 0));
                yy += distribution(generator) * sqrt(pts_cov(1, 1));
#endif

                int n_id = before_feature_id[k].find(i) == before_feature_id[k].end() ?
                           -1 /*current_id++*/ : before_feature_id[k][i];
                ids[k].push_back(n_id);
                gr_ids[k].push_back(i);
                pro_pts[k].push_back(Vector3d(xx, yy, zz));
            }
        }
    }

    for (int k = 0; k < NUMBER_OF_CAMERA - 1; k++)
    {
        for (int i = 0; i < int(ids[k].size()); i++)
        {
            if (ids[k][i] != -1)
                continue;
            for (int j = 0; j < int(ids[k + 1].size()); j++)
                if (ids[k + 1][j] == -1 && gr_ids[k][i] == gr_ids[k + 1][j])
                    ids[k][i] = ids[k + 1][j] = current_id++;
        }
    }

    for (int k = 0; k < NUMBER_OF_CAMERA; k++)
    {
        for (int i = 0; i < int(ids[k].size()); i++)
        {
            if (ids[k][i] == -1)
                ids[k][i] = current_id++;
            current_feature_id[k][gr_ids[k][i]] = ids[k][i];
            image.push_back(make_pair(ids[k][i] * NUMBER_OF_CAMERA + k, pro_pts[k][i]));
        }
        std::swap(before_feature_id[k], current_feature_id[k]);
        current_feature_id[k].clear();
    }
    return image;
}

vector<Vector3d> DataGenerator::getCloud()
{
    vector<Vector3d> cloud;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        double xx = pts[i * 3 + 0];
        double yy = pts[i * 3 + 1];
        double zz = pts[i * 3 + 2];
        cloud.push_back(Vector3d(xx, yy, zz));
    }
    return cloud;
}
