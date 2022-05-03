#include <cstdlib>
#include <cmath>
#include <vector>
#include <tuple>
#include <map>
#include <algorithm>
#include <random>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;


class DataGenerator
{
public:
    DataGenerator();
    void update();

    double getTime();
    Vector3d getPoint(int i);
    Vector3d getAP(int i);
    vector<Vector3d> getCloud();
    Vector3d getPosition();
    Matrix3d getRotation();
    Quaterniond _getRotation();
    Vector3d getVelocity();

    Vector3d getAngularVelocity();
    Vector3d getLinearAcceleration();

    vector<pair<int, Vector3d>> getImage();

    static int const NUMBER_OF_CAMERA = 2;
    //static int const NUMBER_OF_AP     = 1;
    static int const NUM_POINTS       = 500;
    static int const MAX_BOX          = 10;
    static int const FREQ             = 10;
    static int const IMU_PER_IMG      = 50;
    static int const IMU_PER_WIFI     = 5;
    static int const MAX_TIME         = 10;
    static int const FOV              = 90;
private:
    int pts[NUM_POINTS * 3];
    double t;
    double t_x;
    double t_y;
    map<int, int> before_feature_id[NUMBER_OF_CAMERA];
    map<int, int> current_feature_id[NUMBER_OF_CAMERA];
    int current_id;

    Matrix3d Ric[NUMBER_OF_CAMERA];
    Vector3d Tic[NUMBER_OF_CAMERA];
    //Vector3d ap[NUMBER_OF_AP];
    Matrix3d acc_cov, gyr_cov;
    Matrix2d pts_cov;
    default_random_engine generator;
    normal_distribution<double> distribution;
};
