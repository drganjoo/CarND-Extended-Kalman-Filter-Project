#ifndef EKF_MEASUREMENTS_H
#define EKF_MEASUREMENTS_H

//struct GroundTruth
//{
//    double x;
//    double y;
//    double x_dot;
//    double y_dot;
//    double rho;
//    double rho_dot;
//};

struct Measurement
{
    long long timestamp;
};

struct Radar : Measurement
{
    double rho;
    double phi;
    double rhodot;
};

struct Laser : Measurement
{
    double x;
    double y;
};


#endif //EKF_MEASUREMENTS_H
