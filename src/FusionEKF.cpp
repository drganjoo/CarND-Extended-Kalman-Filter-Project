//
// Created by Fahad Zubair on 22/06/2017.
//

#include <memory>
#include <iostream>
#include <cassert>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;
using namespace Eigen;


SensorFusion::SensorFusion() {

    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;


    /**
    * Update the state transition matrix F according to the new elapsed time.
    * Time is measured in seconds.
    * Update the process noise covariance matrix.
    * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */

    kf_ = std::unique_ptr<KalmanFilter>(new KalmanFilter(3, 3));
    last_timestamp_ = 0;
    initialized_ = false;
}

void SensorFusion::ProcessMeasurement(const Laser &l) {
    assert(initialized_);

    Eigen::VectorXd z(2);
    z << l.x, l.y;

    double dt = GetDtAndSaveTimeStamp(l);
    kf_->Predict(dt);
    kf_->Update(z, R_laser_);
}

void SensorFusion::ProcessMeasurement(const Radar &r) {
    assert(initialized_);

    Eigen::VectorXd z(3);
    z << r.rho, r.phi, r.rhodot;

    double dt = GetDtAndSaveTimeStamp(r);
    kf_->Predict(dt);
    kf_->UpdateEKF(z, R_radar_);
}

void SensorFusion::Initialize(const Laser &l) {
    Eigen::Vector4d init_state(l.x, l.y, 0, 0);

    kf_->Initialize(init_state);
    last_timestamp_ = l.timestamp;

    initialized_ = true;
}

void SensorFusion::Initialize(const Radar &r) {
    double x = r.rho * cos(r.phi);
    double y = r.rho * sin(r.phi);

    Eigen::Vector4d init_state(x, y, 0, 0);

    kf_->Initialize(init_state);
    last_timestamp_ = r.timestamp;

    initialized_ = true;
}