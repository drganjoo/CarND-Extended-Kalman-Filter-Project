//
// Created by Fahad Zubair on 22/06/2017.
//
#define _USE_MATH_DEFINES

#include <math.h>
#include <limits>
#include <iostream>
#include <cassert>
#include "kalman_filter.h"
#include "tools.h"

using namespace Eigen;
using namespace std;

KalmanFilter::KalmanFilter(double noise_ax, double noise_ay) {
    auto dt = 1.0;

    F_ = MatrixXd(4, 4);
    F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // set initial state matrix with NaN so that we can figure out
    // if it has been assigned to or not
    auto nan = std::numeric_limits<double>::quiet_NaN();
    X_ = VectorXd(4);
    X_ << nan, nan, nan, nan;

    Q_ = MatrixXd(4, 4);
    Q_ << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;

    noise_ay_ = noise_ay;
    noise_ax_ = noise_ax;

    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

#ifdef _DEBUG
    initialized_ = false;
#endif
}


void KalmanFilter::Predict(double dt) {
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4_4 = (dt_3 * dt) / 4.0;
    double dt_3_2 = dt_3 / 2.0;

    auto noise_ax_2 = noise_ax_ * noise_ax_;
    auto noise_ay_2 = noise_ay_ * noise_ay_;

    Q_ << dt_4_4 * noise_ax_2, 0, dt_3_2 * noise_ax_2, 0,
            0, dt_4_4 * noise_ay_2, 0, dt_3_2 * noise_ay_2,
            dt_3_2 * noise_ax_2, 0, dt_2 * noise_ax_2, 0,
            0, dt_3_2 * noise_ay_2, 0, dt_2 * noise_ay_2;

    // since our model is still using linear equations for predicting state, we do not need
    // to use Jacobian f(x) for predicting

    // lesson 5, slide 20 says that we are not using nu_ and that uncertainty will show up in Q
    // nu_ << noise_ax_ * dt_2 / 2.0, noise_ay_ * dt_2 / 2.0, noise_ax_ * dt, noise_ay_ * dt;
    // X_predict_ = F_ * X_ + nu_;

    X_predict_ = F_ * X_;
    auto Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

VectorXd KalmanFilter::H_Radar(const VectorXd &predicted) {
    // Non-Linear function to convert state prediction to Radar measurements
    // h'(x) = sqrt(px^2 + py^2), arctan(py/px), (px*vx + pyvy) / sqrt(px^2 + py^2)
    double px = predicted(0);
    double py = predicted(1);
    double vx = predicted(2);
    double vy = predicted(3);
    double px_2 = px * px;
    double py_2 = py * py;
    double px_py = px_2 + py_2;
    double px_py_root = sqrt(px_py);

    VectorXd X_polar(3);
    X_polar << px_py_root, atan2(py, px), (px * vx + py * vy) / px_py_root;

    // normalize angle to be in between -M_pi and M_PI
    X_polar[1] = Tools::NormalizeAngleRad(X_polar[1]);
    assert(X_polar[1] >= -M_PI && X_polar[1] <= M_PI);

    return X_polar;
}

MatrixXd KalmanFilter::CalculateJacobian(const Eigen::VectorXd &x_state) {
    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    double px_2 = px * px;
    double py_2 = py * py;

    //check division by zero, just set it to a very small negligible value
    if (px_2 == 0) {
        px_2 = 1e-7;
        assert(0);    // stop debugger in case this happens, would like to see why
    }
    if (py_2 == 0) {
        py_2 = 1e-7;
        assert(0);    // stop debugger in case this happens, would like to see why
    }

    double px_py = px_2 + py_2;
    double px_py_root = sqrt(px_py);
    double px_py_3_2 = px_py * px_py_root; // (px^2 + py^2)^3/2

    MatrixXd Hj(3, 4);
    Hj << px / px_py_root, py / px_py_root, 0, 0,
            -py / px_py, px / px_py, 0, 0,
            py * (vx * py - vy * px) / px_py_3_2, px * (vy * px - vx * py) / px_py_3_2, px / px_py_root, py /
                                                                                                         px_py_root;

    return Hj;
}

void KalmanFilter::ApplyKalmanGain(const VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    long x_size = X_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    X_ = X_predict_ + (K * y);
    P_ = (I - K * H) * P_;
}

void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd &R) {
    VectorXd y = z - H_laser_ * X_predict_;
    ApplyKalmanGain(y, H_laser_, R);
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &R) {
    VectorXd y = z - H_Radar(X_predict_);

    // normalize y to have values in between -pi to pi
    y[1] = Tools::NormalizeAngleRad(y[1]);
    assert(y[1] >= -M_PI && y[1] <= M_PI);

    Eigen::MatrixXd Hj = CalculateJacobian(X_predict_);
    ApplyKalmanGain(y, Hj, R);
}