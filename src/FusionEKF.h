#ifndef EKF_FUSIONEKF_H
#define EKF_FUSIONEKF_H

#include <memory>
#include "kalman_filter.h"
#include "measurements.h"

class SensorFusion {
public:
    SensorFusion();

    void ProcessMeasurement(const Laser &l);
	void ProcessMeasurement(const Radar &r);

	void Initialize(const Laser &l);
	void Initialize(const Radar &r);

    Eigen::VectorXd GetState() {
        return kf_->GetState();
    }

	Eigen::MatrixXd GetStateCovariance() {
		return kf_->GetStateCovariance();
	}

	__inline double GetDtAndSaveTimeStamp(const Measurement &measurement) {
		double dt = (measurement.timestamp - last_timestamp_) / 1000000.0;	//dt - expressed in seconds
		last_timestamp_ = measurement.timestamp;

		return dt;
	}

	bool IsInitialized() const {
		return initialized_;
	}

private:
	std::unique_ptr<KalmanFilter> kf_;
    long long last_timestamp_;
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
	bool initialized_;
};


#endif // EKF_FUSIONEKF_H
