#ifndef EKF_KALMANFILTER_H
#define EKF_KALMANFILTER_H

#include "Eigen/Dense"

#ifdef _WIN32
__declspec(align(16))
#endif

class KalmanFilter {
public:
    KalmanFilter(double noise_ax, double noise_ay);

    void Predict(double dt);
    void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &R);
	void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &R);
	
    Eigen::VectorXd GetState() {
        return X_;
    }

    Eigen::MatrixXd GetStateCovariance() {
        return P_;
    }

    void Initialize(const Eigen::VectorXd &init_state) {
        X_ = init_state;
    }

#ifdef _WIN32
	void* operator new(size_t size) {
		return _aligned_malloc(size, 16);
	}

	void operator delete(void *p){
		return _aligned_free(p);
	}
#endif

protected:
	Eigen::VectorXd H_Radar(const Eigen::VectorXd &predicted);
	Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

private:
	void ApplyKalmanGain(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd & R);

private:
    Eigen::MatrixXd F_;			// State Transition Matrix
    Eigen::VectorXd X_;			// State Matrix
    Eigen::VectorXd X_predict_;	// Predicted State Matrix 
    Eigen::MatrixXd Q_;			// Process Noise Covariance Matrix
    Eigen::MatrixXd P_;			// State Covariance Matrix
	Eigen::MatrixXd H_laser_;	// Prediction to laser measurement matrix

    double noise_ax_;
    double noise_ay_;

#ifdef _DEBUG
	bool initialized_;
#endif
};


#endif //EKF_KALMANFILTER_H
