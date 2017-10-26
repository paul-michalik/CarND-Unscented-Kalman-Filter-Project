#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <utility>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
    ///* constants
public:
    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    const double std_a_ = 0.2;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    const double std_yawdd_ = 0.3;

    ///* Laser measurement noise standard deviation position1 in m
    const double std_laspx_ = 0.15;

    ///* Laser measurement noise standard deviation position2 in m
    const double std_laspy_ = 0.15;

    ///* Radar measurement noise standard deviation radius in m
    const double std_radr_ = 0.3;

    ///* Radar measurement noise standard deviation angle in rad
    const double std_radphi_ = 0.03;

    ///* Radar measurement noise standard deviation radius change in m/s
    const double std_radrd_ = 0.3;

    ///* radar measurement vector dimension 

    ///* State dimension
    const int n_x_ = 5;

    ///* Augmented state dimension
    const int n_aug_ = 7;

    ///* Sigma point spreading parameter
    const int lambda_ = 3 - n_aug_;

    ///* Radar measurement dimension
    const int n_z_radr_ = 3;

    ///* Weights of sigma points
    const VectorXd weights_ = ukf::calculate_weights(lambda_, n_aug_);

    const MatrixXd R_ = ukf::calculate_radar_measurement_noise(n_z_radr_, std_radr_, std_radphi_, std_radrd_);
public:
    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_ = false;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_ = true;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_ = true;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_{n_x_};

    ///* augmented state vector
    VectorXd x_aug_{n_aug_};

    ///* state covariance matrix
    MatrixXd P_{n_x_, n_x_};

    ///* augmented state covariance matrix
    MatrixXd P_aug_{n_aug_, n_aug_};

    ///* predicted augmented sigma points matrix
    MatrixXd Xsig_aug_{n_aug_, 2 * n_aug_ + 1};

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_{n_x_, 2 * n_aug_ + 1};

    ///* 
    ///* time when the state is true, in us
    long previous_timestamp_ = 0;

    ///* Normalized Innovation Squared (NIS) for respective measurements
    double NIS_radar_ = 0.;

    double NIS_lidar_ = 0.;
    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);

    void AugmentedSigmaPoints();
    void SigmaPointPrediction(double delta_t);
    void PredictMeanAndCovariance();
    void PredictRadarMeasurement(VectorXd& z_out, MatrixXd& S_out);
    void PredictLidarMeasurement();
    void UpdateState(VectorXd& x_out, MatrixXd& P_out);
};

namespace ukf {
    int lambda(int n);
    VectorXd calculate_weights(int lambda_, int n_aug_);
    MatrixXd augmented_sigma_points(int n_aug_, int lambda_, VectorXd const& x_, MatrixXd const& P_, double std_a_, double std_yawdd_);
    MatrixXd predict_sigma_points(int n_x_, int n_aug_, MatrixXd const& Xsig_aug, double delta_t);
    std::pair<VectorXd, MatrixXd> predict_mean_and_covariance(
        int n_aug_,
        int n_x_,
        VectorXd const& weights_,
        MatrixXd const& Xsig_pred_);
}
#endif /* UKF_H */
