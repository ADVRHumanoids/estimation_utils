#ifndef ESTIMATION_UTILS_PAYLOAD_ESTIMATION_H
#define ESTIMATION_UTILS_PAYLOAD_ESTIMATION_H

#include <Eigen/Dense>
#include <XBotInterface/ModelInterface.h>
#include <estimation_utils/payload/force_estimation.h>

#include "../kalman/kalman.h"

namespace estimation_utils
{

class PayloadEstimation
{

public:

    PayloadEstimation(XBot::ModelInterface::ConstPtr model,
                      std::string payload_link,
                      double torque_noise_cov,
                      double mass_noise_cov,
                      double com_noise_cov,
                      double rate);

    bool set_noise_covariance(double torque_noise_cov);

    bool set_state_covariance(double mass_noise_cov,
                              double com_noise_cov);

    bool initialize(const Eigen::Vector4d& x0,
                    double mass_cov, double com_cov);

    bool compute(const Eigen::VectorXd& residual,
                 Eigen::VectorXd& payload_torque,
                 Eigen::Vector4d& payload_params);

    bool compute_static(Eigen::VectorXd& payload_torque,
                        Eigen::Vector4d& payload_params);

    bool compute_momentum_based(Eigen::VectorXd& payload_torque,
                        Eigen::Vector4d& payload_params);

    void compute_payload_torque(const Eigen::Vector4d& payload_params,
                                Eigen::VectorXd& payload_torque);

    void compute_payload_torque(const Eigen::Vector4d& payload_params,
                                const Eigen::Matrix4d& payload_covariance,
                                Eigen::VectorXd& payload_torque,
                                Eigen::MatrixXd& payload_torque_covariance);

    Eigen::Matrix4d getCovariance() const;


private:

    void compute_meas_matrix();

    XBot::ModelInterface::ConstPtr _model;
    KalmanFilter _kalman;
    std::string _payload_link;

    Eigen::MatrixXd _J, _C, _Q, _R, _A, _Y;
    Eigen::VectorXd _r, _tau, _grav;
    Eigen::VectorXd _zero4d;

    std::unique_ptr<estimation_utils::ForceEstimationMomentumBased> _force_estimation;

};

}

#endif // PAYLOAD_ESTIMATION_H
