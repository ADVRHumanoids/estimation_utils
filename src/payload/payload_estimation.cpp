#include <estimation_utils/payload/payload_estimation.h>
#include <xbot2_interface/common/utils.h>

using namespace estimation_utils;


PayloadEstimation::PayloadEstimation(XBot::ModelInterface::ConstPtr model,
                                     std::string payload_link,
                                     double torque_noise_cov,
                                     double mass_noise_cov,
                                     double com_noise_cov,
                                     double rate):
    _model(model),
    _kalman(4),
    _payload_link(payload_link)
{
    set_state_covariance(mass_noise_cov, com_noise_cov);

    set_noise_covariance(torque_noise_cov);

    _A.setIdentity(4, 4);

    if(!model->getJacobian(payload_link, _J))
    {
        throw std::invalid_argument("invalid payload link: " + payload_link);
    }

    _Y.setZero(6, 4);

    _zero4d.setZero(4);

    if(!initialize(_zero4d, 1.0, 1.0))
    {
        throw std::runtime_error("kalman init failed");
    }

    _force_estimation = std::make_unique<ForceEstimationMomentumBased>(model, rate);

}

bool PayloadEstimation::set_noise_covariance(double torque_noise_cov)
{
    _R = Eigen::VectorXd::Constant(_model->getJointNum(),
                                   torque_noise_cov).asDiagonal();

    return true;  // TODO check input
}

bool PayloadEstimation::set_state_covariance(double mass_noise_cov, double com_noise_cov)
{
    _Q = Eigen::Vector4d(
             mass_noise_cov,
             mass_noise_cov*com_noise_cov,
             mass_noise_cov*com_noise_cov,
             mass_noise_cov*com_noise_cov).asDiagonal();

    return true;  // TODO check input
}

bool PayloadEstimation::initialize(const Eigen::Vector4d& x0,
                                   double mass_cov, double com_cov)
{
    Eigen::MatrixXd P0;
    P0.setIdentity(4, 4);
    P0.diagonal() << mass_cov, mass_cov*com_cov, mass_cov*com_cov, mass_cov*com_cov;

    if(!_kalman.initialize(x0, P0))
    {
        return false; 
    }

    return true;
}

void PayloadEstimation::compute_meas_matrix()
{
    // gravity
    Eigen::Vector3d g(0, 0, -9.81);

    // skew(g)
    Eigen::Matrix3d Sg = XBot::Utils::skewSymmetricMatrix(g);

    // payload orientation w.r.t. world
    Eigen::Matrix3d R;
    _model->getOrientation(_payload_link, R);

    // payload jacobian
    _model->getJacobian(_payload_link, _J);

    // don't use virtual joints on floating base models
    // (it breakes the estimate)
    if(_model->isFloatingBase())
    {
        _J.leftCols<6>().setZero();
    }

    // payload param -> wrench matrix
    // F = Y*x
    _Y.topLeftCorner<3, 1>() = g;
    _Y.bottomRightCorner<3, 3>() = -Sg*R;

    // form measurment equation
    // C*x = J'Y*x = r
    _C.noalias() = _J.transpose()*_Y;

}

bool PayloadEstimation::compute(const Eigen::VectorXd &residual,
                                Eigen::VectorXd &payload_torque,
                                Eigen::Vector4d &payload_params)
{
    // compute _C
    compute_meas_matrix();
    _r = residual;

    // kalman
    _kalman.predict(_A, _zero4d, _Q);
    payload_params = _kalman.correct(_r, _C, _R);
    payload_torque.noalias() = -_C*payload_params;

    return true;
}

bool PayloadEstimation::compute_static(Eigen::VectorXd &payload_torque,
                                       Eigen::Vector4d &payload_params)
{
    _model->computeGravityCompensation(_grav);
    _model->getJointEffort(_tau);
    _r = _grav - _tau;

    if(_model->isFloatingBase())
    {
        _r.head<6>().setZero();
    }

    return compute(_r, payload_torque, payload_params);
}

bool PayloadEstimation::compute_momentum_based(Eigen::VectorXd &payload_torque,
                                       Eigen::Vector4d &payload_params)
{
    _force_estimation->compute_residual(_r);

    if(_model->isFloatingBase())
    {
        _r.head<6>().setZero();
    }

    return compute(_r, payload_torque, payload_params);
}

void PayloadEstimation::compute_payload_torque(const Eigen::Vector4d& payload_params, 
                            Eigen::VectorXd& payload_torque)
{
    // compute _C
    compute_meas_matrix();

    // compute torque
    payload_torque.noalias() = -_C*payload_params;
}

void PayloadEstimation::compute_payload_torque(const Eigen::Vector4d &payload_params,
                                               const Eigen::Matrix4d &payload_covariance,
                                               Eigen::VectorXd &payload_torque,
                                               Eigen::MatrixXd &payload_torque_covariance)
{
    compute_payload_torque(payload_params, payload_torque);

    payload_torque_covariance = _C * payload_covariance * _C.transpose();
}

Eigen::Matrix4d PayloadEstimation::getCovariance() const
{
    return _kalman.P();
}
