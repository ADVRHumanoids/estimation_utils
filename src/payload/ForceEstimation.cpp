#include <estimation_utils/payload/ForceEstimation.h>

using namespace estimation_utils;

const double ForceEstimation::DEFAULT_SVD_THRESHOLD = 0.05;

ForceEstimation::ForceEstimation(XBot::ModelInterface::ConstPtr model, 
                                 double svd_threshold):
    _model(model),
    _ndofs(0)
{
    _svd.setThreshold(svd_threshold);
    _y.setZero(model->getJointNum());
}


XBot::ForceTorqueSensor::ConstPtr ForceEstimation::add_link(std::string name, 
                                                            std::vector<int> dofs,
                                                            std::vector<std::string> chains)
{
    // check link exists
    auto urdf_link = _model->getUrdf().getLink(name);
    
    if(!urdf_link)
    {
        throw std::invalid_argument("Invalid link '" + name + "'");
    }
    
    // wrench dofs if not provided
    if(dofs.size() == 0)
    {
        dofs = {0, 1, 2, 3, 4, 5};
    }
    
    // chains to use for estimation if not provided
    if(chains.size() == 0)
    {
        chains = _model->getChainNames();
    }
    
    // check dofs are valid
    auto it = std::find_if(dofs.begin(), 
                           dofs.end(), 
                           [](int dof){ return dof >= 6 || dof < 0; });
    if(it != dofs.end())
    {
        throw std::invalid_argument("Dofs must be >= 0 && < 6");
    }
    
    // add torque sensing dofs for the requested chains
    std::vector<int> meas_dofs;
    for(auto ch : chains)
    {
        if(!_model->hasChain(ch))
        {
            throw std::invalid_argument("Invalid chain '" + ch + "'");
        }
        
        for(int id : _model->chain(ch).getJointIds())
        {
            meas_dofs.push_back(_model->getDofIndex(id));
        }
    }
    
    _meas_idx.insert(meas_dofs.begin(), meas_dofs.end());

    // remove ignored ids
    for(int ignid : _ignore_idx)
    {
        _meas_idx.erase(ignid);
    }
    
    // make virtual sensor and task info struct
    TaskInfo t;
    t.link_name = name;
    static int id = -1;
    t.sensor = std::make_shared<XBot::ForceTorqueSensor>(urdf_link, id--);
    t.dofs = dofs;
    
    _tasks.push_back(t);
    
    _ndofs += dofs.size();
    
    return t.sensor;
    
}

void ForceEstimation::setIgnoredJoint(const std::string &jname)
{
    int idx = _model->getDofIndex(jname);
    if(idx < 0)
    {
        throw std::invalid_argument("invalid joint '" + jname + "'");
    }

    // add to ignored ids set
    _ignore_idx.insert(idx);

    // remove from meas ids set
    _meas_idx.erase(idx);
}


void ForceEstimation::compute_A_b()
{
    _Jtot.setZero(_ndofs, _model->getJointNum());
    _Jtmp.setZero(6, _model->getJointNum());
    _b.setZero(_meas_idx.size());
    _A.setZero(_meas_idx.size(), _ndofs);
    
    int dof_idx = 0;
    for(TaskInfo& t : _tasks)
    {
        _model->getJacobian(t.link_name, _Jtmp);
        for(int i : t.dofs)
        {
            _Jtot.row(dof_idx++) = _Jtmp.row(i);
        }
    }
    
    compute_residual(_y);
    
    int idx = 0;
    for(int i : _meas_idx)
    {
        _b(idx) = _y(i);
        _A.row(idx) = _Jtot.col(i);
        idx++;
    }
    
}

void ForceEstimation::solve()
{
    _svd.compute(_A, Eigen::ComputeThinU|Eigen::ComputeThinV);
    _sol = _svd.solve(_b);
}

void ForceEstimation::compute_residual(Eigen::VectorXd& res)
{
    _model->getJointEffort(_tau);
    _model->computeGravityCompensation(_g);

    res = _g - _tau;

}


void ForceEstimation::update()
{
    compute_A_b();

    if(_b.size() == 0)
    {
        return;
    }
    
    solve();
    
    int dof_idx = 0;
    for(TaskInfo& t : _tasks)
    {
        Eigen::Vector6d wrench;
        wrench.setZero();
        
        for(int i : t.dofs)
        {
            wrench(i) = _sol(dof_idx++);
        }
        
        Eigen::Matrix3d sensor_R_w;
        _model->getOrientation(t.link_name, sensor_R_w);
        sensor_R_w.transposeInPlace();
        
        wrench.head<3>() = sensor_R_w * wrench.head<3>();
        wrench.tail<3>() = sensor_R_w * wrench.tail<3>();
        
        t.sensor->setWrench(wrench, 0.0);
        
    }
}

bool ForceEstimation::getResiduals(Eigen::VectorXd& res) const
{
    res = _y;
    return true;
}

void estimation_utils::ForceEstimation::log(XBot::MatLogger2::Ptr logger) const
{
    for(const TaskInfo& t : _tasks)
    {
        Eigen::Vector6d wrench;
        t.sensor->getWrench(wrench);
        
        Eigen::Matrix3d w_R_s;
        _model->getOrientation(t.link_name, w_R_s);
        
        wrench.head<3>() = w_R_s * wrench.head<3>();
        wrench.tail<3>() = w_R_s * wrench.tail<3>();
        
        logger->add(t.link_name + "_f_est", wrench);
    }
    
    logger->add("fest_A", _A);
    logger->add("fest_b", _b);
    logger->add("fest_tau", _tau);
    logger->add("fest_g", _g);
}


ForceEstimationMomentumBased::ForceEstimationMomentumBased(XBot::ModelInterface::ConstPtr model,
                                                           double rate,
                                                           double svd_threshold,
                                                           double obs_bw):
    ForceEstimation(model, svd_threshold),
    _k_obs(2.0 * M_PI * obs_bw),
    _rate(rate)
{
    init_momentum_obs();
}

void ForceEstimationMomentumBased::compute_residual(Eigen::VectorXd& res)
{
    _model->getJointVelocity(_qdot);
    _model->getJointEffort(_tau);
    _model->computeGravityCompensation(_g);

    /* Observer */
    _model->getInertiaMatrix(_M);
    _p1 = _M * _qdot;

    _Mdot = (_M - _M_old) * _rate;
    _M_old = _M;
    _model->computeNonlinearTerm(_h);
    _model->computeGravityCompensation(_g);
    _coriolis = _h - _g;
    _p2 += (_tau + (_Mdot * _qdot - _coriolis) - _g + res) / _rate;

    res = _k_obs*(_p1 - _p2 - _p0);
    
}

void ForceEstimationMomentumBased::init_momentum_obs()
{
    _p1.setZero(_model->getJointNum());
    _p2.setZero(_model->getJointNum());
    _coriolis.setZero(_model->getJointNum());
    _h.setZero(_model->getJointNum());

    _model->getInertiaMatrix(_M);
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _p0 = _M * _qdot;

    _M_old = _M;
    _q_old = _q;
}
