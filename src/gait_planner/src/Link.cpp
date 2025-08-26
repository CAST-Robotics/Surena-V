#include "Link.h"

_Link::_Link(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, Vector3d com_pose, _Link *parent)
{
    this->ID_ = ID;
    this->parent_ = parent;
    this->a_ = a;
    this->b_ = b;
    this->m_ = m;
    this->I_ = inertia;
    this->c_ = com_pose;

    this->q_ = 0.0;
    this->dq_ = 0.0;
    this->ddq_ = 0.0;
    this->v_ = Vector3d::Zero();
    this->w_ = Vector3d::Zero();
}

_Link::_Link(const _Link &source)
{
    ID_ = source.ID_;
    parent_ = source.parent_;
    a_ = source.a_;
    b_ = source.b_;
    m_ = source.m_;
    I_ = source.I_;

    q_ = 0.0;
    dq_ = 0.0;
    ddq_ = 0.0;
    eulerAtitude_ << 0.0, 0.0, 0.0;
    p_ << 0.0, 0.0, 0.0; // might produce problem because puts each link position at (0, 0, 0). it should be checked.
}

short int _Link::getID()
{
    return this->ID_;
}

double _Link::q()
{
    return this->q_;
}

double _Link::getMass()
{
    return this->m_;
}

double _Link::dq()
{
    return this->dq_;
}

void _Link::update(double q, double dq, double ddq)
{
    this->q_ = q;
    this->dq_ = dq;
    this->ddq_ = ddq;
    if (parent_ == NULL)
    {
        this->w_ = Vector3d::Zero();
        // this->v_ = Vector3d::Zero();
    }
    else
    {
        this->w_ = this->parent_->getOmega() + this->dq_ * this->getRot() * this->a_;
        this->v_ = this->parent_->getLinkVel() + this->parent_->getOmega().cross(this->parent_->getRot() * this->b_);
    }
}

Vector3d _Link::getLinkVel()
{
    return this->v_;
}
Vector3d _Link::getOmega()
{
    return this->w_;
}

Vector3d _Link::getPose()
{
    return this->p_;
}

Vector3d _Link::getLinkCoM()
{
    return this->c_;
}

Matrix3d _Link::getRot()
{
    return this->R_;
}

_Link *_Link::getParent()
{
    return this->parent_;
}

void _Link::initPose(Vector3d p, Matrix3d r)
{
    this->p_ = p;
    this->R_ = r;
}

void _Link::setParams(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, _Link *parent)
{
    this->ID_ = ID;
    this->parent_ = parent;
    this->a_ = a;
    this->b_ = b;
    this->m_ = m;
    this->I_ = inertia;

    this->q_ = 0.0;
    this->dq_ = 0.0;
    this->ddq_ = 0.0;
}

Matrix3d _Link::rodrigues(Vector3d w, double dt)
{
    // Helper Function for calculating attitude in Forward Kinematics

    if (w.norm() < numeric_limits<double>::epsilon())
    {
        return Matrix3d::Identity(3, 3);
    }
    else
    {
        Vector3d wn = w / w.norm();
        double th = w.norm() * dt;
        Matrix3d w_wedge;
        w_wedge << 0.0, -wn(2), wn(1),
            wn(2), 0.0, -wn(0),
            -wn(1), wn(0), 0.0;
        Matrix3d R = Matrix3d::Identity(3, 3) + sin(th) * w_wedge + (1 - cos(th)) * w_wedge * w_wedge;
        return R;
    }
}

MatrixXd _Link::FK()
{
    if (this->parent_ == NULL)
    {
        return this->transformation();
    }
    else
    {
        this->parent_->FK();
        this->p_ = this->parent_->getRot() * this->b_ + this->parent_->p_;
        this->R_ = this->parent_->getRot() * this->rodrigues(this->a_, this->q_);
        return this->transformation();
    }
}

MatrixXd _Link::transformation()
{
    // returns homogeneous transformation matrix
    MatrixXd T(4, 4);
    T << this->R_(0, 0), this->R_(0, 1), this->R_(0, 2), this->p_(0),
        this->R_(1, 0), this->R_(1, 1), this->R_(1, 2), this->p_(1),
        this->R_(2, 0), this->R_(2, 1), this->R_(2, 2), this->p_(2),
        0.0, 0.0, 0.0, 1.0;
    return T;
}

MatrixXd _Link::updateJacobian()
{
    vector<_Link *> idx;
    _Link *base = this;
    // cout << this->getRot() << endl;
    while (base->getID() != 0)
    {
        idx.push_back(base);
        base = base->getParent();
    }
    // cout << "Route:\n";
    // for (int i = 0; i < idx.size(); i++){
    //     cout << idx[i]->getID() << "-->";
    // }
    // cout << endl;
    MatrixXd jacobian = MatrixXd::Zero(6, idx.size());
    Vector3d target = this->getPose();
    for (int n = idx.size() - 1; n >= 0; n--)
    {
        Vector3d a = idx[n]->getRot() * idx[n]->a_;
        jacobian.block<3, 1>(0, idx.size() - 1 - n) = a.cross(target - idx[n]->getPose());
        jacobian.block<3, 1>(3, idx.size() - 1 - n) = a;
    }

    return jacobian;
}

MatrixXd _Link::getVel()
{
    MatrixXd jacobian = this->updateJacobian();
    int rows = jacobian.cols();
    MatrixXd dq(rows, 1);
    _Link *link = this;
    for (int i = rows - 1; i >= 0; i--)
    {
        dq(i, 0) = link->dq();
        link = link->getParent();
    }
    return jacobian * dq;
}

_Link::~_Link()
{
}
