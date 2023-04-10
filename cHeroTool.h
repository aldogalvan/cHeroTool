#ifndef CHEROTOOL_CHEROTOOL_H
#define CHEROTOOL_CHEROTOOL_H

#include "chai3d.h"

using namespace chai3d;

class OBB {
public:
    cVector3d m_position;      // position of the OBB center
    cMatrix3d m_orientation;   // orientation matrix of the OBB
    cVector3d m_dimensions;    // dimensions of the OBB

    // constructor
    OBB(const cVector3d& position, const cMatrix3d& orientation, const cVector3d& dimensions)
            : m_position(position), m_orientation(orientation), m_dimensions(dimensions) {}

    // compute the vertices of the OBB
    std::vector<cVector3d> vertices() const {
        std::vector<cVector3d> verts;
        verts.reserve(8);
        const double& x = m_dimensions.x();
        const double& y = m_dimensions.y();
        const double& z = m_dimensions.z();
        verts.push_back(m_position + m_orientation * cVector3d(-x, -y, -z) / 2.0);
        verts.push_back(m_position + m_orientation * cVector3d(x, -y, -z) / 2.0);
        verts.push_back(m_position + m_orientation * cVector3d(-x, y, -z) / 2.0);
        verts.push_back(m_position + m_orientation * cVector3d(x, y, -z) / 2.0);
        verts.push_back(m_position + m_orientation * cVector3d(-x, -y, z) / 2.0);
        verts.push_back(m_position + m_orientation * cVector3d(x, -y, z) / 2.0);
        verts.push_back(m_position + m_orientation * cVector3d(-x, y, z) / 2.0);
        verts.push_back(m_position + m_orientation * cVector3d(x, y, z) / 2.0);
        return verts;
    }
};


class cHeroTool : public cMultiMesh
{
    cHeroTool(cGenericHapticDevicePtr a_device)
    {
        m_device = a_device;
        m_force = cVector3d(0,0,0);
        m_torque = cVector3d(0,0,0);
        m_gripper_force = 0;
        updateDevicePosition();
    }

    ~cHeroTool()
    {

    }

public:

    // this function transfers any necessary data to the GPU
    virtual void transferToGPU(){}

    // this function updates the tool position
    void updateDevicePosition(void);

    // this function starts the haptic device
    bool start(void);

    // this function updates the force
    void setForceTorqueAndGripperForce(const cVector3d&, const cVector3d&, const double&);

    // this function sets the scale factor
    void setScaleFactor(double a_scale);

    // this function creates an OBB BVH
    void computeTriangleMeshOBB(cMultiMesh* multimesh, OBB& obb);

public:

    // the haptic device
    cGenericHapticDevicePtr m_device;

    // the position
    cVector3d m_pos;

    // the orientation
    cMatrix3d m_rot;

    // the velocity
    cVector3d m_vel;

    // the angular velocity
    cVector3d m_rot_dot;

    // the gripper angle
    double m_gripper;

    // the force
    cVector3d m_force;

    // the torque
    cVector3d m_torque;

    // the gripper force
    double m_gripper_force;

    // the scale factor
    double m_scaleFactor = 4;

};



#endif //CHEROTOOL_CHEROTOOL_H
