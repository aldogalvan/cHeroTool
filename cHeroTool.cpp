#include "cHeroTool.h"


void cHeroTool::updateDevicePosition(void)
{
    if (m_device != NULL)
    {
        m_device->getPosition(m_pos);
        m_pos = m_pos*m_scaleFactor;
        m_device->getLinearVelocity(m_vel);
        m_device->getRotation(m_rot);
        m_device->getAngularVelocity(m_rot_dot);
        m_device->getGripperAngleRad(m_gripper);
    }
}

bool cHeroTool::start()
{
    // check if device is available
    if (m_device == nullptr)
    {
        return (C_ERROR);
    }

    // open connection to device
    if (m_device->open())
    {
        m_enabled = true;
    }
    else
    {
        m_enabled = false;
        return (C_ERROR);
    }

}

void cHeroTool::setForceTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, const double& a_gripper_force)
{
    m_device->setForceAndTorqueAndGripperForce(a_force,a_torque,a_gripper_force);
}

void cHeroTool::setScaleFactor(double a_scale)
{
    m_scaleFactor = a_scale;
}

void cHeroTool::computeTriangleMeshOBB(cMultiMesh* multimesh, OBB& obb) {
    // compute the minimum and maximum values along each axis
    cVector3d mins(1e9, 1e9, 1e9);
    cVector3d maxs(-1e9, -1e9, -1e9);

    // we assume theres only one mesh
    cMesh* mesh = multimesh->getMesh(0);

    for (int i = 0; i < mesh->getNumVertices(); i++) {
        cVector3d vert = mesh->m_vertices->getLocalPos(i);
        mins(0) = std::min(mins.x(), vert.x());
        mins(1) = std::min(mins.y(), vert.y());
        mins(2) = std::min(mins.z(), vert.z());
        maxs(0) = std::max(maxs.x(), vert.x());
        maxs(1) = std::max(maxs.y(), vert.y());
        maxs(2) = std::max(maxs.z(), vert.z());
    }

    // compute the center and dimensions of the OBB
    cVector3d center = (mins + maxs) / 2.0;
    cVector3d dimensions = maxs - mins;

    // compute the orientation of the OBB
    cMatrix3d orientation;
    orientation.setCol0( cNormalize(maxs - mins));
    orientation.setCol1( cNormalize(cCross(orientation.getCol0(), cVector3d(1, 0, 0))));
    orientation.setCol2( cNormalize(cCross(orientation.getCol0(), orientation.getCol1())));

    // create the OBB
    obb = OBB(center, orientation, dimensions);
}