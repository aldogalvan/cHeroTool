#ifndef CHEROTOOL_CHEROTOOL_H
#define CHEROTOOL_CHEROTOOL_H

#include "chai3d.h"
#include "cOBB.h"
#include "external/hierarchical-face-clustering/ClusteringAlgorithm.h"

#include <vector>

using namespace chai3d;
using namespace std;

class cHeroTool : public cMultiMesh
{
public:

    cHeroTool(cGenericHapticDevicePtr a_device)
    {
        m_device = a_device;
        m_force = cVector3d(0,0,0);
        m_torque = cVector3d(0,0,0);
        m_gripper_force = 0;
        m_obbTree = new OBB(cVector3d(), cMatrix3d(), cVector3d() );
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
    void setForceAndTorqueAndGripperForce(const cVector3d&, const cVector3d&, const double&);

    // this function sets the scale factor
    void setScaleFactor(double a_scale);

    // this function builds the OBB tree
    void buildOBBTree(void);

    // this function partitions the triangle mesh
    void clusterMesh(void);


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
    double m_scaleFactor = 1;

    // the oriented boundng box tree
    OBB* m_obbTree;

public:

    // the vector od clusters
    vector<vector<int>> m_clusters;

    // boolean flag if true
    bool m_visualizeClusters = true;

};



#endif //CHEROTOOL_CHEROTOOL_H
