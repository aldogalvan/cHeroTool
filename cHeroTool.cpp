#include "cHeroTool.h"
#include <set>
#include <unordered_set>
#include <igl/readOBJ.h>

std::tuple<int, int, int> tint_color(std::tuple<int, int, int> base_color) {
    int pastel_r = (std::get<0>(base_color) + 255) / 2;
    int pastel_g = (std::get<1>(base_color) + 255) / 2;
    int pastel_b = (std::get<2>(base_color) + 255) / 2;
    return std::make_tuple(pastel_r, pastel_g, pastel_b);
}

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

void cHeroTool::setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, const double& a_gripper_force)
{
    m_device->setForceAndTorqueAndGripperForce(a_force,a_torque,a_gripper_force);
}

void cHeroTool::setScaleFactor(double a_scale)
{
    m_scaleFactor = a_scale;
}

void cHeroTool::buildOBBTree()
{

    // build the top level OBB
    computeMultiMeshOBB(this, *m_obbTree);

    // build a child for every cluster
    if (!m_visualizeClusters)
    {

    }

    else
    {

    }
}

void cHeroTool::clusterMesh(void)
{
    ClusteringAlgorithm alg;
    double th = 0.02;
    string filename = "/home/agalvan-admin/cHeroTool/resources/longhorn2.obj";
    m_clusters = alg.Cluster(filename,th);


    if (!m_visualizeClusters)
    {
        this->loadFromFile(filename);
    }
    else
    {
        Eigen::MatrixXd V; Eigen::MatrixXi F;
        igl::readOBJ(filename, V, F);

        for (int cidx = 0 ; cidx < m_clusters.size(); cidx++)
        {

            this->addMesh(new cMesh());

            for (int vidx = 0; vidx < V.rows(); vidx++)
            {
                this->m_meshes->back()->newVertex(cVector3d(V.row(vidx)));
            }

            for(int tidx = 0; tidx < m_clusters[cidx].size(); tidx++)
            {
                int tid = m_clusters[cidx][tidx];
                this->m_meshes->back()->newTriangle(F(tid,0),F(tid,1),F(tid,2));
            }

            // Generate a random color and convert it to pastel
            int rand_r = rand() % 256;
            int rand_g = rand() % 256;
            int rand_b = rand() % 256;
            std::tuple<int, int, int> rand_color = std::make_tuple(rand_r, rand_g, rand_b);
            std::tuple<int, int, int> pastel_color = tint_color(rand_color);

            // Set the pastel color as the material color of the mesh
            chai3d::cColorf color(std::get<0>(pastel_color) / 255.0,
                                  std::get<1>(pastel_color) / 255.0,
                                  std::get<2>(pastel_color) / 255.0);

            cout << std::get<0>(pastel_color) / 255.0 <<  " , " <<
                                                                std::get<1>(pastel_color) / 255.0 << " , "
                                                                        << std::get<2>(pastel_color) / 255.0 << endl;
            this->m_meshes->back()->m_material->setColor(color);

            assert(this->m_meshes->back()->m_material != NULL);
        }
    }
}
