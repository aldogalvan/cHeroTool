
#ifndef CHEROTOOL_COBB_H
#define CHEROTOOL_COBB_H

#include "chai3d.h"
#include <vector>

using namespace chai3d;
using namespace std;


class OBB {

public:

    cVector3d m_position;      // position of the OBB center
    cMatrix3d m_orientation;   // orientation matrix of the OBB
    cVector3d m_dimensions;    // dimensions of the OBB
    cTransform m_localTransform; //  the local transform

public:

    vector<OBB*> m_children; // the child oriented bounding boxes

public:

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

    void computeLocalTransform(const cTransform& globalTransform)
    {
        // compute the inverse of the parent transform
        cTransform parentInverse = globalTransform;
        parentInverse.invert();

        // compute the local transform
        m_localTransform = parentInverse * cTransform(m_position, m_orientation);
    }

    bool intersectChildren() const
    {
        // Check for intersection between this OBB and each of its children
        for (OBB* child : m_children) {
            if (intersect(*this, *child)) {
                return true;
            }
        }

        // Recursively check for intersection between this OBB's children
        for (OBB* child : m_children) {
            if (child->intersectChildren()) {
                return true;
            }
        }

        // No intersections found
        return false;
    }

    static bool intersect(const OBB& a, const OBB& b)
    {
        // Compute the vertices of the two OBBs
        std::vector<cVector3d> verts_a = a.vertices();
        std::vector<cVector3d> verts_b = b.vertices();

        // Check for intersection on all 15 axes (3 axes per face of each box, plus the 9 cross products)
        for (int i = 0; i < 15; i++) {
            // Get the axis normal
            cVector3d axis;
            if (i < 3) {
                axis = a.m_orientation.getRow(i);
            } else if (i < 6) {
                axis = b.m_orientation.getRow(i - 3);
            } else {
                int j = i - 6;
                cVector3d axis_a = a.m_orientation.getRow(j % 3);
                cVector3d axis_b = b.m_orientation.getRow(j / 3);
                axis_a.cross(axis_b);
                axis = axis_a;
            }

            // Project the vertices of the two boxes onto the axis
            double min_a = INFINITY, max_a = -INFINITY;
            double min_b = INFINITY, max_b = -INFINITY;
            for (const cVector3d& vert : verts_a) {
                double proj = vert.dot(axis);
                min_a = std::min(min_a, proj);
                max_a = std::max(max_a, proj);
            }
            for (const cVector3d& vert : verts_b) {
                double proj = vert.dot(axis);
                min_b = std::min(min_b, proj);
                max_b = std::max(max_b, proj);
            }

            // Test for intersection
            if (max_a < min_b || max_b < min_a) {
                return false;
            }
        }

        // If all axes test positive for intersection, the OBBs overlap
        return true;
    }

};


static void computeTriangleMeshOBB(cMesh* mesh , OBB& obb) {

    // compute the minimum and maximum values along each axis
    cVector3d mins(1e9, 1e9, 1e9);
    cVector3d maxs(-1e9, -1e9, -1e9);


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

static void computeMultiMeshOBB(cMultiMesh* multiMesh, OBB& obb) {
    // Initialize OBB values
    cVector3d mins(1e9, 1e9, 1e9);
    cVector3d maxs(-1e9, -1e9, -1e9);
    cVector3d center;
    cVector3d dimensions;
    cMatrix3d orientation;

    // Iterate over each mesh in the multimesh
    for (int i = 0; i < multiMesh->getNumMeshes(); i++) {
        cMesh* mesh = multiMesh->getMesh(i);
        // Compute OBB for this mesh
        computeTriangleMeshOBB(mesh, obb);

        // Update overall OBB values
        cVector3d meshMins = obb.m_position - obb.m_orientation * obb.m_dimensions / 2.0;
        cVector3d meshMaxs = obb.m_position + obb.m_orientation * obb.m_dimensions / 2.0;
        mins(0) = std::min(mins.x(), meshMins.x());
        mins(1) = std::min(mins.y(), meshMins.y());
        mins(2) = std::min(mins.z(), meshMins.z());
        maxs(0) = std::max(maxs.x(), meshMaxs.x());
        maxs(1) = std::max(maxs.y(), meshMaxs.y());
        maxs(2) = std::max(maxs.z(), meshMaxs.z());
    }

    // Compute center and dimensions of the overall OBB
    center = (mins + maxs) / 2.0;
    dimensions = maxs - mins;

    // Compute orientation of the overall OBB
    orientation.setCol0(cNormalize(cVector3d(dimensions.x(), 0, 0)));
    orientation.setCol1(cNormalize(cCross(orientation.getCol0(), cVector3d(0, 1, 0))));
    orientation.setCol2(cNormalize(cCross(orientation.getCol0(), orientation.getCol1())));

    // Update the OBB values with the overall OBB
    obb = OBB(center, orientation, dimensions);
}

static void computeTriangleMeshOBB(cMesh* mesh, OBB& obb, const std::vector<int>& triangleIndices) {
    // Compute the minimum and maximum values along each axis
    cVector3d mins(1e9, 1e9, 1e9);
    cVector3d maxs(-1e9, -1e9, -1e9);

    // Iterate over the specified triangle indices
    for (int i = 0; i < triangleIndices.size(); i++) {
        int tid = triangleIndices[i];
        cVector3d v0 = mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex(tid, 0));
        cVector3d v1 = mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex(tid, 1));
        cVector3d v2 = mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex(tid, 2));
        mins(0) = std::min(mins.x(), std::min(v0.x(), std::min(v1.x(), v2.x())));
        mins(1) = std::min(mins.y(), std::min(v0.y(), std::min(v1.y(), v2.y())));
        mins(2) = std::min(mins.z(), std::min(v0.z(), std::min(v1.z(), v2.z())));
        maxs(0) = std::max(maxs.x(), std::max(v0.x(), std::max(v1.x(), v2.x())));
        maxs(1) = std::max(maxs.y(), std::max(v0.y(), std::max(v1.y(), v2.y())));
        maxs(2) = std::max(maxs.z(), std::max(v0.z(), std::max(v1.z(), v2.z())));
    }

    // Compute the center and dimensions of the OBB
    cVector3d center = (mins + maxs) / 2.0;
    cVector3d dimensions = maxs - mins;

    // Compute the orientation of the OBB
    cMatrix3d orientation;
    orientation.setCol0(cNormalize(maxs - mins));
    orientation.setCol1(cNormalize(cCross(orientation.getCol0(), cVector3d(1, 0, 0))));
    orientation.setCol2(cNormalize(cCross(orientation.getCol0(), orientation.getCol1())));

    // Create the OBB
    obb = OBB(center, orientation, dimensions);
}



#endif //CHEROTOOL_COBB_H
