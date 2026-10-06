#include "transform.hpp"

Point3D transform_point(const Point3D &p, const Pose3D &pose) {
    Point3D out;
    out.x = pose.R[0] * p.x + pose.R[1] * p.y + pose.R[2] * p.z + pose.t[0];
    out.y = pose.R[3] * p.x + pose.R[4] * p.y + pose.R[5] * p.z + pose.t[1];
    out.z = pose.R[6] * p.x + pose.R[7] * p.y + pose.R[8] * p.z + pose.t[2];
    return out;
}
