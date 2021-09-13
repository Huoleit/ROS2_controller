#pragma once

#include <Eigen/Eigen>

namespace UtilQuad
{
    Eigen::Matrix3d hat(Eigen::Vector3d v);
    
    Eigen::Matrix4d L_matrix(Eigen::Vector4d q);
    Eigen::Matrix<double, 4, 3> H_matrix();
    Eigen::Matrix<double, 13, 12> E_matrix(Eigen::Vector4d q);
    Eigen::Vector3d qtorp(Eigen::Vector4d q);
}


