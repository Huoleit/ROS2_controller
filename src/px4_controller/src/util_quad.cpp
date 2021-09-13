#include "px4_controller/util_quad.hpp"

// function qtoQ(q)
//     return H'*T*L(q)*T*L(q)*H
// end

// function G(q)
//     G = L(q)*H
// end

// function rptoq(ϕ)
//     (1/sqrt(1+ϕ'*ϕ))*[1; ϕ]
// end

// function qtorp(q)
//     q[2:4]/q[1]
// end

// function hat(v)
//     return [0 -v[3] v[2];
//             v[3] 0 -v[1];
//             -v[2] v[1] 0]
// end

// function L(q)
//     s = q[1]
//     v = q[2:4]
//     L = [s    -v';
//             v  s*I+hat(v)]
//     return L
// end

    // function hat(v)
//     return [0 -v[3] v[2];
//             v[3] 0 -v[1];
//             -v[2] v[1] 0]
// end
using namespace Eigen;

namespace UtilQuad
{
    

    Matrix3d hat(Vector3d v)
    {
        Matrix3d h;
        h << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return h;
    }

    Matrix4d L_matrix(Vector4d q)
    {
        Matrix4d res;
        res.block<1,3>(0,1) = -q.segment<3>(1).transpose();
        res.block<3,1>(1,0) = q.segment<3>(1);
        res.block<3,3>(1,1) = hat(q.segment<3>(1));
        return res + q(0)*Matrix4d::Identity();
    }

    Matrix<double, 4, 3> H_matrix()
    {
        Matrix<double, 4, 3> H = Matrix<double, 4, 3>::Zero();
        H.block<3,3>(1,0) = Matrix3d::Identity();
        return H;
    }
    Matrix<double, 13, 12> E_matrix(Vector4d q)
    {
        Matrix<double, 13, 12> res = Matrix<double, 13, 12>::Zero();
        res.block<3,3>(0,0) = Matrix3d::Identity();
        res.block<4,3>(3,3) = L_matrix(q) * H_matrix();
        res.block<6,6>(7,6) = Matrix<double, 6, 6>::Identity();
        return res;
    }

    Vector3d qtorp(Vector4d q)
    {
        return q.segment<3>(1)/q(0);
    }
    
}
