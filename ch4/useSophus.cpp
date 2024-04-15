#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

/// sophus 사용법

int main(int argc, char **argv) {

  // z축 기준 90도 rotation matrix
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  // Quaternion
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);              // Sophus::SO3d는 rotation matrix로부터 직접 구축 가능
  Sophus::SO3d SO3_q(q);              // quaternion을 통해서도 가능
  // 동치
  cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
  cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
  cout << "they are equal" << endl;

  // log를 이용한 lie algebra 변환
  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat skew symmetric matrix
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
  cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  // 증분 perturbation model update
  Vector3d update_so3(1e-4, 0, 0); // 임의의 update량 설정
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
  cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

  cout << "*******************************" << endl;
  // SE(3)
  Vector3d t(1, 0, 0);                  // x축을 따라 1 이동
  Sophus::SE3d SE3_Rt(R, t);            // SE(3) -> R|t
  Sophus::SE3d SE3_qt(q, t);            // quaternion,translation to SE(3)
  cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
  cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;

  // lie algebra se(3)는 6차원 vector -> 사전 정의
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;
  cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

  Vector6d update_se3; // update
  update_se3.setZero();
  update_se3(0, 0) = 1e-4;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
  cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

  return 0;
}
