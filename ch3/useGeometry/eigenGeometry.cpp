#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

// using namespace Eigen;



int main(int argc, char **argv) {

  // Eigen/Geometry 모듈은 다양한 회전 및 이동 표현을 제공
  // 3D 회전 행렬은 Matrix3d 또는 Matrix3f를 사용
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

  // 회전 방향량은 AngleAxis를 사용
  Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));     // Z축을 따라 45도 회전
  cout.precision(3);
  cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;   // matrix()를 통해 행렬로 변환
  // 배열을 직접 할당도 가능
  rotation_matrix = rotation_vector.toRotationMatrix();

  // AngleAxis로 좌표 변환 가능
  Eigen::Vector3d v(1, 0, 0);
  Eigen::Vector3d v_rotated = rotation_vector * v;
  cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
  // 또는 회전행렬을 사용
  v_rotated = rotation_matrix * v;
  cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

  // Euler angle : 회전 행렬을 euler angle로 바로 변환할 수 있다.
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX 순서，즉yaw-pitch-roll 순서
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

  // Eigen::Isometry를 사용한 euclidean 변환 행렬
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();           // 3d라고 명시되어 있지만，실질적으로는 4*4행렬
  T.rotate(rotation_vector);                                     // rotation_vector에 따라 회전
  T.pretranslate(Eigen::Vector3d(1, 3, 4));                      // 변환벡터를 (1,3,4)로 설정
  cout << "Transform matrix = \n" << T.matrix() << endl;

  // 변환행렬을 사용한 좌표 변환
  Eigen::Vector3d v_transformed = T * v;                              // R*v+t와 동일
  cout << "v tranformed = " << v_transformed.transpose() << endl;

  //* affine 변환 및 projection 변환에는 Eigen::Affine3d 和 Eigen::Projective3d를 사용

  //* Quaternion
  // Quaternion에 AngleAxis를 직접 할당할 수 있으며 그 반대도 가능
  Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
  cout << "quaternion from rotation vector = " << q.coeffs().transpose()
       << endl;   // 계수의 순서는 (x,y,z,w)이고, w는 실수부이고，처음 3개는 허수부

  // 회전행렬 할당도 가능
  q = Eigen::Quaterniond(rotation_matrix);
  cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
  // Quaternion을 사용하여 벡터를 회전하고 오버로드된 곱셈을 사용
  v_rotated = q * v; // 수학에서 qvq^{-1}을 의미함
  cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  // 통상적인 벡터의 곱으로 나타내면 아래와 같이 계산됨
  cout << "should be equal to " << (q * Eigen::Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

  return 0;
}
