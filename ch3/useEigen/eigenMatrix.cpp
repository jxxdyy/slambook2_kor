#include <iostream>

using namespace std;

#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// using namespace Eigen;

#define MATRIX_SIZE 50

/****************************
* 이 프로그램은 Eigen 기본 유형의 사용을 시연합니다.
****************************/

int main(int argc, char **argv) {
     // Eigen의 모든 벡터 및 행렬은 Eigen::Matrix로 템플릿 클래스입니다.그것의 처음 세 개의 인자는 데이터 형식, 행, 열입니다.
     // 또한 Eigen은 typedef를 통해 많은 내장형을 제공하지만, 하위 계층은 Eigen::Matrix

     Eigen::Matrix<float, 2, 3> matrix_23; // 2*3 float matrix
     // 예를 들어 Vector3d는 실질적으로 Eigen::Matrix<double, 3, 1>이며,즉 3차원 벡터
     Eigen::Vector3d v_3d;
     Eigen::Matrix<float, 3, 1> vd_3d;

     // Matrix3d는 실질적으로 Eigen:: Matrix<double, 3, 3>
     Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); // 0으로 초기화

     // Matrix크기가 확실하지 않은 경우, 동적으로 크기가 조정된 행렬을 사용할 수 있다.
     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
  
     Eigen::MatrixXd matrix_x;


     // 고유 행렬에 대한 연산
     // input (initialization)
     matrix_23 << 1, 2, 3, 4, 5, 6;
     // output
     cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

     // (i,j)를 사용하여 요소에 접근
     cout << "print matrix 2x3: " << endl;
     for (int i = 0; i < 2; i++) 
     {
          for (int j = 0; j < 3; j++) cout << matrix_23(i, j) << "\t";
          cout << endl;
     }

     v_3d << 3, 2, 1;
     vd_3d << 4, 5, 6;

     // Eigen에서는 아래와 같이 서로 다른 두 가지 유형의 행렬 혼합 불가
     //! Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
     // 명시적 변환을 허용
     Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
     cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << endl;

     Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
     cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;

     // 또한 행렬의 차원을 잘못 가져올 수 없음
     // 아래 주석 해제를 통해 확인 가능
     //* Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

     // 부분 행렬 연산
     // 사직연산은 시연 x, 바로 +, -, *, / 하면 됨
     matrix_33 = Eigen::Matrix3d::Random();                      // 난수행렬
     cout << "random matrix: \n" << matrix_33 << endl;
     cout << "transpose: \n" << matrix_33.transpose() << endl;   // 전치
     cout << "sum: " << matrix_33.sum() << endl;                 // 각 원소의 합
     cout << "trace: " << matrix_33.trace() << endl;             // 추적
     cout << "times 10: \n" << 10 * matrix_33 << endl;           // 곱셈
     cout << "inverse: \n" << matrix_33.inverse() << endl;       // 역행렬
     cout << "det: " << matrix_33.determinant() << endl;         // 행렬식
     
     // 고유값
     // 실제 대칭 행렬은 성공적인 대학화를 보장
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
     cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
     cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

     // 방정식 풀기 => (matrix_NN * x = v_Nd)의 해
     // N의 크기는 앞의 매크로에서 정의되며, 이는 난수로 생성
     // 직접 역행하는 것이 가장 직접적이지만, 연산량이 많음

     Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
          = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
     matrix_NN = matrix_NN * matrix_NN.transpose();
     Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

     clock_t time_stt = clock();
     // 역행렬 구하기
     Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
     cout << "time of normal inverse is "
          << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
     cout << "x = " << x.transpose() << endl;

     // 일반적으로 행렬 분해로 풀면, 예를 들어 QR decomp는 속도가 더 빠를 것
     time_stt = clock();
     x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
     cout << "time of Qr decomposition is "
          << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
     cout << "x = " << x.transpose() << endl;

     // 정행렬에 대해서는, cholesky 분해로 방정식을 풀 수도 있음
     time_stt = clock();
     x = matrix_NN.ldlt().solve(v_Nd);
     cout << "time of ldlt decomposition is "
          << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
     cout << "x = " << x.transpose() << endl;

     return 0;
}