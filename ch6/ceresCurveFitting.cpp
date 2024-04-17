//
// Created by xiang on 18-11-19.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// 대가함수계산모델
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

  // 잔차의 계산
  template<typename T>
  bool operator()(
    const T *const abc, // 模모델 파라미터, 3차원
    T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
    return true;
  }

  const double _x, _y;    // x,y 데이터
};

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;         // 실제 인자값
  double ae = 2.0, be = -1.0, ce = 5.0;        // 추정 파라미터 값
  int N = 100;                                 // 데이터 포인트
  double w_sigma = 1.0;                        // 노이즈 시그마 값
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV 난수 생성기

  vector<double> x_data, y_data;      // 데이터
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }

  double abc[3] = {ae, be, ce};

  // Least Square Method
  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(     // 문제에 오차 항목을 추가하다
      // 자동 안내를 사용합니다. 템플릿 매개 변수: 오류 형식, 출력 차원, 입력 차원, 차원이 앞의 struct에 일치해야 합니다
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(x_data[i], y_data[i])
      ),
      nullptr,            // 핵함수, 여기 사용하지 않음, 비어 있음
      abc                 // 추정 인자
    );
  }

  // ceres::Solver 설정 부분
  ceres::Solver::Options options;     // 다양한 옵션 설정 가능
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 증분 방정식을 어떻게 풀어야 할지?
  options.minimizer_progress_to_stdout = true;   // cout으로 출력

  ceres::Solver::Summary summary;                 // optimization
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);      // optimization 시작
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // results
  cout << summary.BriefReport() << endl;
  cout << "estimated a,b,c = ";
  for (auto a:abc) cout << a << " ";
  cout << endl;

  return 0;
}