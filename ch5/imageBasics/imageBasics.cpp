#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
  // argv[1]에서 지정한 이미지 읽기
  cv::Mat image;
  image = cv::imread(argv[1]); //cv::imread 지정한 경로의 이미지

  // image 파일을 정상적으로 읽어왔는지 확인
  if (image.data == nullptr) { // 데이터가 존재하지 않음
    cerr << "File" << argv[1] << " not existed." << endl;
    return 0;
  }

  // 이미지 기본 정보
  cout << "<image info>" << endl;
  cout << "width : " << image.cols << ", height : " << image.rows << ", channel : " << image.channels() << endl;
  cv::imshow("image", image);   
  cv::waitKey(0);                 

  // image 종류 판단
  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    // 이미지 형식이 부적합
    cout << "input the color image or gray scale image." << endl;
    return 0;
  }

  // std::chrono를 사용하여 알고리즘 시간을 재기
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < image.rows; y++) {
    // cv::Mat:::ptr로 그림의 행 포인터를 가져옴
    unsigned char *row_ptr = image.ptr<unsigned char>(y); 
    for (size_t x = 0; x < image.cols; x++) {
      // x, y에 있는 픽셀에 접근
      unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 접근할 픽셀 데이터를 가리킵니다
      // 이 화소를 출력하는 각 채널은, 계조도라면, 하나의 채널만 있다.
      for (int c = 0; c != image.channels(); c++) {
        unsigned char data = data_ptr[c]; // data는 I(x, y)의 c번째 channel 값
      }
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
  cout << "loop 동안 모든 픽셀에 접근하는 데에 소요 시간：" << time_used.count() << " sec" << endl;

  // cv::Mat 복사 정보
  // 직접 값을 매긴다고 해서 데이터가 복사되는 것은 아님
  cv::Mat image_another = image;
  // image_another를 수정하면 image가 변경됨
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 왼쪽 상단 모서리 100*100의 블록을 0으로 설정
  cv::imshow("image", image);
  cv::waitKey(0);

  // clone 함수를 이용하여 데이터를 복사
  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);

  // 클립, 회전, 스케일링 등 이미지에 대한 기본적인 조작이 많이 있으며, OpenCV 공식 문서를 참조하여 각 함수의 호출 방법 확인
  return 0;
}
