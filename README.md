## OpenCV 설치
1. OpenCV 4.11.0 다운로드 : [https://opencv.org/releases/](https://opencv.org/releases/)
2. 'Libs/opencv-4.11.0/' 폴더에 OpenCV 폴더 복사
3. Visual Studio 프로젝트 링커 경로확인
    - **프로젝트 속성 > 링커 > 일반 > 추가 라이브러리 디렉터리**에서 `Libs/opencv-4.11.0/build/x64/vc16/lib` 경로를 추가.
    - **링커 > 입력 > 추가 종속성**에서 `opencv_world4110.lib`를 추가.

## PCL 설치
1. PCL 다운로드 : [https://pointclouds.org/downloads/#cross-platform](https://pointclouds.org/downloads/#cross-platform)
2. PowerShell, vcpkg를 이용한 PCL 설치
   - PS> .\vcpkg install pcl
   - PS> $env:Path -split ";"       환경변수 등록 확인
  

## 프로젝트 실행에 필요한 데이터
1. 매칭되는 Point Cloud 데이터 (main.cpp "NotchSeg_realTopPC.txt, NotchSeg_idealTopPC.txt")
2. 실제 Point Cloud 데이터 (main.cpp "REAL_TOP.tiff");
