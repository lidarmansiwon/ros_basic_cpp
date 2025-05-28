# ros_basic_cpp

**ROS2 C++ 프로그래밍 기초 학습용 패키지**

## 패키지 개요

`ros_basic_cpp`는 C++ 기반으로 ROS2 프로그래밍의 기초를 학습하고, 실제 로봇 소프트웨어에 응용할 수 있는 역량을 기르기 위한 패키지입니다. 이 패키지는 다음과 같은 목적으로 제작되었습니다:

- **표운석 저자의 『ROS2로 시작하는 로봇 프로그래밍』 예제를 따라하며 ROS의 기본 개념 학습**
- **C++ 언어의 기초와 알고리즘을 ROS 환경에 적용해보는 실습**
- 여러 기본 튜토리얼 패키지가 존재함!
- QOS 및 DDS 이론 공부

---

## 패키지 목표

1. `cpp_study` 패키지에서 학습한 C++ 문법과 알고리즘을 ROS 환경에서 실제로 적용
2. **ROS2 Developer Guide**, **ROS2 코드 스타일 가이드**, **Google C++ Style Guide**를 준수하며 클린 코드 작성 연습

---

## 포함된 내용

- ROS2 기본 노드 작성 및 실행
- ROS2에서 Publisher / Subscriber 구성
- ROS2에서 C++ 알고리즘 활용
- 메시지 정의 및 커스텀 인터페이스 사용
- `rclcpp` 라이브러리 활용 등

---

## 🧑‍💻 코드 작성 가이드

### 기본 규칙

- C++14 표준 준수

### 라인 길이

- 한 줄은 **최대 100자**까지 허용

### 이름 규칙

| 항목                       | 규칙 유형        | 예시                             |
|----------------------------|------------------|----------------------------------|
| 타입 / 클래스 / 구조체 / 열거형 | `CamelCased`     | `MyRobot`, `SensorType`          |
| 변수 / 함수 / 메소드 / 네임스페이스 | `snake_case`     | `compute_path()`, `odom_data`    |
| 패키지 / 파일 이름           | `snake_case`     | `ros_basic_cpp`, `main_node.cpp` |
| 상수 / 매크로               | `ALL_CAPITALS`   | `MAX_SPEED`, `PI`                |

- 소스 파일 확장자: `.cpp`
- 헤더 파일 확장자: `.hpp`
- 전역 변수: `g_` 접두어 사용 (`g_global_data`)
- 클래스 멤버 변수: 변수명 끝에 `_` 접미어 사용 (`position_`)

### 공백 문자 vs 탭

- **공백(Space)** 사용 (탭 미사용)
- 기본 들여쓰기(Indent)는 공백 문자(space) 2개를 사용한다(Tab 문자 사용 금지).
- Class의 접근 지정자(public, protected, private)는 들여쓰기를 하지 않는다.

### 괄호(Brace)
- if, else, do, while, for 구문에 괄호를 사용한다.
- 
---

## 🚀 사용 방법

```bash
# 워크스페이스 생성 및 빌드 예시
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/사용자명/ros_basic_cpp.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 📝 참고 문서
ROS2 공식 문서

Google C++ Style Guide

표운석, 『ROS2로 시작하는 로봇 프로그래밍』
