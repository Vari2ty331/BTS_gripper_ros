# Gripper Control

그리퍼 운용을 위한 ROS 패키지입니다.

## Description

Ubuntu version : 20.04

ROS version:noetic

## Dependencies

* Dynamixel  Workbench
* Dynamixel SDK
* rosserial package

## Installing

* {ros workspace }/ src/ 아래에 package 설치 후 
    ```
    $ catkin_make
    ```
* [Arduino IDE 설치](https://hjh1023.tistory.com/65)

* [Dynamixel  Wizard 2.0 설치](https://emanual.robotis.com/docs/kr/software/dynamixel/dynamixel_wizard2/#%EB%A6%AC%EB%88%85%EC%8A%A4%EC%97%90%EC%84%9C-%EC%84%A4%EC%B9%98%ED%95%98%EA%B8%B0)

* 아두이노, 다이나믹셀 연결 USB장치 (U2D2) PC 연결 후
    ```
    $ sudo chmod 666 /dev/ttyUSB0 #U2D2 연결 권한
    $ sudo chmod 666 /dev/ttyACM0 #아두이노 연결 권한
    ```
    장치를 찾을수 없을경우,
    ```
    $ls  /dev/tty*
    ```
    로 USB 장치 검색 후 진행

*  Dynamixel Wizard 설치 후 다이나믹셀 주소 ID 설정 필요,
    
    singleFinger (한손가락 연결 다이나믹셀) ID : 1
    
    doubleFinger (두손가락 연결 다이나믹셀) ID : 2

    ID 설정시 초기설정시에는 daisychain 상태로 연결하면 ID 충돌로 모터 인식이 되지 않으므로 (초기값이 둘다 0임) 한개씩만 연결 후 ID 변경 해야함.

* Arduino IDE 설치 후
    
    Tools->Manage Libraries->Rosserial Arduino Library 0.7.9 버전 설치 (최신버전은 0.9.1)

    button_example/twobuttons.pde   아두이노에 업로드

    Arduino Micro 사용시 첫줄 #define USE_USBCON 주석 해제

* 버튼 설치
    
    GND->Arduino gnd핀에 연결

    OUT->single finger : pin 9 / double finger : pin 11

    VCC 은 연결필요 x


## Node Description

* hanyang_gripper_controller

    pc->U2D2->dynamixel 로 통신 전달하는 node

* hanyang_gripper_operator/velocity_operator
    
    teleop control node

* hanyang_gripper_operator/gripper_initialize

    위치 초기화 노드(velocity control시에는 실행필요x)

* hanyang_gripper_operator/grip_operator

    초기화 후 상대위치 전달 노드


## Executing program

### Velocity control (teleop control)

* Controller node
    ```
    $ roslaunch hanyang_gripper_controller gripper_controllers_velocity.launch
    ```
* Operation node
    ```
    $ roslaunch hanyang_gripper_operator velocity_operator.launch 
    ```
* teleop
    ```
      Finger operations:
  
       w      /       e        /     r      : finger  open velocity
        s      /       d        /     f      : finger close velocity
  all Fingers / single Finger  / double Fingers
  x : force stop
  
  CTRL-C to quit
    ```

### Position control (initialize required)

 * Controller node
    ```
    $ roslaunch hanyang_gripper_controller gripper_controllers.launch 
    ```

    Arduino와 통신을 위한 serial node도 동시에 실행.
* Initialize node
    ```
    $ roslaunch hanyang_gripper_operator initialize.launch 
    ```
    초기화 시 single finger부터 안쪽으로 이동, button 터치 후 double finger 안쪽으로 이동.

    두 세팅 전부 안쪽으로 이동 후 initialize 종료 (node 종료x)

* Operation node
    ```
    $ roslaunch hanyang_gripper_operator grip_position_control.launch 
    ```

    * 미리 설정해놓은 yaml file 이용해서 특정 위치로 이동 가능(Work In Progress)
    * rosservice call로 특정 상대위치로 이동 가능

    ```
    $ rosservice call /hanyang_gripper/grip_size "single_finger_pos: 0 double_finger_pos: 0 move_time: 0.0" 
    ```
    * 각 finger position : 0~23000(folding 없이)
    * move  time으로 이동시간 조절
* gripSize.srv
    ```
    int32 single_finger_pos
    int32 double_finger_pos
    float64 move_time
    ---
    bool state
    ```

## Author
Contributors names and contact info

Jeeho Won ( 원 지 호 )

Robot Design Engineering Lab., Mechanical Engineering, Hanyang University
222 Wangsimni-ro, Seoul 04763, Republic of Korea

Email:jhwon331@gmail.com

Tel: +82-10-3342-9392

## Work in progress

* Initialize 이동 linear하게 변경
* Dish gripping position control safety profile 생성
