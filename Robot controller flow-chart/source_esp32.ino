/*
  Quadruped robot arduino sketch.
  3/10/2020 by Alexandros Petkos
  Updates available at https://github.com/maestrakos/warp

  This kinematics sketch is placed under CC-BY.

  This file is part of warp_kinematics.

  [source] This is the main file that manages [kinematics] & [hardware]
  all the important parameters are set in this file.

  Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/

#include "datatypes.h"

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*#include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
  MPU6050 mpu;*/

#include <PS4Controller.h> ///使用PS4控制

/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
//> stores the frequency of the loop function
const float frequency = 440.0; ///設定頻率參數單位Hz

/// Kinematics Parameters

//: stores the location, rotation and scale of the main [body]
const datatypes::Transform body_transform = {
  {0, 0, 0},  // ## {mm, mm, mm}設定主體的位置三維參數單位
  {0, 0, 0},   // ## {deg, deg, deg}設定主體的旋轉參數單位
  {300, 40, 180} // ## {mm, mm, mm}設定主體的比例三維參數單位
};

//: stores the parent joint location relative to the [body]
const datatypes::Vector p_joint_origin[] = {
  { -50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { -50, 0, 0}  // ## {mm, mm, mm} ///parent joint location relative to the body參數單位mm
};
const float bone_length = 105; ///設定骨骼長度參數單位mm

//: high level parameters for the step function
const datatypes::Vector step_extent = {40, 40, 26}; // ## {mm, mm}
float vrt_offset = - 16.50; // ## mm
float hrz_offset = - 6.00; // ## mm

float base_offset[] = { 0, -1, 0, -2};
const float precision = 0.001; ///設定高等參數單位mm

void setup() {
  Serial.begin(115200);

  init_hardware(); ///初始化hardware&input fn
  init_input();
}

//: those local variables control the step direction and period
datatypes::Vector2D _direction = {0, 0};
float turn = 0; ///define 旋轉方向
float height = 0; ///define 腿部伸展

int state = 0; ///指示步伐的型態(0) 閒置, (1) 小跑步, (2) 橫擺, (3) 俯仰滾動, (4) 物體偵測 (2) yaw, (3) pitch-roll, (4) object-detection
float _period = 10.0; ///define 指示每秒步伐的數量

datatypes::Rotator _sRotation; ///儲存身體的相對旋轉

unsigned long duration; ///define 持續時間變數
int sample_sum, sample_num = 10, ///define 設定樣本和與樣本數量
                sample_index;
float freq; /// 設定頻率

void loop() {
  duration = millis(); ///紀錄時間

  handle_hardware(); ///呼叫handle_hardware()函式
  handle_kinematics(_direction, turn, height, _period);
  ///呼叫handle_kinematics()函式並引入持續時間、選轉方向、高度、週期參數到函式裡
  handle_input(); ///呼叫handle_input()函式

  if (Serial.available()) ///判斷Serial有沒有數據進來
    handle_serial(); ///呼叫handle_serial()函式

  // 下面這段程式碼取得loop function的頻率
  /*sample_sum += 1000.0 / (millis() - duration);
    sample_index++;

    if (sample_index > sample_num) {
    freq = sample_sum / sample_num;
    Serial.println(freq);
    sample_sum = 0;
    sample_index = 0;
    }*/
}

float vo, ho;
void init_input() { //This fn is for PS4 input
  PS4.begin("F8:C3:9E:3F:F8:10"); /// 開啟控制器的藍芽MAC地址
  vo = vrt_offset; ///assign垂直補償
  ho = hrz_offset; ///assign水平補償
}

bool _tb = false;
float stick_min = 6.f;
float lx, ly, rx, ry;
void handle_input() { //這個函式用來計算PS4操控動作的數值,之後送進kinematics算舵機角度
  if (PS4.isConnected()) {
    lx = inter(lx, PS4.data.analog.stick.lx / 4.f, 0.5f); //> gets the interpolated x-position of the left  analog stick
    ly = inter(ly, PS4.data.analog.stick.ly / 4.f, 0.5f); //> gets the interpolated y-position of the left  analog stick
    rx = inter(rx, PS4.data.analog.stick.rx / 4.f, 0.5f); //> gets the interpolated x-position of the right analog stick
    ry = inter(ry, PS4.data.analog.stick.ry / 4.f, 0.5f); //> gets the interpolated y-position of the right analog stick

    if (abs(lx) > stick_min) { ///確認stick左邊的X位置是否在死角
      float x0 = lx - stick_min * sign(lx); ///減掉死角角度做修正
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { ///確認stick左邊的Y位置是否在死角
      float y0 = ly - stick_min * sign(ly); ///減掉死角角度做修正
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
        else
          vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { ///確認stick右邊的X位置是否在死角
      float x1 = rx - stick_min * sign(rx); ///減掉死角角度做修正
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { ///確認stick右邊的Y位置是否在死角
      float y1 = ry - stick_min * sign(ry); ///減掉死角角度做修正
      height = y1;
    } else height = 0;
  }

  if (PS4.data.button.touchpad) { ///確認觸控板的狀態
    if (_tb == true) {
      _tb = false; state++;
      if (state > 4) state = 0;
    }
  } else _tb = true;
}

// !! make sure you have enabled Newline or Carriage return
#define _mode 1 // (0) used for calibration and testing, (1) uses serial as input
void handle_serial() {
  //: reads and stores the serial data
  int i = 0; float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Serial.available()) {
    char c = Serial.read(); ///將Serial讀到的數值assign到c
    if (c == 13 || c == 32 || c == '\n') { ///儲存數值在Buffer
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    } else
      s_buff += c;
  }

  if (_mode == 0) ///若_mode等於0，執行修正和測試
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1) ///若_mode等於1，使用Serial數值當作輸入
    if (state == 4) {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

/// 這是一個插值函式被用於平滑
float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}

#define properties 0
void commands_exe(float val1, float val2, float val3) { ///校正函式
  //: properties 0 is used to calibrate the joints
  if (properties == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" joint ");
    Serial.print(joint);
    Serial.print(" set to ");
    Serial.print(servo);
    Serial.print(".\n");

    set_servo(leg, joint, servo); ///呼叫set_servo()函式
  }
  //: properties 1 is used for small adjustments to balance the weight
  else if (properties == 1) { ///小調整和平衡重量
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" null ");
    Serial.print(empty);
    Serial.print(" set to ");
    Serial.print(ammount);
    Serial.print(".\n");

    base_offset[leg] = ammount;
  }
}
