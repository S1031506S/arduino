/* - ::: HARDWARE :::

  This file is part of warp_kinematics.
  [hardware] This file manages the basic hardware functions.

  [BACK] [LEFT], LOWER JOINT (0, 0) : servo00,
  UPPER JOINT (0, 1) : servo02, SHLDR JOINT (0, 2) :servo08

  [FRONT][LEFT], LOWER JOINT (1, 0) : servo04,
  UPPER JOINT (1, 1) : servo06, SHLDR JOINT (1, 2) :servo09

  [FRONT][RIGHT], LOWER JOINT (2, 0) : servo05,
  UPPER JOINT (2, 1) : servo07, SHLDR JOINT (2, 2) :servo10

  [BACK] [RIGHT], LOWER JOINT (3, 0) : servo01,
  UPPER JOINT (3, 1) : servo03, SHLDR JOINT (3, 2) :servo11

*/

/*
  ==============================
  HARDWARE - SERVO PARAMETERS
  ==============================
*/

const int s_output[4][3] = { ///output陣列參數設定單位chnl
  {8, 2, 0},  // ## {chnl, chnl, chnl}
  {9, 6, 4},  // ## {chnl, chnl, chnl}
  {10, 7, 5}, // ## {chnl, chnl, chnl}
  {11, 3, 1}  // ## {chnl, chnl, chnl}
};

const int s_optinv[4][3] = { ///optinv陣列參數設定單位dir
  {0, 0, 0}, // ## {dir, dir, dir}
  {1, 0, 0}, // ## {dir, dir, dir}
  {0, 1, 1}, // ## {dir, dir, dir}
  {1, 1, 1}  // ## {dir, dir, dir}
};

const int s_offset_min[4][3] = { ///補償最小值陣列參數設定單位脈衝寬
  {940, 925, 900},    // ## {pulse-width, pulse-width, pulse-width}
  {750, 1025, 1025},  // ## {pulse-width, pulse-width, pulse-width}
  {920, 1080, 1050},  // ## {pulse-width, pulse-width, pulse-width}
  {675, 750, 1160}    // ## {pulse-width, pulse-width, pulse-width}
};

const int s_offset_max[4][3] = { ///補償最大值陣列參數設定單位脈衝寬
  {2875, 2800, 2400}, // ## {pulse-width, pulse-width, pulse-width}
  {2720, 2850, 2500}, // ## {pulse-width, pulse-width, pulse-width}
  {3025, 2950, 2540}, // ## {pulse-width, pulse-width, pulse-width}
  {2590, 2500, 2640}  // ## {pulse-width, pulse-width, pulse-width}
};

const int d_constraint_min[] { -50, 30, 60}; ///最小角度限制單位deg
const int d_constraint_max[] {80, 150, 140}; ///最大角度限制單位deg

/*
  ==============================
  HARDWARE - MPU VARIABLES
  ==============================
*/

uint16_t packetSize; ///define預期DMP封包大小(預設42bytes)
uint8_t fifoBuffer[64]; ///define FIFO儲存緩衝區

//Quaternion q;           // [w, x, y, z]         quaternion container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//VectorFloat gravity;    // [x, y, z]            gravity vector

/*
  ::: SETUP :::
*/

void init_hardware() { ///開啟hardware
  init_servo();
  //init_mpu();

  Wire.begin();
  Wire.setClock(400000);
}

void init_servo() { ///開啟servo
  pwm.begin(); ///開啟pwm
  pwm.setOscillatorFrequency(27000000); ///設定振盪器頻率
  pwm.setPWMFreq(330); ///設定PWM頻率
}

void init_mpu() {
  /*mpu.initialize();
    uint8_t dmp_s = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (dmp_s == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    if (DEBUG == 1)
      Serial.println(":ERROR[" + String(dmp_s) + "]");
    while (1); /*:pause_sketch:
    }*/
}

/*
  ::: HANDLE LOOP :::
*/

void handle_hardware() { ///更新mpu資料
  //update_mpu_data();
}


/*
  ::: [SERVO] FUNCTIONS :::
*/

void set_leg(int leg, datatypes::Rotator rot) {
  set_joint(leg, 0, rot.yaw); ///傳入腿部yaw資料
  set_joint(leg, 1, rot.pitch); ///傳入腿部pitch資料
  set_joint(leg, 2, rot.roll); ///傳入腿部roll資料
}

void set_joint(int leg, int joint, float deg) {
  int _min = s_offset_min[leg][joint]; //set腿部關節最小補償值
  int _max = s_offset_max[leg][joint]; //set腿部關節最大補償值
  int _num = s_output[leg][joint]; //腿部關節輸出值
  int _inv = s_optinv[leg][joint]; //腿部關節optinv

  int _minC = d_constraint_min[joint]; //限制關節最小值
  int _maxC = d_constraint_max[joint]; //限制關節最大值

  if (deg < _minC) //若最小值大於角度,set最小值assign角度
    deg = _minC;
  else if (deg > _maxC) //若最大值小於角度,set最大值assign角度
    deg = _maxC;

  if (_inv == 0)
    pwm.setPWM(_num, 0, map(deg, _minC, _maxC, _min, _max));
  else if (_inv == 1)
    pwm.setPWM(_num, 0, map(deg, _minC, _maxC, _max, _min));
}

void set_servo(int leg, int joint, float pulse) {
  int _num = s_output[leg][joint];
  pwm.setPWM(_num, 0, pulse);
}

/*
  ::: [Gyroscope/Accelerometer Sensor] FUNCTIONS :::
*/

void update_mpu_data() { ///更新mpu資料
  /*if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    _sRotation = {ypr[0] * 80 / M_PI,
                  -ypr[1] * 80 / M_PI,
                  ypr[2] * 80 / M_PI
                 };

    /*if (DEBUG == 0) {
      Serial.print(ypr[0] * 60 / M_PI);
      Serial.print("/");
      Serial.print(-ypr[1] * 60 / M_PI);
      Serial.print("/");
      Serial.println(ypr[2] * 60 / M_PI);
      }
    }*/
}
