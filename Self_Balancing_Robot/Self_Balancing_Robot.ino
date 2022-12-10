// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include "I2Cdev.h"


#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      PID Controller                      ===
// ================================================================
class Queue{
  private:
    int static const max_size = 50;
    int cur_size;
    float queue[max_size];
    bool isfull;
  public:
    Queue(){
      this->isfull = false;
      this->initialize();
      this->cur_size = 0;
    }

    void enqueue(float value){
      if (this->cur_size >= this->max_size - 1){
        this->shift();
        this->queue[max_size-1] = value;
      }
      else{
        this->queue[this->cur_size] = value;
      }
      this->cur_size += 1;
    }
    
    void shift(){
      for (int i = 1; i < this->max_size; i++){
        this->queue[i-1] = this->queue[i];
      }
    }

    void initialize(){
      for (int i = 0; i < this->max_size; i++){
        this->queue[i] = -1.0;
      }
    }

    float getSum(){
      float sum = 0;
      for (int i = 0; i <= this->cur_size; i++){
        sum += this->queue[i];
      }
    }
    
};
enum system_state {
    POS_HOLD,
    POS_FREE
};
class PID_Controller{
  private:
    float prev_x_err;
    float prev_theta_err;
    float prev_time;
    system_state state;
    float x_i_err;
    float theta_i_err;
    Queue q_x;
    Queue q_theta;
  public:
    PID_Controller(){
      this->prev_x_err = 0;
      this->prev_theta_err = 0;
      this->prev_time = millis();
      this->state = POS_FREE;
      //Takes integral as sum of all errors since t=0
      this->x_i_err = 0;
      this->theta_i_err = 0;
    }

    float get_action(float theta, float x, float goal){
//      int Kpp = 35;
//      int Kpd = 10;
//      int Kpi = 2;
//
//      int Kcp = 3;
//      int Kcd = 1;
//      int Kci = 0;

      int Kpp = 70;
      int Kpd = 6;
      int Kpi = 2;

      float rps = 0;
      float cur_time = millis();
      float c_err = goal - x;
      float theta_err = 0;
      
      //POS_HOlD wants to stay at the x goal
      if (this->state == POS_HOLD){ //pos is current x-pos, goal is desired x-pos
        q_x.enqueue(c_err);
        float theta_goal = 0;//Kcp * c_err + Kcd * this->get_d_err(c_err, this->prev_x_err, cur_time) + Kci * q_x.getSum(); //replace c_err for needed error functions
        theta_err = theta_goal - theta;
      }
      else if (this->state == POS_FREE){//POS_FREE does not care about x, only theta
        theta_err = goal - theta;
        //Serial.print(theta); Serial.print(" : "); Serial.println(theta_err);
      }
      q_theta.enqueue(theta_err);
      rps = Kpp * theta_err + Kpd * this->get_d_err(theta_err, this->prev_theta_err, cur_time) + Kpi * q_theta.getSum();

      this->prev_x_err = c_err;
      this->prev_theta_err = theta_err;
      this->prev_time = cur_time;
      return rps;
    }

    float get_d_err(float cur_err, float prev_err, float cur_time){
      float d_err = 0;
      d_err = (cur_err - prev_err) / (cur_time - this->prev_time);
      return d_err;
    }

    void set_pos_state(system_state state){
      this->state = state;
    }

    system_state get_pos_state(){
      return this->state;
    }
    
};

//PID_Controller setup
PID_Controller pid;
int ml_step_pin = 8;
int ml_dir_pin = 7; //left motor
int mr_step_pin = 6;
int mr_dir_pin = 5; //right motor
int enable_pins = 4;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Initialize motor pins
    pinMode(ml_step_pin, OUTPUT);
    pinMode(mr_step_pin, OUTPUT);
    pinMode(ml_dir_pin, OUTPUT);
    pinMode(mr_dir_pin, OUTPUT);
    pinMode(enable_pins, OUTPUT);

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    //configure pos system state
    pid.set_pos_state(POS_FREE);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
float const DEGREES_PER_STEP = 0.2;
int const STEPS_PER_REVOLUTION = 1800;
int const CIRCUMFERENCE_OF_WHEELS = 314; //millimeters
float const DISTANCE_PER_STEP = 0.174; //millimeters

float accel = 1;
float cur_rps = 0;
int mot_dir = 1; // 1 = forwards, -1 = backwards. Only used for pos estimation
float cur_pos = 0;
float speed_delay = 0;
float action = 0;

void loop() {
    // if programming failed, don't try to do anything  
    if (!dmpReady) {
      
      return;
    }
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//
//      if (pid.get_pos_state() == POS_HOLD){
//        cur_rps = pid.get_action(ypr[2], cur_pos, 0); // 0 here is for x goal
//      }
//      else{
//        cur_rps = pid.get_action(ypr[2], cur_pos, 0); // 0 here is for theta goal
//      }
      cur_rps = pid.get_action(ypr[2], cur_pos, 0);
      if (cur_rps < 0){ //negative = backwards
        digitalWrite(ml_dir_pin, HIGH);
        digitalWrite(mr_dir_pin, LOW);
        mot_dir = -1;
        cur_rps = abs(cur_rps);
      }
      else if (cur_rps > 0){ //positive = forwards
        digitalWrite(ml_dir_pin, LOW);
        digitalWrite(mr_dir_pin, HIGH);
        mot_dir = 1;
      }
      Serial.print(cur_rps);
      Serial.print(" : ");
      cur_rps *= cur_rps * cur_rps;
      if (cur_rps > 3950){
        cur_rps = 3950;
      }
      Serial.print(cur_rps);
      Serial.print(" : ");
      cur_rps = 4000 - cur_rps;
      Serial.println(cur_rps);

    }
    
    //Takes step
    digitalWrite(ml_step_pin, HIGH);
    digitalWrite(ml_step_pin, LOW);
    digitalWrite(mr_step_pin, HIGH);
    digitalWrite(mr_step_pin, LOW);
    delayMicroseconds(cur_rps);

    //Dead Reakoning
    cur_pos += mot_dir * DISTANCE_PER_STEP;
}
