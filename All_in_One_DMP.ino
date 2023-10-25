#define RESTRICT_PITCH 
#define _eTaskGetState
#define MIN_PULSE_LENGTH 900
#define MAX_PULSE_LENGTH 1900
#include "HMC5883L.h"
#include "MS5611.h"
#include "filters.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <esp_now.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <ESP32_Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Definitions.h"
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13
//Declare intances 
MPU6050 accelgyro;
HMC5883L mag(0x1E);
MS5611 MS5611(0x77);

//Declare for comple filter
const float cutoff_freq = 10.0;
const float sample_time = 0.005;
IIR::ORDER  order  = IIR::ORDER::OD2;
Filter f(cutoff_freq, sample_time, order);

// Heading variables
int16_t mx, my, mz;
float heading, declinationAngle, headingdeg;

//Declare Kalman 


//Declare PID Lib

double input_1,out_Thrust,input_2,out_Roll,input_3,out_Pitch,input_4,out_Yaw;
double setpoint_1,setpoint_2,setpoint_3,setpoint_4;
PID pid_Thrust(&input_1, &out_Thrust, &setpoint_1, Pid_1[0].Kp, Pid_1[0].Ki, Pid_1[0].Kd, DIRECT);
PID pid_Roll(&input_2, &out_Roll, &setpoint_2, Pid_1[1].Kp, Pid_1[1].Ki, Pid_1[1].Kd, DIRECT);
PID pid_Pitch(&input_3, &out_Pitch, &setpoint_3, Pid_1[2].Kp, Pid_1[2].Ki, Pid_1[2].Kd, DIRECT);
PID pid_Yaw(&input_4, &out_Yaw, &setpoint_4, Pid_1[3].Kp, Pid_1[3].Ki, Pid_1[3].Kd, DIRECT);

//Declare motor vars
double motor_1,motor_2,motor_3,motor_4,motor_5,motor_6;

//Declare servo lib - output for motors
Servo servo1,servo2,servo3,servo4,servo5,servo6;

const int servo1Pin = 18;
const int servo2Pin = 5;
const int servo3Pin = 17;
const int servo4Pin = 16;
const int servo5Pin = 4;
const int servo6Pin = 15;
bool Limit = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  while (!Serial);
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  dmp_config();
  if (!MS5611.begin()) 
  {
    Serial.println("MS5611 not found, check wiring!");
    while (1);
  } 

    WiFi.mode(WIFI_STA); //WIFI mode
    if (esp_now_init() != ESP_OK) //initiliza ESP-NOW
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(OnDataSent);  
    esp_now_register_recv_cb(OnDataRecv);
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // initialize device
  // mag.initialize();
  // Serial.println("Testing device connections...");
  // Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  // MS5611.setOversampling(OSR_STANDARD);
  // delay(1000);
  //devStatus = accelgyro.dmpInitialize();
  analogSetWidth(10);
  analogSetAttenuation(ADC_0db);

  //Setpoint
  setpoint_2 = 0;
  setpoint_3 = 0;
  setpoint_4 = 0;
  // Get Setpoint height
  MS5611.read();
  ref_pres = MS5611.getPressure();
  altitude_r = MS5611.getAltitude(ref_pres,1013.25);
  // filteredval = f.filterIn(altitude_r);
  setpoint_1 = (int)altitude_r+1.5;
  delay(1000);
  //PID setup
  pid_Thrust.SetMode(AUTOMATIC);
  pid_Thrust.SetSampleTime(10);
  pid_Thrust.SetOutputLimits(-400, 400);
  pid_Roll.SetMode(AUTOMATIC);
  pid_Roll.SetOutputLimits(-400, 400);
  pid_Roll.SetSampleTime(10);
  pid_Pitch.SetMode(AUTOMATIC);
  pid_Pitch.SetOutputLimits(-400, 400);
  pid_Pitch.SetSampleTime(10);
  pid_Yaw.SetMode(AUTOMATIC);
  pid_Yaw.SetOutputLimits(-400, 400);
  pid_Yaw.SetSampleTime(10);

  //
  servo1.attach(servo1Pin,MIN_PULSE_LENGTH,1400);
  servo2.attach(servo2Pin,MIN_PULSE_LENGTH,1400);
  servo3.attach(servo3Pin,MIN_PULSE_LENGTH,1400);
  servo4.attach(servo4Pin,MIN_PULSE_LENGTH,1400);
  servo5.attach(servo5Pin,MIN_PULSE_LENGTH,1400);
  servo6.attach(servo6Pin,MIN_PULSE_LENGTH,1400);
  //Declare first task on core0
  xTaskCreatePinnedToCore
 ( 
    Task1core,
    "Task1",
    2048,
    NULL,
    2,
    &Task1,
    1    
 );
  allstop();
  vTaskSuspend(Task1);
 Serial.println("Task1 is created and suspended");
 delay(500);
 //Declare second task on core1
  xTaskCreatePinnedToCore
 ( 
    Task2core,
    "Task2",
    1024,
    NULL,
    2,
    &Task2,
    0    
 );
 vTaskSuspend(Task2);
 Serial.println("Task2 is created and suspended");
 delay(500);
 xTaskCreatePinnedToCore
 (
   TaskCore1Pid,
   "TaskPID",
   1024,
   NULL,
    1,
    &Task3,
    0   
 );
 vTaskSuspend(Task3);
 allstop();
 Serial.println("Task3 is created and suspended");
 delay(500);
  xTaskCreatePinnedToCore
 (
   TaskCore1BLDC,
   "TaskBLDC",
   1024,
   NULL,
   1,
    &Task4,
    0   
 );
 vTaskSuspend(Task4);
 Serial.println("Task BLDC Calibration is created and suspended");
 delay(500);
}

void Task1core(void * parameter)
{
  for(;;)
  { 
    sensorgather();
    pid_bldc();
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }
}

void Task2core (void * parameter)
{
    for(;;)
    {
        
        myData.b = ypr[1]* 180/M_PI;
        myData.c = ypr[2]* 180/M_PI;
        myData.d = ypr[0]* 180/M_PI;
        myData.e = filteredval;
        myData.f = motor_1;
        myData.g = motor_2;
        myData.h = motor_3;
        myData.i = motor_4;
        myData.l = motor_5;
        myData.m = motor_6;
        myData.n = out_Roll;
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));   
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
}

void TaskCore1Pid (void* parameter)
{
  for(;;)
  {
    // mag.getHeading(&mx, &my, &mz);
    // heading = atan2(my , mx );
    // declinationAngle = (0 - (46.0 / 60.0)) / (180 / M_PI);
    // heading += declinationAngle;
    // if (heading < 0) heading += 2 * PI;
    // if (heading > 2 * PI) heading -= 2 * PI;
    // headingdeg = heading * 180/M_PI;
    
    if(abs(ypr[1]* 180/M_PI) >=30 | abs(ypr[2]* 180/M_PI) >=30)
    {
      Limit = false;
      allstop();
    }
    
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }

}

void TaskCore1BLDC (void* parameter)
{
  for(;;)
  {
    double Time = micros();
    if(Time < 8000)
    {
    servo1.writeMicroseconds(MIN_PULSE_LENGTH);
    servo2.writeMicroseconds(MIN_PULSE_LENGTH);
    servo3.writeMicroseconds(MIN_PULSE_LENGTH);
    servo4.writeMicroseconds(MIN_PULSE_LENGTH);
    servo5.writeMicroseconds(MIN_PULSE_LENGTH);
    servo6.writeMicroseconds(MIN_PULSE_LENGTH);
    }
    else
    {
      test();
    }
    vTaskSuspend(Task4);
    vTaskDelay(1/ portTICK_PERIOD_MS);
    
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&deli_package, incomingData, sizeof(deli_package));
  Serial.print(deli_package.A);
  strlength = deli_package.A.length();
  Data = deli_package.A;
  char res = deli_package.A.charAt(strlength-strlength);
  char code = deli_package.A.charAt(1);
  char code_2 = deli_package.A.charAt(2);
  Serial.println(code);
  switch (res)
  {
    case 'A':
    Limit = false;
    allstop();
    vTaskSuspend(Task3);
    //Serial.print((int)eTaskGetState(Task3));
    Alert("PID-OFF");
    break;
    case 'B':
    Limit = true;
    vTaskResume(Task3);
    //Serial.print((String)eTaskGetState(Task3));
    Alert("PID-ON");
    break;
    case 'C':
    Serial.println("Resume all Tasks"); 
    vTaskResume(Task1);
    vTaskResume(Task2);
    Alert("SENSORS-ENABLED");
    break;
    case 'D':
    Serial.println("Entering Calibration mode:");
    vTaskResume(Task4);
    break;
    case 'E':
    Limit = false;
    Serial.println("Pause all");
    allstop();
    vTaskSuspend(Task1);
    vTaskSuspend(Task2);
    vTaskSuspend(Task4);
    break;
    case 'F':
    dex = deli_package.A.indexOf("F");
    height = deli_package.A.substring(1,strlength-1);
    Serial.println(height);
    setpoint_1 = height.toDouble();
    Alert(Data);
    break;
    case 'P':
    changevalPD(code, res);
    Alert(Data);
    break;
    case 'K':
    changevalPD(code, res);
    Alert(Data);
    break;
    case 'I':
    changevalPD(code, res);
    Alert(Data);
    break;
    case 'S':
    feedback(code);
    break;
    case 'T':
    throttle = deli_package.A.substring(1,strlength-1).toDouble();
    Serial.println(throttle);
    Alert(Data);
    break;        
    default:
    Serial.println("Wrong Code");
    Alert("Wrong Code");
    break;

  }
}

void Alert(String mess)
{
  
  mess.toCharArray(myData.a,mess.length());
}
void feedback(char code)
{
  switch(code)
  {
    case 'T':
    TOP_SPEED = deli_package.A.substring(2,strlength).toInt();
    deli_package.A.toCharArray(Command,strlength);
    break;
    case 'I':
    {
      IDLE_SPEED =  deli_package.A.substring(2,strlength).toInt();
      deli_package.A.toCharArray(Command,strlength);
      break;
    }
    case 'E':
    {
      char check = deli_package.A.charAt(2);
      switch (check)
      {
        case 'R':
        setpoint_2 = deli_package.A.substring(3,strlength).toInt();
        Serial.println(setpoint_2);
        break;
        case 'P':
        setpoint_3 = deli_package.A.substring(3,strlength).toInt();
        Serial.println(setpoint_3);
        break;
        case 'Y':
        setpoint_4 = deli_package.A.substring(3,strlength).toInt();
        Serial.println(setpoint_4);
        break;
        default:
        Serial.println("Wrong code");
        break;
      }
      deli_package.A.toCharArray(Command,strlength);
      break;
    }
    default:
    Alert("Wrong Code");
    break;
  }
}
void changevalPD(char code, char cd)
{
  if (code == '2')
  {
    rev_Pid = deli_package.A.substring(2,strlength+1);
    Serial.println(rev_Pid);
    if(cd == 'P')
    {
      Pid_1[1].Kp = rev_Pid.toDouble();
      pid_Roll.SetTunings(Pid_1[1].Kp,Pid_1[1].Ki,Pid_1[1].Kd);
      Serial.println(pid_Roll.GetKp());
    }
    else if (cd =='K')
    {
      Pid_1[1].Kd = rev_Pid.toDouble();
      pid_Roll.SetTunings(Pid_1[1].Kp,Pid_1[1].Ki,Pid_1[1].Kd);
      Serial.println(pid_Roll.GetKd());
    }
    else if (cd =='I')
    {
      Pid_1[1].Ki = rev_Pid.toDouble();
      pid_Roll.SetTunings(Pid_1[1].Kp,Pid_1[1].Ki,Pid_1[1].Kd);
      Serial.println(pid_Roll.GetKi());
    }
  }
  else if (code == '3')
  {
    rev_Pid = deli_package.A.substring(2,strlength);
    if(cd == 'P')
    {
      Pid_1[2].Kp = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[2].Kp,Pid_1[2].Ki,Pid_1[2].Kd);
      Serial.println(pid_Pitch.GetKp());
    }
    else if (cd =='K')
    {
      Pid_1[2].Kd = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[2].Kp,Pid_1[2].Ki,Pid_1[2].Kd);
      Serial.println(pid_Pitch.GetKd());
    }
    else if (cd =='I')
    {
      Pid_1[2].Ki = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[2].Kp,Pid_1[2].Ki,Pid_1[2].Kd);
      Serial.println(pid_Roll.GetKi());
    }   
  }
  else if (code == '1')
  {
    rev_Pid = deli_package.A.substring(2,strlength);
    switch(cd)
    {
      case 'P':
      Pid_1[0].Kp = rev_Pid.toDouble();
      pid_Thrust.SetTunings(Pid_1[0].Kp,Pid_1[0].Ki,Pid_1[0].Kd);
      Serial.println(pid_Thrust.GetKp());
      break;
      case 'I':
      Pid_1[0].Ki = rev_Pid.toDouble();
      pid_Thrust.SetTunings(Pid_1[0].Kp,Pid_1[0].Ki,Pid_1[0].Kd);
      Serial.println(pid_Thrust.GetKi());
      break;
      case 'K':
      Pid_1[0].Kd = rev_Pid.toDouble();
      pid_Thrust.SetTunings(Pid_1[0].Kp,Pid_1[0].Ki,Pid_1[0].Kd);
      Serial.println(pid_Thrust.GetKd());
      break;
      default:
      break;
    }
  }
  else if (code == '4')
  {
    rev_Pid = deli_package.A.substring(2,strlength);
    switch(cd)
    {
      case 'P':
      Pid_1[3].Kp = rev_Pid.toDouble();
      pid_Thrust.SetTunings(Pid_1[3].Kp,Pid_1[3].Ki,Pid_1[3].Kd);
      Serial.println(pid_Yaw.GetKp());
      break;
      case 'I':
      Pid_1[3].Ki = rev_Pid.toDouble();
      pid_Thrust.SetTunings(Pid_1[3].Kp,Pid_1[3].Ki,Pid_1[3].Kd);
      Serial.println(pid_Yaw.GetKi());
      break;
      case 'K':
      Pid_1[3].Kd = rev_Pid.toDouble();
      pid_Thrust.SetTunings(Pid_1[3].Kp,Pid_1[3].Ki,Pid_1[3].Kd);
      Serial.println(pid_Yaw.GetKd());
      break;
      default:
      break;
    }
  }
}
void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= 1100; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        servo1.writeMicroseconds(i);
        servo2.writeMicroseconds(i);
        servo3.writeMicroseconds(i);
        servo4.writeMicroseconds(i);
        servo5.writeMicroseconds(i);
        servo6.writeMicroseconds(i);
        delay(100);
    }

    Serial.println("STOP");
    servo1.writeMicroseconds(MIN_PULSE_LENGTH);
    servo2.writeMicroseconds(MIN_PULSE_LENGTH);
    servo3.writeMicroseconds(MIN_PULSE_LENGTH);
    servo4.writeMicroseconds(MIN_PULSE_LENGTH);
    servo5.writeMicroseconds(MIN_PULSE_LENGTH);
    servo6.writeMicroseconds(MIN_PULSE_LENGTH);
}
void allstop()
{
    servo1.writeMicroseconds(MIN_PULSE_LENGTH);
    servo2.writeMicroseconds(MIN_PULSE_LENGTH);
    servo3.writeMicroseconds(MIN_PULSE_LENGTH);
    servo4.writeMicroseconds(MIN_PULSE_LENGTH);
    servo5.writeMicroseconds(MIN_PULSE_LENGTH);
    servo6.writeMicroseconds(MIN_PULSE_LENGTH);
}

void sensorgather()
{
  if (!dmpReady) return;
  // MS5611.read();
  // ref_pres = MS5611.getPressure();
  // altitude_r = MS5611.getAltitude(ref_pres,1013.25);
  // filteredval = f.filterIn(altitude_r);

  if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { 
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
  }
}
void pid_bldc()
{
  if(Limit==true)
  {
    // input_1 = (int)filteredval;
    input_2 = ypr[1]* 180/M_PI; //this is 100% correct
    input_3 = ypr[2]* 180/M_PI;
    input_4 =  ypr[0]* 180/M_PI;
;
    // pid_Thrust.Compute();
    pid_Roll.Compute();
    pid_Pitch.Compute();
    pid_Yaw.Compute();

    motor_2 = throttle + out_Thrust - out_Roll + out_Pitch - out_Yaw; //front left (CW)
    motor_3 = throttle + out_Thrust - out_Roll /*========*/+ out_Yaw; //rear left (CCW)
    motor_4 = throttle + out_Thrust - out_Roll - out_Pitch - out_Yaw; //back left (CW)
    motor_5 = throttle + out_Thrust + out_Roll - out_Pitch + out_Yaw; // back right (CCW)
    motor_6 = throttle + out_Thrust + out_Roll /*======*/  - out_Yaw; //rear right (CW)
    motor_1 = throttle + out_Thrust + out_Roll + out_Pitch + out_Yaw; //front right (CCW)

    if(A1Raw<1240 && A1Raw>800 )
    {
      motor_1 += motor_1 * ((1240 - A3Raw)/(float)3500);
      motor_2 += motor_2 * ((1240 - A3Raw)/(float)3500);
      motor_3 += motor_3 * ((1240 - A3Raw)/(float)3500);
      motor_4 += motor_4 * ((1240 - A3Raw)/(float)3500);
      motor_5 += motor_5 * ((1240 - A3Raw)/(float)3500);
      motor_6 += motor_6 * ((1240 - A3Raw)/(float)3500);
    }

    servo1.writeMicroseconds(motor_1 = motor_1 > TOP_SPEED ? TOP_SPEED : motor_1);
    servo2.writeMicroseconds(motor_2 = motor_2 > TOP_SPEED ? TOP_SPEED : motor_2);
    servo3.writeMicroseconds(motor_3 = motor_3 > TOP_SPEED ? TOP_SPEED : motor_3);
    servo4.writeMicroseconds(motor_4 = motor_4 > TOP_SPEED ? TOP_SPEED : motor_4);
    servo5.writeMicroseconds(motor_5 = motor_5 > TOP_SPEED ? TOP_SPEED : motor_5);
    servo6.writeMicroseconds(motor_6 = motor_6 > TOP_SPEED ? TOP_SPEED : motor_6);
  }
}
void battery_com()
{
  
  for (byte n =0; n<5;n++)
  {
    A3Raw += analogRead(adc3);
    A2Raw += analogRead(adc2);
    A1Raw += analogRead(adc1);
    delay(2);
  }
  A3Raw = A3Raw/5;
  A2Raw = A2Raw/5;
  A1Raw = A1Raw/5;
  cell_1 = A3Raw * 3.29/1023;
  cell_2 = A2Raw * 3.29/1023;
  cell_3 = A1Raw * 3.29/1023;
  

}
void dmp_config()
{
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    delay(2000);
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = accelgyro.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    accelgyro.setXAccelOffset(-3270);
    accelgyro.setYAccelOffset(2095);
    accelgyro.setZAccelOffset(3485);
    accelgyro.setXGyroOffset(21);
    accelgyro.setYGyroOffset(164);
    accelgyro.setZGyroOffset(30); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        // accelgyro.CalibrateAccel(6);
        // accelgyro.CalibrateGyro(6);
        // accelgyro.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        accelgyro.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = accelgyro.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = accelgyro.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
void loop() 
{
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}