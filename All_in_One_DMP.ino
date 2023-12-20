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
#include <SimpleKalmanFilter.h>
#include <HCSR04.h>
#include <RunningMedian.h>
RunningMedian Alitudez = RunningMedian(15);
// movingAvg mySensor(5);
// const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
// const float sampling_time = 0.005; //Sampling time in seconds.
// IIR::ORDER  order  = IIR::ORDER::OD2; // Order (OD1 to OD4)
// Filter dis(cutoff_freq, sampling_time, order);
const byte triggerpin = 27;
const byte echopin = 14;
UltraSonicDistanceSensor distanceSensor(triggerpin, echopin);
#define INTERRUPT_PIN 2  
#define LED_PIN 13
//Declare intances 
MPU6050 accelgyro;
HMC5883L mag(0x1E);
MS5611 MS5611(0x77);

//

// Heading variables
int16_t mx, my, mz;
float heading, declinationAngle, headingdeg;

//Declare Kalman 
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

//Declare PID Lib

double input_1,out_Thrust,input_2,out_Roll,input_3,out_Pitch,input_4,out_Yaw, input_5, out_Xd, input_6, out_Yd;
double setpoint_1,setpoint_2,setpoint_3,setpoint_4,setpoint_5,setpoint_6;
PID pid_Thrust(&input_1, &out_Thrust, &setpoint_1, Pid_1[0].Kp, Pid_1[0].Ki, Pid_1[0].Kd, DIRECT);
PID pid_Roll(&input_2, &out_Roll, &setpoint_2, Pid_1[1].Kp, Pid_1[1].Ki, Pid_1[1].Kd, DIRECT);  //wrong this pitch
PID pid_Pitch(&input_3, &out_Pitch, &setpoint_3, Pid_1[2].Kp, Pid_1[2].Ki, Pid_1[2].Kd, DIRECT); //roll
PID pid_Yaw(&input_4, &out_Yaw, &setpoint_4, Pid_1[3].Kp, Pid_1[3].Ki, Pid_1[3].Kd, DIRECT);
PID pid_X(&input_5, &out_Xd, &setpoint_5, Pid_1[4].Kp, Pid_1[4].Ki, Pid_1[4].Kd, DIRECT);
PID pid_Y(&input_6, &out_Yd, &setpoint_6, Pid_1[5].Kp, Pid_1[5].Ki, Pid_1[5].Kd, DIRECT);
double pid_error_gain_altitude;
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
  analogSetWidth(10); 
  analogReadResolution(10);
  // analogSetAttenuation(ADC_0db);
  // mySensor.begin();
  //Setpoint
  setpoint_2 = 0;
  setpoint_3 = 0;
  setpoint_4 = 0;
  setpoint_5 = 1.3;
  setpoint_6 = 2.5;
  // Get Setpoint height
  // for(byte n = 0; n<50;n++ )
  // {
  //   MS5611.read();
  //   ref_pres = MS5611.getPressure();
  //   double fix_height = MS5611.getAltitude(ref_pres,1013.25);
  //   base_height += pressureKalmanFilter.updateEstimate(fix_height);
  // }
  // base_height /= 50;;
  setpoint_1 = 80;
  delay(1000);
  //PID setup
  pid_Thrust.SetMode(AUTOMATIC);
  pid_Thrust.SetSampleTime(10);
  pid_Thrust.SetOutputLimits(-100, 100);
  pid_Roll.SetMode(AUTOMATIC);
  pid_Roll.SetOutputLimits(-400, 400);
  pid_Roll.SetSampleTime(10);
  pid_Pitch.SetMode(AUTOMATIC);
  pid_Pitch.SetOutputLimits(-400, 400);
  pid_Pitch.SetSampleTime(10);
  pid_Yaw.SetMode(AUTOMATIC);
  pid_Yaw.SetOutputLimits(-400, 400);
  pid_Yaw.SetSampleTime(10);
  pid_X.SetMode(AUTOMATIC);
  pid_X.SetOutputLimits(-5, 5);
  pid_X.SetSampleTime(10);
  pid_Y.SetMode(AUTOMATIC);
  pid_Y.SetOutputLimits(-5, 5);
  pid_Y.SetSampleTime(10);

  //
  servo1.attach(servo1Pin,MIN_PULSE_LENGTH,1900);
  servo2.attach(servo2Pin,MIN_PULSE_LENGTH,1900);
  servo3.attach(servo3Pin,MIN_PULSE_LENGTH,1900);
  servo4.attach(servo4Pin,MIN_PULSE_LENGTH,1900);
  servo5.attach(servo5Pin,MIN_PULSE_LENGTH,1900);
  servo6.attach(servo6Pin,MIN_PULSE_LENGTH,1900);
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
        myData.b = ypr[1]* 180/M_PI; //roll
        myData.c = ypr[2]* 180/M_PI; //pitch
        myData.d = ypr[0]* 180/M_PI;
        myData.e = true_height;
        myData.n = Distance.d1;
        myData.o = Distance.d2;
        myData.q = Distance.d3;
        myData.r = Px;
        myData.t = Py;
        myData.s = out_Xd;
        myData.i = out_Yd;
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));   
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
}

void TaskCore1Pid (void* parameter)
{
  for(;;)
  {
    if(abs(ypr[1]* 180/M_PI) >=30 | abs(ypr[2]* 180/M_PI) >=30)
    {
      Limit = false;
      throttle=1100;
      allstop();
    }
    if (takeoff == true)
    {
      lifting();
    }
    if (takedown == true)
    {
      landing();
    }
    // battery_com();
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
  char macstr[18];
  snprintf(macstr, sizeof(macstr), "%02x:%02x:%02x:%02x:%02x:%02x",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  String address = macstr;
  // Serial.println(address);
  if(address == "fc:b4:67:4f:40:80")
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
      throttle = 1100;
      resetall();
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
      // Serial.println("Resume all Tasks"); 
      vTaskResume(Task1);
      vTaskResume(Task2);
      Alert("SENSORS-ENABLED");
      break;
      case 'D':
      Alert("Calibration");
      vTaskResume(Task4);
      break;
      case 'E':
      Limit = false;
      allstop();
      vTaskSuspend(Task1);
      vTaskSuspend(Task2);
      vTaskSuspend(Task4);
      break;
      case 'F':
      dex = deli_package.A.indexOf("F");
      height = deli_package.A.substring(1,strlength-1);
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
      Alert(Data);
      break;
      case 'T':
      {
      throttle = deli_package.A.substring(1,strlength-1).toDouble();
        // double* Out = Defuzzy();
        // pid_Roll.SetTunings(Out[0],Out[1],Out[2]);
        // free(Out);
      Alert(Data);
      }
      break;
      case 'Y':
      {
        takeoff = true;
        takeoff_set = deli_package.A.substring(1,strlength-1).toDouble();
        Serial.println(takeoff_set);
      }
      break;
      case 'L':
        takedown = true;
      break;
      case 'H':
        setpoint_5 = Px;
        setpoint_6 = Py;
        holdpos = true;
        break;
      case 'U':
      pid_Roll.SetTunings(5,20,1.3);
      pid_Pitch.SetTunings(6,15,1.3);
      pid_Yaw.SetTunings(3.5,0.5,0);
      break;
      default:
      Alert("Wrong code");
      break;
    }
  }
  else
  {
    memcpy(&Distance, incomingData, sizeof(Distance));
  }
  
 
}

//UWB cal
double positionY_3D()
{
  Py = (pow(Distance.d1+D1_addon,2)+pow(O1O2,2)-pow(Distance.d2+D2_addon,2))/(2*O1O2);
  return Py;
}
double positionX_3D()
{
  Px =  (pow(Distance.d2+D2_addon,2)+pow(O2O3,2)-pow(Distance.d3+D3_addon,2))/(2*O2O3);
  return Px;
}
double altitude_3D()
{
  return sqrt(pow(Distance.d1,2)-pow(Px,2)-pow(Py,2));
}

double positionY()
{
  OO_1 = (true_height + 11);
  Oh = sqrt(pow(Distance.d1,2)-pow(OO_1,2)); // d1^2 > OO_1^2
  double Ah = sqrt(pow(Distance.d2,2)-pow(OO_1,2)); // d2^2 > OO_1^2
  double cos_alpha = (pow(OA,2)+pow(Oh,2)-pow(Ah,2))/(2*OA*Oh);
  Py = cos_alpha*Oh;
  return Py;
}
double positionX()
{
  Px = sqrt(pow(Oh,2)- pow(Py,2));
  return Px;
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
        case 'S':
        setpoint_4 = deli_package.A.substring(3,strlength).toInt();
        Serial.println(setpoint_4);
        break;
        case 'X':
        setpoint_5 = deli_package.A.substring(3,strlength).toDouble();
        Serial.println(setpoint_5);
        break;
        case 'Y':
        setpoint_6 = deli_package.A.substring(3,strlength).toDouble();
        Serial.println(setpoint_6);
        break;
        case 'B':
        D1_addon = deli_package.A.substring(3,strlength).toDouble();
        Serial.println(D1_addon);
        case 'N':
        D2_addon = deli_package.A.substring(3,strlength).toDouble();
        Serial.println(D2_addon);
        break;
        case 'M':
        D3_addon = deli_package.A.substring(3,strlength).toDouble();
        Serial.println(D3_addon);
        break;
        default:
        Serial.println("Wrong code");
        break;
      }
      deli_package.A.toCharArray(Command,strlength);
      break;
    }
    case 'A':
    {
      char check = deli_package.A.charAt(2);
      switch(check)
      { 
        case 'X':
        Px_error = deli_package.A.substring(3,strlength).toDouble();
        break;
        case 'Y':
        Py_error = deli_package.A.substring(3,strlength).toDouble();
        break;
        default:
        break;
      }
    }
    default:
    Alert("Wrong Code");
    break;
  }
}
void changevalPD(char code, char cd)
{
  if (code == '2') // Roll
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
  else if (code == '3') // Pitch
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
  else if (code == '1') // Thrust
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
  else if (code == '4') //Yaw
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
  else if (code == '5')
  {
    rev_Pid = deli_package.A.substring(2,strlength);
    switch(cd)
    {
      case 'P':
      Pid_1[4].Kp = rev_Pid.toDouble();
      pid_X.SetTunings(Pid_1[4].Kp,Pid_1[4].Ki,Pid_1[4].Kd);
      Serial.println(pid_X.GetKp());
      break;
      case 'I':
      Pid_1[4].Ki = rev_Pid.toDouble();
      pid_X.SetTunings(Pid_1[4].Kp,Pid_1[4].Ki,Pid_1[4].Kd);
      Serial.println(pid_X.GetKi());
      break;
      case 'K':
      Pid_1[4].Kd = rev_Pid.toDouble();
      pid_X.SetTunings(Pid_1[4].Kp,Pid_1[4].Ki,Pid_1[4].Kd);
      Serial.println(pid_X.GetKd());
      break;
      default:
      break;
    }    
  }
  else if (code == '6')
  {
    rev_Pid = deli_package.A.substring(2,strlength);
    switch(cd)
    {
      case 'P':
      Pid_1[5].Kp = rev_Pid.toDouble();
      pid_Y.SetTunings(Pid_1[5].Kp,Pid_1[5].Ki,Pid_1[5].Kd);
      Serial.println(pid_Y.GetKp());
      break;
      case 'I':
      Pid_1[5].Ki = rev_Pid.toDouble();
      pid_Y.SetTunings(Pid_1[5].Kp,Pid_1[5].Ki,Pid_1[5].Kd);
      Serial.println(pid_Y.GetKi());
      break;
      case 'K':
      Pid_1[5].Kd = rev_Pid.toDouble();
      pid_Y.SetTunings(Pid_1[5].Kp,Pid_1[5].Ki,Pid_1[5].Kd);
      Serial.println(pid_Y.GetKd());
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
  // true_height =  pressureKalmanFilter.updateEstimate(distanceSensor.measureDistanceCm());
  Alitudez.add(distanceSensor.measureDistanceCm());
  true_height = Alitudez.getMedian();
  if (!dmpReady) return;
  if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { 
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);   
  }
  positionY_3D();
  positionX_3D();
}

void pid_bldc()
{
  if(Limit==true)
  {
    input_1 = true_height;
    input_2 = ypr[1]* 180/M_PI; //this is 100% correct // roll
    input_3 = ypr[2]* 180/M_PI; //pitch 
    input_4 = ypr[0]* 180/M_PI; //yaw
    input_5 = Px;
    input_6 = Py;
    pid_Thrust.Compute();
    pid_Roll.Compute();
    pid_Pitch.Compute();
    pid_Yaw.Compute();
    pid_error_gain_altitude = 0;
    pid_altitude_Kp_gain(setpoint_1-true_height);
    if (holdpos==true)
    {
    pid_X.Compute();
    pid_Y.Compute();
    setpoint_3 = out_Yd;
    setpoint_2 = out_Xd;
    }
    motor_2 = throttle + out_Thrust - out_Roll + out_Pitch - out_Yaw; //front left (CW)
    motor_3 = throttle + out_Thrust - out_Roll /*========*/+ out_Yaw; //rear left (CCW)
    motor_4 = throttle + out_Thrust - out_Roll - out_Pitch - out_Yaw; //back left (CW)
    motor_5 = throttle + out_Thrust + out_Roll - out_Pitch + out_Yaw; // back right (CCW)
    motor_6 = throttle + out_Thrust + out_Roll /*======*/  - out_Yaw; //rear right (CW)
    motor_1 = throttle + out_Thrust + out_Roll + out_Pitch + out_Yaw; //front right (CCW)

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
    delay(2);
  }
  A3Raw = A3Raw/5;
  A3ADC = A3Raw;
  cell_1 = A3Raw * 3.29/1023;
}
double* Defuzzy()
{
  for(int i=0;i<nMember;i++)
  {
    nuy[i]=Nuy[i].Priority();
  }
  return suge.defuzzication(nuy, Roll_Out, 8, 3);
}
void dmp_config()
{
    Serial.println(F("\nBegin DMP: "));
    delay(2000);
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = accelgyro.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    accelgyro.setXAccelOffset(-3376);
    accelgyro.setYAccelOffset(2066);
    accelgyro.setZAccelOffset(3437);
    accelgyro.setXGyroOffset(38);
    accelgyro.setYGyroOffset(164);
    accelgyro.setZGyroOffset(47); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        // accelgyro.CalibrateAccel(6);
        // accelgyro.CalibrateGyro(6);
        // accelgyro.Pr intActiveOffsets();
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
void pid_altitude_Kp_gain(double error)
{
  if(error > 10 || error <-10)
  {
    pid_error_gain_altitude = (abs(error)-10)/20;
    if(pid_error_gain_altitude >3 ) pid_error_gain_altitude=3;
    pid_Thrust.SetTunings(pid_error_gain_altitude,Pid_1[0].Ki,Pid_1[0].Kd);
  }
 
}
void loop() 
{
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void landing()
{
    if(throttle>1100)
  {
    throttle--;
    delay(10);
  }
  else 
  {
    takedown= false;
  }
}
void lifting()
{
  if (throttle < takeoff_set)
  {
    throttle ++;
    delay(30);
  }
  else
  {
    takeoff = false;
    // double* Out = Defuzzy();resetall
    // pid_Roll.SetTunings(Out[0],Out[1],Out[2]);
  }
}
void resetall()
{
  takeoff = false;
  takedown = false;
  holdpos = false;
  setpoint_3 = 0;
  setpoint_2 = 0;
  
}