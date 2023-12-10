#include "FuzzySimple.h"
double Timer; //1000 is enough
int TOP_SPEED = 1500;
int IDLE_SPEED = 950;
//Battery declare
const int adc1 = 36;
const int adc2 = 39;
const int adc3 = 34;
double A3Raw,A2Raw,A1Raw;
double A3ADC, A2ADC, A1ADC;
float cell_1, cell_2, cell_3;
char loader[70];
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// X and Y Position
double Position_X;
double Position_Y;

//Task Handle
TaskHandle_t Task1, Task2, Task3, Task4;

//MS5611 Vars
double ref_pres, altitude_r, filteredval, base_height, true_height;

// MAC Address
uint8_t broadcastAddress[] = { 0xC4, 0xDE, 0xE2, 0x13, 0xC7, 0x78 };

//Structure for data transfer
typedef struct struct_message {
	char a[32];
	float b;
	float c;
	float d;
	double e;
	int f;
	int g;
	int h;
	int i;
	int l;
	int m;
  float n;
  float o;
  float q;
} struct_message;
esp_now_peer_info_t peerInfo;
struct_message myData;

//Data Received
String Data_recv;
char dat;
typedef struct delivery
{
	String A;
} delivery;
delivery deli_package;

typedef struct Position
{
	double X;
  double Y;
} Position;
Position Position_UWB;
typedef struct Dis
{
	double d1;
  double d2;
  double d3;
} Dis;
Dis Distance;
//Other Variables
String Data, height, rev_Pid;
int strlength, dex;
char Command[32];
double throttle = 1100;

typedef struct PID_Val
{
  double Kp;
  double Ki;
  double Kd;
}PID_Val;

PID_Val Pid_1[6] =
{
  {6,0.01,1.5},
  {3.7,0.35,1.4}, //Kd = 10(tested). Increase Kp increase react speed of motor. (7-8 degrees) 3.2,0.35,1.45 , 1400:Kp-4.5,
  {3.7,0.085,1.4}, // 3.7, 0.085, 1.4
  {3,0.03,0},
  {0,0,0},
  {0,0,0}
};
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[32]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
double ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//Fuzzy 
const int nMember = 5;
typedef struct DatA
{
  double left;
  double right;
  double top;
}DatA;
double nuy[nMember]={};
double K_Out[3][nMember]=
{
  {3.4, 3.25, 3.4, 3.1, 4.5},
  {0.35, 0.30, 0.35, 0.35, 0.45},
  {1.4, 1.2, 1.4, 1.45, 1.95},
};
double* K_K = NULL;

char buffer[70];
DatA Data_in[nMember]=
{
  {1200,1250},
  {1200,1300,1250},
  {1250,1350,1300},
  {1300,1400,1350},
  {1350,1400},
};

SimpleFuzzy Nuy[nMember]={
  {1,&throttle,Data_in[0].left,Data_in[0].right},
  {3,&throttle,Data_in[1].left, Data_in[1].right, Data_in[1].top},
  {3,&throttle,Data_in[2].left, Data_in[2].right, Data_in[2].top},
  {3,&throttle,Data_in[3].left, Data_in[3].right, Data_in[3].top},
  {2,&throttle,Data_in[4].left, Data_in[4].right},
};

sugeno suge(3,3);

//UWB
double O1O2 = 1.2;
double O2O3 = 2.7;
double Oh,Px,Py;

