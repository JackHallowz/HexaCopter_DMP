double Timer; //1000 is enough
int TOP_SPEED = 1440;
int IDLE_SPEED = 950;
const int adc1 = 36;
const int adc2 = 39;
const int adc3 = 34;
double A3Raw,A2Raw,A1Raw;
float cell_1, cell_2, cell_3;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter


//Task Handle
TaskHandle_t Task1, Task2, Task3, Task4;

//MS5611 Vars
double ref_pres, altitude_r, filteredval;

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
  double n;
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
PID_Val Pid_1[4] =
{
  {80,20,1},
  {3.2,0.35,1.4}, //Kd = 10(tested). Increase Kp increase react speed of motor. (7-8 degrees) 3.2,0.35,1.45
  {3.7,0.085,1.4}, // 3.7, 0.085, 1.4
  {3,0.03,0},
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





