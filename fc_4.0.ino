/*****************************************************************************
*                                                                            *
*   Lorenzo Santos Raymundo & Caio Joaquim da Cruz Mariani                   *
*   ROCKET FLIGHT COMPUTER VERSION 4.0 - FINAL COURSE PROJECT AT FETLSVC     *
*                                                                            *
*   This software was meant to run on an ESP32 inside a watter-bottle rocket *
*   It can do math, record/send flight data and deploy a parachute!          *
*                                                                            *
*   You can find this code at <https://github.com/LorenzoRaymundo>           *
*                                                                            *
******************************************************************************/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h> // INSTALL VERSION 3.7
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define PI 3.1415926535897932384626433832795
#define BMP280 0x76
#define MPU6050 0x68
#define SPEED 1
#define CAL_TIME 1
#define FILTER_SIZE 10
#define ACC_G 9.80665
#define ACC_DEV 20.0   // acc standard deviation  [cm/s²]
#define BARO_DEV 30.0  // baro standard deviation [cm]
#define GPS_BAUD 9600

#define BUZZER_PIN 15
#define SERVO_PIN 4
#define RXD2 16
#define TXD2 17
#define LED_PIN 36

char *altitude_file = "/altitude.txt";            // kalman, baro
char *acc_readings = "/acc.txt";                  // ax ay az azi;
char *orientation_readings = "/orientation.txt";  // pitch roll
char *velocity_file = "/vertical_velocity.txt";

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t recieverAddress[] = { 0x94, 0x54, 0xC5, 0x76, 0xE2, 0xD4 };  // 94:54:C5:76:E2:D4

typedef struct mpu {
  float ang_roll;
  float ang_pitch;
  float ax;
  float ay;
  float az;
  float az_i;
  float gx;
  float gy;
  float gz;
  float vertical_velocity;
} MPU6050_struct;

typedef struct bmp {
  float altitude;
  float altitude_filtered;
  float apogee;
} BMP280_struct;

typedef struct gps {
  double lat;
  double lng;
  double speed;
  int sat;
  double alt;
} GPS_struct;

typedef struct {
  MPU6050_struct mpu;
  BMP280_struct bmp;
  GPS_struct gps;
} Telemetry_t;


GPS_struct gps_data;
MPU6050_struct mpu_data;
BMP280_struct bmp_data;

esp_now_peer_info_t peerInfo;


// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

Servo servo;

void sendUBX(const uint8_t *msg, uint8_t len) {
  gpsSerial.write(msg, len);
}


float vertical_velocity;
float acc_z_inercial;
float dt = 0.001;
float loopTimer;
unsigned long lastTime;  // for calculating dt

bool apogeuTravado = false;
float altitude_max;
int seconds = 1;

float roll, pitch, yaw;
float roll_calibration, pitch_calibration, yaw_calibration;
float accX, accY, accZ;
float angle_roll, angle_pitch;

// pressure sensor calibration values
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;

// altitude variabels
float altitude_barometer, altitude_barometer_startup;

// moving avarge simpllle filter
float altitude_buffer[FILTER_SIZE];
int buffer_index = 0;
bool buffer_filled = false;

float moving_average(float new_value) {
  altitude_buffer[buffer_index] = new_value;
  buffer_index = (buffer_index + 1) % FILTER_SIZE;

  float sum = 0;
  int count = buffer_filled ? FILTER_SIZE : buffer_index;
  for (int i = 0; i < count; i++) {
    sum += altitude_buffer[i];
  }
  if (buffer_index == 0) buffer_filled = true;
  return sum / count;
}

// SD CARD STANDARD FUNCTIONS
void initSDCard() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}


void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
  file.close();
}


void tocarSomESC() {
  tone(BUZZER_PIN, 1000, 150);
  delay(250);
  tone(BUZZER_PIN, 1500, 150);
  delay(250);
  tone(BUZZER_PIN, 2000, 400);

  for (int f = 1000; f <= 2000; f += 100)
    tone(BUZZER_PIN, f, 20);
  for (int f = 2000; f >= 1000; f -= 100)
    tone(BUZZER_PIN, f, 20);

  noTone(BUZZER_PIN);
}

// KALMAN FILTER VARIABLES
float altitude_kalman, vertical_velocity_kalman;

BLA::Matrix<2, 2> A;    // State transition matrix
BLA::Matrix<2, 1> B;    // Control input matrix
BLA::Matrix<2, 2> P;    // Estimation error covariance
BLA::Matrix<2, 2> Q;    // Process noise covariance
BLA::Matrix<2, 1> x_k;  // State vector [position; velocity]
BLA::Matrix<1, 2> H;    // Observation matrix
BLA::Matrix<2, 2> I;    // Identity matrix
BLA::Matrix<1, 1> z_k;  // Measurement (from the barometer)
BLA::Matrix<2, 1> K;    // Kalman gain
BLA::Matrix<1, 1> R;    // Measurement noise covariance
BLA::Matrix<1, 1> y_k;  // Innovation (residual)
BLA::Matrix<1, 1> S_k;  // Innovation covariance

void kalman_2d(float altitude_baro, float acc_z_inertial) {
  BLA::Matrix<1, 1> u_k = { acc_z_inertial };

  // predictiont
  x_k = A * x_k + B * u_k;  // defines the statespace matrix
  P = A * P * ~A + Q;       // actual uncertainty of the prediction based on previous state prediction

  // correction
  z_k = { altitude_baro };  // measurement

  S_k = H * P * ~H + R;             // calculates the estimated error
  K = P * ~H * Invert(S_k);         // kalman gain
  x_k = x_k + K * (z_k - H * x_k);  // actual estimate
  P = (I - K * H) * P;              // updates the uncertainty of the predicted state based on the gain

  // final result based on the acutal state estimation
  altitude_kalman = x_k(0, 0);
  vertical_velocity_kalman = x_k(1, 0);
}


// Kalman filter variables 
float Q_angle = 0.006;   // Process noise variance for the angle (how much we trust the angle model)
float Q_bias = 0.003;    // Process noise variance for the gyroscope bias (how much the bias can vary)
float R_measure = 0.03;  // Measurement noise variance of the accelerometer (sensor reliability)

float angleRollKF = 0;                                // Kalman-estimated angle (roll)
float biasRoll = 0;                                   // Estimated gyroscope bias (roll)
float P_roll[2][2] = { { 0.1, 0.1 }, { 0.1, 0.1 } };  // State covariance matrix (confidence in angle and bias)

float anglePitchKF = 0;  // Kalman-estimated angle (pitch)
float biasPitch = 0;
float P_pitch[2][2] = { { 0.1, 0.1 }, { 0.1, 0.1 } };


// Kalman filter update function
float kalmanUpdate(float newAngle, float newRate, float dt,
                   float &angle, float &bias, float P[2][2]) {

  // ------------------ PREDICTION ------------------

  // Compute the true rate by subtracting the gyroscope bias
  float rate = newRate - bias;

  // Update the predicted angle using the true rate and elapsed time
  angle += dt * rate;

  // ------------------ COVARIANCE PREDICTION ------------------
  // Update matrix P = F * P * F^T + Q_d
  // Each term comes from the expanded matrix multiplication
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);  // P00
  P[0][1] -= dt * P[1][1];                                       // P01
  P[1][0] -= dt * P[1][1];                                       // P10
  P[1][1] += Q_bias * dt;                                        // P11

  // ------------------ MEASUREMENT UPDATE ------------------

  // Compute measurement uncertainty (S = H * P * H^T + R)
  float S = P[0][0] + R_measure;

  // Compute the Kalman gain (K = P * H^T / S)
  float K[2];
  K[0] = P[0][0] / S;  // how much to correct the angle
  K[1] = P[1][0] / S;  // how much to correct the bias

  // Compute the innovation (measurement residual)
  float y = newAngle - angle;

  // Correct the angle and bias using the Kalman gain
  angle += K[0] * y;
  bias  += K[1] * y;

  // Store previous covariance values before update
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  // Update the covariance matrix after correction: P = (I - K*H) * P
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  // Return the corrected angle
  return angle;
}



void BMP_280_init(void) {

  Wire.beginTransmission(BMP280);
  Wire.write(0xF4);
  Wire.write(0x2F);  // ctrl_meas reg
  Wire.endTransmission();

  Wire.beginTransmission(BMP280);
  Wire.write(0xF5);
  Wire.write(0x04);  // config reg -> 0.5ms and a 2x low-pass filter
  Wire.endTransmission();

  // acquaring calibration data
  uint8_t data[24], i = 0;
  Wire.beginTransmission(BMP280);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BMP280, 24);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];

  delay(250);

  int iteraions = CAL_TIME * 1000;
  for (int i = 0; i < iteraions; i++) {
    barometer_signals();
    altitude_barometer_startup += altitude_barometer;
    delay(1);
  }
  altitude_barometer_startup /= (float)iteraions;
  Serial.println("Finished bmp cal!");
}

void barometer_signals(void) {

  Wire.beginTransmission(BMP280);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BMP280, 6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();

  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);  // raw pressure, uncalibrated
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);     // raw temperature

  // compensated temp
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;

  // compensated pressure - datasheet code
  unsigned long int p;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0) {
    return;
  }
  p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((unsigned long int)var1);
  } else {
    p = (p / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));  // pascal

  double pressure = (double)p / 100;                                              // hetopascal
  altitude_barometer = 44330 * (1 - pow(pressure / 1013.25, 1.0 / 5.255)) * 100;  // altitude in hpa -> cm
}

#define MPU6050 0x68  // std i2c sensor address

void MPU6050_init(int speed = SPEED) {
  // wake up the sensor
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up
  Wire.endTransmission();

  // configs the Low Pass Filter
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1A);
  Wire.write(0x05);  // LPF ~10Hz
  Wire.endTransmission();

  // sets up the acc to ±16g
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1C);
  Wire.write(0x18);  //16g
  Wire.endTransmission();

  // configs the gyro (±500°/s or ±1000°/s based on speed)
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1B);
  Wire.write(speed == 0 ? 0x10 : 0x08);
  Wire.endTransmission();
}


void gyro_signals(int speed = SPEED) {

  // acc
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050, 6);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // gyro
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  roll = (float)GyroX / (speed == 0 ? 32.8 : 65.5);  // deg/s
  pitch = (float)GyroY / (speed == 0 ? 32.8 : 65.5);
  yaw = (float)GyroZ / (speed == 0 ? 32.8 : 65.5);

  // acc in g
  accX = (float)AccXLSB / 2048 - 0.01;  // gravity
  accY = (float)AccYLSB / 2048 + 0.01;
  accZ = (float)AccZLSB / 2048 + 0.06;

  // returned angles by gravity vectors 
  angle_roll  =  atan(accY / sqrt(accX * accX + accZ * accZ)) * (1 / (3.142 / 180));
  angle_pitch = -atan(accX / sqrt(accY * accY + accZ * accZ)) * (1 / (3.142 / 180));
}


void calibrate_gyro(int seconds) {
  float roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
  int samples = seconds * 1000;
  for (int i = 0; i < samples; i++) {
    gyro_signals();
    roll_sum += roll;
    pitch_sum += pitch;
    yaw_sum += yaw;
    delay(1);
  }
  roll_calibration = roll_sum / samples;
  pitch_calibration = pitch_sum / samples;
  yaw_calibration = yaw_sum / samples;
}

// callback when data is sent
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // exemplo: acessar o MAC do destinatário
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           info->des_addr[0], info->des_addr[1], info->des_addr[2],
           info->des_addr[3], info->des_addr[4], info->des_addr[5]);

  /*if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.printf("Sucesso ao enviar para %s\n", macStr);
    } else {
        Serial.printf("Falha ao enviar para %s\n", macStr);
    }*/
}
/*
//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Se for pacote de MPU
  if (len == sizeof(uint8_t)) {
        uint8_t btnState = *incomingData;
        bool servo_btn = btnState == 1;
        if (btnState) {
          servo.write(90);
        }
    }
}
*/

double lat;
double lng;
double speed;
double alt;
double hdop;
int sats;

void gps_signals(void) {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
    speed = gps.speed.mps();
    alt = gps.altitude.meters();
    hdop = gps.hdop.value() / 100.0;
    sats = gps.satellites.value();
  }
}

void gps_init(void) {

  /* GPS CONFIG */
  Serial.println("GPS initialized at 9600 baud");
  // configs the response freq to 5 Hz
  uint8_t setRate5Hz[] = {
    0xB5, 0x62,  // Header
    0x06, 0x08,  // CFG-RATE
    0x06, 0x00,  // Length = 6
    0xC8, 0x00,  // measRate = 200 ms (5 Hz)
    0x01, 0x00,  // navRate = 1
    0x01, 0x00,  // timeRef = 1 (GPS time)
    0xDE, 0x6A   // Checksum
  };

  sendUBX(setRate5Hz, sizeof(setRate5Hz));
  Serial.println("GPS AT 5HZ RESPONSE FREQ.");
}


float derivative(float variable, float past_variable, float dt) {
  return (variable - past_variable) / dt;
}


void releaseParachute() {
  int currentPos = servo.read();
  if (currentPos > 80 && currentPos < 100) {
    Serial.println("Released!");
    Serial.println("apogee: ");
    Serial.println(altitude_max);
    servo.write(0);
  }
}

unsigned long startMillis;

void setup() {
  Serial.begin(115200);
  delay(50);

  initSDCard();

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  Serial.println("Creating file...");
  writeFile(SD, altitude_file, "time, altitude Kalman Filter, altitude Barometer \r\n");

  Serial.println("Creating file...");
  writeFile(SD, acc_readings, "time, ax, ay, az \r\n");

  Serial.println("Creating file...");
  writeFile(SD, orientation_readings, "time, roll, pitch \r\n");

  Serial.println("Creating file...");
  writeFile(SD, velocity_file, "time, vertical velocity, Inertial acceleration \r\n");

  servo.attach(SERVO_PIN);
  delay(100);
  servo.write(0);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  gps_init();
  Serial.println("GPS CCONFIGURED!");

  /* ESP32 NOW CONFIG  */
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  /* Register a callback function that is triggered upon sending data.
  When a message is sent, a function is called – this function returns
  whether the delivery was successful or not. */

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, recieverAddress, 6);  // copies the 6-hex adress array to the peerInfo struct
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else
    Serial.println("ESPNOW CCONFIGURED!");

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  BMP_280_init();
  MPU6050_init(SPEED);

  Serial.println("Starting mpu clibration...");
  calibrate_gyro(CAL_TIME);

  tocarSomESC();
  digitalWrite(LED_PIN, HIGH);

  Serial.println("Finished mpu calibration! Reaady to launch.");

  delay(100);
  servo.write(90);

  lastTime = micros();

  H = { 1, 0 };
  I = { 1, 0,
        0, 1 };
  R = { BARO_DEV * BARO_DEV };
  P = { 0, 0, 0, 0 };
  x_k = { 0, 0 };

  loopTimer = micros();
  startMillis = millis();
}

int descent_counter;
unsigned long previousMillis = 0;

void loop() {

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  barometer_signals();
  altitude_barometer -= altitude_barometer_startup;
  float altitude_filtered = moving_average(altitude_barometer);

  gyro_signals();
  roll -= roll_calibration;
  pitch -= pitch_calibration;
  yaw -= yaw_calibration;

  A = { 1, dt, 0, 1 };
  B = { 0.5 * dt * dt, dt };
  Q = B * ~B * ACC_DEV * ACC_DEV;

  // Kalman filter for roll and pitch
  angleRollKF  = kalmanUpdate(angle_roll,   roll, dt, angleRollKF,  biasRoll,  P_roll);
  anglePitchKF = kalmanUpdate(angle_pitch, pitch, dt, anglePitchKF, biasPitch, P_pitch);

  acc_z_inercial = -sin(anglePitchKF * PI / 180) * accX
                   + cos(anglePitchKF * PI / 180) * sin(angleRollKF * PI / 180) * accY
                   + cos(anglePitchKF * PI / 180) * cos(angleRollKF * PI / 180) * accZ;
  acc_z_inercial = (acc_z_inercial - 1.0) * ACC_G * 100;  // cm/s²

  kalman_2d(altitude_barometer, acc_z_inercial);

  //float acc_kalman = derivative(vertical_velocity_kalman, past_vertical_velocity, dt);
  //past_vertical_velocity = vertical_velocity_kalman;

  if (altitude_kalman > altitude_max) {
    altitude_max = altitude_kalman;
  }
  // apogee detection
  if (altitude_max - altitude_barometer >= 200 && altitude_max > 100) {
    releaseParachute();
  }

  gps_signals();

  mpu_data.ang_pitch = anglePitchKF;
  mpu_data.ang_roll = angleRollKF;
  mpu_data.ax = accX;
  mpu_data.ay = accY;
  mpu_data.az = accZ;
  mpu_data.az_i = acc_z_inercial;
  mpu_data.gx = roll;
  mpu_data.gy = pitch;
  mpu_data.gz = yaw;
  mpu_data.vertical_velocity = vertical_velocity_kalman;

  bmp_data.altitude = altitude_barometer;
  bmp_data.altitude_filtered = altitude_kalman;
  bmp_data.apogee = altitude_max;

  gps_data.lat = lat;
  gps_data.lng = lng;
  gps_data.speed = speed;
  gps_data.alt = alt;
  gps_data.sat = sats;

  // sends everything by just one struct of data
  Telemetry_t tel;
  tel.mpu = mpu_data;
  tel.bmp = bmp_data;
  tel.gps = gps_data;

  esp_err_t result = esp_now_send(recieverAddress, (uint8_t *)&tel, sizeof(Telemetry_t));
  if (result != ESP_OK) {
    //Serial.printf("esp_now_send erro: %d\n", result);
  }

  if (altitude_barometer >= 100) {
    // records in every 25 ms
    if (millis() - previousMillis >= 25) {
      previousMillis = millis();

      float t_sec = (millis() - startMillis) / 1000.0;  // real time in seconds

      // altitudes
      String dataMessage = String(t_sec, 3) + "," + String(altitude_kalman, 2) + "," + String(altitude_barometer, 2) + "\r\n";
      Serial.print("Saving altitude data: ");
      Serial.println(dataMessage);
      appendFile(SD, altitude_file, dataMessage.c_str());

      // acelerations
      dataMessage = String(t_sec, 3) + "," + String(accX, 3) + "," + String(accY, 3) + "," + String(accZ, 3) + "\r\n";
      Serial.print("Saving accelerometer data: ");
      Serial.println(dataMessage);
      appendFile(SD, acc_readings, dataMessage.c_str());

      // orientation
      dataMessage = String(t_sec, 3) + "," + String(angleRollKF, 2) + "," + String(anglePitchKF, 2) + "\r\n";
      Serial.print("Saving orientation data: ");
      Serial.println(dataMessage);
      appendFile(SD, orientation_readings, dataMessage.c_str());

      // velocities
      dataMessage = String(t_sec, 3) + "," + String(vertical_velocity_kalman, 2) + "," + String(acc_z_inercial, 2) + "\r\n";
      Serial.print("Saving velocity data: ");
      Serial.println(dataMessage);
      appendFile(SD, velocity_file, dataMessage.c_str());
    }
  }

  delay(5);
}