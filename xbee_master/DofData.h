#ifndef DofData_h
#define DofData_h

struct DofData {
  double accelX;
  double accelY;
  double accelZ;
  double magX;
  double magY;
  double magZ;
  double gyroX;
  double gyroY;
  double gyroZ;
};

struct EulerData {
  double roll;
  double pitch;
  double yaw;
};

struct GyroData {
  int x, y, z;
  char checkSum;
};

#endif
