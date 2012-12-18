#include "DofData.h"

#ifndef DofHandler_h
#define DofHandler_h

#define DOF_DATA_DEFAULT_INTERVAL 70 // Default data interval
#define DOF_DATA_DEFAULT_CONTINUOUS false
#define DOF_DATA_SIZE 30 // Packet's max data size is 30
#define DOF_GYRO_SCALE (0.00390625) // Factor to scale gyro data by (1 / 256)

#define DOF_DATA_MODE_ALL 0 // Send all sensor data (binary)
#define DOF_DATA_MODE_GYRO 1 // Send Gyro data
#define DOF_DATA_MODE_EULER 2 // Send "Euler" angles
#define DOF_DATA_MODE_DEFAULT DOF_DATA_MODE_ALL

const byte DOF_DATA_MODE_SIZE[] = {30, 6, 12};

/**
 * DofHandler is designed to handle communications between a 9Degrees of Freedom board
 * and the Arduino. In order to support HardwareSerial (Serial, Serial1, Serial2, Serial3)
 * and SoftwareSerial, this must be a generic template class, as the base class of
 * HardwareSerial and SoftwareSerial (Stream) does not have the begin() and end() methods
 * needed for this class (ikr?).
 * 
 * This class uses approximately 4.5 KB of memory on the Arduino.
 */
template <class StreamType> class DofHandler {
  public:
    /**
     * Constructs a DofHandler.
     * 
     * @param dofStream Pointer to the Stream used for communication with the 9DoF
     * @param baud Baud rate that the stream was opened at, if it was at all. Optional.
     */
    DofHandler(StreamType *dofStream, int baud = 0);
    
    /**
     * Begins connection with the 9DoF. Does nothing if the stream has already been opened.
     * 
     * If changing the baud rate to something other than the inital rate, pass that in as well.
     * Warning: this method will block for approximately 210 milliseconds if finalBaud is passed.
     * 
     * @param initialBaud The baud rate to first create the connection at.
     * @param baud If changing the baud rate, pass in the new baud rate here. Optional.
     */
    void begin(int initialBaud, int finalBaud = 0);
    
    /**
     * Closes the stream's connections (calls end() on the stream).
     */
    void end();
	
    /**
     * Marks the DofHandler as open. This allows the stream to be opened
     * after it has been passed into the DofHandler, but without needing to open
     * it with DofHandler.begin().
     *
     * @param baud the baud rate that the stream is set to.
     */
    void markOpen(int baud);
    
    /**
     * Runs the code to check incoming stream data. Run this in the loop() function.
     * 
     * @param loop Optional. If true, will loop through the check code until either a packet is found,
     *   or there are no more bytes available in the stream.
     * 
     * @return true if a packet was received (good or bad), false otherwise.
     */
    boolean checkStream(boolean loop = false);
    
    /**
     * If there is a character available to be read from the 9DoF stream, this echos that character.
     * 
     * @param out Stream to echo character to.
     */
    void debugRead(Stream &out);
    
	/**
	 * Gets the baud rate of the stream, as known to the DofHandler.
	 * 
	 * @return the baud rate being used by the stream
	 */
    int getBaudRate() { return baudRate; }
    
    /**
     * Sets the baud rate of the connection. By default, it also (attempts) to tell the 9DoF to change
     * its baud rate as well. This method may block for up to approximately 110 milliseconds.
     * 
     * @param newBaud The baud rate to change to.
     * @param internal Optional. If true, the 9DoF will not be told to change its baud rate as well. Defaults to false.
     */
    void setBaudRate(int newBaud, boolean internal = false);
    
    /**
     * Gets the update interval as known by the DofHandler
     *
     * @return the known update interval
     */
    short getUpdateInterval() { return updateInterval; }
    
    /**
     * Tells the 9DoF to change its update interval.
     * 
     * @param newBaud The new update interval.
     */
    void setUpdateInterval(short interval);
    
    boolean isContinuousStream() { return continuousStream; }
    
    /**
     * Enables or disables continuous data streaming from the 9DoF.
     */
    void setContinuousStream(boolean enable);
    void enableContinuousStream() { setContinuousStream(true); }
    void disableContinuousStream() { setContinuousStream(false); }
    
    void setDataMode(byte mode, boolean force = false);
	
    byte getDataMode() { return dataMode; }
    byte getLastDataMode() { return lastPacketMode; }
	
    
    void zeroCalibrate();
    
    /**
     * Requests a single data frame from the 9DoF.
     * If the time since the last frame and this request is less than the update interval,
     * The frame will not be sent until the update interval passes.
     */
    void requestData();
    
    /**
     * Gets the most recent sensor data. Clears the newData flag.
     *
     * @return the most recent sensor data.
     */
    DofData getData() { newData = false; return data; }
    
    /**
     * Gets the most recent euler angles data (yaw, pitch, roll). Clears the newData flag.
     *
     * @return the most recent euler angle data
     */
      EulerData getEulerData() { newData = false; return eulerData; }
    
    /**
     * Gets the most recent gyroscope data. Clears the newData flag.
     *
     * @return the most recent gyroscope data
     */
    GyroData getGyroData() { newData = false; return gyroData; }
    
    /**
     * Returns the newData flag.
     */
    boolean isNewDataAvailable() { return newData; }
    
    /**
     * Gets the age (in milliseconds) of the last good data frame.
     */
    unsigned long getDataAge() { return millis() - dataTime; }
    
    /**
     * Returns true if the last received packet was valid; false otherwise.
     */
    boolean isPacketGood() { return lastPacketGood; }
    
    /**
     * Prints out the sensor data to the passed in stream.
     */
    void printData(Stream &out);
  
  private:
    StreamType *stream;
    boolean open; // Has stream->begin been called?
    int baudRate; // Baud rate of stream
    
    // Converts the baud rate to an ID used to configure baud of 9DoF remotely.
    int baudRateToId(int rate);
    boolean _checkStream(); // Private version of checkStream(boolean).
    void readPacket(); // Reads the packet data stored in the buffer
    void clearBuffer(); // Clears packet data buffer and resets state
    void read_double(byte startIndex, double &out); // Read a double from a set of bytes
    void read_short(byte startIndex, double &out); // Read a short from a set of bytes
    byte packetState; // The packet's state (Finite state machine)
    byte dataBuffer[DOF_DATA_SIZE]; // Buffer for packet data
    byte dataBufferSize; // Amount of data stored in the buffer
    byte dataModeSize; // Size of the current mode's packet data
    byte dataMode;
    byte lastPacketMode;
    
    short updateInterval; // Number of milliseconds between updates sent by 9DoF
    boolean continuousStream; // True if the 9DoF is configured to send a continous stream, false otherwise
    
    DofData data; // Holds the data retrieved from the 9DoF
    EulerData eulerData;
    GyroData gyroData;
    unsigned long dataTime; // Stores the time that the data was read (millis())
    boolean newData;
    
    
    // Statistics
    short goodCount; // Number of good data frames retreived.
    short badCount; // Number of bad data frames retreived.
    // True if the last incoming packet was good (goodCount was incremented); false otherwise
    boolean lastPacketGood;
    
    
  
};


// Implementation code required in header file to take care of template instantiation.

template <class StreamType>
DofHandler<StreamType>::DofHandler(StreamType *dofStream, int baud) {
  stream = dofStream;
  
  // Check optional parameter presence
  if (baud > 0) {
    baudRate = baud;
    open = true;
  } else {
    open = false;
    baudRate = 0;
  }
  // Initialize data members
  updateInterval = -1;
  continuousStream = false;
  lastPacketGood = false;
  dataModeSize = DOF_DATA_MODE_SIZE[DOF_DATA_MODE_DEFAULT];
  dataMode = DOF_DATA_MODE_DEFAULT;
}

template <class StreamType>
void DofHandler<StreamType>::begin(int initalBaud, int baud) {
  if (open) return; // If the stream is already open, don't begin it again.
  
  stream->begin(initalBaud);
  
  if (baud != 0 && baud != initalBaud) {
    int baudId = baudRateToId(baud);
    // Only change the baud rate if the rate is supported
    if (baudId >= 0) {
      delay(100);
      stream->print("#b");
      stream->print(baudId);
      stream->flush();
      stream->end();
      delay(100);
      stream->begin(baud);
      delay(10);
    } else {
      baud = initalBaud;
    }
  } else {
    baud = initalBaud;
  }
  
  baudRate = baud;
  open = true;
  
}

template <class StreamType>
void DofHandler<StreamType>::end() {
  if (!open)
    return;
  
  stream->end();
  open = false;
}

template <class StreamType>
void DofHandler<StreamType>::markOpen(int baud) {
  open = true;
  baudRate = baud;
}

template <class StreamType>
boolean DofHandler<StreamType>::checkStream(boolean loop) {
  // If loop is true, run _checkStream within a while loop, otherwise, at max once.
  if (loop) {
    while (stream->available()) {
      if (_checkStream()) {
        return true;
      }
    }
  } else {
    if (stream->available()) {
      return _checkStream();
    }
  }
  
  return false;
}

template <class StreamType>
boolean DofHandler<StreamType>::_checkStream() {
  byte in = (byte)stream->read();
  boolean packet = false;
  // Finite state machine to read beginning "9DoF" magic number
  // at the start of a packet (for alignment purposes)
  switch (packetState) {
    case 0:
      if (in == '9') {
        packetState = 1;
      }
      break;
    case 1:
      if (in == 'D')
        packetState = 2;
      else
        packetState = 0;
      break;
    case 2:
      if (in == 'o')
        packetState = 3;
      else
        packetState = 0;
        
      break;
    case 3:
      if (in == 'F')
        packetState = 4;
      else
        packetState = 0;
      break;
    default:
      if (dataBufferSize >= dataModeSize) {
        // The next byte should be an end line character
        if (in == '\n') {
          // Good 
          readPacket();
          lastPacketGood = true;
          // Statistics collecting to check both average age of data,
          // And for seeing how old current data is
          goodCount++;
          dataTime = millis();
          /*Serial.print("Avg "); // The average time between new data
          Serial.println(dofDataTime / dof_good_count);
          Serial.print((double)dof_bad_count / dof_good_count);
          Serial.print(" (");
          Serial.print(dof_bad_count);
          Serial.println(")");*/
          //printDofData();
        } else {
          // Bad line
          //Serial.println("Bad");
          badCount++;
          lastPacketGood = false;
        }
        packet = true;
        newData = true;
        clearBuffer();
      } else {
        dataBuffer[dataBufferSize] = in;
        dataBufferSize++;
        //dof_data_buffer[dof_data_buffer_size] = 0;
      }
      break;
  }
  return packet;
}

template <class StreamType>
void DofHandler<StreamType>::readPacket() {
  // Format:
  // MMMMAAAABBBBCCCCIIIIJJJJKKKKXXYYZZN (35 bytes long; 30 bytes of data)
  // Where MMMM is the magic number "9DoF" (no null terminator),
  // AAAA, BBBB, and CCCC are the X, Y and Z values (respectively) of the accelerometer
  // IIII, JJJJ, and KKKK are the X, Y and Z values (respectively) of the magnetometer
  // XX, YY, and ZZ are the X, Y and Z values (respectively) of the gyroscope
  // N is a new line character (\n)
  // MMMM and N are stripped as data is coming in.
  
  // Packet size is already 36 bytes long
  //DofData data;
  //EulerData euler;
  
  if (dataMode == DOF_DATA_MODE_ALL) {
    read_double(0, data.accelX);
    read_double(4, data.accelY);
    read_double(8, data.accelZ);
    
    read_double(12, data.magX);
    read_double(16, data.magY);
    read_double(20, data.magZ);
    
    read_short(24, data.gyroX);
    read_short(26, data.gyroY);
    read_short(28, data.gyroZ);
    
    data.gyroX *= DOF_GYRO_SCALE;
    data.gyroY *= DOF_GYRO_SCALE;
    data.gyroZ *= DOF_GYRO_SCALE;
    
    gyroData.x = data.gyroX;
    gyroData.y = data.gyroY;
    gyroData.z = data.gyroZ;
    
    gyroData.checkSum = (gyroData.x + gyroData.y + gyroData.z) % 10;
    
    lastPacketMode = DOF_DATA_MODE_ALL;
  } else if (dataMode == DOF_DATA_MODE_GYRO) {
    read_short(0, data.gyroX);
    read_short(2, data.gyroY);
    read_short(4, data.gyroZ);
    
    data.gyroX *= DOF_GYRO_SCALE;
    data.gyroY *= DOF_GYRO_SCALE;
    data.gyroZ *= DOF_GYRO_SCALE;
    
    gyroData.x = data.gyroX;
    gyroData.y = data.gyroY;
    gyroData.z = data.gyroZ;
    
    gyroData.checkSum = (gyroData.x + gyroData.y + gyroData.z) % 10;
    
    lastPacketMode = DOF_DATA_MODE_GYRO;
  } else if (dataMode == DOF_DATA_MODE_EULER) {
    read_double(0, eulerData.roll);
    read_double(4, eulerData.pitch);
    read_double(8, eulerData.yaw);
    
    lastPacketMode = DOF_DATA_MODE_EULER;
  }
  //this->data = data;
}

template <class StreamType>
void DofHandler<StreamType>::read_double(byte startIndex, double &out) {
  // Dirty casting and shifting magic, undoing what was done on the 9Dof
  long val = 0;
  val |= dataBuffer[startIndex]; val <<= 8;
  val |= dataBuffer[startIndex + 1]; val <<= 8;
  val |= dataBuffer[startIndex + 2]; val <<= 8;
  val |= dataBuffer[startIndex + 3];
  out = *((double *)&val);
}

template <class StreamType>
void DofHandler<StreamType>::read_short(byte startIndex, double &out) {
  short val = 0;
  val |= dataBuffer[startIndex]; val <<= 8;
  val |= dataBuffer[startIndex + 1];
  out = val;
}

template <class StreamType>
void DofHandler<StreamType>::clearBuffer() {
  dataBuffer[0] = 0;
  dataBufferSize = 0;
  packetState = 0;
}

template <class StreamType>
void DofHandler<StreamType>::printData(Stream &out) {
  //out.println("\n9DoF Data:");
  
  if (lastPacketMode == DOF_DATA_MODE_ALL) {
    out.print("(A){ { ");
    out.print(data.accelX);
    out.print(", ");
    out.print(data.accelY);
    out.print(", ");
    out.print(data.accelZ);
    out.print(" }, { ");
    out.print(data.magX);
    out.print(", ");
    out.print(data.magY);
    out.print(", ");
    out.print(data.magZ);
    out.print(" }, { ");
    out.print(data.gyroX);
    out.print(", ");
    out.print(data.gyroY);
    out.print(", ");
    out.print(data.gyroZ);
    out.println(" } }");
  } else if (lastPacketMode == DOF_DATA_MODE_GYRO) {
    out.print("(G){ ");
    out.print(data.gyroX);
    out.print(", ");
    out.print(data.gyroY);
    out.print(", ");
    out.print(data.gyroZ);
    out.println(" }");
  } else if (lastPacketMode == DOF_DATA_MODE_EULER) {
    out.print("(E){ ");
    out.print(eulerData.yaw);
    out.print(", ");
    out.print(eulerData.pitch);
    out.print(", ");
    out.print(eulerData.roll);
    out.println(" }");
  }
  
}

template <class StreamType>
void DofHandler<StreamType>::debugRead(Stream &out) {
  if (stream->available()) {
    byte in = (byte)stream->read();
    out.write(in);
  }
}

template <class StreamType>
void DofHandler<StreamType>::setBaudRate(int rate, boolean internal) {
  if (!open) return;
  
  int baudId = baudRateToId(rate);
  if (baudId < 0) {
    return;
  }
  
  if (!internal) {
    stream->print("#b");
    stream->print(baudId);
  }
  
  stream->flush(); // SoftwareSerial does not flush() when end() is called.
  stream->end();
  delay(internal ? 10 : 100);
  
  stream->begin(rate);
  delay(10);
  
  baudRate = rate;
}

template <class StreamType>
void DofHandler<StreamType>::setUpdateInterval(short interval) {
  stream->print("#i");
  stream->write(interval >> 8);
  stream->write(interval);
  updateInterval = interval;
}

template <class StreamType>
void DofHandler<StreamType>::setContinuousStream(boolean continuous) {
  if (continuous) {
    stream->println("#o1");
  } else {
    stream->println("#o0");
  }
}

template <class StreamType>
void DofHandler<StreamType>::requestData() {
  stream->print("#f"); // Request _f_rame
}

template <class StreamType>
int DofHandler<StreamType>::baudRateToId(int rate) {
/*
Baud ID values:
  1 -> 2400 baud
  2 -> 4800
  3 -> 9600 baud (default)
  4 -> 14400 baud
  5 -> 19200 baud
  6 -> 28800 baud (recommended)
  7 -> 38400 baud
  8 -> 57600 baud
  9 -> 115200 baud
  
Baud rates above 28800 do not seem to work with sofware serial.
*/

  switch (rate) {
    case 2400:
      return 1;
    case 4800:
      return 2;
    case 9600:
      return 3;
    case 14400:
      return 4;
    case 19200:
      return 5;
    case 28800:
      return 6;
    case 38400:
      return 7;
    case 57600:
      return 8;
    case 115200:
      return 9;
  }
  return -1;
}

template <class StreamType>
void DofHandler<StreamType>::setDataMode(byte mode, boolean force) {
  switch (mode) {
    case DOF_DATA_MODE_ALL:
    case DOF_DATA_MODE_GYRO:
    case DOF_DATA_MODE_EULER:
      break;
    default:
      mode = DOF_DATA_MODE_DEFAULT;
  }
  
  clearBuffer();
  if (force || dataMode != mode) {
    stream->print("#m");
    stream->write(mode);
  }
  
  dataModeSize = DOF_DATA_MODE_SIZE[mode];
  dataMode = mode;
  
}

template <class StreamType>
void DofHandler<StreamType>::zeroCalibrate() {
  stream->print("#z"); // _z_ero calibrate
}

#endif
