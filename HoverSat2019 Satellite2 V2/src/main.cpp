//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   HoverSat Satellite2
//Version number:  Ver.1.1
//Date:            2019.06.14
//------------------------------------------------------------------//
 
//This program supports the following boards:
//* M5Stack(Gray version)
 
//Include
//------------------------------------------------------------------//
#include <M5Stack.h>
#include <Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <EEPROM.h>
#include "utility/MPU9250.h"
#include "BluetoothSerial.h"


//Define
//------------------------------------------------------------------//
#define   TIMER_INTERRUPT     10      //  ms
#define   LCD

#define BufferRecords 32

#define HX711_DOUT  2
#define HX711_SCLK  5
#define OUT_VOL     0.0007f
#define LOAD        500.0f

#define NOOFPATTERNS  17

int parameters[NOOFPATTERNS][5] =
{
// Accel, Velocity, Decel, TIme
{ 1600, 200, 200, 200, 14000 },
{ 1600, 100, 200, 200, 14500 },
{ 1600, 400, 200, 200, 13750 },
{ 1600, 600, 200, 200, 13667 },
{ 1600, 800, 200, 200, 13625 },
{ 1600, 1000, 200, 200, 13600 },
{ 1600, 200, 200, 100, 14500 },
{ 1600, 200, 200, 400, 13750 },
{ 1600, 200, 200, 600, 13667 },
{ 1600, 200, 200, 800, 13625 },
{ 1600, 200, 200, 1000, 13600 },
{ 1600, 200, 200, 5000, 13500 },
{ 1600, 200, 200, 200, 14000 },
{ 1600, 5000, 200, 5000, 13020 },
{ 1600, 25, 200, 25, 21000 },
{ 1600, 200, 200, 200, 14000 },
{ 1600, 200, 200, 200, 14000 }
};


//Global
//------------------------------------------------------------------//
int     pattern = 0;
int     tx_pattern = 0;
int     rx_pattern = 0;
int     rx_val = 0;
bool    hover_flag = false;
bool    log_flag = false;
bool    telemetry_flag = false;
int     cnt10 = 0;

unsigned long time_ms;
unsigned long time_buff = 0;
unsigned long time_buff2 = 0;
unsigned char current_time = 0; 
unsigned char old_time = 0;  

BluetoothSerial bts;
String  bts_rx;
char bts_rx_buffer[16];
int bts_index = 0;

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Buffalo-G-0CBA";
//char pass[] = "hh4aexcxesasx";
char ssid[] = "X1Extreme-Hotspot";
char pass[] = "5]6C458w";
//char ssid[] = "Macaw";
//char pass[] = "1234567890";

// Time
char ntpServer[] = "ntp.nict.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;

File file;
String fname_buff;
const char* fname;

typedef struct {
    String          log_time;
    int             log_pattern;
    unsigned long   log_time_ms;
    int             log_tension;
    float           log_ax;
    float           log_ay;
    float           log_az;
    float           log_gz;
} RecordType;

static RecordType buffer[2][BufferRecords];
static volatile int writeBank = 0;
static volatile int bufferIndex[2] = {0, 0};

static const int TSND_121 = 13;
int  TSND_121_ENABLE = 0;

// MPU9250
MPU9250 IMU; 

float accelBiasX = 0;
float accelBiasY = 0;
float accelBiasZ = 0;
float gyroBiasZ = 0;

// HX711
float hx711_offset;
float hx711_data;

// DuctedFan
static const int DuctedFanPin = 15;
unsigned char hover_val = 65;
Servo DuctedFan;

unsigned char patternNo = 0;
unsigned long ex_time = 0;

// Timer Interrupt
volatile int interruptCounter;
volatile int interruptCounterS;
int totalInterruptCounter;
int iTimer10;


hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


//Global
//------------------------------------------------------------------//
void AE_HX711_Init(void);
void AE_HX711_Reset(void);
long AE_HX711_Read(void);
long AE_HX711_Averaging(long adc,char num);
float AE_HX711_getGram(char num);
void IRAM_ATTR onTimer(void);
void Timer_Interrupt( void );
void getTimeFromNTP(void);
void getTime(void);
void writeData(void);
void writeDataInitial(void);
void bluetooth_rx(void);
void bluetooth_tx(void);
void eeprom_write(void);
void eeprom_read(void);
void TSND121( void );

//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin();
  Wire.begin();
  EEPROM.begin(128);
  SD.begin(4, SPI, 24000000, "/sd");
  M5.Lcd.clear();
  M5.Lcd.drawJpgFile(SD, "/Image/Picture.jpg");
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(88, 160);
  M5.Lcd.println("HoverSat");
  M5.Lcd.setCursor(82, 200);
  M5.Lcd.println("Satellite2");
  
  delay(1000);

  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(GREEN ,BLACK);
  M5.Lcd.fillScreen(BLACK);

  bts.begin("M5Stack Satellite2");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }

  //eeprom_read();
  ex_time = parameters[0][4];

  // timeSet
  getTimeFromNTP();
  getTime();
  fname_buff  = "/log/Satellite2_log_"
              +(String)(timeinfo.tm_year + 1900)
              +"_"+(String)(timeinfo.tm_mon + 1)
              +"_"+(String)timeinfo.tm_mday
              +"_"+(String)timeinfo.tm_hour
              +"_"+(String)timeinfo.tm_min
              +".csv";
  fname = fname_buff.c_str();
    
  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer);

  AE_HX711_Init();
  AE_HX711_Reset();
  hx711_offset = AE_HX711_getGram(30); 

  file = SD.open(fname, FILE_APPEND);
  if( !file ) {
    M5.Lcd.setCursor(5, 160);
    M5.Lcd.println("Failed to open sd");
  }

  delay(2000);
  M5.Lcd.fillScreen(BLACK);

  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();

  pinMode(TSND_121, OUTPUT);


  
}


//Main
//------------------------------------------------------------------//
void loop() {

  Timer_Interrupt();
  bluetooth_rx();
  bluetooth_tx();

  int readBank = !writeBank;

  if (bufferIndex[readBank] >= BufferRecords) {
    static RecordType temp[BufferRecords];

    memcpy(temp, buffer[readBank], sizeof(temp));
    bufferIndex[readBank] = 0;
    file = SD.open(fname, FILE_APPEND);
    for (int i = 0; i < BufferRecords; i++) {
        file.print(temp[i].log_time);
        file.print(",");
        file.print(temp[i].log_pattern);
        file.print(",");
        file.print(temp[i].log_time_ms);
        file.print(",");
        file.print(temp[i].log_tension);
        file.print(",");
        file.print(temp[i].log_ax);
        file.print(",");
        file.print(temp[i].log_ay);
        file.print(",");
        file.print(temp[i].log_az);
        file.print(",");
        file.print(temp[i].log_gz);
        file.println(",");
    }
    file.close();
  }

  if( telemetry_flag ) {
    bts.print(time_ms);
    bts.print(", ");
    bts.print(pattern);
    bts.print(", ");
    bts.print(hx711_data);
    bts.print(", ");
    bts.print(IMU.ax);
    bts.print(", ");
    bts.print(IMU.ay);
    bts.print(", ");
    bts.print(IMU.az);
    bts.print(", ");
    bts.println(IMU.gz);
    telemetry_flag = false;
  }

  switch (pattern) {
    case 0:
      break;

    // CountDown
    case 111:    
      if( current_time >= 52 ) {
        time_buff2 = millis();
        pattern = 113;
        hover_flag = true;
        M5.Lcd.clear();
        DuctedFan.attach(DuctedFanPin);
        DuctedFan.write(0);
        break;
      }
      M5.Lcd.setCursor(180, 100);
      M5.Lcd.clear();
      M5.Lcd.println(60 - current_time);
      bts.println( 60 - current_time );
      break;

    case 112:    
      if( current_time < 1 ) {
        pattern = 111;
        break;
      }
      M5.Lcd.setCursor(180, 100);
      M5.Lcd.clear();
      M5.Lcd.println(60 - current_time);
      bts.println( 60 - current_time + 60 );
      break;

    case 113:    
      if( millis() - time_buff2 >= 3000 ) {
        DuctedFan.write(hover_val);
        bts.println(" - Start within 5 seconds -");
        time_buff2 = millis();
        log_flag = true;
        pattern = 122;
        break;
      }    
      M5.Lcd.setCursor(180, 100);
      M5.Lcd.clear();
      M5.Lcd.println(60 - current_time);
      bts.println( 60 - current_time );
      break;

    case 122:   
      if( millis() - time_buff2 >= 3000 ) {
        time_buff2 = millis();
        pattern = 114;
        TSND121();
        bts.println( "\n - Log start -" );
        break;
      }        
      break;

    case 114:   
      if( millis() - time_buff2 >= 5000 ) {
        time_buff = millis();
        pattern = 115;
        //tx_pattern = 11;
        bts.println( "\n - Sequence start -" );
        break;
      }        
      break;

    case 115:
      if( time_ms >= ex_time ) {
        log_flag = false;
        pattern = 0;
        tx_pattern = 0;
        TSND121();
        hover_flag = false;
        M5.Lcd.clear();
        DuctedFan.detach();
        break;
      }
      break;


    
  }


  // Button Control
  M5.update();
  if (M5.BtnA.wasPressed()) {
    hover_flag = !hover_flag;
    if(pattern == 115) {
      log_flag = false;
      pattern = 0;
    }
    // Hover Control
    if(hover_flag) {
      M5.Lcd.clear();
      DuctedFan.attach(DuctedFanPin);
      DuctedFan.write(0);
      delay(3000);
      DuctedFan.write(hover_val);
    } else {
      M5.Lcd.clear();
      DuctedFan.detach();
    } 
  } else if (M5.BtnB.wasPressed() && pattern == 0) {     
    patternNo++;
    M5.Lcd.fillScreen(BLACK);
    if( patternNo >= NOOFPATTERNS ) {
      patternNo = 0;
    }
    ex_time = parameters[0][4];

  } else if (M5.BtnC.wasPressed() && pattern == 0) {  
    M5.Lcd.clear();
    M5.Lcd.setCursor(82, 100);
    if( current_time >= 52 ) {   
      pattern = 112;
    } else {
      pattern = 111;
    }
  }

  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();

    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes - accelBiasX;
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes - accelBiasY;
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes - accelBiasZ;

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes - gyroBiasZ;
  }
  
}


// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    cnt10++;
    time_ms = millis() - time_buff;
    getTime();

    if (bufferIndex[writeBank] < BufferRecords && log_flag ) {
      RecordType* rp = &buffer[writeBank][bufferIndex[writeBank]];
      rp->log_time = timeStr;
      rp->log_pattern = pattern;
      rp->log_time_ms = time_ms;
      rp->log_tension = hx711_data * 1000;
      rp->log_ax = IMU.ax;
      rp->log_ay = IMU.ay;
      rp->log_az = IMU.az;
      rp->log_gz = IMU.gz;
      if (++bufferIndex[writeBank] >= BufferRecords) {
          writeBank = !writeBank;
      }      
    }

//    totalInterruptCounter++;

    iTimer10++;
    switch( iTimer10 ) {
    case 1:
      if(hover_flag) {
        //M5.Lcd.fillScreen(BLACK);
        M5.Lcd.fillRect(0, 0, 80, 80, TFT_WHITE);
        M5.Lcd.fillRect(80, 0, 240, 80, TFT_DARKGREY);
        M5.Lcd.fillRect(0, 80, 80, 160, TFT_DARKGREY);
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(13, 23);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.print("S2");
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(96, 30);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.printf("DctF     %3d", hover_val);
        M5.Lcd.setCursor(15, 120);
        M5.Lcd.print("No.");
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(10, 160);
        M5.Lcd.printf("%2d", patternNo+1);

        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(96, 92);
        M5.Lcd.printf("Move Length   %4d", parameters[patternNo][0]);
        M5.Lcd.setCursor(96, 132);
        M5.Lcd.printf("Acceleration  %4d", parameters[patternNo][1]);
        M5.Lcd.setCursor(96, 172);
        M5.Lcd.printf("Velocity      %4d", parameters[patternNo][2]);
        M5.Lcd.setCursor(96, 212);
        M5.Lcd.printf("Deceleration  %4d", parameters[patternNo][3]);
      } else {
        M5.Lcd.fillRect(0, 0, 80, 80, TFT_WHITE);
        M5.Lcd.fillRect(80, 0, 240, 80, TFT_DARKGREY);
        M5.Lcd.fillRect(0, 80, 80, 160, TFT_DARKGREY);
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(13, 23);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.print("S2");
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(96, 30);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.print("DctF Disable");
        M5.Lcd.setCursor(15, 120);
        M5.Lcd.print("No.");
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(10, 160);
        M5.Lcd.printf("%2d", patternNo+1);

        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(96, 92);
        M5.Lcd.printf("Move Length   %4d", parameters[patternNo][0]);
        M5.Lcd.setCursor(96, 132);
        M5.Lcd.printf("Acceleration  %4d", parameters[patternNo][1]);
        M5.Lcd.setCursor(96, 172);
        M5.Lcd.printf("Velocity      %4d", parameters[patternNo][2]);
        M5.Lcd.setCursor(96, 212);
        M5.Lcd.printf("Deceleration  %4d", parameters[patternNo][3]);

      }
    break;

    case 2:      
      //hx711_data = AE_HX711_Read();
      hx711_data = AE_HX711_getGram(1) - hx711_offset;
      break;

    case 3:     
      if( tx_pattern == 11 ) { 
        telemetry_flag = true;
      }
      break;

    case 4:    
      //hx711_data = AE_HX711_Read();
      hx711_data = AE_HX711_getGram(1) - hx711_offset;  
      break;

    case 5:      
      break;

    case 6:    
      //hx711_data = AE_HX711_Read();
      hx711_data = AE_HX711_getGram(1) - hx711_offset;  
      break;

    case 7:      
      break;

    case 8:    
      //hx711_data = AE_HX711_Read();
      hx711_data = AE_HX711_getGram(1) - hx711_offset;  
      break;

    case 9:      
      break;
        
    case 10:
      //hx711_data = AE_HX711_Read();
      hx711_data = AE_HX711_getGram(1) - hx711_offset;
      iTimer10 = 0;
      break;

    }

  }
}



// EEPROM Write
//------------------------------------------------------------------// 
void eeprom_write(void) {
  EEPROM.write(0, hover_val);
  EEPROM.commit();
}

// EEPROM Read
//------------------------------------------------------------------// 
void eeprom_read(void) {
    hover_val = EEPROM.read(0);
}

// Bluetooth RX
//------------------------------------------------------------------//
void bluetooth_rx(void) {

  while (bts.available() > 0) {
    bts_rx_buffer[bts_index] = bts.read();
    bts.write(bts_rx_buffer[bts_index]);
    
    if( bts_rx_buffer[bts_index] == '/' ) {
      bts.print("\n\n"); 
      if( tx_pattern == 1 ) {
        rx_pattern = atoi(bts_rx_buffer);
      } else {
        rx_val = atof(bts_rx_buffer);
      }
      bts_index = 0;
      
      switch ( rx_pattern ) {
          
      case 0:
        tx_pattern = 0;
        break;
        
      case 11:
        rx_pattern = 0;
        tx_pattern = 11;
        break;

      case 20:
        rx_pattern = 0;
        tx_pattern = 20;
        file.print("NTP");
        file.print(",");
        file.print("Pattern");
        file.print(",");
        file.print("Time [ms]");
        file.print(",");
        file.print("Tension [mg]");
        file.print(",");
        file.print("IMUaX [G]");
        file.print(",");
        file.print("IMUaY [G]");
        file.print(",");
        file.print("IMUaZ [G]");
        file.print(",");
        file.print("IMUgZ [deg/s]");
        file.println(",");
        file.close();

        accelBiasX = IMU.ax;
        accelBiasY = IMU.ay;
        accelBiasZ = IMU.az - 1;
        gyroBiasZ = IMU.gz;

        if( current_time >= 52 ) {   
          pattern = 112;
          break;
        } else {
          pattern = 111;
          break;
        }
        break;
        
      case 21:
        rx_pattern = 0;
        tx_pattern = 21;
        hover_flag = !hover_flag;
        if(hover_flag) {
          M5.Lcd.clear();
          DuctedFan.attach(DuctedFanPin);
          DuctedFan.write(0);
          delay(3000);
          DuctedFan.write(hover_val);
        } else {
          M5.Lcd.clear();
          DuctedFan.detach();
          if(pattern == 115) {
            log_flag = false;
            pattern = 0;
          }
        }
        break;

      case 31:
        tx_pattern = 31;
        rx_pattern = 41;
        break;

      case 41:
        hover_val = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;
          

      }
      
    } else {
        bts_index++;
    }
  }


}


// Bluetooth TX
//------------------------------------------------------------------//
void bluetooth_tx(void) {

    switch ( tx_pattern ) {
            
    case 0:
      delay(30);
      bts.print("\n\n\n\n\n\n");
      bts.print(" HoverSat Satellite2 (M5Stack version) "
                         "Test Program Ver1.10\n");
      bts.print("\n");
      bts.print(" Satellite control\n");
      bts.print(" 11 : Telemetry\n");
      bts.print(" 12 : Read log\n");
      bts.print("\n");
      bts.print(" 20 : Sequence Control\n");
      bts.print(" 21 : Start/Stop Hover\n");
      bts.print("\n");
      bts.print(" Set parameters  [Current val]\n");
      bts.print(" 31 : DuctedFan Output [");
      bts.print(hover_val);
      bts.print("%]\n");
      
      bts.print("\n");
      bts.print(" Please enter 11 to 35  ");
      
      tx_pattern = 1;
      break;
        
    case 1: 
      break;
        
    case 2:
      break;
        
    case 11:
      //Teremetry @ Interrupt
      break;

    case 20:
      bts.print(" Starting Sequence...\n");
      tx_pattern = 1;
      break;

    case 21:
      if(hover_flag) {
        bts.print(" Start Hovering...\n");
      } else {
        bts.print(" Stop Hovering...\n");
      }
      delay(1000);
      tx_pattern = 0;
      break;
              
    case 31:
      bts.print(" DuctedFan Output [%] -");
      bts.print(" Please enter 0 to 100 ");
      tx_pattern = 2;
      break;
                 
    }
}


// TSND 121
//------------------------------------------------------------------//
void TSND121( void ) {
    TSND_121_ENABLE = 1;
    digitalWrite( TSND_121, HIGH );
    delay(2000);
    digitalWrite( TSND_121, LOW );

}



// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
 }


//Get Time From NTP
//------------------------------------------------------------------//
void getTimeFromNTP(void){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
  }
}

//Get Convert Time
//------------------------------------------------------------------//
void getTime(void){
  getLocalTime(&timeinfo);
  dateStr = (String)(timeinfo.tm_year + 1900)
          + "/" + (String)(timeinfo.tm_mon + 1)
          + "/" + (String)timeinfo.tm_mday;
  timeStr = (String)timeinfo.tm_hour
          + ":" + (String)timeinfo.tm_min
          + ":" + (String)timeinfo.tm_sec;
  current_time = timeinfo.tm_sec;
}


//Write SD Initial Data
//------------------------------------------------------------------//
void writeDataInitial(void) {
  file = SD.open(fname, FILE_APPEND);
  file.println("Tether extension experiment");
  file.println("Parameters");
  file.println("Time, Pattern, Pattern");
  file.close();
}



//AE HX711 Init
//------------------------------------------------------------------//
void AE_HX711_Init(void)
{
  pinMode(HX711_SCLK, OUTPUT);
  pinMode(HX711_DOUT, INPUT);
}

//AE HX711 Reset
//------------------------------------------------------------------//
void AE_HX711_Reset(void)
{
  digitalWrite(HX711_SCLK,1);
  delayMicroseconds(100);
  digitalWrite(HX711_SCLK,0);
  delayMicroseconds(100); 
}

//AE HX711 Read
//------------------------------------------------------------------//
long AE_HX711_Read(void)
{
  long data=0;
  while(digitalRead(HX711_DOUT)!=0);
  delayMicroseconds(1);
  for(int i=0;i<24;i++)
  {
    digitalWrite(HX711_SCLK,1);
    delayMicroseconds(1);
    digitalWrite(HX711_SCLK,0);
    delayMicroseconds(1);
    data = (data<<1)|(digitalRead(HX711_DOUT));
  }  
  digitalWrite(HX711_SCLK,1);
  delayMicroseconds(1);
  digitalWrite(HX711_SCLK,0);
  delayMicroseconds(1);
  return data^0x800000; 
}


long AE_HX711_Averaging(long adc,char num)
{
  long sum = 0;
  for (int i = 0; i < num; i++) sum += AE_HX711_Read();
  return sum / num;
}

float AE_HX711_getGram(char num)
{
  #define HX711_R1  20000.0f
  #define HX711_R2  8200.0f
  #define HX711_VBG 1.25f
  #define HX711_AVDD      4.2987f//(HX711_VBG*((HX711_R1+HX711_R2)/HX711_R2))
  #define HX711_ADC1bit   HX711_AVDD/16777216 //16777216=(2^24)
  #define HX711_PGA 128
  #define HX711_SCALE     (OUT_VOL * HX711_AVDD / LOAD *HX711_PGA)
  
  float data;

  data = AE_HX711_Averaging(AE_HX711_Read(),num)*HX711_ADC1bit; 
  //Serial.println( HX711_AVDD);   
  //Serial.println( HX711_ADC1bit);   
  //Serial.println( HX711_SCALE);   
  //Serial.println( data);   
  data =  data / HX711_SCALE;


  return data;
}
