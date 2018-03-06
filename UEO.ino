#include "CurieIMU.h"
#include <Wire.h> 
#include "Math.h"
#include <CurieBLE.h>
#include <stdlib.h>

#define wd 10
int16_t SensVal[30];
int16_t UEOtrainAvr[wd][30] = {
{64.71,40.8,53.88,42.27,53.22,0,0,0,0,5.8,-63.9,-89.3,41.61,95.25,55.74,44.13,56.37,0,0,0,0,3.5,-65.4,-90.5,0,0,0,0,0,0},
{64.71,40.8,53.88,42.27,53.22,0,0,0,0,5.8,-63.9,-89.3,40.62,97.74,123.96,48.6,65.4,0,0,0,0,4.6,-62.4,-90.2,0,0,0,0,0,0},
{64.71,40.8,53.88,42.27,53.22,0,0,0,0,5.8,-63.9,-89.3,42.93,97.35,128.28,117.57,68.43,0,0,0,0,7.6,-59.7,-93.5,0,0,0,0,0,0},
{64.71,40.8,53.88,42.27,53.22,0,0,0,0,5.8,-63.9,-89.3,37.14,99.75,126.72,125.76,141.24,0,0,0,0,6.2,-58,-96,0,0,0,0,0,0},
{64.71,40.8,53.88,42.27,53.22,0,0,0,0,5.8,-63.9,-89.3,105.54,39.48,54.75,42.42,51.3,0,0,0,0,3.9,-60.8,-98.1,0,0,0,0,0,0},
{48.24,81.96,55.35,42.78,62.25,0,0,0,0,10.5,13.7,-22.6,52.08,83.46,56.07,43.77,63.15,0,0,0,0,98.9,-126.8,-8.4,48.5,0,29.4,0,0,-39},
{58.44,33.75,55.95,45.12,59.94,0,0,0,0,-4.5,-5.2,-95.3,43.26,78.18,56.97,46.98,62.64,0,0,0,0,-0.3,14.1,-38.7,9.1,-6.9,10.5,0,64.7,0},
{43.29,93.36,52.47,43.53,57.96,0,0,0,0,-0.6,-5.7,-75.7,44.04,98.55,55.38,46.08,61.98,0,0,0,0,5,-1.1,-87,178.9,-184,7,-1.7,9.3,-2.8},
{103.29,62.94,101.88,92.82,123.72,0,0,0,0,115,110.9,-29.2,43.23,85.68,56.43,44.85,57.54,0,0,0,0,24.1,-36,130.9,34.2,-75.8,51,-16,3,-41.1},
{41.22,80.28,60.78,49.2,68.88,0,0,0,0,163.6,-157.5,145.6,50.49,74.82,59.94,48.72,66.09,0,0,0,0,21.1,-4.2,-89.5,0.7,-129.2,2.5,0,8.6,-0.5}
};
  
float UEOtrainImpt[wd][30] = {
{0.9709,0.9125,0.9815,0.98975,0.9835,1,1,1,1,0.983888889,0.988611111,0.9925,0.9871,0.98375,0.987,0.98775,0.98475,1,1,1,1,0.990277778,0.992777778,0.995833333,1,1,1,1,1,1},
{0.9709,0.9125,0.9815,0.98975,0.9835,1,1,1,1,0.983888889,0.988611111,0.9925,0.9898,0.9845,0.9855,0.9875,0.98,1,1,1,1,0.982222222,0.987777778,0.992222222,1,1,1,1,1,1},
{0.9709,0.9125,0.9815,0.98975,0.9835,1,1,1,1,0.983888889,0.988611111,0.9925,0.9787,0.96375,0.9815,0.96475,0.98525,1,1,1,1,0.987222222,0.988055556,0.993055556,1,1,1,1,1,1},
{0.9709,0.9125,0.9815,0.98975,0.9835,1,1,1,1,0.983888889,0.988611111,0.9925,0.9844,0.97125,0.9865,0.9805,0.977,1,1,1,1,0.986666667,0.991666667,0.994444444,1,1,1,1,1,1},
{0.904,0.959,0.96225,0.9795,0.97775,1,1,1,1,0.983611111,0.987777778,0.983611111,0.9646,0.9765,0.97125,0.976,0.9875,1,1,1,1,0.989166667,0.989444444,0.991944444,1,1,1,1,1,1},
{0.9784,0.968,0.98625,0.9815,0.97125,1,1,1,1,0.970833333,0.984166667,0.982222222,0.9868,0.983,0.98525,0.97975,0.97375,1,1,1,1,0.986388889,0.988333333,0.985,0.9725,1,0.982,1,1,0.955},
{0.9484,0.90875,0.98125,0.9815,0.9795,1,1,1,1,0.9875,0.992222222,0.978611111,0.9886,0.956,0.98975,0.984,0.977,1,1,1,1,0.999166667,0.994722222,0.988055556,0.9605,0.9655,0.9775,1,0.9685,1},
{0.9709,0.962,0.98725,0.98025,0.9805,1,1,1,1,0.982222222,0.978611111,0.950833333,0.9754,0.97625,0.989,0.9915,0.9865,1,1,1,1,0.986111111,0.994722222,0.955555556,0.9605,0.92,0.97,0.9915,0.9665,0.986},
{0.9649,0.947,0.9565,0.9185,0.931,1,1,1,1,0.988888889,0.985833333,0.986666667,0.9733,0.9565,0.98025,0.97875,0.9805,1,1,1,1,0.978055556,0.988888889,0.885833333,0.959,0.911,0.945,0.955,0.985,0.9505},
{0.9778,0.909,0.9815,0.98,0.904,1,1,1,1,0.985,0.968055556,0.948888889,0.9301,0.961,0.9795,0.9835,0.96825,1,1,1,1,0.983611111,0.991111111,0.926388889,0.9965,0.971,0.9875,1,0.982,0.9975}
};

#define M_PI  3.14159265358979323846

void AngleXYZ();
void FingerShape();
boolean MovingState();
void dAngle();
void MLdataOut_start();
void MLdataOut_end();
void Sprint();

float Mv_stop_time;
float dt, t1, t2;
///////////////////////////////////////////////////////////////////////////// Mag /////////////////////////////////////
#define address 0x1E //0011110b, I2C 7bit address of HMC5883
#define MagAngleX_dy -37
#define MagAngleX_dz 233
float Xavrmd = 713.5;
#define MagAngleX_mdy 779
#define MagAngleX_mdz 648

#define MagAngleY_dx 31
#define MagAngleY_dz 206
float Yavrmd = 678.50;
#define MagAngleY_mdx 706
#define MagAngleY_mdz 651

#define MagAngleZ_dx 37
#define MagAngleZ_dy -59
float Zavrmd = 774.50;
#define MagAngleZ_mdx 774
#define MagAngleZ_mdy 775

float MagAngleX_y, MagAngleX_z, MagAngleY_x, MagAngleY_z, MagAngleZ_x, MagAngleZ_y;
int16_t MagAngleOffset;
int16_t atanAngleX_Mag, atanAngleY_Mag, atanAngleZ_Mag;
int16_t AngleX_Mag, AngleY_Mag, AngleZ_Mag;
int16_t mx, my, mz;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// Acc /////////////////////////////////////
float a;
float ax2, ay2, az2;
float AngleX_Acc, AngleY_Acc, AngleZ_Acc;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// Gyr /////////////////////////////////////
float dgx, dgy, dgz;
float gx2, gy2, gz2;
float gx1, gy1, gz1;
float dAngleX_Gyr, dAngleY_Gyr, dAngleZ_Gyr;
float AngleX_Gyr, AngleY_Gyr, AngleZ_Gyr;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// Ang /////////////////////////////////////
float Kx, Ky, Kz;
int16_t AngleX, AngleY, AngleZ;
float fAngleX, fAngleY, fAngleZ;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// Flex ////////////////////////////////////
uint8_t pulse;
#define T  0
#define I  1
#define M  2
#define R  3
#define L  4
uint16_t FlexS[5];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// Reed ////////////////////////////////////
uint16_t T_ReedS_I, I_ReedS_M, M_ReedS_R, R_ReedS_L;
boolean TRI, IRM, MRR, RRL;
uint16_t All_ReedS;
uint8_t fsc;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// Move ////////////////////////////////////
#define AF 5

uint8_t sc;

float aa[5], aMax, aMin;
float mv_t, st_t;
uint8_t Acc_mov;

uint16_t o_Reed; 
uint8_t  Rd_mov,  Rd_st_cnt;

uint16_t fa[5][5], fMax[5], fMin[5];
uint8_t Fs_mv[6], Fs_MmM[5] = {38, 45, 45, 45, 80}, Fs_st_cnt, Fs_mov;

boolean mov = false;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// MLdata //////////////////////////////////
uint16_t startFlexT, startFlexI, startFlexM, startFlexR, startFlexL;
boolean startTRI, startIRM, startMRR, startRRL;
int16_t startAngX, startAngY, startAngZ;
int16_t dAngleX_p, dAngleX_m, dAngleY_p, dAngleY_m, dAngleZ_p, dAngleZ_m;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// BLE /////////////////////////////////////
BLEService TransService("19B10000-E8F2-537E-4F6C-D104768A1215");
char data[20];

BLECharacteristic UEOCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
BLECharacteristic UEOCharacteristic2("19B10002-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
BLECharacteristic UEOCharacteristic3("19B10003-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
BLECharacteristic UEOCharacteristic4("19B10004-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
BLECharacteristic UEOCharacteristic5("19B10005-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
BLECharacteristic UEOCharacteristic6("19B10006-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
BLECharacteristic UEOCharacteristic7("19B10007-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
BLECharacteristic RHandMv("19B10008-E8F2-537E-4F6C-D104768A1215",BLEWrite|BLERead,20);
//BLECharacteristic switchCharacteristic9("19B10010-E8F2-537E-4F6C-D104768A1214",BLEWrite|BLERead,20);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################## setup 
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("start");

  CurieIMU.begin();
  CurieIMU.setGyroRange(2000);
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();  
  
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);

  while(!(6<=Wire.available()));
  mx = Wire.read()<<8; //X msb
  mx |= Wire.read(); //X lsb
  mz = Wire.read()<<8; //Z msb
  mz |= Wire.read(); //Z lsb
  my = Wire.read()<<8; //Y msb
  my |= Wire.read(); //Y lsb

  MagAngleZ_x = (mx - MagAngleZ_dx)*(Zavrmd/MagAngleZ_mdx);
  MagAngleZ_y = (my - MagAngleZ_dy)*(Zavrmd/MagAngleZ_mdy);
  MagAngleOffset = atan2(-MagAngleZ_x, MagAngleZ_y) * (180/M_PI);

  pinMode(9, OUTPUT); pinMode(10, INPUT); pinMode(11, INPUT); pinMode(12, INPUT); pinMode(13, INPUT);

  BLE.begin();
  BLE.setLocalName("Trans");
  BLE.setAdvertisedService(TransService);

  TransService.addCharacteristic(UEOCharacteristic);
  TransService.addCharacteristic(UEOCharacteristic2);
  TransService.addCharacteristic(UEOCharacteristic2);
  TransService.addCharacteristic(UEOCharacteristic3);
  TransService.addCharacteristic(UEOCharacteristic4);
  TransService.addCharacteristic(UEOCharacteristic5);
  TransService.addCharacteristic(UEOCharacteristic6);
  TransService.addCharacteristic(UEOCharacteristic7);
  TransService.addCharacteristic(RHandMv);  
//  TransService.addCharacteristic(switchCharacteristic9);

  BLE.addService(TransService);
  BLE.advertise();

  RHandMv.setValue("0");
  pinMode(5, OUTPUT); pinMode(6, OUTPUT); pinMode(7, OUTPUT);
}
//##############################################################################################
//############################################################################################## loop
void loop() {
 boolean MvSt, oMvSt, sig = 0;
  digitalWrite(5, HIGH);
  digitalWrite(7, LOW); digitalWrite(6, LOW);
  BLEDevice central = BLE.central();

  if(central){
    digitalWrite(5, LOW);
    while(central.connected()) {
      FingerShape();
      AngleXYZ();
      MvSt = MovingState();
//      Serial.print(MvSt); Serial.print("\t");
//      Serial.print(LHandMv.value()); Serial.println("\t");
      if(MvSt) { dAngle(); RHandMv.setValue("0"); sig = 0;
                 digitalWrite(7, HIGH); digitalWrite(6, LOW); digitalWrite(5, LOW);}
      else     { digitalWrite(6, HIGH); digitalWrite(7, LOW);}                 
      if(oMvSt != MvSt) {
        if(MvSt) MLdataOut_start();
        else     Mv_stop_time = millis();
      }
      if(!MvSt) 
        if((millis()-Mv_stop_time) >= 1500) { 
          if(!sig) { digitalWrite(5, HIGH); 
                     MLdataOut_end();
                      Sprint();             
                      sig = 1;               }   
        } 
      delay(10);
      oMvSt = MvSt;
    }
  }
}
//##############################################################################################
//############################################################################################## FingerShape
void FingerShape() { 
  pulse = ~pulse;
  if(pulse==0){ digitalWrite(9, HIGH); FlexS[T] = analogRead(A3);}
  else        { digitalWrite(9, LOW);  FlexS[L] = analogRead(A3);}
  FlexS[I] = analogRead(A2);
  FlexS[M] = analogRead(A1);
  FlexS[R] = analogRead(A0); 

  if(digitalRead(10)) T_ReedS_I |= 0x01<<fsc;
  else                T_ReedS_I &= ~(0x01<<fsc);
  if(digitalRead(11)) I_ReedS_M |= 0x01<<fsc;
  else                I_ReedS_M &= ~(0x01<<fsc);
  if(digitalRead(12)) M_ReedS_R |= 0x01<<fsc;
  else                M_ReedS_R &= ~(0x01<<fsc);  
  if(digitalRead(13)) R_ReedS_L |= 0x01<<fsc;
  else                R_ReedS_L &= ~(0x01<<fsc);
  fsc++; if(fsc>12) fsc = 0;
  
}
//##############################################################################################
//############################################################################################## AngleXYZ
void AngleXYZ() {
  CurieIMU.readAccelerometerScaled(ax2, ay2, az2);
  CurieIMU.readGyroScaled(gx2, gy2, gz2);
  a = sqrt((ax2 * ax2) + (ay2 * ay2) + (az2 * az2));

  Wire.beginTransmission(address);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    mx = Wire.read()<<8;
    mx |= Wire.read();
    mz = Wire.read()<<8;
    mz |= Wire.read();
    my = Wire.read()<<8;
    my |= Wire.read();
  }
  MagAngleX_y = (my - MagAngleX_dy)*(Xavrmd/MagAngleX_mdy);
  MagAngleX_z = (mz - MagAngleX_dz)*(Xavrmd/MagAngleX_mdz);
  MagAngleY_x = (mx - MagAngleY_dx)*(Yavrmd/MagAngleY_mdx);
  MagAngleY_z = (mz - MagAngleY_dz)*(Yavrmd/MagAngleY_mdz);
  MagAngleZ_x = (mx - MagAngleZ_dx)*(Zavrmd/MagAngleZ_mdx);
  MagAngleZ_y = (my - MagAngleZ_dy)*(Zavrmd/MagAngleZ_mdy);
  
  t2 = millis();
  dt = t2 - t1;
  
 if (abs(gx2) <= 5) gx2 = 0;
  dgx = gx2 - gx1;
  dAngleX_Gyr = (gx1 + (dgx / 2)) * (dt / 1000);
  //AngleX_Gyr += dAngleX_Gyr;
  AngleX_Acc = atan2(ay2, az2) * 180 / M_PI;
  atanAngleX_Mag = (atan2(MagAngleX_z, MagAngleX_y) * (180/M_PI))-MagAngleOffset;
  if     (atanAngleX_Mag >180)  AngleX_Mag = atanAngleX_Mag - 360;  
  else if(atanAngleX_Mag <-180) AngleX_Mag = atanAngleX_Mag + 360;
  else                          AngleX_Mag = atanAngleX_Mag;
  if ((abs(ax2) > abs(ay2)) and (abs(ax2) > abs(az2))) Kx = 0.00;
  else                                                 Kx = 0.05;
  fAngleX = (0.95 * (fAngleX + dAngleX_Gyr)) + (Kx * AngleX_Acc) + ((0.05 - Kx) * AngleX_Mag);
  AngleX = fAngleX;
  
  if (abs(gy2) <= 5) gy2 = 0;
  dgy = gy2 - gy1;
  dAngleY_Gyr = (gy1 + (dgy / 2)) * (dt / 1000);
  //AngleY_Gyr += dAngleY_Gyr;
  AngleY_Acc = atan2(-ax2, az2) * 180 / M_PI;
  atanAngleY_Mag = ((atan2(MagAngleY_x, MagAngleY_z) * (180/M_PI))-MagAngleOffset) + 180;
  if     (atanAngleY_Mag >180)  AngleY_Mag = atanAngleY_Mag - 360;  
  else if(atanAngleY_Mag <-180) AngleY_Mag = atanAngleY_Mag + 360;
  else                          AngleY_Mag = atanAngleY_Mag;
  if ((abs(ay2) > abs(ax2)) and (abs(ay2) > abs(az2))) Ky = 0.00;
  else                                                 Ky = 0.05;
  fAngleY = (0.95 * (fAngleY + dAngleY_Gyr)) + (Ky * AngleY_Acc) + ((0.05 - Ky) * AngleY_Mag);
  AngleY = fAngleY;

  if (abs(gz2) <= 5) gz2 = 0;
  dgz = gz2 - gz1;
  dAngleZ_Gyr = -(gz1 + (dgz / 2)) * (dt / 1000);
  //AngleZ_Gyr += dAngleZ_Gyr;
  AngleZ_Acc = atan2(-ax2, ay2) * 180 / M_PI;
  atanAngleZ_Mag = (atan2(-MagAngleZ_x, MagAngleZ_y) * (180/M_PI))-MagAngleOffset;
  if     (atanAngleZ_Mag >180)  AngleZ_Mag = atanAngleZ_Mag - 360;  
  else if(atanAngleZ_Mag <-180) AngleZ_Mag = atanAngleZ_Mag + 360;
  else                          AngleZ_Mag = atanAngleZ_Mag;
  if ((abs(az2) > abs(ax2)) and (abs(az2) > abs(ay2))) Kz = 0.00;
  else                                                 Kz = 0.05;
  fAngleZ = (0.95 * (fAngleZ + dAngleZ_Gyr)) + (Kz * AngleZ_Acc) + ((0.05 - Kz) * AngleZ_Mag);
  AngleZ = fAngleZ;
  
  t1 = t2;
  gx1 = gx2; gy1 = gy2; gz1 = gz2;
}
//##############################################################################################
//############################################################################################## MovingState
boolean MovingState() {
  
  aa[sc] = a; 
  unsigned char i;
  aMax = aa[0]; aMin = aa[0];
  for(i=0;i<5;i++){
    if(aMax<aa[i]) aMax = aa[i];
    if(aMin>aa[i]) aMin = aa[i];
  } 
  if((aMax-aMin)<0.20) {
    mv_t = millis();
    if(millis()-st_t >= 300)  Acc_mov = 0;
  }
  else {    
    st_t = millis();
    if(millis()-mv_t >= 10) Acc_mov = 1;
  }

  if(T_ReedS_I) TRI = true;
  else          TRI = false; 
  if(I_ReedS_M) IRM = true;
  else          IRM = false; 
  if(M_ReedS_R) MRR = true;
  else          MRR = false; 
  if(R_ReedS_L) RRL = true;
  else          RRL = false;  

  if(FlexS[T]<220)   TRI = false;
  if(FlexS[I]<280) { TRI = false; IRM = false; }
  if(FlexS[M]<300) { IRM = false; MRR = false; } 
  if(FlexS[R]<300) { MRR = false; RRL = false; }
  if(FlexS[L]<300)   RRL = false;
  
  All_ReedS = (TRI) + (IRM*10) + (MRR*100) + (RRL*1000);
  
  if(o_Reed==All_ReedS) Rd_st_cnt++;
  else                { Rd_mov = 1; Rd_st_cnt = 0; }
  if(Rd_st_cnt >= 80) { Rd_mov = 0; Rd_st_cnt = 80;}

  unsigned int j,k;
  for(j=0;j<5;j++){
    for(k=0;k<5;k++){
      fa[sc][k] = FlexS[k];
      if(j==0) { fMax[k] = fa[0][k]; fMin[k] = fa[0][k]; }
      if(fMax[k]<fa[j][k]) fMax[k] = fa[j][k];
      if(fMin[k]>fa[j][k]) fMin[k] = fa[j][k];
      if(fMax[k]-fMin[k] > Fs_MmM[k]) Fs_mv[k] = 1;
      else                     Fs_mv[k] = 0;
      Fs_mv[AF] = Fs_mv[T] + Fs_mv[I] + Fs_mv[M] + Fs_mv[R] + Fs_mv[L];
    }
  }
  if(Fs_mv[AF]==0) Fs_st_cnt++;
  else           { Fs_st_cnt = 0; Fs_mov = 1; }
  if(Fs_st_cnt>= 80) { Fs_st_cnt = 80; Fs_mov = 0; }
       
  if(Acc_mov or Rd_mov or Fs_mov) mov = true;
  else                            mov = false;

  sc++; if(sc==5) sc = 0;
  o_Reed = All_ReedS;  

////

//  Serial.print(AngleX); Serial.print("\t");
//  Serial.print(AngleY); Serial.print("\t");
//  Serial.print(AngleZ); Serial.println("\t");
 
//  Serial.print(T_ReedS_I); Serial.print("\t");
//  Serial.print(I_ReedS_M); Serial.print("\t");
//  Serial.print(M_ReedS_R); Serial.print("\t");
//  Serial.print(R_ReedS_L); Serial.print("\t"); Serial.print("\t");
//
//  Serial.print(TRI); Serial.print("\t");
//  Serial.print(IRM); Serial.print("\t");
//  Serial.print(MRR); Serial.print("\t");
//  Serial.print(RRL); Serial.print("\t");
//
//  Serial.print(fMax[T]-fMin[T]); Serial.print("\t");
//  Serial.print(fMax[I]-fMin[I]); Serial.print("\t");
//  Serial.print(fMax[M]-fMin[M]); Serial.print("\t");
//  Serial.print(fMax[R]-fMin[R]); Serial.print("\t");
//  Serial.print(fMax[L]-fMin[L]); Serial.print("\t");
//  Serial.print(FlexS[T]); Serial.print("\t");
//  Serial.print(FlexS[I]); Serial.print("\t");
//  Serial.print(FlexS[M]); Serial.print("\t");
//  Serial.print(FlexS[R]); Serial.print("\t");
//  Serial.print(FlexS[L]); Serial.print("\t");
//  Serial.print("acc_mv : "); Serial.print(Acc_mov); Serial.print("\t");
//  Serial.print("rd_mv : "); Serial.print(Rd_mov); Serial.print("\t");
//  Serial.print("fs_mv : "); Serial.print(Fs_mov); Serial.print("\t");
//  Serial.println(mov);
  

  return mov;
}
//##############################################################################################
//############################################################################################## dAngle
void dAngle() {
  if(dAngleX_Gyr > 0) dAngleX_p += dAngleX_Gyr;
  else                dAngleX_m += dAngleX_Gyr;
  
  if(dAngleY_Gyr > 0) dAngleY_p += dAngleY_Gyr;
  else                dAngleY_m += dAngleY_Gyr;
  
  if(dAngleZ_Gyr > 0) dAngleZ_p += dAngleZ_Gyr;
  else                dAngleZ_m += dAngleZ_Gyr;
}
//##############################################################################################
//############################################################################################## MLdataOut_start
void MLdataOut_start() {
  SensVal[0] = FlexS[T]*0.3;
  SensVal[1] = FlexS[I]*0.3;
  SensVal[2] = FlexS[M]*0.3;
  SensVal[3] = FlexS[R]*0.3;
  SensVal[4] = FlexS[L]*0.3;
  SensVal[5] = TRI*100;
  SensVal[6] = IRM*100;
  SensVal[7] = MRR*100;
  SensVal[8] = RRL*100;
  SensVal[9] = AngleX;
  SensVal[10] = AngleY;
  SensVal[11] = AngleZ;
  
  dAngleX_p = 0; dAngleX_m = 0; 
  dAngleY_p = 0; dAngleY_m = 0; 
  dAngleZ_p = 0; dAngleZ_m = 0;    
}
//##############################################################################################
//############################################################################################## MLdataOut_end
void MLdataOut_end() {  
  SensVal[12]  = FlexS[T]*0.3;
  SensVal[13]  = FlexS[I]*0.3;
  SensVal[14]  = FlexS[M]*0.3;
  SensVal[15]  = FlexS[R]*0.3;
  SensVal[16]  = FlexS[L]*0.3;
  SensVal[17] = TRI*100;
  SensVal[18] = IRM*100;
  SensVal[19] = MRR*100;
  SensVal[20] = RRL*100;
  SensVal[21] = AngleX;
  SensVal[22] = AngleY;
  SensVal[23] = AngleZ;
  SensVal[24] = dAngleX_p; 
  SensVal[25] = dAngleX_m; 
  SensVal[26] = dAngleY_p;
  SensVal[27] = dAngleY_m;
  SensVal[28] = dAngleZ_p;
  SensVal[29] = dAngleZ_m;
  
  int8_t dataN, w, c;
  float ProbMax = 0, Prob = 0;
  uint8_t MaxNum = 0;
  
  for(w=0;w<wd;w++){
    for(c=0;c<30;c++){
      float eFact = (abs(SensVal[c]-UEOtrainAvr[w][c]))/(abs(UEOtrainAvr[w][c]));
      if((SensVal[c]-UEOtrainAvr[w][c])==0) eFact = 0; 
      if(UEOtrainAvr[w][c]==0)              eFact = 1; 
      Prob += (1-eFact) * UEOtrainImpt[w][c];       
    }
//      Serial.print("word Num : "); Serial.print(w);
//      Serial.print("Prob : "); Serial.println(Prob);
    if(ProbMax<Prob) { ProbMax = Prob; MaxNum = w; }   
    Prob = 0;
  }   
  
  sprintf(data,"%d",MaxNum);
  UEOCharacteristic.setValue((unsigned char*)data,strlen(data));
  Serial.println(MaxNum);
       
  RHandMv.setValue("1");
}
//##############################################################################################
//############################################################################################## Sprint
void Sprint() { 
  Serial.print(SensVal[0]);   Serial.print("\t");
  Serial.print(SensVal[1]);   Serial.print("\t");
  Serial.print(SensVal[2]);   Serial.print("\t");
  Serial.print(SensVal[3]);   Serial.print("\t");
  Serial.print(SensVal[4]);   Serial.print("\t");
  Serial.print(SensVal[5]);     Serial.print("\t");
  Serial.print(SensVal[6]);     Serial.print("\t");
  Serial.print(SensVal[7]);     Serial.print("\t");
  Serial.print(SensVal[8]);     Serial.print("\t");
  Serial.print(SensVal[9]);    Serial.print("\t");
  Serial.print(SensVal[10]);    Serial.print("\t");
  Serial.print(SensVal[11]);    Serial.print("\t");    
  Serial.print(SensVal[12]);     Serial.print("\t");
  Serial.print(SensVal[13]);     Serial.print("\t");
  Serial.print(SensVal[14]);     Serial.print("\t");
  Serial.print(SensVal[15]);     Serial.print("\t");
  Serial.print(SensVal[16]);     Serial.print("\t");
  Serial.print(SensVal[17]);    Serial.print("\t");
  Serial.print(SensVal[18]);    Serial.print("\t");
  Serial.print(SensVal[19]);    Serial.print("\t");
  Serial.print(SensVal[20]);    Serial.print("\t");
  Serial.print(SensVal[21]);       Serial.print("\t");
  Serial.print(SensVal[22]);       Serial.print("\t");
  Serial.print(SensVal[23]);       Serial.print("\t");    
  Serial.print(SensVal[24]);    Serial.print("\t");
  Serial.print(SensVal[25]);    Serial.print("\t");
  Serial.print(SensVal[26]);    Serial.print("\t");
  Serial.print(SensVal[27]);    Serial.print("\t");
  Serial.print(SensVal[28]);    Serial.print("\t");
  Serial.print(SensVal[29]);    Serial.println("\t");   
}



