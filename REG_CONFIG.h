#define ID        1 
 

// Register Sensor
#define _PM2_5  0       //  0.
#define _PM10 1      //  1.
#define _HUM  2      //  2.          
#define _TEMP 3      //  3.
#define _ATM 4      //  3.
#define _LUX  5
#define _TVOC 7
#define _CO2 8
#define _O3 10
#define _CO 14
#define _NO2 15
#define _SO2 16 
 

uint16_t const SensorAddr[12] = {
  _PM2_5,
  _PM10,
  _HUM,
  _TEMP,
  _ATM,
  _LUX,
  _TVOC,
  _CO2,
  _O3,
  _CO,
  _NO2,
  _SO2
  
};
