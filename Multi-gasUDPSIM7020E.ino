#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif



BluetoothSerial SerialBT;


#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <ArduinoJson.h>

#include <EEPROM.h>

#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>

#include "AIS_SIM7020E_API.h"



String deviceToken = "";


String address      = "147.50.151.130";    // Your Server IP
String serverPort   = "19956";    // Your Server Port
String payload = "";  //"{\\\"Tn\\\":\\\"8966031940014311892\\\",\\\"temp\\\":33}";
String data_return;

const long interval = 120000;  //millisecond
unsigned long previousMillis = 0;
int cnt = 0;

/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
const char* ssid = "greenio"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password

//WiFi&OTA 参数
String HOSTNAME = "CPNMultiGas01";
#define PASSWORD "green7650" //the password for OTA upgrade, can set it in any char you want

/************************************************  注意编译时要设置此值 *********************************
   是否使用静态IP
*/
#define USE_STATIC_IP false
#if USE_STATIC_IP
IPAddress staticIP(192, 168, 1, 22);
IPAddress gateway(192, 168, 1, 9);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(114, 114, 114, 114);
#endif



ModbusMaster node;
HardwareSerial modbus(2);
AIS_SIM7020E_API nb;
#define trigWDTPin    33

StaticJsonDocument<400> doc;

struct Meter
{

  String PM2_5;
  String PM10;
  String HUM;
  String TEMP;
  String ATM;
  String LUX;
  String TVOC;
  String CO2;
  String CH2O;
  String O3;
  String O;
  String H2S;
  String CH4;
  String CO;
  String NO2;
  String SO2;


};

Meter meter;

String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}






void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);

  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());

  //No authentication by default
  ArduinoOTA.setPassword(PASSWORD);

  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");

    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });

  ArduinoOTA.onEnd([]()
  {

    Serial.println("Update Complete!");
    SerialBT.println("Update Complete!");
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);


    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }


    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void setupWIFI()
{

  Serial.println("Connecting...");
  Serial.println(String(ssid));


  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  WiFi.disconnect(true);
  delay(1000);

  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
#if USE_STATIC_IP
  WiFi.config(staticIP, gateway, subnet);
#endif

  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }


  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");

}

void _init() {

  Serial.begin(115200);
  SerialBT.begin(HOSTNAME); //Bluetooth
  HOSTNAME.concat(getMacAddress());
  setupWIFI();
  setupOTA();

  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  delay(500);
  digitalWrite(25, LOW);

  boolean isConnected = nb.begin(address, serverPort);
  if (isConnected) {
    nb.debug = true;
    Serial.println(F("-------------BEGIN-------------"));
    Serial.print(F(">>DeviceIP: "));
    Serial.println(nb.getDeviceIP());
    Serial.print(F(">>Signal: "));
    Serial.println(nb.getSignal());


    nb.pingIP(address);
  } else {

    Serial.println("NB Fail");
    SerialBT.println("NB Fail");
  }
  deviceToken = nb.getICCID();
  Serial.print(F(">>ICCID: "));
  Serial.println(deviceToken);
  SerialBT.print(F(">>ICCID: "));
  SerialBT.println(deviceToken);
  modbus.begin(4800, SERIAL_8N1, 16, 17);

}


void setup() {

  _init();




  Serial.println();
  Serial.println(F("***********************************"));
  Serial.println("Initialize...");
  SerialBT.println(F("***********************************"));
  SerialBT.println("Initialize...");



}

void getPayload() {
  payload = "";
  payload.concat("{\\\"Tn\\\":\\\"");
  payload.concat(deviceToken);
  payload.concat("\\\"");
  payload.concat(",\\\"pm2.5\\\":");
  payload.concat(meter.PM2_5);
  payload.concat(",\\\"pm10\\\":");
  payload.concat(meter.PM10);
  payload.concat(",\\\"hum\\\":");
  payload.concat(meter.HUM);
  payload.concat(",\\\"temp\\\":");
  payload.concat(meter.TEMP);
  payload.concat(",\\\"atm\\\":");
  payload.concat(meter.ATM);
  payload.concat(",\\\"lux\\\":");
  payload.concat(meter.LUX);
  payload.concat(",\\\"tvoc\\\":");
  payload.concat(meter.TVOC);
  payload.concat(",\\\"co2\\\":");
  payload.concat(meter.CO2);
  payload.concat(",\\\"o3\\\":");
  payload.concat(meter.O3);
  payload.concat(",\\\"co\\\":");
  payload.concat(meter.CO);
  payload.concat(",\\\"no2\\\":");
  payload.concat(meter.NO2);
  payload.concat(",\\\"so2\\\":");
  payload.concat(meter.SO2);
  payload.concat(",\\\"CH2O\\\":");
  payload.concat(meter.CH2O);
  payload.concat(",\\\"RSSI\\\":");
  payload.concat(nb.getSignal());

  payload.concat("}");
  Serial.println(payload);
  SerialBT.println(payload);
}
void loop() {
  data_return = "";
  unsigned long currentMillis = millis();


  if (currentMillis - previousMillis >= interval) {
    cnt++;
    // Send data in String
    //Serial.print("IMSI:");    Serial.println(nb.getIMSI());
    //Serial.print("NBstatus:"); Serial.println(nb.getSignal());
    readMeter();
    Serial.println("Start concat payload");
    SerialBT.println("Start concat payload");
    getPayload();
    nb.sendMsgSTR(address, serverPort, payload);

    // Send data in HexString
    //    nb.sendMsgHEX(address,serverPort,payload);
    previousMillis = currentMillis;
  }
  nb.waitResponse(data_return, address);
  if (data_return != ""){
    Serial.println("# Receive : " + data_return);
    SerialBT.println("# Receive : " + data_return);
  }


  ArduinoOTA.handle();

  if (currentMillis % 60000 == 0)
  {
    Serial.println("v2.0 Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );
    SerialBT.println("v2.0 Attach WiFi for OTA"); SerialBT.println(WiFi.RSSI() );
    setupWIFI();
    HeartBeat();
    setupOTA();

  }
}


int readModbus1Byte(char addr, uint16_t  REG)
{
  static uint32_t i;
  int8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(addr, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 1);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 1; j++)
    {
      data[j] = node.getResponseBuffer(j);
      Serial.println(data[j]);
    }
    value = data[0];

    return value;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return NULL;
  }
}



void readMeter()
{


  meter.PM2_5 = readModbus1Byte(ID, SensorAddr[0]);// Scan registers
  meter.PM10 = readModbus1Byte(ID, SensorAddr[1]); // Scan registers

  meter.HUM = readModbus1Byte(ID, SensorAddr[2]); // Scan registers
  meter.TEMP = readModbus1Byte(ID, SensorAddr[3]); // Scan registers
  meter.ATM = readModbus1Byte(ID, SensorAddr[4]); // Scan registers
  meter.LUX = readModbusFloat(ID, SensorAddr[5]); // Scan registers
  meter.TVOC = readModbus1Byte(ID, SensorAddr[6]); // Scan registers
  meter.CO2 = readModbus1Byte(ID, SensorAddr[7]); // Scan registers
  meter.O3 = readModbus1Byte(ID, SensorAddr[8]); // Scan registers
  meter.CO = readModbus1Byte(ID, SensorAddr[9]); // Scan registers
  meter.NO2 = readModbus1Byte(ID, SensorAddr[10]); // Scan registers
  meter.SO2 = readModbus1Byte(ID, SensorAddr[11]); // Scan registers
  meter.CH2O = readModbus1Byte(ID, SensorAddr[12]); // Scan registers


  Serial.print("pm2.5: ");  Serial.println(meter.PM2_5);
  Serial.print("pm10: ");  Serial.println(meter.PM10);
  Serial.print("hum: ");  Serial.println(meter.HUM);
  Serial.print("temp: ");  Serial.println(meter.TEMP);
  Serial.print("atm: ");  Serial.println(meter.ATM);
  Serial.print("LUX: ");  Serial.println(meter.LUX);
  Serial.print("TVOC: ");  Serial.println(meter.TVOC);
  Serial.print("CO2: ");  Serial.println(meter.CO2);
  Serial.print("O3: ");  Serial.println(meter.O3);
  Serial.print("CO: ");  Serial.println(meter.CO);
  Serial.print("NO2: ");  Serial.println(meter.NO2);
  Serial.print("SO2: ");  Serial.println(meter.SO2);
  Serial.print("CH2O: ");  Serial.println(meter.CH2O);


  Serial.println("");
  Serial.println("");
  HeartBeat();
}

void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);

  pinMode(trigWDTPin, INPUT);

  Serial.println("Heartbeat");
  SerialBT.println("Heartbeat");

}

float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}



uint32_t FloatTohex(float x) {
  return (*(uint32_t*)&x);
}

float readModbusFloat(char addr , uint16_t  REG) {
  float i = 0;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  for (int i = 0; i < 2; i++) {
    node.begin(addr, modbus);
    result = node.readInputRegisters (REG, 2); ///< Modbus function 0x04 Read Input Registers
    delay(500);

    if (result == node.ku8MBSuccess) {
      for (j = 0; j < 2; j++)
      {
        data[j] = node.getResponseBuffer(j);
        Serial.print(i); Serial.print(":");
        Serial.println(data[j]);
      }
      value = data[0];
      value = value >> 16;
      value = value + data[1];
      Serial.print("t1:");
      Serial.println(value);
      //      i = HexTofloat(value);
      //      Serial.print("t2:");
      //      Serial.println(i);
      //Serial.println("Connec modbus Ok.");
      return value;
    }

  }

  Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
  return 0;
}




void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;

    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;

    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;

    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;

    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;

    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;

    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}
