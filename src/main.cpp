/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 * 2. The network access status of LoRaWAN is displayed on the screen.
 *
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 *
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 */

#include "LoRaWan_APP.h"
#include <Arduino.h>

#define LED_PIN 35

// ------------ คีย์ LoRaWAN (ใช้ของคุณเดิม) ------------
// TTN Applications Heltec V3 End devices eui-70b3d57ed00683d9
// AT+CDKEY=C42BE5106951CF976CC56C4160D6BD6D
// uint32_t license[4] = { 0xC42BE510,0x6951CF97,0x6CC56C41,0x60D6BD6D };

/* OTAA para*/
// devEui MSB APPKEY MSB
uint8_t devEui[] = {0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x4E, 0x49};
uint8_t appEui[] = {0xA1, 0xA2, 0xA3, 0xA4, 0x55, 0x55, 0x55, 0x55};
uint8_t appKey[] = {0x64, 0x2B, 0xA5, 0xBA, 0x29, 0x95, 0xE8, 0xC9, 0x37, 0xE9, 0xF2, 0x8D, 0xC3, 0xEA, 0x64, 0x2D};

/* ABP para*/
uint8_t nwkSKey[] = {0x7F, 0xA8, 0xE0, 0xEA, 0xE8, 0x71, 0x60, 0xC9, 0x5C, 0xE6, 0x0D, 0xD6, 0xD4, 0x2A, 0x38, 0x49};
uint8_t appSKey[] = {0xAB, 0x0E, 0xFC, 0x07, 0xA7, 0x11, 0xD3, 0xC7, 0x9C, 0xB8, 0x57, 0x3E, 0xCE, 0xAE, 0xBA, 0x5D};
uint32_t devAddr = (uint32_t)0x27FA3400;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/* Application port */
uint8_t appPort = 2;

/* Number of trials for confirmed messages */
uint8_t confirmedNbTrials = 4;

/* ID สถานี */
int station_id = 1;

/* พิกัดของ Node (ตัวอย่าง: Bangkok คุณแก้เป็นของจริงได้) */
const float LATITUDE  = 13.7563;   // <-- ใส่ lat จริงของ node
const float LONGITUDE = 100.5018;  // <-- ใส่ lng จริงของ node

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port)
{
  // จำลองค่าข้อมูลเซนเซอร์
  uint16_t pressure   = random(1, 5);    // ตัวเลขตัวอย่าง
  uint16_t waterlevel = random(1, 600);  // ตัวเลขตัวอย่าง

  // คำนวณ lat/lng เป็น int32 (× 1e6)
  int32_t latInt = (int32_t)(LATITUDE * 1000000.0f);
  int32_t lngInt = (int32_t)(LONGITUDE * 1000000.0f);

  // แปลงเป็น unsigned สำหรับตัด byte ตาม two's complement
  uint32_t latU = (uint32_t)latInt;
  uint32_t lngU = (uint32_t)lngInt;

  Serial.printf("station=%d, P=%u, WL=%u, lat=%.6f, lng=%.6f\n",
                station_id, pressure, waterlevel, LATITUDE, LONGITUDE);

  // ----------------- สร้าง payload -----------------
  // format:
  // [0]  : stationId
  // [1-2]: pressure (uint16, big-endian)
  // [3-4]: waterlevel (uint16, big-endian)
  // [5-8]: latInt (int32, big-endian)
  // [9-12]: lngInt (int32, big-endian)

  appDataSize = 13;

  appData[0] = (uint8_t)station_id;

  appData[1] = (pressure >> 8) & 0xFF;
  appData[2] = pressure & 0xFF;

  appData[3] = (waterlevel >> 8) & 0xFF;
  appData[4] = waterlevel & 0xFF;

  appData[5] = (latU >> 24) & 0xFF;
  appData[6] = (latU >> 16) & 0xFF;
  appData[7] = (latU >> 8) & 0xFF;
  appData[8] = latU & 0xFF;

  appData[9]  = (lngU >> 24) & 0xFF;
  appData[10] = (lngU >> 16) & 0xFF;
  appData[11] = (lngU >> 8) & 0xFF;
  appData[12] = lngU & 0xFF;
}

/* callback ตอนรับ downlink */
void onRxData(uint8_t *payload, uint8_t size, uint8_t port)
{
  Serial.printf("Received Data on Port %d: ", port);
  for (int i = 0; i < size; i++)
  {
    Serial.printf("%02X ", payload[i]);
  }
  Serial.println();
}

RTC_DATA_ATTR bool firstrun = true;

void setup()
{
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

#ifdef WIFI_LORA_32_V4
  // เปิดไฟเลี้ยงจอ (สำหรับบอร์ด V4 ถ้ามีใช้)
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (firstrun)
  {
    LoRaWAN.displayMcuInit();  // Heltec จะขึ้นหน้าจอ initial
    firstrun = false;
  }
}

void loop()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
#if (LORAWAN_DEVEUI_AUTO)
    LoRaWAN.generateDeveuiByChipID();
#endif
    LoRaWAN.init(loraWanClass, loraWanRegion);
    // กำหนด DR default เวลา ADR ปิดได้ถ้าอยาก
    // LoRaWAN.setDefaultDR(3);
    break;
  }

  case DEVICE_STATE_JOIN:
  {
    // แสดงบนจอว่า กำลัง Join (เหมือน "Disconnect / กำลังเชื่อมต่อ" กับ TTN)
    LoRaWAN.displayJoining();
    LoRaWAN.join();
    break;
  }

  case DEVICE_STATE_SEND:
  {
    // แสดงบนจอว่า กำลังส่ง (แสดงว่า join แล้ว / connect อยู่)
    LoRaWAN.displaySending();

    digitalWrite(LED_PIN, HIGH);      // ไฟติดตอนส่ง
    prepareTxFrame(appPort);          // สร้างข้อมูล Uplink
    LoRaWAN.send();
    digitalWrite(LED_PIN, LOW);

    deviceState = DEVICE_STATE_CYCLE;
    break;
  }

  case DEVICE_STATE_CYCLE:
  {
    // เวลารอรอบถัดไป
    txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
  }

  case DEVICE_STATE_SLEEP:
  {
    // ถ้าส่งแบบ confirmed แล้วได้รับ ACK จะโชว์บนจอ (สื่อว่า connect OK)
    LoRaWAN.displayAck();
    LoRaWAN.sleep(loraWanClass);
    break;
  }

  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
  }
}
