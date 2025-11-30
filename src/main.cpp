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
 * */
 
#include "LoRaWan_APP.h"
#include <Arduino.h>
 
#define LED_PIN 35
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
/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */
uint8_t confirmedNbTrials = 4;
 
int station_id = 1;
 
/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port)
{
  // จำลองค่าข้อมูลเซนเซอร์
  uint16_t pressure = random(1, 5); // 20.00°C - 35.00°C (×100)
  uint16_t waterlevel = random(1, 600);    // 30.00% - 80.00% (×100)
 
  Serial.printf("T: %.2f, H: %.2f, L: %d\n", pressure, waterlevel);
 
  // เตรียม payload
  appDataSize = 7;
  appData[0] = station_id;
 
  appData[1] = (pressure >> 8) & 0xFF;
  appData[2] = pressure  & 0xFF;
 
  appData[3] = (waterlevel >> 8) & 0xFF;
  appData[4] = waterlevel & 0xFF;

}
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
 
  if (firstrun)
  {
    LoRaWAN.displayMcuInit();
    firstrun = false;
  }
}
 
void loop()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
    LoRaWAN.init(loraWanClass, loraWanRegion);
    break;
 
  case DEVICE_STATE_JOIN:
    LoRaWAN.join();
    break;
 
  case DEVICE_STATE_SEND:
    prepareTxFrame(appPort); // สร้างข้อมูล Uplink
    LoRaWAN.send();
    deviceState = DEVICE_STATE_CYCLE;
    break;
 
  case DEVICE_STATE_CYCLE:
    txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
 
  case DEVICE_STATE_SLEEP:
    LoRaWAN.sleep(loraWanClass);
    break;
 
  default:
    deviceState = DEVICE_STATE_INIT;
    break;
  }
}