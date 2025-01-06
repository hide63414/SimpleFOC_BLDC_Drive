#include <Arduino.h>
#include "M5Dial.h"
#include "ESP32_NOW.h"
#include "WiFi.h"


#define ESPNOW_WIFI_CHANNEL 4  // 1 - 14
//#define ESPNOW_ADVERTISING_STRING "ADVERTISING_ESP32_ESPNOW_STRING"
#define ADV_NUM 123456


M5Canvas canvas(&M5Dial.Display);

HWCDC USBSerial; 
// struct struct_Message {
//   int advNum;
//   float targetVelocity;
//   float currentVelocity;
// };

struct struct_Message {
  int advNum;
  float targetVelocity;
  float currentVelocity;
  float currentCurrent;
  float accx;
  float accy;
  float accz;
  float gyrox;
  float gyroy;
  float gyroz;
};

class MY_ESP_NOW_Peer : public ESP_NOW_Peer {
public:
  MY_ESP_NOW_Peer(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
  ~MY_ESP_NOW_Peer() {
  }
  bool Begin() {
    return add();
  }
  bool Send(const uint8_t *data, size_t len) {
    return send(data, len);
  }
  struct_Message commData;
  
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    memcpy(&commData, data, sizeof(commData));
    if (broadcast && (commData.advNum == ADV_NUM)) {
      // ADVERTISING Echo
      this->Send((const uint8_t *)&commData, sizeof(commData));
    } else {
      // Communication
      const uint8_t *mac = addr();
      //USBSerial.printf("Peer Receive(%02X:%02X:%02X:%02X:%02X:%02X) %s : %d,%f,%f\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (broadcast ? "Broadcast" : "Unicast"), commData.advNum, commData.targetVelocity, commData.currentVelocity);
    }
  }
};

MY_ESP_NOW_Peer *espnow_peer_broadcast = nullptr;
MY_ESP_NOW_Peer *espnow_peer = nullptr;

struct_Message advertisingData;
struct_Message sndData;             //送信用データ //受信データはsepnow_peer内で受けている


void espnow_receive(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  struct_Message rcvData;
  memcpy(&rcvData, data, sizeof(rcvData));
  if ((espnow_peer == nullptr) && (rcvData.advNum == ADV_NUM)) {
    // Add peer
    USBSerial.println("Add peer");
    espnow_peer = new MY_ESP_NOW_Peer(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);
    espnow_peer->Begin();

    // ADVERTISING Echo
    espnow_peer->Send((const uint8_t *)&rcvData, sizeof(rcvData));
    USBSerial.printf("Peer Send[%s] : %d\n", ARDUINO_BOARD, rcvData.advNum);
  }
  USBSerial.printf("Non Peer Receive(%02X:%02X:%02X:%02X:%02X:%02X) : %d\n", info->src_addr[0], info->src_addr[1], info->src_addr[2], info->src_addr[3], info->src_addr[4], info->src_addr[5], rcvData.advNum);
}

void task0(void* arg);
void task1(void* arg);

void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  USBSerial.begin(115200);
  delay(2000);

  canvas.createSprite(240, 240);

  advertisingData.advNum = ADV_NUM;
  sndData.advNum = 0;
  sndData.targetVelocity = 0.0f;
  sndData.currentVelocity = 0.0f;

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  if (!ESP_NOW.begin()) {
    ESP.restart();
  }

  ESP_NOW.onNewPeer(espnow_receive, NULL);

  espnow_peer_broadcast = new MY_ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);
  espnow_peer_broadcast->Begin();


  canvas.setTextDatum(top_left);
  canvas.setTextColor(GREEN);
  canvas.setTextFont(&fonts::Orbitron_Light_24);
  canvas.setTextSize(1);

  xTaskCreatePinnedToCore(task0, "Task_0", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task1, "Task_1", 4096, NULL, 1, NULL, 1);
}

long oldPosition = -999;
long newPosition = 0;
void dispDial();

void loop() {
  M5Dial.update();
  newPosition = M5Dial.Encoder.read();
  if (newPosition != oldPosition) {
      M5Dial.Speaker.tone(8000, 20);
      oldPosition = newPosition;
      //USBSerial.println(newPosition);
      sndData.targetVelocity = newPosition;
      //dispDial();
  }
  if (M5Dial.BtnA.wasPressed()) {
      M5Dial.Encoder.readAndReset();
  }
}

void dispDial(){
  canvas.setTextDatum(top_left);
  canvas.setTextFont(&fonts::Orbitron_Light_32);
  canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
  canvas.setCursor(32, 25);
  canvas.printf("DIAL:%3d",newPosition);
}

void dispConnection(){
  canvas.setTextDatum(top_center);
  canvas.setTextFont(&fonts::Orbitron_Light_24);
  if (espnow_peer == nullptr) {
    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.drawString("ADV",120,0);
  } else {
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString("CON",120,0);
  }
}

void dispMotor(){
  canvas.setTextDatum(top_left);
  if (espnow_peer == nullptr) {

  } else {
    canvas.setTextFont(&fonts::Orbitron_Light_24);

    canvas.setTextColor(TFT_GREEN, TFT_BLACK);
    canvas.drawString("PV", 10, 70);
    canvas.drawString("SV", 130, 70);
    canvas.setTextSize(1,2);
    canvas.drawString("/", 110, 90);
    canvas.setTextFont(&fonts::Font7);
    canvas.setTextSize(0.8);
    canvas.setCursor(10, 100);
    canvas.printf("%04.1f", espnow_peer->commData.currentVelocity);
    canvas.setCursor(130, 100);
    canvas.printf("%04.1f", espnow_peer->commData.targetVelocity);
    canvas.setTextSize(1);

    canvas.setTextFont(&fonts::Orbitron_Light_24);
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString("Current", 10, 140);
    canvas.setTextFont(&fonts::Font7);
    canvas.setTextSize(0.8);
    canvas.setCursor(30, 170);
    canvas.printf("%.2f", espnow_peer->commData.currentCurrent);
    canvas.setTextSize(1);

    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextFont(&fonts::Orbitron_Light_24);
    canvas.setCursor(80, 210);    
    canvas.printf("%d",espnow_peer->commData.advNum);
  }
}

void task0(void* arg) {
  while (1) {
      canvas.clear(); 
      dispDial();
      dispConnection();
      dispMotor();
      canvas.pushSprite(0,0);
      vTaskDelay(100);    
  }
}

void espComm(){
  espnow_peer->Send((uint8_t *)&sndData, sizeof(sndData));
  USBSerial.printf("%4.1f,%4.1f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n", 
    espnow_peer->commData.targetVelocity,
    espnow_peer->commData.currentVelocity,
    espnow_peer->commData.currentCurrent,
    espnow_peer->commData.accx,
    espnow_peer->commData.accy,
    espnow_peer->commData.accz,
    espnow_peer->commData.gyrox,
    espnow_peer->commData.gyroy,
    espnow_peer->commData.gyroz    
  );  
}

void task1(void* arg) {
  while (1) {
    if (espnow_peer == nullptr) {
      // ADVERTISING
      espnow_peer_broadcast->Send((const uint8_t *)&advertisingData, sizeof(advertisingData));
      USBSerial.printf("Broadcast Send[%s] : %d\n", ARDUINO_BOARD, advertisingData.advNum);
      vTaskDelay(3000);
    } else {
      // Communication
      espComm();
      vTaskDelay(100);
    }
  }
}
