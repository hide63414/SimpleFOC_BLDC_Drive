#include <Arduino.h>
#include <M5Unified.h>
// Open loop motor control example
#include <SimpleFOC.h>

#include "ESP32_NOW.h"
#include "WiFi.h"

#define ESPNOW_WIFI_CHANNEL 4  // 1 - 14
#define ADV_NUM 123456

M5Canvas canvas(&M5.Display);

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

  //登録済みのピアから受信した場合にはこの関数が呼び出される
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    memcpy(&commData, data, sizeof(commData));
    if (broadcast && (commData.advNum == ADV_NUM)) {
      // ADVERTISING Echo
      this->Send((const uint8_t *)&commData, sizeof(commData));
    } else {
      // Communication
      const uint8_t *mac = addr();
      //Serial.printf("Peer Receive(%02X:%02X:%02X:%02X:%02X:%02X) %s : %d,%f,%f\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (broadcast ? "Broadcast" : "Unicast"), commData.advNum, commData.currentVelocity, commData.targetVelocity);
    }
  }
};

MY_ESP_NOW_Peer *espnow_peer_broadcast = nullptr;
MY_ESP_NOW_Peer *espnow_peer = nullptr;

struct_Message advertisingData;     //アドバタイジング用データ
struct_Message sndData;             //送信用データ //受信データはsepnow_peer内で受けている

//ピアに登録していない場合には個別のコールバック関数が呼び出される
void espnow_receive(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  struct_Message rcvData;
  memcpy(&rcvData, data, sizeof(rcvData));
  if ((espnow_peer == nullptr) && (rcvData.advNum == ADV_NUM)) {
    // Add peer
    Serial.println("Add peer");
    espnow_peer = new MY_ESP_NOW_Peer(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);
    espnow_peer->Begin();

    // ADVERTISING Echo
    espnow_peer->Send((const uint8_t *)&rcvData, sizeof(rcvData));
    Serial.printf("Peer Send[%s] : %d\n", ARDUINO_BOARD, rcvData.advNum);
  }
  Serial.printf("Non Peer Receive(%02X:%02X:%02X:%02X:%02X:%02X) : %d\n", info->src_addr[0], info->src_addr[1], info->src_addr[2], info->src_addr[3], info->src_addr[4], info->src_addr[5], rcvData.advNum);
}

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
// https://www.amazon.co.jp/dp/B089SYW5W3  MAX1.3A
BLDCMotor motor = BLDCMotor(7, 5.6, 260);  // BDUAV 2206-260KV 14 poles 5.6Ω 260KV 
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);
// velocity set point variable
float targetVelocity = 0;
float currentVelotity = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
   command.scalar(&targetVelocity, cmd);
   espnow_peer->commData.targetVelocity = targetVelocity;
}
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void task0(void* arg);
void task1(void* arg);

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;   // default=115200. if "Serial" is not needed, set it to 0.
  M5.begin(cfg);
  // use monitoring with serial 
  Serial.begin(115200);
  M5.Display.setCursor(0, 0);
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.printf("Motor\n Setting!");
  delay(2000);

  canvas.createSprite(120, 120);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 5;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 5;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

   // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 5.0f;
  motor.PID_velocity.D = 0.0001f;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // comment out if not needed
  motor.useMonitoring(Serial);
  //motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
  motor.monitor_downsample = 0; // disable monitor at first - optional

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('M', doMotor,"my motor");
  //command.add('A', doMotor,"motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));

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
 
  xTaskCreatePinnedToCore(task0, "Task_0", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task1, "Task_1", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(targetVelocity);
  //motor.move();
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();
  
  // user communication
  command.run();
}

void espComm(){  
  sndData.advNum++;                                   //送信カウント
  sndData.targetVelocity = targetVelocity;
  sndData.currentVelocity = motor.shaft_velocity;
  sndData.currentCurrent = motor.current_sp;
  espnow_peer->Send((uint8_t *)&sndData, sizeof(sndData));
  //Serial.printf("MotorState:TVel%3.1f,CVel%3.1f\n", sndData.targetVelocity, sndData.currentVelocity);
}

void task1(void* arg) {
  while (1) {
    if (espnow_peer == nullptr) {
      // ADVERTISING
      espnow_peer_broadcast->Send((const uint8_t *)&advertisingData, sizeof(advertisingData));
      Serial.printf("Broadcast Send[%s] : %d\n", ARDUINO_BOARD, advertisingData.advNum);
      vTaskDelay(3000);
    } else {
      // Communication
      espComm();
      vTaskDelay(1000);
    }
  }
}

void task0(void* arg) {
  while (1) {
    if (M5.Imu.isEnabled()) {
      float val[6];
      M5.Imu.getAccel(&val[0], &val[1], &val[2]);
      M5.Imu.getGyro(&val[3], &val[4], &val[5]);
      sndData.accx = val[0];
      sndData.accy = val[1];
      sndData.accz = val[2];
      sndData.gyrox = val[3];
      sndData.gyroy = val[4];
      sndData.gyroz = val[5];
      //Serial.printf("%f,%f,%f,%f,%f,%f\n", val[0],val[1],val[2],val[3],val[4],val[5]);
    }
    canvas.clear();
    if (espnow_peer == nullptr) {
      canvas.setCursor(0, 0);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(TFT_CYAN, TFT_BLACK);
      canvas.printf("Motor Ready!");

      canvas.setFont(&fonts::Font4);
      canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
      canvas.drawString("Advertise",0,20);
    } else {
      targetVelocity = espnow_peer->commData.targetVelocity;
      canvas.setTextSize(1);
      canvas.setFont(&fonts::Font4);
      canvas.setTextColor(TFT_CYAN, TFT_BLACK);
      canvas.drawString("Connect",0,0);
      canvas.setTextSize(0.8);
      canvas.setTextColor(TFT_GREEN, TFT_BLACK);
      canvas.drawString("PV",0,25);
      canvas.drawString("SV",0,75);

      canvas.setTextSize(1);
      canvas.setFont(&fonts::Font7);
      canvas.setTextColor(TFT_GREEN, TFT_BLACK);
      canvas.setTextSize(0.8);
      canvas.setCursor(20, 30);      
      canvas.printf("%4.1f",motor.shaft_velocity);
      canvas.setTextSize(0.8);
      canvas.setCursor(20, 80);      
      canvas.printf("%4.1f",targetVelocity);
    }
    canvas.pushSprite(0,0);
    vTaskDelay(100);
  }
}