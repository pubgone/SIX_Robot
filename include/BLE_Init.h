<<<<<<< HEAD
#ifndef _BLE_INIT_H_
#define _BLE_INIT_H_
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
// #include <main.h>
// #include <Robot.h>
#include <Arduino.h>
#include <Servo.h>

#define SERVICE_UUID "b408e1a0-3d8a-11ed-b878-0242ac120002"        // 服务UUID
#define CHARACTERISTIC_UUID "dfd84064-3d8a-11ed-b878-0242ac120002" // 特征UUID
#define CONTROL_UUID "de045162-3d97-11ed-b878-0242ac120002"        // 控制特征UUID

extern TaskHandle_t BLEServer_TaskHandle;
extern TaskHandle_t BLEMainLogic_TaskHandle;
extern TaskHandle_t Control_TaskHandle;
extern BLECharacteristic controlCharacteristic;
// extern char state;
extern bool connected_state;
extern bool flag;
extern char state;
extern const char default_state;
// extern bool connected_state;

void BLE_Init();
void BLEServer_Task(void *pvParam);
void BLEMainLogic_Task(void *pvParam);
#endif
=======
// #ifndef _BLE_INIT_H_
// #define _BLE_INIT_H_
// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>
// #include <Arduino.h>

// class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
// {
//   void onResult(BLEAdvertisedDevice advertisedDevice)
//     {
//     Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
//   }
// };
// class BLE_Config : public BLEDevice, public BLEServer, public BLEService, public BLECharacteristic, public BLEScan
// {
//   BLEScan *pBLEScan;                                  // BLE扫描
//   BLEServer *pServer;                                 // BLE服务端
//   String SERVICE_UUID;                                // 服务UUID
//   String CHARACTERISTIC_UUID;                         // 特征UUID
//   String DeviceName;                                  // 设备密码
//   String DeviceValue;                                 // 设备值
//   BLE_Config();                                       // 默认使用BLE客户端
//   BLE_Config(String DeviceName, String SERVICE_UUID); // BLE服务端
//   ~BLE_Config();
// };

// #endif
>>>>>>> 0faa1efc23617dff3a3c3e1d4285a0a950e91692
