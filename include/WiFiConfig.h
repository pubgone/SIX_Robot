#ifndef _WIFICONFIG_H_
#define _WIFICONFIG_H_
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <UARTConfig.h>
#include <ArduinoOTA.h>
#include <TCPConfig.h>
// #include <TCP.h>

/*********WiFi Config*********/
// #define defaultSSID "666666"       // 默认WiFi名称
// #define defaultPassward "88888888" // 默认WiFi密码
#define defaultSSID "DebugPC"       // 默认WiFi名称
#define defaultPassward "asdasdasd" // 默认WiFi密码



/*********Static IP Config*********/

#define defaultLocalIP \
    {                  \ 
        192,168, 137, 9} // 默认本地IP
#define defaultGateway \
    {                  \
        192, 168, 31, 1} // 默认网关
#define defaultSubnetMask \
    {                     \
        255, 255, 255, 0} // 默认子网掩码
#define defaultPrimaryDNS \
    {                     \
        223, 5, 5, 5} // 默认首选DNS
#define defaultSecondaryDNS \
    {                       \
        114, 114, 114, 114} // 默认备用DNS
/*********OTA Config*********/
#define defaultOTAHostname "ESP32" // 默认OTA主机名
#define defaultOTAPassword "OTAP"  // 默认OTA密码
#define defaultOTAPort 8266        // 默认OTA端口
/*********Other Config*********/
#define defaultReconnectTime 1000 // 默认重连时间

class WiFiConfig : public WiFiClient, public WiFiClass
{
public:
    String ssid;     // WiFi名称
    String passward; // WiFi密码
    IPAddress localIP;
    IPAddress gateway;
    IPAddress subnetMask;
    IPAddress primaryDNS;
    IPAddress secondaryDNS;

    WiFiClass WiFi;

    TaskHandle_t OTA_TaskHandle = NULL;

    const char *OTAHostname;
    const char *OTAPassword;

    WiFiConfig();
    bool WiFiInit();
    bool WiFiInit(String ssid, String passward);
    bool WiFiInit(String ssid, String passward, IPAddress localIP, IPAddress gateway, IPAddress subnetMask, IPAddress primaryDNS, IPAddress secondaryDNS);
    void OTAconfig();
    void OTAconfig(const char *OTAHostname, const char *OTAPassword);
};
void OTATask(void *pvParam);
void WiFiStatusTask(void *pvParam);
#endif //_WIFICONFIG_H_