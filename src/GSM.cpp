/*
              _   _     _                _                            _                   
             | | | |   (_)              | |                          | |                  
  _ __   ___ | |_| |__  _ _ __   __ _   | |_ ___     ___  ___  ___   | |__   ___ _ __ ___ 
 | '_ \ / _ \| __| '_ \| | '_ \ / _` |  | __/ _ \   / __|/ _ \/ _ \  | '_ \ / _ \ '__/ _ \
 | | | | (_) | |_| | | | | | | | (_| |  | || (_) |  \__ \  __/  __/  | | | |  __/ | |  __/
 |_| |_|\___/ \__|_| |_|_|_| |_|\__, |   \__\___/   |___/\___|\___|  |_| |_|\___|_|  \___|
                                 __/ |                                                 
                                |___/                                                  

This code is not part of the project yet and still very messy

*/

/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-sim800l-publish-data-to-cloud/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include "Keys.h"
#include "GSM.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Your GPRS credentials (leave empty, if not needed)
const char apn[] = "pinternet.interkom.de"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "";                 // GPRS User
const char gprsPass[] = "";                 // GPRS Password

// SIM card PIN (leave empty, if not defined)
const char simPIN[] = "";

// Server details
// The server variable can be just a domain name or it can have a subdomain. It depends on the service you are using
const char server[] = "maker.ifttt.com"; // domain name: example.com, maker.ifttt.com, etc
//char resource[] = "/trigger/gps/with/key/eNW13xRWBsX5nRiC5BR50-Jr2sXy4zFaA2FTfq1rUx?value1=52.443035807275834&value2=13.411784355487839";         // resource path, for example: /post-data.php
const int port = 80; // server port number

// TTGO T-Call pins
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800   // Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

#define uS_TO_S_FACTOR 1000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 3600  /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

bool GSM::setPowerBoostKeepOn(int en)
{
    I2CPower.beginTransmission(IP5306_ADDR);
    I2CPower.write(IP5306_REG_SYS_CTL0);
    if (en)
    {
        I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
    }
    else
    {
        I2CPower.write(0x35); // 0x37 is default reg value
    }
    return I2CPower.endTransmission() == 0;
}

TinyGPSPlus gps;

void GSM::GSMsetup()
{
    // Set serial monitor debugging window baud rate to 115200
    SerialMon.begin(9600);

    // Keep power when running from battery
    bool isOk = setPowerBoostKeepOn(1);
    SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

    // Set modem reset, enable, power pins
    pinMode(MODEM_PWKEY, OUTPUT);
    pinMode(MODEM_RST, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);
    digitalWrite(MODEM_PWKEY, LOW);
    digitalWrite(MODEM_RST, HIGH);
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000);

    // Restart SIM800 module, it takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.print("Initializing modem... ");
    //modem.restart();
    modem.init();
    SerialMon.println("OK");
    // use modem.init() if you don't need the complete restart

    // Unlock your SIM card with a PIN if needed
    if (strlen(simPIN) && modem.getSimStatus() != 3)
    {
        modem.simUnlock(simPIN);
    }

    // Configure the wake up source as timer wake up
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

float latitude;
float longitude;

void GSM::sendPosition(int wpNr)
{

    SoftwareSerial serialConnection(19, 21);
    serialConnection.begin(9600);

    do
    {
        while (serialConnection.available()) //while gps sends characters
        {
            gps.encode(serialConnection.read());
        }
        latitude = gps.location.lat();
        longitude = gps.location.lng();
    } while (gps.location.lat() == 0.0 && gps.location.lng() == 0.0); //until it has read a decent value

    SerialMon.println(latitude);
    SerialMon.println(longitude);

    Keys myKey;

    String resource = "/trigger/gps/with/key/" + myKey.ifttt_key 
    + "?value1=" + String(latitude, 6) + "&value2=" + String(longitude, 6) + "&value3=" + String(wpNr);

    Serial.print("wp1: ");
    Serial.println(wpNr);

    SerialMon.print("Connecting to APN: ");
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
        SerialMon.println(" fail");
    }
    else
    {
        SerialMon.println(" OK");

        SerialMon.print("Connecting to ");
        SerialMon.print(server);
        if (!client.connect(server, port))
        {
            SerialMon.println(" fail");
        }
        else
        {
            SerialMon.println(" OK");

            // Making an HTTP POST request
            SerialMon.println("Performing HTTP POST request...");

            String httpRequestData = "asdf";

            client.print(String("POST ") + resource + " HTTP/1.1\r\n");
            client.print(String("Host: ") + server + "\r\n");
            client.println("Connection: close");
            client.println("Content-Type: application/x-www-form-urlencoded");
            client.print("Content-Length: ");
            client.println(httpRequestData.length());
            client.println();
            client.println(httpRequestData);

            unsigned long timeout = millis();
            while (client.connected() && millis() - timeout < 10000L)
            {
                // Print available data (HTTP response from server)
                while (client.available())
                {
                    char c = client.read();
                    SerialMon.print(c);
                    timeout = millis();
                }
            }
            SerialMon.println();

            // Close client and disconnect
            client.stop();
            SerialMon.println(F("Server disconnected"));
            modem.gprsDisconnect();
            SerialMon.println(F("GPRS disconnected"));
        }
    }

    // delay(40000);
    // Put ESP32 into deep sleep mode (with timer wake up)
    //esp_deep_sleep_start();
}