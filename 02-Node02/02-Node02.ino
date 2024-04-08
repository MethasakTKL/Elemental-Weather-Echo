//! Slave Node 02
String NodeID = "0x429";
////////////////////////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <GUVA_S12SD.h>
////////////////////////////////////////////////////////////////////////////////////
//* Dust–
#define DustPin 39
#define dustLedPower 26
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
////////////////////////////////////////////////////////////////////////////////////
//* UV
#define UV_sensor 4
GUVA_S12SD guva_s12sd;

////////////////////////////////////////////////////////////////////////////////////
//* Rain
#define rainAnalogPin 36
////////////////////////////////////////////////////////////////////////////////////
#define DHTPIN 16
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
////////////////////////////////////////////////////////////////////////////////////
//* LoRa
#define ss 5
#define rst 13
#define dio0 27
////////////////////////////////////////////////////////////////////////////////////
//* LED Packet
#define LED_Slave 14
int CountPacket = 0;
////////////////////////////////////////////////////////////////////////////////////
//* Set RTC Button
#define buttonPin 12
unsigned long pressStartTime = 0;
bool holding = false;
////////////////////////////////////////////////////////////////////////////////////
//* Var
String Date, Month, Year, Hour, Min, Sec, PacketCheck;
float TemperatureValue, HumidityValue, DustValue, UvValue, RainValue;
String RT_NodeID, RT_Date, RT_Month, RT_Year, RT_Hour, RT_Min, RT_Sec, RT;
unsigned long takeActionStartTime = 0;
////////////////////////////////////////////////////////////////////////////////////
String addLeadingZero(int number)
{
    if (number < 10)
    {
        return "0" + String(number);
    }
    else
    {
        return String(number);
    }
}

void ResetESP()
{
    if (millis() - takeActionStartTime >= 10 * 60 * 1000) // todo 10 minute Masternode will reboot
    {
        Serial.println("Safe Mode *************************");
        for (int i = 0; i < 10; i++)
        {
            Serial.println(".");
        }
        takeActionStartTime = millis();
        ESP.restart();
    }
}
void alert()
{
    digitalWrite(LED_Slave, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    digitalWrite(LED_Slave, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    digitalWrite(LED_Slave, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    digitalWrite(LED_Slave, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}


void SetupLoRa()
{
    while (!Serial)
        ;
    Serial.println("EWE Slava Node2 (0x429)");
    Serial.println("Starting...");
    LoRa.setPins(ss, rst, dio0);

    if (!LoRa.begin(433E6))
    {
        Serial.println("Starting LoRa failed!");
        alert();
        ESP.restart();
        while (1)
            ;
    }
}

void sendPacket(String NodeID,
                float TemperatureValue, float HumidityValue, float DustValue,
                float UvValue, float RainValue, String PacketCheck)
{
    LoRa.beginPacket();
    //  [NodeID,Date,Month,Year,Hour,Min,Sec,TemperatureValue,HumidityValue,DustValue,UvValue,RainValue]
    String DataPacked = NodeID + "," +
                        TemperatureValue + "," + HumidityValue + "," + DustValue + "," + UvValue + "," + RainValue + "," + PacketCheck;

    // LoRa Sent
    LoRa.print(DataPacked);
    // Serial Monitor
    Serial.println(DataPacked);
    // Count Packet Sent
    CountPacket++;
    Serial.print("Sent packet : ");
    Serial.println(CountPacket);
    LoRa.endPacket();
}

void Data()
{
    digitalWrite(dustLedPower, LOW); // power on the LED
    delayMicroseconds(280);
    voMeasured = analogRead(DustPin); // read the dust value
    digitalWrite(dustLedPower, HIGH); // turn the LED off
    delayMicroseconds(9620);
    calcVoltage = voMeasured * (5.0 / 4095);
    dustDensity = ((0.17 * calcVoltage - 0.1) * 1000);
    dustDensity = abs(dustDensity);

    TemperatureValue = dht.readTemperature();
    HumidityValue = dht.readHumidity();
    DustValue = dustDensity;
    UvValue = analogRead(UV_sensor) * (5.0 / 4095.0) * 1000;
    RainValue = analogRead(rainAnalogPin);
    PacketCheck = "1";

    if (isnan(HumidityValue) || isnan(TemperatureValue))
    {
        Serial.println("Failed to read from DHT sensor!");
        alert();
        ESP.restart();
        return;
    }
}

//! Get Dust Value Task
void dustTask(void *parameter)
{
    // task initialization here
    while (1)
    {
        digitalWrite(dustLedPower, LOW); // power on the LED
        delayMicroseconds(280);
        voMeasured = analogRead(DustPin); // read the dust value
        digitalWrite(dustLedPower, HIGH); // turn the LED off
        delayMicroseconds(9620);
        calcVoltage = voMeasured * (5.0 / 4095);
        dustDensity = ((0.17 * calcVoltage - 0.1) * 1000);
        dustDensity = abs(dustDensity);
        // Serial.print("Raw Signal Value (0-1023): ");
        // Serial.print(voMeasured);
        // Serial.print(" - Voltage: ");
        // Serial.print(calcVoltage);
        // Serial.print(" - Dust Density: ");
        // Serial.println(dustDensity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

//! Sent Packet Task
void packetTask(void *parameter)
{
    while (1)
    {
        Data(); // Data

        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            // Read Packet and seperate data to variable
            //  read packet
            String receivedData = "";
            while (LoRa.available())
            {
                receivedData += (char)LoRa.read();
            }
            Serial.println();
            String dataArray[8]; // จำนวนข้อมูลทั้งหมดมี 13 ตัว
            int index = 0;
            int lastIndex = 0;
            while (index < 8)
            {
                int commaIndex = receivedData.indexOf(',', lastIndex);
                if (commaIndex != -1)
                {
                    dataArray[index] = receivedData.substring(lastIndex, commaIndex);
                    lastIndex = commaIndex + 1;
                }
                else
                {
                    dataArray[index] = receivedData.substring(lastIndex);
                    break;
                }
                index++;
            }
            uint16_t RT_Year;
            uint8_t RT_Date, RT_Month, RT_Hour, RT_Min, RT_Sec;

            RT_NodeID = dataArray[0];
            RT_Date = dataArray[1].toInt();
            RT_Month = dataArray[2].toInt();
            RT_Year = dataArray[3].toInt();
            RT_Hour = dataArray[4].toInt();
            RT_Min = dataArray[5].toInt();
            RT_Sec = dataArray[6].toInt();
            RT = dataArray[7];
            // Serial.println("RecievedData : " + receivedData); // For debug
            // Check Node
            if (receivedData == NodeID)
            {
                Serial.println("RequestID : " + receivedData);
                sendPacket(NodeID, TemperatureValue, HumidityValue, DustValue, UvValue, RainValue, PacketCheck);
                alert();
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    // Determine pinMode
    pinMode(LED_Slave, OUTPUT);
    pinMode(dustLedPower, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    // Setup Function
    guva_s12sd.initialize(UV_sensor);
    SetupLoRa();
    dht.begin();

    // Task
    xTaskCreatePinnedToCore(packetTask, "led-task", 2048, NULL, 1, NULL, 1);
}

void loop()
{
    ResetESP();
}
