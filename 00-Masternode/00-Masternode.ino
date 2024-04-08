//! Master Node
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <RTClib.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define APP_WIFI_SSID "Ohm"
#define APP_WIFI_PASS "methasak2023"

#define APP_MQTT_SERVER "172.20.10.14"
#define APP_MQTT_PORT 1883

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
////////////////////////////////////////////////////////////////////////////////////
//* Keypad
#define I2CADDR 0x20
const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

byte rowPins[ROWS] = {7, 6, 5, 4};
byte colPins[COLS] = {3, 2, 1, 0};

Keypad_I2C keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS, I2CADDR);
String keyinputDate = "";
String keyinputTime = "";

byte keyindexDate = 0;
byte keyindexTime = 0;

String ModeState = "A";
int LineState = 0;
////////////////////////////////////////////////////////////////////////////////////
//* LCD 2004
LiquidCrystal_I2C lcd(0x27, 20, 4);
////////////////////////////////////////////////////////////////////////////////////
//* RTC
RTC_DS3231 rtc;
DateTime now;
////////////////////////////////////////////////////////////////////////////////////
//* LoRa
#define ss 5
#define rst 13
#define dio0 26
////////////////////////////////////////////////////////////////////////////////////
//* LED Packet
#define LED_Master 14
int CountPacketrecieved = 1;
int NodeSate = 1;
////////////////////////////////////////////////////////////////////////////////////
//* Var
String NodeID, Date, Month, Year, Hour, Min, Sec, TempValue, HumidityValue, DustValue, UvValue, RAW_DustValue, RAW_UvValue, RainValue, Rainstr, PacketCheck;
String NameNode;
uint16_t RT_Year;
uint8_t RT_Date, RT_Month, RT_Hour, RT_Min, RT_Sec;
unsigned long previousMillis = 0;
unsigned long takeActionStartTime = 0;

///////////////////////////////////////////////////\////////////////////////
String TempValue_1, HumidityValue_1, DustValue_1, UvValue_1, RAW_DustValue_1, RAW_UvValue_1, RainValue_1;
String TempValue_2, HumidityValue_2, DustValue_2, UvValue_2, RAW_DustValue_2, RAW_UvValue_2, RainValue_2;

///////////////////////////////////////////////////\////////////////////////
float countAVG_1 = 0;
float countAVG_2 = 0;
float AVGDustValue_1, AVGUvValue_1;
float AVGDustValue_2, AVGUvValue_2;
// int DustValue_1, UvValue_1;
// int DustValue_2, UvValue_2;
////////////////////////////////////////////////////////////////////////////////////

void setupWiFi()
{
    Serial.println("Connecting to " + String(APP_WIFI_SSID) + " ...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(APP_WIFI_SSID, APP_WIFI_PASS);

    if (WiFi.waitForConnectResult() == WL_CONNECTED)
    {
        Serial.println("Wi-Fi connected, IP address: " + WiFi.localIP().toString());
    }
    else
    {
        Serial.println("Failed to connect to the Wi-Fi network, restarting...");
        delay(2000);
        ESP.restart();
    }
}

void mqttCallback(const char *topic, byte *message, unsigned int length)
{
    // Not used in this example
}

void setupMqttClient()
{
    mqttClient.setServer(APP_MQTT_SERVER, APP_MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    Serial.println("Connecting to MQTT broker " + String(APP_MQTT_SERVER) + ":" + String(APP_MQTT_PORT));

    while (!mqttClient.connected())
    {
        if (mqttClient.connect("iot-controller"))
        {
            Serial.println("Connected to MQTT broker");
        }
        else
        {
            Serial.println("Failed to connect to MQTT broker, retrying in 5 seconds...");
            delay(5000);
        }
    }
}

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

void SetupLoRa()
{
    while (!Serial)
        ;
    Serial.println("\n");
    Serial.println("EWE (Master node) is Starting..., Wait to recieve from EWE (Slave Node)");
    LoRa.setPins(ss, rst, dio0);
    if (!LoRa.begin(433E6))
    {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }
}

void SetupRTC()
{
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        ESP.restart();
        while (1)
            ;
    }

    // ตรวจสอบว่า RTC ได้หายไปเวลาหรือไม่
    if (rtc.lostPower())
    {
        Serial.println("RTC lost power, let's set the time!");
        rtc.adjust(DateTime(__DATE__, __TIME__));
    }
}

void ResetESP()
{
    if (millis() - takeActionStartTime >= 15 * 60 * 1000) // todo 10 minute Masternode will reboot
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

// Check Level Fuction
void DustLevel(int DustValue)
{
    if (DustValue < 12)
    {
        Serial.print("Very Good,(");
        Serial.print(DustValue);
        Serial.print(" ug/m^2)");
    }
    else if (DustValue < 55)
    {
        Serial.print("Moderate,(");
        Serial.print(DustValue);
        Serial.print(" ug/m^2)");
    }
    else if (DustValue < 70)
    {
        Serial.print("Unhealthy,Sensitive (");
        Serial.print(DustValue);
        Serial.print(" ug/m^2)");
    }
    else if (DustValue < 150)
    {
        Serial.print("Unhealthy,(");
        Serial.print(DustValue);
        Serial.print(" ug/m^2)");
    }
    else if (DustValue < 250)
    {
        Serial.print("Very Unhealthy,(");
        Serial.print(DustValue);
        Serial.print(" ug/m^2)");
    }
    else if (DustValue >= 250)
    {
        Serial.print("Hazardous,(");
        Serial.print(DustValue);
        Serial.print(" ug/m^2)");
    }
}

String UvLevel(int UvValue)
{
    if (UvValue < 227)
    {
        return "0";
    }
    else if (UvValue < 318)
    {
        return "1";
    }
    else if (UvValue < 408)
    {
        return "2";
    }
    else if (UvValue < 503)
    {
        return "3";
    }
    else if (UvValue < 606)
    {
        return "4";
    }
    else if (UvValue < 696)
    {
        return "5";
    }
    else if (UvValue < 795)
    {
        return "6";
    }
    else if (UvValue < 881)
    {
        return "7";
    }
    else if (UvValue < 976)
    {
        return "8";
    }
    else if (UvValue < 1079)
    {
        return "9";
    }
    else if (UvValue < 1170)
    {
        return "10";
    }
    else // if (UvValue >= 1170)
    {
        return "11";
    }
}

void RainLevel(int RainValue)
{
    if (RainValue < 3000)
    {
        Rainstr = "Yes ";
        Serial.print(Rainstr + " (Value = " + RainValue + ")");
    }
    else
    {
        Rainstr = "No ";
        Serial.print(Rainstr + " (Value = " + RainValue + ")");
    }
}



// Read Packet Function
void dataToVar()
{
    // read packet
    String receivedData = "";
    Serial.println(receivedData);
    while (LoRa.available())
    {
        receivedData += (char)LoRa.read();
    }
    String dataArray[7];
    int index = 0;
    int lastIndex = 0;

    while (index < 7)
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
    NodeID = dataArray[0];
    TempValue = dataArray[1];
    HumidityValue = dataArray[2];
    DustValue = dataArray[3];
    UvValue = dataArray[4];
    RainValue = dataArray[5];
    PacketCheck = dataArray[6];

    /////////////////////////////////////////////////////////////////////
    // Check Node to display Value Dust & UV
    if (NodeID == "0x137")
    {
        TempValue_1 = TempValue;
        HumidityValue_1 = HumidityValue;
        DustValue_1 = DustValue;
        UvValue_1 = UvValue;
        RainValue_1 = RainValue;

        mqttClient.publish("node1/Temp", String(TempValue_1).c_str());
        mqttClient.publish("node1/Humidity", String(HumidityValue_1).c_str());
        mqttClient.publish("node1/Dust", String(DustValue_1).c_str());
        mqttClient.publish("node1/UV", String(UvValue_1).c_str());
        mqttClient.publish("node1/Rain", String(RainValue_1).c_str());
    }
    else if (NodeID == "0x429")
    {
        TempValue_2 = TempValue;
        HumidityValue_2 = HumidityValue;
        DustValue_2 = DustValue;
        UvValue_2 = UvValue;
        RainValue_2 = RainValue;

        mqttClient.publish("node2/Temp", String(TempValue_2).c_str());
        mqttClient.publish("node2/Humidity", String(HumidityValue_2).c_str());
        mqttClient.publish("node2/Dust", String(DustValue_2).c_str());
        mqttClient.publish("node2/UV", String(UvValue_2).c_str());
        mqttClient.publish("node2/Rain", String(RainValue_2).c_str());
    }
}

// Display Function
void DisplaySerial()
{
    Serial.println("\n");
    Serial.print("Packet recieved : ");
    Serial.println(CountPacketrecieved);
    CountPacketrecieved++;
    if (NodeID == "0x137")
    {
        NameNode = "Node1";
    }
    else if (NodeID == "0x429")
    {
        NameNode = "Node2";
    }
    Serial.println("--------------------------------------------");
    Serial.println("Slave " + NameNode + " (" + NodeID + ")   PacketCheck: " + PacketCheck);
    Serial.print("RSSI : ");
    Serial.println(LoRa.packetRssi());
    Serial.println("----------------" + NodeID + "----------------");

    // todo ------------------Time
    Serial.print("Time: " + Date + "-" + Month + "-" + Year + "  " + Hour + ":" + Min + ":" + Sec);
    Serial.println();

    // todo ------------------Temperature
    float temperature = TempValue.toFloat(); // Convert string to float
    Serial.print("Temp: ");
    Serial.print(temperature, 2); // Display temperature with 2 decimal places
    Serial.println(" °C");

    // todo ------------------Humidity
    Serial.print("Humidity: " + HumidityValue + " %");
    Serial.println();

    // todo ------------------Air quality
    Serial.print("Air quality: ");
    DustLevel(DustValue.toInt());
    Serial.println();
    // todo ------------------UVindex
    Serial.print("UVindex: ");
    UvLevel(UvValue.toInt());
    Serial.printf(" (%d mV) ", UvValue.toInt());
    Serial.println();

    // todo ------------------RainDetect
    Serial.print("RainDetect: ");
    RainLevel(RainValue.toInt());
    Serial.println();
    Serial.println("--------------------------------------------");
}

void DisplayLCD()
{
    lcd.clear();
    // todo ------------------1's line
    lcd.setCursor(0, 0);
    String TimeLCD = Date + "-" + Month + " " + Hour + ":" + Min + ":" + Sec;
    lcd.print(NameNode + " " + TimeLCD);

    // todo ------------------2nd line
    lcd.setCursor(0, 1);
    String TempLCD = "T:" + TempValue + "    H:" + HumidityValue;
    lcd.print(TempLCD);

    // todo ------------------3rd line
    lcd.setCursor(0, 2);
    // lcd.print("D:" + DustValue + "   Uv:" + UvValue);
    lcd.print("D:" + DustValue + " ug/m3 Uv:" + UvLevel(UvValue.toInt()));

    // todo ------------------4th line
    lcd.setCursor(0, 3);
    lcd.print("R:" + Rainstr + " " + RainValue + " " + UvValue);
}

void sendRequest(String RequestID)
{
    LoRa.beginPacket();
    LoRa.print(RequestID);
    Serial.println();
    Serial.println("RequestID : " + RequestID);
    LoRa.endPacket();
}

void sendSetRTC(String Data_SetRTC)
{
    LoRa.beginPacket();
    LoRa.print(Data_SetRTC);
    Serial.println();
    Serial.println("SentSetRTC : " + Data_SetRTC);
    LoRa.endPacket();
}

//! Get Dust Value Task
void DisplayTask(void *parameter)
{
    while (1)
    {
        int i;
        char key = keypad.getKey();
        if (key != NO_KEY)
        {
            if (key == 'A')
            {
                lcd.clear();
                ModeState = "A";
                lcd.setCursor(7, 1);
                lcd.print("Display");
                lcd.setCursor(8, 2);
                lcd.print("Mode");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                lcd.clear();
            }
            else if (key == 'B' && ModeState == "A")
            {
                lcd.clear();
                //! In Mode B always Input = ""
                keyinputDate = "";
                keyinputTime = "";
                /////////////////////////
                ModeState = "B";
                lcd.setCursor(7, 1);
                lcd.print("Set RTC");
                lcd.setCursor(8, 2);
                lcd.print("Mode");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                lcd.clear();
                LineState = 0;
            }
            //////////////////////////////////////////////////////////////////
            if (ModeState == "B")
            {
                if (isdigit(key))
                { // ตรวจสอบว่าเป็นตัวเลขหรือไม่
                    lcd.setCursor(19, 2);
                    lcd.print(key);
                    if (LineState == 1)
                    {
                        keyinputDate += key;
                        keyindexDate++;
                        lcd.setCursor(1, 0);
                        lcd.print(keyinputDate);
                    }
                    else if (LineState == 2)
                    {
                        keyinputTime += key;
                        keyindexTime++;
                        lcd.setCursor(1, 1);
                        lcd.print(keyinputTime);
                    }
                }
                else if (key == 'B')
                {
                    if (LineState == 0)
                    {
                        Serial.println("Line 1");
                        LineState = 1;
                    }
                    else if (LineState == 1)
                    {
                        Serial.println("Line 2");
                        LineState = 2;
                    }
                    else if (LineState == 2)
                    {
                        Serial.println("Line 1");
                        LineState = 1;
                    }
                }
                else if (key == 'C')
                {
                    if (LineState == 1)
                    {
                        keyinputDate += ",";
                        keyindexDate++;
                        lcd.setCursor(1, 0);
                        lcd.print(keyinputDate);
                    }
                    else if (LineState == 2)
                    {
                        keyinputTime += ",";
                        keyindexTime++;
                        lcd.setCursor(1, 1);
                        lcd.print(keyinputTime);
                    }
                }
                else if (key == 'D')
                {
                    if (LineState == 1 && keyindexDate > 0)
                    {
                        keyinputDate.remove(keyinputDate.length() - 1);
                        keyindexDate--;
                        lcd.clear();
                        lcd.setCursor(1, 0);
                        lcd.print(keyinputDate);
                    }
                    else if (LineState == 2 && keyindexTime > 0)
                    {
                        keyinputTime.remove(keyinputTime.length() - 1);
                        keyindexTime--;
                        lcd.clear();
                        lcd.setCursor(1, 1);
                        lcd.print(keyinputTime);
                    }
                }
                else if (key == '*')
                {
                    lcd.clear();
                    lcd.setCursor(7, 1);
                    lcd.print("Clear");
                    keyinputDate = "";
                    keyinputTime = "";
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    lcd.clear();
                }
                else if (key == '#')
                {
                    lcd.clear();
                    lcd.setCursor(6, 1);
                    lcd.print("Set Time");
                    vTaskDelay(1500 / portTICK_PERIOD_MS);
                    lcd.clear();
                    lcd.setCursor(6, 1);
                    lcd.print("Set Time");
                    lcd.clear();
                    //////////////////////////////////////////
                    String Data_date = keyinputDate + "," + keyinputTime;
                    String dataArray[8];
                    int index = 0;
                    int lastIndex = 0;
                    while (index < 8)
                    {
                        int commaIndex = Data_date.indexOf(',', lastIndex);
                        if (commaIndex != -1)
                        {
                            dataArray[index] = Data_date.substring(lastIndex, commaIndex);
                            lastIndex = commaIndex + 1;
                        }
                        else
                        {
                            dataArray[index] = Data_date.substring(lastIndex);
                            break;
                        }
                        index++;
                    }
                    uint16_t RT_Year;
                    uint8_t RT_Date, RT_Month, RT_Hour, RT_Min, RT_Sec;

                    RT_Date = dataArray[0].toInt();
                    RT_Month = dataArray[1].toInt();
                    RT_Year = dataArray[2].toInt();
                    RT_Hour = dataArray[3].toInt();
                    RT_Min = dataArray[4].toInt();
                    RT_Sec = dataArray[5].toInt();
                    rtc.adjust(DateTime(RT_Year, RT_Month, RT_Date, RT_Hour, RT_Min, RT_Sec));
                    Serial.println("--------------------SetTime----------------------");
                    //////////////////////////////////////////
                    keyinputDate = "";
                    keyinputTime = "";
                    //////////////////////////////////////////
                    lcd.setCursor(6, 1);
                    lcd.print("Press A");
                }
            }
        }
        // todo Display Data //////////////////////////////////////////////////////////////////////////////////
        if (ModeState == "A")
        {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= 10000) // todo Time to Test sent
            {
                if (NodeSate == 1)
                {
                    sendRequest("0x137");
                    NodeSate = 2;
                }
                else if (NodeSate == 2)
                {
                    sendRequest("0x429");
                    NodeSate = 1;
                }
                previousMillis = currentMillis;
            }
            int packetSize = LoRa.parsePacket();
            if (packetSize)
            {
                // Read Packet and seperate data to variable
                dataToVar();
                // Check Node
                if (NodeID == "0x137" & PacketCheck == "1") // Check NodeID and PacketCheck to identify Node1
                {
                    DisplaySerial();
                    DisplayLCD();
                }
                else if (NodeID == "0x429" & PacketCheck == "1") // Check NodeID and PacketCheck to identify Node2
                {
                    DisplaySerial();
                    DisplayLCD();
                }
            }
        }
        if (ModeState == "B")
        {
            lcd.setCursor(19, 0);
            lcd.print(ModeState);
            if (LineState == 1)
            {
                lcd.setCursor(0, 0);
                lcd.print(">");
                lcd.setCursor(0, 1);
                lcd.print(" ");
            }
            else if (LineState == 2)
            {
                lcd.setCursor(0, 0);
                lcd.print(" ");
                lcd.setCursor(0, 1);
                lcd.print(">");
            }
            //////////////////////////////
            lcd.setCursor(1, 0);
            lcd.print(keyinputDate);
            lcd.setCursor(1, 1);
            lcd.print(keyinputTime);
            /////////////////////
            lcd.setCursor(0, 2);
            lcd.print("D Back");
            lcd.setCursor(13, 2);
            lcd.print("Input ");
            lcd.setCursor(0, 3);
            lcd.print("* Cancel");
            lcd.setCursor(13, 3);
            lcd.print("# Save");
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // Determine pinMode
    pinMode(LED_Master, OUTPUT);

    // Setup Function
    SetupLoRa();
    lcd.begin();
    Wire.begin();
    keypad.begin();
    SetupRTC();
    setupWiFi();
    setupMqttClient();

    // Start Screen LCD
    lcd.backlight();
    lcd.setCursor(5, 1);
    lcd.print("Masternode");
    delay(2000);
    lcd.clear();

    // Task
    xTaskCreatePinnedToCore(DisplayTask, "Display LCD", 2048, NULL, 1, NULL, 1);
    // xTaskCreatePinnedToCore(packetTask, "led-task", 2048, NULL, 1, NULL, 1);
}

void loop()
{
    ResetESP(); // Masternode will reboot every 10 min
    //  Masternode Request Data fom Slave Node
    mqttClient.loop();
    now = rtc.now();
    Date = addLeadingZero(now.day());
    Month = addLeadingZero(now.month());
    Year = String(now.year());
    Hour = addLeadingZero(now.hour());
    Min = addLeadingZero(now.minute());
    Sec = addLeadingZero(now.second());
    // Serial.println(Date + '/' + Month + '/' + Year + "----" + Hour + ':' + Min + ':' + Sec);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}