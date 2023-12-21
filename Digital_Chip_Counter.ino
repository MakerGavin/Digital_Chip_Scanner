
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <SD.h>
File myFile;

#include "WizFi360.h"
/* Wi-Fi info */
 char ssid[] = "wiznet";       // your network SSID (name)
 char pass[] = "KUvT5sT1Ph";   // your network password
//char ssid[] = "WLAN_Gavin";       // your network SSID (name)
//char pass[] = "changxizheng";   // your network password
IPAddress ip;
WiFiClient client1;
char Cloud_Printer[] = "10.0.1.61"; 

#include "TouchyTouch.h"
const int touch_threshold_adjust = 300;
const int touch_pins[] = {7, 14, 15};
const int touch_count = sizeof(touch_pins) / sizeof(int);
TouchyTouch touches[touch_count];

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PIN_QR_TRIG 12
#define PIN_POWER_EN 29
#define PIN_Wifi_RST 11
#define PIN_CHARGE 25
#define PIN_Joy_Stick_Left 28
#define PIN_Joy_Stick_Mid 27
#define PIN_Joy_Stick_Right 26
#define PIN_IR_1 8
#define PIN_IR_2 9
#define PIN_IR_3 13

uint16_t Hole_num = 2;
uint16_t Hole_num_Now = 2;
uint16_t QR_num = 0;
uint16_t QR_num_Now = 0;

int frame = 0;

SoftwareSerial QR(3, 2);

uint8_t IR_Previous;
uint8_t IR_current;

String data_String; 
char file_name[16] = "recorder.txt";

unsigned long Time;
unsigned long Time_Touch_Key;
uint32_t Timeout_M = 5;

typedef enum 
{
  DO_Menu = 0,
  DO_QR_Scan,
  DO_QR_Classify,
  DO_Chip_Counter,
  DO_Setting,
  DO_QR_Classify_Print
}STATE_;
STATE_ CurrentState;

typedef enum 
{
  Power_On_wifi = 0,
  Link_To_Network,
  QR_Code_print
}WIFI_STATE_;
WIFI_STATE_ Wifi_State;

uint8_t Touch_Key_Num = 128;
bool Touch_Key_Enter = false;

bool Scan_En = false; 

#define Product_Box_max 64

struct _Product_Box{
  String MODEL;
  String LOT;
  String DC;
  uint32_t Qty;
} ;
_Product_Box Product_Box[Product_Box_max];
uint8_t Product_Box_cnt = 0;
uint8_t DO_QR_Classify_Step=0;

int status = WL_IDLE_STATUS;  // the Wifi radio's status

void setup() {
  // initialize serial for debugging
  Serial.begin(115200);
  delay(2000);
  
  pinMode(PIN_Wifi_RST, OUTPUT);
  digitalWrite(PIN_Wifi_RST, LOW);
  //check the power on
  pinMode(PIN_POWER_EN, OUTPUT);
  digitalWrite(PIN_POWER_EN, HIGH);
  pinMode(PIN_QR_TRIG, OUTPUT);
  digitalWrite(PIN_QR_TRIG, HIGH);  
  pinMode(PIN_CHARGE, INPUT);
  
  // initialize serial for WizFi360 module
  Serial2.setFIFOSize(4096);
  Serial2.begin(2000000);
  //Serial2.begin(115200);  

  // Init the oled
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.fillScreen(SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.display();

  for (int i = 0; i < touch_count; i++) {
    touches[i].begin( touch_pins[i] );
    touches[i].threshold += touch_threshold_adjust; // make a bit more noise-proof
  }

  SD.begin(17);
  QR.begin(9600);
  
  pinMode(PIN_IR_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_1), IR_COUNTER, CHANGE);
  pinMode(PIN_IR_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_2), IR_COUNTER, CHANGE);
  // initialize the pushbutton pin as an input:
  pinMode(PIN_Joy_Stick_Mid, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_Joy_Stick_Mid), onChange, CHANGE);
  CurrentState = DO_Menu;
  Time = millis();
}

void loop() {
  // Check for touch button input
  // Add your touch button logic here
  Touch_handling();  
  switch(CurrentState){
  case DO_Menu:
     {
        display.clearDisplay();
        if(Touch_Key_Num%4 == 0)
        {
          display.drawBitmap(8, 8, frames_QR[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(65, 20);
          display.print("QR Code");
          display.setCursor(65, 30);
          display.print("Scanner");
          display.display();
          frame = (frame + 1) % FRAME_COUNT_QR;
          delay(FRAME_DELAY);
          Touch_handling();
          delay(FRAME_DELAY);
          if(Touch_Key_Enter)
          {
            CurrentState = DO_QR_Scan;
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(18, 1);
            display.print("QR Code Scanner");
            display.drawBitmap(4, 8, frames_tick[0], FRAME_WIDTH, FRAME_HEIGHT, 1);
            display.fillRect(0, 55, 128, 8, SSD1306_WHITE);
            Touch_Key_Enter = false;
          }
        }
        else if(Touch_Key_Num%4 == 1)
        {          
          display.drawBitmap(8, 8, frames_Classify[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(65, 20);
          display.print("QR Code");
          display.setCursor(65, 30);
          display.print("Classify");
          display.display();
          frame = (frame + 1) % FRAME_COUNT_Classify;
          delay(FRAME_DELAY);
          Touch_handling();
          delay(FRAME_DELAY);
          if(Touch_Key_Enter)
          {
            CurrentState = DO_QR_Classify;
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(16, 1);
            display.print("QR Code Classify");      
            display.drawBitmap(4, 8, frames_tick[0], FRAME_WIDTH, FRAME_HEIGHT, 1);
            display.drawRect(56, 12, 68, 11, SSD1306_WHITE);
            display.drawRect(56, 22, 68, 11, SSD1306_WHITE);
            display.drawRect(56, 32, 68, 11, SSD1306_WHITE);
            display.drawRect(56, 42, 68, 11, SSD1306_WHITE);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(58, 14);
            display.print("MODEL");
            display.setCursor(58, 24);
            display.print("Lot");
            display.setCursor(58, 34);
            display.print("D/C");
            display.setCursor(58, 44);
            display.print("Qty");
            display.drawRoundRect(92, 14, 22,7,7,SSD1306_WHITE);
            display.drawRoundRect(92, 24, 22,7,7,SSD1306_WHITE);
            display.drawRoundRect(92, 34, 22,7,7,SSD1306_WHITE);
            display.drawRoundRect(92, 44, 22,7,7,SSD1306_WHITE);
            display.fillRect(0, 55, 128, 8, SSD1306_WHITE);
            Touch_Key_Enter = false;
          }
        }        
        else if(Touch_Key_Num%4 == 2)
        {
          display.clearDisplay();
          display.drawBitmap(8, 8, frames_Counter[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(65, 20);
          display.print("Chip");
          display.setCursor(65, 30);
          display.print("Counter");
          display.display();
          frame = (frame + 1) % FRAME_COUNT_Counter;
          delay(FRAME_DELAY);
          if(Touch_Key_Enter)
          {
            CurrentState = DO_Chip_Counter;
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(32, 1);
            display.print("Chip Counter");
            display.drawBitmap(8, 8, frames_CountX[0], FRAME_WIDTH, FRAME_HEIGHT, 1);
            display.display();
            Touch_Key_Enter = false;
          }
        }
        else if(Touch_Key_Num%4 == 3)
        {
          display.clearDisplay();
          display.drawBitmap(8, 8, frames_setting[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);          
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(65, 25);
          display.print("Setting");          
          display.display();
          frame = (frame + 1) % FRAME_COUNT_setting;
          delay(FRAME_DELAY);
          if(Touch_Key_Enter)
          {
            CurrentState = DO_Setting;
            Touch_Key_Enter = false;
          }
        }
     }
    break;
  case DO_QR_Scan:
    {      
      if(Touch_Key_Enter == true)
      {
        display.drawBitmap(4, 8, frames_tick[1], FRAME_WIDTH, FRAME_HEIGHT, 0);
        display.drawBitmap(4, 8, frames_tick[0], FRAME_WIDTH, FRAME_HEIGHT, 1);        
        display.fillRect(0, 55, 128, 8, SSD1306_WHITE);
        digitalWrite(PIN_QR_TRIG, LOW);
        Scan_En = true;
        Touch_Key_Enter = false;
      }
      if(Scan_En)
      {
        if( millis()- Time_Touch_Key > 6000)
        {
          digitalWrite(PIN_QR_TRIG, HIGH);
          display.writeLine(17, 21, 37, 41,SSD1306_WHITE);
          display.writeLine(17, 41, 37, 21,SSD1306_WHITE);
        }
        if( millis()- Time_Touch_Key > 10000)
        {
          display.writeLine(17, 21, 37, 41,SSD1306_BLACK);
          display.writeLine(17, 41, 37, 21,SSD1306_BLACK);  
          Scan_En = false;
        }          
      }
      while (QR.available())
      {
        char a = QR.read();
        Serial.print(a);
        data_String += a;
        digitalWrite(PIN_QR_TRIG, HIGH);
        delay(10);
      }
      if(data_String != "")
      {        
        display.drawBitmap(4, 8, frames_tick[0], FRAME_WIDTH, FRAME_HEIGHT, 0);
        display.drawBitmap(4, 8, frames_tick[1], FRAME_WIDTH, FRAME_HEIGHT, 1);
        display.setTextColor(SSD1306_BLACK);
        display.setTextSize(2);
        display.setCursor(65, 15);
        display.println(QR_num_Now);
        display.setCursor(65, 35);
        display.println(QR_num);
        QR_num ++;
        QR_num_Now ++;

        display.setTextColor(SSD1306_BLACK);
        display.setTextSize(1);
        display.setCursor(1, 56);
        display.print(data_String);
        
        myFile = SD.open(file_name, FILE_WRITE);
        if (myFile) {
          myFile.println(data_String);
          myFile.close();
          data_String = "";  
        }
        Scan_En = false;
      }
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(65, 15);
      display.println(QR_num_Now);
      display.setCursor(65, 35);
      display.println(QR_num);
      display.display();
    }
    break;
  case DO_QR_Classify:
    {
      if(Touch_Key_Enter == true)
      {
        display.drawBitmap(4, 8, frames_tick[1], FRAME_WIDTH, FRAME_HEIGHT, 0);
        display.drawBitmap(4, 8, frames_tick[0], FRAME_WIDTH, FRAME_HEIGHT, 1);        
        display.fillRect(0, 55, 128, 8, SSD1306_WHITE);
        digitalWrite(PIN_QR_TRIG, LOW);
        Scan_En = true;
        Touch_Key_Enter = false;
      }
      if(Scan_En)
      {
        if( millis()- Time_Touch_Key > 6000)
        { 
          digitalWrite(PIN_QR_TRIG, HIGH);
          display.writeLine(17, 21, 37, 41,SSD1306_WHITE);
          display.writeLine(17, 41, 37, 21,SSD1306_WHITE);
        }
        if( millis()- Time_Touch_Key > 10000)
        {
          display.writeLine(17, 21, 37, 41,SSD1306_BLACK);
          display.writeLine(17, 41, 37, 21,SSD1306_BLACK);  
          Scan_En = false;
        }          
      }
      while (QR.available())
      {
        char a = QR.read();
        Serial.print(a);
        data_String += a;
        digitalWrite(PIN_QR_TRIG, HIGH);
        delay(10);
      }
      if(data_String != "")
      {
        display.drawBitmap(4, 8, frames_tick[0], FRAME_WIDTH, FRAME_HEIGHT, 0);
        display.drawBitmap(4, 8, frames_tick[1], FRAME_WIDTH, FRAME_HEIGHT, 1);
        display.setTextColor(SSD1306_BLACK);
        display.setTextSize(2);
        if(DO_QR_Classify_Step == 0)
        {
          if(data_String.substring(0, data_String.indexOf(":", 0)) == "P/N")
          {
            Product_Box[Product_Box_cnt].MODEL = data_String.substring(data_String.indexOf(":", 0)+1, data_String.length()-1);    
            display.fillRoundRect(95, 16, 16,3,3,SSD1306_WHITE);           
            DO_QR_Classify_Step ++;
          }
        }else if(DO_QR_Classify_Step == 1)
        {          
          if(data_String.substring(0, data_String.indexOf(":", 0)) == "LOT")
          {
            Product_Box[Product_Box_cnt].LOT = data_String.substring(data_String.indexOf(":", 0)+1, data_String.length()-1);
            display.fillRoundRect(95, 26, 16,3,3,SSD1306_WHITE);
            DO_QR_Classify_Step ++;            
          }          
        }else if(DO_QR_Classify_Step == 2)
        {
          Product_Box[Product_Box_cnt].DC = data_String.substring(data_String.indexOf(":", 0)+1, data_String.length()-1);
          
          if(data_String.substring(0, data_String.indexOf(":", 0)) == "D/C")
          {
            display.fillRoundRect(95, 36, 16,3,3,SSD1306_WHITE);
            DO_QR_Classify_Step ++;
          }
        }else if(DO_QR_Classify_Step == 3)
        {
          if(data_String.substring(0, data_String.indexOf(":", 0)) == "QTY")
          {
              Product_Box[Product_Box_cnt].Qty = data_String.substring(data_String.indexOf(":", 0)+1, data_String.length()-1).toInt();
              display.fillRoundRect(95, 46, 16,3,3,SSD1306_WHITE);
              myFile = SD.open(file_name, FILE_WRITE);
              if (myFile) {
                myFile.print("BOX No:");
                myFile.println(Product_Box_cnt);
                myFile.print("MODEL:");
                myFile.println(Product_Box[Product_Box_cnt].MODEL);
                myFile.print("LOT:");
                myFile.println(Product_Box[Product_Box_cnt].LOT);
                myFile.print("D/C:");
                myFile.println(Product_Box[Product_Box_cnt].DC);
                myFile.println("");
                myFile.close();
              }        
              DO_QR_Classify_Step = 0;
              Product_Box_cnt ++;
              display.fillRoundRect(92, 14, 22,7,7,SSD1306_BLACK);
              display.fillRoundRect(92, 24, 22,7,7,SSD1306_BLACK);
              display.fillRoundRect(92, 34, 22,7,7,SSD1306_BLACK);
              display.fillRoundRect(92, 44, 22,7,7,SSD1306_BLACK);
              display.drawRoundRect(92, 14, 22,7,7,SSD1306_WHITE);
              display.drawRoundRect(92, 24, 22,7,7,SSD1306_WHITE);
              display.drawRoundRect(92, 34, 22,7,7,SSD1306_WHITE);
              display.drawRoundRect(92, 44, 22,7,7,SSD1306_WHITE);   
          }       
        }        
        display.setTextColor(SSD1306_BLACK);
        display.setTextSize(1);
        display.setCursor(1, 56);
        display.print(data_String); 
        data_String = "";
      }
      display.display();
    }
    break;
  case DO_Chip_Counter:
    {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(32, 1);
      display.print("Chip Counter");
      if(Hole_num > Hole_num_Now)
      {
        display.drawBitmap(8, 8, frames_CountX[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);
        frame = (frame + 1) % FRAME_COUNT_CountX;
        if(frame == FRAME_COUNT_CountX-1)
        {
          Hole_num_Now = Hole_num;
        }
      }
      else
      {
        display.drawBitmap(8, 8, frames_CountX[0], FRAME_WIDTH, FRAME_HEIGHT, 1);    
      }
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(65, 25);
      display.println(Hole_num/3); 
      display.display();
      delay(FRAME_DELAY);
    }
    break;
  case DO_Setting:
    {
      
    }
    break;
  case DO_QR_Classify_Print:
    {
       switch(Wifi_State){
        case Power_On_wifi:
          {
            Serial.println("POWER_ON1");
            digitalWrite(PIN_Wifi_RST, HIGH);
            delay(1000);
            WiFi.init(&Serial2);
            Serial2.flush();
            Serial2.begin(2000000);
            // attempt to connect to WiFi network
            while ( status != WL_CONNECTED) {
              // Connect to WPA/WPA2 network
              status = WiFi.begin(ssid, pass);
            }
            // print your WiFi shield's IP address
            ip = WiFi.localIP(); 
            Wifi_State = Link_To_Network;
          }
          break;
        case Link_To_Network:
          {
             if (client1.connect(Cloud_Printer,80)){
                delay(3000);                
                client1.print(String("TEXT")+String("QR CODE Classify"));
                Wifi_State = QR_Code_print;
             }
          }
          break;
        case QR_Code_print:
          {
          }
          break;
       }
    }
    break;
  }
  if((millis() - Time) == Timeout_M*60*1000)
  {
    digitalWrite(PIN_POWER_EN, LOW);
    delay(2000);
  }
}

void onChange()
{
  if (digitalRead(PIN_Joy_Stick_Mid) == HIGH)
  {    
    IR_current = 0;
    delay(2000);
    if (digitalRead(PIN_Joy_Stick_Mid) == HIGH)
    {
      digitalWrite(PIN_POWER_EN, LOW);
    }
  }
}

void Touch_handling()
{
  // key handling
  for ( int i = 0; i < touch_count; i++) {
    touches[i].update();
    if ( touches[i].rose()) {
//      Serial.print("Button:");
//      Serial.println(i);
      Time = millis();
      Time_Touch_Key = millis();
      if ( i == 0 ) {
        if(CurrentState == DO_Menu)
        {
          Touch_Key_Num --;
          frame = 0;  
        }
        if(CurrentState == DO_QR_Classify)
        {
          DO_QR_Classify_Step = 0;
          display.fillRoundRect(92, 14, 22,7,7,SSD1306_BLACK);
          display.fillRoundRect(92, 24, 22,7,7,SSD1306_BLACK);
          display.fillRoundRect(92, 34, 22,7,7,SSD1306_BLACK);
          display.fillRoundRect(92, 44, 22,7,7,SSD1306_BLACK);
          display.drawRoundRect(92, 14, 22,7,7,SSD1306_WHITE);
          display.drawRoundRect(92, 24, 22,7,7,SSD1306_WHITE);
          display.drawRoundRect(92, 34, 22,7,7,SSD1306_WHITE);
          display.drawRoundRect(92, 44, 22,7,7,SSD1306_WHITE);
          display.display();
        }
      }
      else if ( i == 1 ) {
      
      }
      else if ( i == 2 ) {
        if(CurrentState == DO_Menu)
        {
          Touch_Key_Num ++; 
          frame = 0;
        }
        if(CurrentState == DO_QR_Scan)
        {
          display.setTextColor(SSD1306_BLACK);
          display.setTextSize(2);
          display.setCursor(65, 15);
          display.println(QR_num_Now);
          QR_num_Now = 0;
        }
        if(CurrentState == DO_Chip_Counter)
        {
          IR_current = 0;
          Hole_num = 2;
          Hole_num_Now = 2;
        }
      }
    }
    if ( touches[i].fell() ) {
//      Serial.print("Release:");
//      Serial.println(i);
      if ( i == 0 ) 
      {
        if(CurrentState == DO_QR_Classify)
        {
           if( millis()- Time_Touch_Key > 1000)
          {             
             Wifi_State = Power_On_wifi;
             CurrentState = DO_QR_Classify_Print;
          }
        }
      }
      if ( i == 1 ) 
      {
        if( millis()- Time_Touch_Key > 1000)
        {
          CurrentState = DO_Menu;
          Touch_Key_Num = 126;
          frame = 0;
        }
        else
        {
          Touch_Key_Enter = true;
        }
      }
      else if ( i == 2 ) 
      {
        if( millis()- Time_Touch_Key > 1000)
        {
          if(CurrentState == DO_QR_Scan)
          {
            display.setTextColor(SSD1306_BLACK);
            display.setTextSize(2);
            display.setCursor(65, 35);
            display.println(QR_num);
            QR_num = 0;
          }            
        }
      }
    }
  }
}

void IR_COUNTER()
{
  IR_current = ((IR_current << 2) | (digitalRead(PIN_IR_1) << 1) | (digitalRead(PIN_IR_2)));
  Serial.println((digitalRead(PIN_IR_1) << 1) | (digitalRead(PIN_IR_2)), HEX);
  if ((IR_current == 0b00101101) || (IR_current == 0b11101101)|| (IR_current == 0b00100001) || (IR_current == 0b00101001)|| (IR_current == 0b10001000))
  {
    Hole_num ++ ;
//    Serial.print("NUM:");
//    Serial.println(Hole_num);
  }
}
