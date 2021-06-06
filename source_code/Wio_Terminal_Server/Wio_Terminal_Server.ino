#include <TFT_eSPI.h>
#include "lcd_backlight.hpp"
#include "rpcWiFi.h"
#include <WiFiMulti.h>
#include <Thread.h>
#include "RTC_SAMD51.h"
#include <sfud.h>
#include <SPI.h>


// for use by the Adafuit RTClib library
char daysOfTheWeek[8][10] = { "Sun.", "Mon.", "Tues.", "Wed.", "Thurs.", "Fri.", "Sat.", "." };
static uint8_t onTimeOfDay[8]; // on time of day in % format

// Use WiFiClient class to create TCP connections
WiFiClient client;
WiFiUDP udp;

TFT_eSPI tft;
static LCDBackLight backLight;
WiFiMulti wifiMulti;
bool ac_on = true;
uint32_t ac_on_time = 0;         // increas every 1 second when AC turned on
uint8_t dd=7,hh=0,mm=0,ss=0;
uint8_t omm = 99, oss = 99, odd=99;
uint8_t xcolon = 0, xsecs = 0;

const char timeServer[] = "cn.ntp.org.cn"; // local NTP server 
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
const unsigned int localPort = 2390;      // local port to listen for UDP packets
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
unsigned long devicetime;// localtime
DateTime now;
RTC_SAMD51 rtc;

//time set
char number1 = 0;
char cc;
char a[2],b[2],c[2],d[2];
struct Time_Num {
  char num;
  char posx;
  char posy;
};

struct Time_Set {
  int8_t index;
  char set_enable;
  char start_x;
  char start_y;
  struct Time_Num time_num[4];
};
struct Time_Set time_set;
static uint8_t testbuf[4]; 

#define ssid1 "bbear601"
#define password1 "xj13687227078"
#define ssid2 "xjWIFIHOST"
#define password2 "12345679"

#define TIME_TEXT_SIZE  6
#define AC_STOP_TIME_HOUR 19
#define AC_STOP_TIME_MINUTE 29

#define AC_OFF_CMD {0xFE, 0x08, 0x91, 0x90, 0xFE, 0xFD, 0xFE, 0xFD, 0x00, 0xAE, 0x13, 0x33, 0xFF,}
#define AC_FORCE_OFF_CMD {0xFE, 0x08, 0x91, 0x90, 0xFE, 0xFD, 0xFE, 0xFD, 0x02, 0xAE, 0x13, 0x33, 0xFF,}

Thread timeThread = Thread(updateTimeCB);
Thread ntpUpdateThread = Thread(nptUpdateCB);
Thread keyThread = Thread(keyCB);
Thread acThread = Thread(acControllCB);
Thread TimeSetThread = Thread(timeSetCB);

void setup() 
{    
    int i,j;
    char a[2],b[2];
    Serial.begin(115200);  
    Serial.println("start");
    Serial1.begin(115200);

    while(!(sfud_init() == SFUD_SUCCESS));
    Serial.println("Flash is OK");

    sfud_qspi_fast_read_enable(sfud_get_device(SFUD_W25Q32_DEVICE_INDEX), 2);
    readFlash(0,sizeof(testbuf), testbuf);

    pinMode(WIO_KEY_A, INPUT_PULLUP);
    pinMode(WIO_KEY_B, INPUT_PULLUP);
    pinMode(WIO_KEY_C, INPUT_PULLUP);
    pinMode(WIO_5S_UP, INPUT_PULLUP);
    pinMode(WIO_5S_DOWN, INPUT_PULLUP);
    pinMode(WIO_5S_LEFT, INPUT_PULLUP);
    pinMode(WIO_5S_RIGHT, INPUT_PULLUP);
    pinMode(WIO_5S_PRESS, INPUT_PULLUP);

    wifiMulti.addAP(ssid1, password1);
    wifiMulti.addAP(ssid2, password2);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    tft.begin();
    backLight.initialize();
    backLight.setBrightness(10);
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Connecting Wifi...",60,100,4);
    if(wifiMulti.run() != WL_CONNECTED)
    //if(0)
    {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("Connect Wifi Failed",60,40,4);
        tft.drawString("Check Wifi and retry",60,60,4);
        tft.drawString("wifi1:",60,100,2);
        tft.drawString(ssid1,60,115,2);
        tft.drawString(password1,60,130,2);
        
        tft.drawString("wifi2:",60,145,2);
        tft.drawString(ssid2,60,160,2);
        tft.drawString(password2,60,175,2);
        while(1);
    }
    tft.fillScreen(TFT_BLACK);

    timeThread.setInterval(1000);
    ntpUpdateThread.setInterval(10000);//first time init set interval to 10s
    keyThread.setInterval(50);
    acThread.setInterval(60000);
    TimeSetThread.setInterval(100);

    memset(&time_set, 0x00, sizeof(time_set));

    for(i = 0;i < sizeof(testbuf); ++i)
      time_set.time_num[i].num = testbuf[i];

    time_set.start_x = 20;
    time_set.start_y = 193;
    time_set.time_num[0].posy = time_set.start_y;
    time_set.time_num[1].posy = time_set.start_y;
    time_set.time_num[2].posy = time_set.start_y;
    time_set.time_num[3].posy = time_set.start_y;

    itoa(time_set.time_num[0].num, a, 10); 
    itoa(time_set.time_num[1].num, b, 10); 
    itoa(time_set.time_num[2].num, c, 10); 
    itoa(time_set.time_num[3].num, d, 10); 

    //draw the layout
    tft.drawFastHLine(0,60,320,TFT_WHITE);//A black horizontal line starting from (0, 120)
    tft.drawFastHLine(0,88,320,TFT_WHITE);//A black horizontal line starting from (0, 120)
    tft.drawFastHLine(0,164,212,TFT_WHITE);//A black horizontal line starting from (0, 120)
    tft.drawFastVLine(106,0,60,TFT_WHITE); // A black vertical line starting from (160, 0)
    tft.drawFastVLine(212,0,60,TFT_WHITE); // A black vertical line starting from (160, 0)
    tft.drawFastVLine(212,88,152,TFT_WHITE); // A black vertical line starting from (160, 0)

    tft.drawString("button C",5,0,4);
    tft.drawString("Reserved",5,25,2);
    tft.drawString("button B",111,0,4);
    tft.drawString("Turn off all",111,25,2);
    tft.drawString("immadiately",111,40,2);
    tft.drawString("button A",217,0,4);
    tft.drawString("Turn off all",217,25,2);
    tft.drawString("without people",217,40,2);
    
    i = tft.drawString("Wifi Host:",10,61,4);
    tft.drawString(WiFi.SSID(),i+10,61,4);

    tft.drawString("Press twice to",217,90,2);
    tft.drawString("setup;",217,105,2);

    tft.drawString("Long press to",217,120,2);
    tft.drawString("save off time;",217,135,2);

    tft.drawString("Left,Right,Up",217,165,2);
    tft.drawString("Down to adjust",217,180,2);
    tft.drawString("off time;",217,195,2);

    tft.drawString("Current time:",5,90,4);
    //tft.drawString("00:00:00",20,115,6);

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("Off time:",5,170,4);
    time_set.time_num[0].posx = time_set.start_x;
    i = tft.drawString(a, time_set.start_x,time_set.time_num[0].posy,6);
    time_set.time_num[1].posx = i+time_set.start_x;
    i += tft.drawString(b, i+time_set.start_x, time_set.time_num[1].posy, 6);
    i += tft.drawString(":", i+time_set.start_x, time_set.start_y, 6);
    time_set.time_num[2].posx = i+time_set.start_x;
    i += tft.drawString(c, i+time_set.start_x, time_set.time_num[2].posy, 6);
    time_set.time_num[3].posx = i+time_set.start_x;
    i += tft.drawString(d, i+time_set.start_x, time_set.time_num[3].posy, 6);

    Serial.println("set up over");
}

void loop() 
{
  if (timeThread.shouldRun()) timeThread.run();
  if (ntpUpdateThread.shouldRun()) ntpUpdateThread.run();
  if (keyThread.shouldRun()) keyThread.run();
  if (acThread.shouldRun()) acThread.run();
  if (TimeSetThread.shouldRun()) TimeSetThread.run();
}



unsigned long getNTPtime() {

    // module returns a unsigned long time valus as secs since Jan 1, 1970 
    // unix time or 0 if a problem encounted

    //only send data when connected
    if (WiFi.status() == WL_CONNECTED) {
        //initializes the UDP state
        //This initializes the transfer buffer
        udp.begin(WiFi.localIP(), localPort);

        sendNTPpacket(timeServer); // send an NTP packet to a time server
        // wait to see if a reply is available
        delay(1000);
        if (udp.parsePacket()) {
            Serial.println("udp packet received");
            Serial.println("");
            // We've received a packet, read the data from it
            udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

            //the timestamp starts at byte 40 of the received packet and is four bytes,
            // or two words, long. First, extract the two words:

            unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
            unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
            // combine the four bytes (two words) into a long integer
            // this is NTP time (seconds since Jan 1 1900):
            unsigned long secsSince1900 = highWord << 16 | lowWord;
            // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
            const unsigned long seventyYears = 2208988800UL;
            // subtract seventy years:
            unsigned long epoch = secsSince1900 - seventyYears;

            // adjust time for timezone offset in secs +/- from UTC
            // WA time offset from UTC is +8 hours (28,800 secs)
            // + East of GMT
            // - West of GMT
            long tzOffset = 28800UL;

            // WA local time 
            unsigned long adjustedTime;
            return adjustedTime = epoch + tzOffset;
        }
        else {
            // were not able to parse the udp packet successfully
            // clear down the udp connection
            udp.stop();
            return 0; // zero indicates a failure
        }
        // not calling ntp time frequently, stop releases resources
        udp.stop();
    }
    else {
        // network not connected
        return 0;
    }

}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(const char* address) {
    // set all bytes in the buffer to 0
    for (int i = 0; i < NTP_PACKET_SIZE; ++i) {
        packetBuffer[i] = 0;
    }
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}

void updateOnTimeOfDay() {
    if (ac_on) {
        ac_on_time++;
        if (ac_on_time > (60*60*24)) {
            ac_on_time = 60*60*24;
        }
        float  x = ((100.0*ac_on_time / (60*60*24)));
        // Serial.println(x);
        onTimeOfDay[dd] = (uint8_t)x;
    }
    
    if(dd != odd) {
        if (odd != 99) {  //not initial update onTimeOfDay
            onTimeOfDay[dd]=0;
            ac_on_time = 0;
        }
        odd = dd;
    }
    // update_histogram();

}

void updateTimeCB() {
    // Update digital time

    updateOnTimeOfDay();
    ss++;              // Advance second
    if (ss == 60) {    // Check for roll-over
        ss = 0;          // Reset seconds to zero
        omm = mm;        // Save last minute time for display update
        mm++;            // Advance minute
        if (mm > 59) {   // Check for roll-over
            mm = 0;
            hh++;          // Advance hour
            if (hh > 23) { // Check for 24hr roll-over (could roll-over on 13)
                hh = 0;      // 0 for 24 hour clock, set to 1 for 12 hour clock
                dd++;
                if (dd >= 7) {
                    dd =0;
                }
                timeUpdateRTC();  // update time every 1 day
            }
            ///writeFlash(0,sizeof(onTimeOfDay), onTimeOfDay); //save to flash every 1 hour
        }
        ///update_histogram(); //update histogram every 1 minute
    }
    // Update digital time
    int xpos = 20;
    int ypos = 115; // Top left corner ot clock text, about half way down
    int ysecs = ypos+15;
        
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);

    if (omm != mm) { // Redraw hours and minutes time every minute
        omm = mm;
        // Draw hours and minutes
        if (hh < 10) {
            xpos += tft.drawChar('0', xpos, ypos, TIME_TEXT_SIZE);    // Add hours leading zero for 24 hr clock
        }
        xpos += tft.drawNumber(hh, xpos, ypos, TIME_TEXT_SIZE);             // Draw hours
        xcolon = xpos; // Save colon coord for later to flash on/off later
        xpos += tft.drawChar(':', xpos, ypos, TIME_TEXT_SIZE);
        if (mm < 10) {
            xpos += tft.drawChar('0', xpos, ypos, TIME_TEXT_SIZE);    // Add minutes leading zero
        }
        xpos += tft.drawNumber(mm, xpos, ypos, TIME_TEXT_SIZE);             // Draw minutes
        xsecs = xpos; // Sae seconds 'x' position for later display updates
    }
    if (oss != ss) { // Redraw seconds time every second
        oss = ss;
        xpos = xsecs;

        if (ss % 2) { // Flash the colons on/off
            tft.setTextColor(0x39C4, TFT_BLACK);        // Set colour to grey to dim colon
            tft.drawChar(':', xcolon, ypos, TIME_TEXT_SIZE);     // Hour:minute colon
            xpos += tft.drawChar(':', xsecs, ysecs, 4); // Seconds colon
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);    // Set colour back to yellow
        } else {
            tft.drawChar(':', xcolon, ypos, TIME_TEXT_SIZE);     // Hour:minute colon
            xpos += tft.drawChar(':', xsecs, ysecs, 4); // Seconds colon
        }

        //Draw seconds
        if (ss < 10) {
            xpos += tft.drawChar('0', xpos, ysecs, 4);    // Add leading zero
        }
        tft.drawNumber(ss, xpos, ysecs, 4);                     // Draw seconds
    }
}

void nptUpdateCB() {
    // get the time via NTP (udp) call to time server
    // getNTPtime returns epoch UTC time adjusted for timezone but not daylight savings time
    devicetime = getNTPtime();
    if (devicetime == 0) {
        Serial.println("Failed to get time from network time server.");
        ntpUpdateThread.setInterval(10000);//wait 10s retry
        now = rtc.now();
    }
    else {
        now = DateTime(devicetime);
        rtc.adjust(now);
        ntpUpdateThread.setInterval(7*1000*60*60*24);//every 7 days update time from ntp server 1000*60*60*24*7
    }
    hh = now.hour();
    mm = now.minute();
    ss = now.second();
    dd = now.dayOfTheWeek();
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    String ddate = now.timestamp(DateTime::TIMESTAMP_DATE);
    Serial.println(ddate);

    //tft.drawString(ddate,10,10,4);
    //tft.setTextColor(TFT_CYAN, TFT_BLACK);
    //tft.drawRightString(daysOfTheWeek[dd],310,10,4);
    //update_histogram();
}

void timeUpdateRTC() {
    now = rtc.now();
    hh = now.hour();
    mm = now.minute();
    ss = now.second();
    dd = now.dayOfTheWeek();
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    String ddate = now.timestamp(DateTime::TIMESTAMP_DATE);
    Serial.println(ddate);

    tft.drawString(ddate,10,10,4);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawRightString(daysOfTheWeek[dd],310,10,4);
//    update_histogram();
}
    
void keyCB() {
    static bool keyAPressed = false;
    static bool keyBPressed = false;
    static bool keyCPressed = false;
    static bool joyPressed = false;
    static bool upPressed = false;
    static bool downPressed = false;
    static bool leftPressed = false;
    static bool rightPressed = false;

    static bool last_keyAPressed = false;
    static bool last_keyBPressed = false;
    static bool last_keyCPressed = false;
    static bool last_joyPressed = false;
    static bool last_upPressed = false;
    static bool last_downPressed = false;
    static bool last_leftPressed = false;
    static bool last_rightPressed = false;

    static int joyPressed_cnt = 0;
    static int joyPressed_twice_cnt = 0;

    if(joyPressed_twice_cnt > 0)
      joyPressed_twice_cnt--;
  
    if (!digitalRead(WIO_KEY_A)){
      keyAPressed = true;
      if(!last_keyAPressed)
        turnOff_All_Ac();
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("button A",217,0,4);
        Serial1.println("A pressed down");
    } else {
      keyAPressed = false;
      if(last_keyAPressed){
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("button A",217,0,4);
        Serial1.println("A pressed up");
      }
    }
    last_keyAPressed = keyAPressed;

    if (!digitalRead(WIO_KEY_B)){
      keyBPressed = true;
      if(!last_keyBPressed)
        turnOff_All_Ac_Force();
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("button B",111,0,4);
        Serial1.println("B pressed down");
    } else {
      keyBPressed = false;
      if(last_keyBPressed){
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("button B",111,0,4);
        Serial1.println("B pressed up");
      }
    }
    last_keyBPressed = keyBPressed;

    if (!digitalRead(WIO_KEY_C)){
      keyCPressed = true;
      if(!last_keyCPressed){
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("button C",5,0,4);
        Serial1.println("C pressed down");
      }
    } else {
      keyCPressed = false;
      if(last_keyCPressed){
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("button C",5,0,4);
        Serial1.println("C pressed up");
      }
    }
    last_keyCPressed = keyCPressed;   

    //up buttom---------------------------------------------------
    if (!digitalRead(WIO_5S_UP)){
      upPressed = true;
      if(!last_upPressed){
        number1 = time_set.time_num[time_set.index].num;
        if(number1 > 9)
          number1 = 0;
        else 
          number1++;

        if(time_set.index == 0){
          if(number1 > 2)  
            number1 = 0;
        } else if(time_set.index == 1){
          if(time_set.time_num[0].num == 2){
            if(number1 > 3)
            number1 = 0;
          }
        } else if(time_set.index == 2){
          if(number1 > 5)
            number1 = 0;
        }
        number1 = number1%10;
        time_set.time_num[time_set.index].num = number1;
        Serial.println("up pressed down");
      }
    } else {
      upPressed = false;
      if(last_upPressed)
        Serial.println("up pressed up");
    }
    last_upPressed = upPressed;

    //down buttom-------------------------------------------------
    if (!digitalRead(WIO_5S_DOWN)){
      downPressed = true;
      if(!last_downPressed){
        number1 = time_set.time_num[time_set.index].num;
        if(number1 == 0)
          number1 = 9;
        else
          number1--;

        if(time_set.index == 0){
          if(number1 == 9)  
            number1 = 2;
        } else if(time_set.index == 1){
          if(time_set.time_num[0].num == 2){
            if(number1 == 9)
            number1 = 3;
          }
        } else if(time_set.index == 2){
          if(number1 == 9)
            number1 = 5;
        }
        
        time_set.time_num[time_set.index].num = number1;
        Serial.println("down pressed down");
      }
    } else {
      downPressed = false;
      if(last_downPressed)
        Serial.println("down pressed up");
    }
    last_downPressed = downPressed;

    //left buttom-------------------------------------------------
    if (!digitalRead(WIO_5S_LEFT)){
      leftPressed = true;
      if(!last_leftPressed){
        time_set.index--;
        if(time_set.index < 0)
          time_set.index = 3;
        
        Serial.println("left pressed down");
      }
    } else {
      leftPressed = false;
      if(last_leftPressed)
        Serial.println("left pressed up");
    }
    last_leftPressed = leftPressed;

    //right buttom-----------------------------------------------
    if (!digitalRead(WIO_5S_RIGHT)){
      rightPressed = true;
      if(!last_rightPressed){
        time_set.index++;
        if(time_set.index > 3)
          time_set.index = 0;
        
        Serial.println("right pressed down");
      }
    } else {
      rightPressed = false;
      if(last_rightPressed)
        Serial.println("right pressed up");
    }
    last_rightPressed = rightPressed;

    //joy buttom------------------------------------------------
    if (!digitalRead(WIO_5S_PRESS)){
      joyPressed = true;
      if(!last_joyPressed)
        Serial.println("joy pressed down");
      if(joyPressed_twice_cnt > 0 && !last_joyPressed) {
        time_set.set_enable = 1;
        Serial.println("joy twice pressed");
      }
      joyPressed_cnt++;
      if(joyPressed_cnt > 20) {//long press
        joyPressed_cnt = 0;
        time_set.set_enable = 0;
        testbuf[0] = time_set.time_num[0].num;
        testbuf[1] = time_set.time_num[1].num;
        testbuf[2] = time_set.time_num[2].num;
        testbuf[3] = time_set.time_num[3].num;
        writeFlash(0,sizeof(testbuf), testbuf);
        Serial.println("joy long pressed");
      }
    } else {
      joyPressed = false;
      if(last_joyPressed){
        joyPressed_twice_cnt = 20;
        Serial.println("joy pressed up");
      }
    }
    last_joyPressed = joyPressed;
}

void acControllCB() {
  static uint8_t temp_hh,temp_mm;
  temp_hh = time_set.time_num[0].num * 10 + time_set.time_num[1].num;
  temp_mm = time_set.time_num[2].num * 10 + time_set.time_num[3].num;
  if(hh == temp_hh && mm == temp_mm)
    turnOff_All_Ac();
}

void turnOff_All_Ac() {
    char buf[] = AC_OFF_CMD;
    Serial1.write(buf, sizeof(buf));
}

void turnOff_All_Ac_Force() {
    char buf[] = AC_FORCE_OFF_CMD;
    Serial1.write(buf, sizeof(buf));
}

void timeSetCB() {
  if(time_set.set_enable){
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    itoa(time_set.time_num[time_set.index].num, &cc, 10); 
    tft.drawString(&cc, time_set.time_num[time_set.index].posx,time_set.time_num[time_set.index].posy,6);
    delay(25);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString(&cc, time_set.time_num[time_set.index].posx,time_set.time_num[time_set.index].posy,6);
    delay(25);
  }
}

void writeFlash(uint32_t addr, size_t size, uint8_t *data) {
    sfud_err result = SFUD_SUCCESS;
    const sfud_flash *flash = sfud_get_device_table() + 0;
    size_t i;

    result = sfud_erase_write(flash, addr, size, data);
    if (result == SFUD_SUCCESS) {
        Serial.println("Write the flash data finish");
    } else {
        Serial.println("Write the flash data failed");
        return;
    }
}

void readFlash(uint32_t addr, size_t size, uint8_t *data) {
    sfud_err result = SFUD_SUCCESS;

    const sfud_flash *flash = sfud_get_device_table() + 0;
    size_t i;
    /* read test */
    result = sfud_read(flash, addr, size, testbuf);
    if (result == SFUD_SUCCESS) {
        Serial.println("Read the flash data success.");
        Serial.println("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        for (int i = 0; i < size; i++) {
            if (i % 16 == 0) {
                Serial.print("0x");
                Serial.print(addr + i,HEX);
                Serial.print("\t");
            }
            Serial.print(testbuf[i],HEX);
            Serial.print("\t");
            if (((i + 1) % 16 == 0) || i == size - 1) {
                Serial.println("");
            }
        }
        Serial.println(" ");
    } else {
        Serial.println("Read the flash data failed.");
    }
}
