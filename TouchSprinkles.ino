/*
 * Project TouchSprinkles
 * Description:
 * Author:
 * Date:
 */
// Define I/O:  Pump Controller - switch numbers refer to switches on board and exclude the main power switch.
// Variable     Pin      Physical     Action
//  INPUTS
// automan     D0        Switch 1     Auto on(1) Manual off(0)
// p1s         D1        Switch 2     Pump 1 Manual on(1)-off(0)
// p2s         D2        Switch 3     Pump 2 Manual on(1)-off(0)
// as          D3        Switch 4     Auger Manual on(1)-off(-)
// f4          D4        Switch 5     Float 4 Top up(1)-down(0)
// f3          D5        Switch 6     Float 3 Mid high up(1)-down(0)
// f2          D6        Switch 7     Float 2 Mid low up(1)-down(0)
// f1          D7        Switch 8     Float 1 Bottom up(1)-down(0)
// ext         D8        Switch 9     External Contact Automatic Permissive Run on(1)-off(0)
//  OUTPUTS
// pmp1        A1        Pump 1-in    2kHz PWM
// pmp2        A2        Pump 2-out   2kHz PWM
// aug         A3        Auger        2kHz PWM

#include"Particle.h"
#include"Nextion.h"
#include"math.h"
#include <JsonParserGeneratorRK.h>

SYSTEM_THREAD(ENABLED);

USARTSerial& nexSerial = Serial1;

#define DEBUG_SERIAL_ENABLE
#define dbSerial Serial
#define CONTROL_INTERVAL 1000
#define MOTOR_START_DELAY 1000
#define PWM_F 2000     //pwm frequency 500Hz to 10kHz available on Boron
#define PWM_O 255    //pwm output bits, default 8 bit on boron 0 - 255
#define HMI_O 100    //HMI defined input range 0 - 100 

int pmp = A0;
unsigned long lastcloudupdate;
unsigned long lastsensorinterval;
unsigned long motorinterval;

int automan = D0;  // automanual switch on pin D0
int p1s = D1;      // pump1 switch on pin D1
int p2s = D2;      // pump2 switch on pin D2
int as = D3;       // auger switch on pin D3
int f4 = D4;       // float 4 switch (top) on pin D4
int f3 = D5;       // float 3 switch (mid-top) on pin D5
int f2 = D6;       // float 2 switch (mid-bottom) on pin D6
int f1 = D7;       // float 1 switch (bottom) on pin D7
int ext = D8;      // external contactor switch (mid-bottom) on pin D4
int pmp1 = A1;     // pump 1 analog output on A1
int pmp2 = A2;     // pump 2 analog output on A2
int aug = A3;      // auger analog output on A3
int pmp1_s = 0;    // pump 1 speed startat 0, 0to255 (8bit analog out)
int pmp2_s = 0;    // pump 1 speed startat 0, 0to255 (8bit analog out)
int aug_s = 0;    // pump 1 speed startat 0, 0to255 (8bit analog out)

char buffer[100] = {0};

//Nextion Setup
//Dual State Buttons
NexDSButton bt0 = NexDSButton(0, 1, "bt0");     // Man-Auto
NexDSButton bt1 = NexDSButton(0, 2, "bt1");     //Pump 1
NexDSButton bt2 = NexDSButton(0, 8, "bt2");     //Pump 2
NexDSButton bt3 = NexDSButton(0, 9, "bt3");     //Auger
NexDSButton bt4 = NexDSButton(0, 10, "bt3");     //External Contact

//Buttons
NexButton b3 = NexButton (1, 4, "b3");          // Sets pump 1 set point n4 to slider value h0=n3

NexButton b6 = NexButton (1, 14, "b6");          // Sets pump 2 set point n6 to slider value h1=n5

NexButton b9 = NexButton (1, 21, "b9");          // Sets auger set point n8 to slider value h2=n7

//Numbers
NexNumber n0 = NexNumber(0, 11, "n0");   //pump 1 speed
NexNumber n1 = NexNumber(0, 12, "n1");   //pump 2 speed
NexNumber n2 = NexNumber(0, 13, "n2");   //auger speed
NexNumber n4 = NexNumber(1, 7, "n4");   //pump 1 set point
NexNumber n6 = NexNumber(1, 14, "n6");   //pump 2 set point
NexNumber n8 = NexNumber(1, 22, "n8");   //auger set point

//Text
NexText t1 = NexText(0, 4, "t1");     //float 4 status
NexText t2 = NexText(0, 5, "t2");     //float 3 status
NexText t3 = NexText(0, 6, "t3");     //float 2 status
NexText t4 = NexText(0, 7, "t4");     //float 1 status

void bt0PushCallback(void *ptr) {}

void bt0PopCallback(void *ptr) {}

void bt1PushCallback(void *ptr) {}

void bt1PopCallback(void *ptr) {}

void bt2PushCallback(void *ptr) {}

void bt2PopCallback(void *ptr) {}

void bt3PushCallback(void *ptr) {}

void bt3PopCallback(void *ptr){}

void bt4PushCallback(void *ptr) {}

void bt4PopCallback(void *ptr){}

void b3PushCallback(void *ptr){}

void b6PushCallback(void *ptr){}

void b9PushCallback(void *ptr){}

void n0PushCallback(void *ptr){}

void n1PushCallback(void *ptr){}

void n2PushCallback(void *ptr){}

void n4PushCallback(void *ptr){}

void n6PushCallback(void *ptr){}

void n8PushCallback(void *ptr){}

void t1PushCallback(void *ptr){}

void t2PushCallback(void *ptr){}

void t3PushCallback(void *ptr){}

void t4PushCallback(void *ptr){}

NexTouch *nex_listen_list[] = 
{
    &bt0,
    &bt1,
    &bt2,
    &bt3,
    &bt4,
    &b3,
    &b6,
    &b9,
    &n0,
    &n1,
    &n2,
    &n4,
    &n6,
    &n8,
    &t1,
    &t2,
    &t3,
    &t4,
    NULL
};


void setup() 
{
  nexInit();
  
  delay(500);

  bt0.attachPush(bt0PushCallback, &bt0);
  bt0.attachPop(bt0PopCallback, &bt0);
  bt1.attachPush(bt1PushCallback, &bt1);
  bt1.attachPop(bt1PopCallback, &bt1);
  bt2.attachPush(bt2PushCallback, &bt2);
  bt2.attachPop(bt2PopCallback, &bt2);
  bt3.attachPush(bt3PushCallback, &bt3);
  bt3.attachPop(bt3PopCallback, &bt3);
  bt4.attachPush(bt4PushCallback, &bt4);
  bt4.attachPop(bt4PopCallback, &bt4);
  b3.attachPush(b3PushCallback, &b3);
  b6.attachPush(b6PushCallback, &b6);
  b9.attachPush(b9PushCallback, &b9);
  n0.attachPush(n0PushCallback, &n0);
  n1.attachPush(n1PushCallback, &n1);
  n2.attachPush(n2PushCallback, &n2);
  n4.attachPush(n4PushCallback, &n4);
  n6.attachPush(n6PushCallback, &n6);
  n8.attachPush(n8PushCallback, &n8);
  t1.attachPush(t1PushCallback, &t1);
  t2.attachPush(t2PushCallback, &t2;
  t3.attachPush(t3PushCallback, &t3;
  t4.attachPush(t4PushCallback, &t4;

  dbSerial.println("Touch Listen Set Up");


  pinMode(automan, INPUT_PULLDOWN);
  pinMode(p1s, INPUT_PULLDOWN);
  pinMode(p2s, INPUT_PULLDOWN);
  pinMode(as, INPUT_PULLDOWN);
  pinMode(f4, INPUT_PULLDOWN);
  pinMode(f3, INPUT_PULLDOWN);
  pinMode(f2, INPUT_PULLDOWN);
  pinMode(f1, INPUT_PULLDOWN);
  pinMode(ext, INPUT_PULLDOWN);
  pinMode(pmp1, OUTPUT);
  pinMode(pmp2, OUTPUT);
  pinMode(aug, OUTPUT);

  lastsensorinterval = millis();
}


void loop()
{
  nexLoop(nex_listen_list);

  Time.zone(-7);

  if ((millis() - lastsensorinterval) > CONTROL_INTERVAL)
  {
    digitalRead(automan);
    digitalRead(p1s);
    digitalRead(p2s);
    digitalRead(as);
    digitalRead(f4);
    digitalRead(f3);
    digitalRead(f2);
    digitalRead(f1);
    digitalRead(ext);

  
    if(automan == 0)
    {
      // Pump 1 on(1) if switch 2 is on(1), otherwise Pump 1 off(0)
      if(p1s == 1)
      {
       analogWrite(pmp1, pmp1_s); 
      }
      else
      {
        analogWrite(pmp1, 0);
      }
      
      // Pump 2 on(1) if switch 3 is on(1), otherwise Pump 2 off(0)
      if(p2s == 1)
      {
        analogWrite(pmp2, pmp2_s);
      }
      else
      {
        analogWrite(pmp2, 0);
      }

      // Auger on(1) if switch 4 is on(1), otherwise Pump 2 off(0)
      if(as == 1)
      {
        analogWrite(aug, aug_s);
      }
      else
      {
        analogWrite(aug, 0);
      }
    }
    // Automatic Program if Auto/Man Switch 1 on(0) and external contactor is closed(0)
    if(automan == 1 && ext == 1)
    {
      // Pump 1 on(1) if float 3 down(0)
      if(f3 == 0)
      {
        analogWrite(pmp1, pmp1_s);
      }

      // Pump 1 off(0) if float 4 up(1)
      if(f4 == 1)
      {
        digitalWrite(pmp1, 0);
      }

      // Pump 2 on(1) if float 2 up(1)
      if(f2 == 1)
      {
        analogWrite(pmp2, pmp2_b);
      }

      // Pump 2 off(0) if float 1 down(0)
      if(f1 == 0)
      {
        analogWrite(pmp2, 0);
      }

      // Auger on(1) if either pump on
      if(pmp1 == HIGH || pmp2 == HIGH)
      {
        digitalWrite(aug, HIGH);
        delay(500);
      }
      else
      {
        digitalWrite(aug, LOW);
      }
      // control loop checks inputs and adjust outputs every 1 second
      delay(1000);
    }

 
  }
Serial.println(Time.timeStr());
    Serial.printlnf("SYSTEM IN MANUAL");
    Serial.printlnf("Auto(0)/Man(1)            : %d", automan_b);
    Serial.printlnf("Pump1 Switch  on(0)/off(1): %d", p1s_b);
    Serial.printlnf("Pump2 Switch  on(0)/off(1): %d", p2s_b);
    Serial.printlnf("Auger Switch  on(0)/off(1): %d", as_b);
    Serial.printlnf("Float 4      up(0)/down(1): %d", f4_b);
    Serial.printlnf("Float 3      up(0)/down(1): %d", f3_b);
    Serial.printlnf("Float 2      up(0)/down(1): %d", f2_b);
    Serial.printlnf("Float 1      up(0)/down(1): %d", f1_b);
    Serial.printlnf("External      on(0)/off(1): %d", ext_b);
    Serial.printlnf("Pump 1        on(1)/off(0): %d", pmp1_b);
    Serial.printlnf("Pump 2        on(1)/off(0): %d", pmp2_b);
    Serial.printlnf("Auger         on(1)/off(0): %d", aug_b);

    CreateEventPayload(automan_i, p1s_i, p2s_i, as_i, f4_i, f3_i, f2_i, f1_i, ext_i, pmp1_i, pmp2_i, aug_i);
  digitalWrite(pmp1, LOW);
  digitalWrite(pmp2, LOW);
  digitalWrite(aug, LOW);
}



void CreateEventPayload(int automan_i, int p1s_i, int p2s_i, int as_i, int f4_i, int f3_i, int f2_i,
                        int f1_i, int ext_i, int pmp1_i, int pmp2_i, int aug_i) 
{
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue("Auto(0)/Man(1): %d", automan_i);
    jw.insertKeyValue("P1 Sw on(0): %d", p1s_i);
    jw.insertKeyValue("P2 Sw on(0): %d", p2s_i);
    jw.insertKeyValue("Aug Sw on(0): %d", as_i);
    jw.insertKeyValue("F4 on(0): %d", f4_i);
    jw.insertKeyValue("F3 on(0): %d", f3_i);
    jw.insertKeyValue("F2 on(0): %d", f2_i);
    jw.insertKeyValue("F1 on(0): %d", f1_i);
    jw.insertKeyValue("Ext on(0): %d", ext_i);
    jw.insertKeyValue("Pump 1: %d", pmp1_i);
    jw.insertKeyValue("Pump 2: %d", pmp2_i);
    jw.insertKeyValue("Auger: %d", aug_i);
    
  }

  Particle.publish("Sprinkles Speaks:", jw.getBuffer(), PRIVATE);
}
