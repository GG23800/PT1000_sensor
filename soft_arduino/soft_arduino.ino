// LCD screen
#include<LiquidCrystal.h>

LiquidCrystal lcd(A0,A1,4,5,6,7);

// Rotary encoder
int encoderPinA = 2;   // right (DT)
int encoderPinB = 3;   // left (CLK)
int clearButton = 8;    // switch (SW)

volatile int encoderPos = 0;  // un compteur
static boolean rotating=false;      // gestion de l'anti-rebonds
int8_t test_id;
// variable pour les routines d'interruption
boolean A_set = false;             
boolean B_set = false;
boolean A_change = false;
boolean B_change= false;

// RTC
#include<Wire.h>
#include<RTClib.h>
// doc: https://adafruit.github.io/RTClib/html/index.html

RTC_DS3231 rtc;
bool rtc_connected; // check if RTC connected
bool edit_save_step_time;
DateTime adt; // object containig date and time
uint8_t val;
uint16_t lyear;
int save_step_time;
uint32_t next_save_timestamp;

// watchdog
#include <avr/wdt.h>

// SD card
#include <SPI.h>
#include <SD.h>
//#include <SdFat.h>
//SdFat SD;
const int chipSelect = 10;
int8_t last_day;
File myFile;
bool card_inserted;
char filename[13];

// Temperature sensor
int Nts = 4; //number of sensors
float A = 0.0039083f;
float B = -0.0000005775f;
float T1[10], T2[10], T3[10], T4[10];
unsigned int loopid;

// Pump
int pump_pin = 9;

void setup() {
  // if watchdog activated
  wdt_disable();
  // LCD
  lcd.begin(16,2);
  lcd.print("Booting");

  // Rotary encoder
  pinMode(encoderPinA, INPUT_PULLUP); // utilisation du pullup
  pinMode(encoderPinB, INPUT_PULLUP); // utilisation du pullup
  pinMode(clearButton, INPUT_PULLUP); // utilisation du pullup
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);
  test_id = 0;

  //RTC
  #ifdef AVR
    Wire.begin();
  #else
    Wire1.begin();
  #endif

  rtc_connected = rtc.begin();
  save_step_time = 300; // 5 minutes
  edit_save_step_time = false;
  if (rtc_connected)
  {
    adt = rtc.now();
    next_save_timestamp = adt.secondstime() + uint32_t(10*save_step_time);
  }
  else
  {
    lcd.setCursor(0,0);
    lcd.print("No clock");
    adt = DateTime(uint32_t(0));  
  }
  // uncomment only if RTC hour not good
  //if (rtc_connected) {rtc.adjust(DateTime(__DATE__,__TIME__));}

  // temperature sensor setup
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  for (int i=0 ; i<10 ; i++)
  {
    T1[i] = 0.f;
    T2[i] = 0.f;
    T3[i] = 0.f;
    T4[i] = 0.f;
  }
  loopid = 0;

  // watchdog
  wdt_enable(WDTO_8S);

  // SD card
  last_day = -1;
  filename[0] = '\0';
  if (!SD.begin(chipSelect)) 
  {
    card_inserted = false;
    lcd.setCursor(11,0);
    lcd.print("NO SD");
  }
  else 
  {
    card_inserted = true;
    lcd.setCursor(11,0);
    lcd.print("SD OK");
    myFile = SD.open("info.log", FILE_WRITE);
    if (myFile)
    {
      adt = rtc.now();
      myFile.print(adt.timestamp(DateTime::timestampOpt(2))); // date format YYYY-MM-DD
      myFile.print(' ');
      myFile.print(adt.timestamp(DateTime::timestampOpt(1))); // time format hh:mm:ss
      myFile.print(' ');
      myFile.println("new boot");
      myFile.close();
    }
    delay(500);
  }

  // Pump
  pinMode(pump_pin, OUTPUT);
}

// Interrupt on A change state
void doEncoderA()
{  
  // debounce
  if ( rotating ) delay (1);  // attendre un petit peut
  rotating = true; //activation de l'anti-rebond
  // Confirmation du changement
  if( digitalRead(encoderPinA) != A_set ) 
  {
    A_set = !A_set;
    if (B_change) 
    {
      if (edit_save_step_time)
      {
        save_step_time++;
      }
      else
      {
        encoderPos += 1;
        if (encoderPos == Nts) {encoderPos = 0;}
      }
      B_change = false;
    } 
    else
    {
      A_change = true;
    }

    rotating = false;  //libération de l'anti-rebond
  }
}

// Interrupt on B change state
void doEncoderB()
{
  if ( rotating ) delay (1);
  rotating = true;
  if( digitalRead(encoderPinB) != B_set ) 
  {
    B_set = !B_set;
    if (A_change) 
    {
      if (edit_save_step_time)
      {
        save_step_time--;
        if (save_step_time < -2) {save_step_time = -2;}
      }
      else
      {
        encoderPos -= 1;
        if (encoderPos < 0) {encoderPos = Nts - 1;}
      }
      A_change = false;
    } 
    else
    {
      B_change = true;
    }
    rotating = false;
  }
}

void print_time()
{
  if (rtc_connected)
  {
    lcd.setCursor(0,0);
    adt = rtc.now();
    lcd.print(adt.timestamp(DateTime::timestampOpt(1)));
  }
}

float temperature(int val, int sensor)
{
  float Rr = 0.f;
  switch(sensor)
  {
    case 1:
      Rr = 0.998f;
      break;
    case 2:
      Rr = 0.991f;
      break;
    case 3:
      Rr = 1.002f;
      break;
    case 4:
      Rr = 0.993f;
      break;
    default:
      Rr = 1.f;
      break;
  }
  float V = float(val);
  float R = Rr*V/(1024.f-V);
  float delta = A*A - 4.f*B*(1.f-R);
  float t = (-A+sqrt(delta))/(2.f*B);
  return t;
}

void print_sensor()
{
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  int val = 10;
  float temp = 0.f;
  switch(encoderPos)
  {
    case 0:
      lcd.print("T1:");
      val = analogRead(A7);
      for (int i=0 ; i<10 ; i++) {temp += T1[i];}
      break;
    case 1:
      lcd.print("T2:");
      val = analogRead(A6);
      for (int i=0 ; i<10 ; i++) {temp += T2[i];}
      break;
    case 2:
      lcd.print("T3:");
      val = analogRead(A3);
      for (int i=0 ; i<10 ; i++) {temp += T3[i];}
      break;
    case 3:
      lcd.print("T4:");
      val = analogRead(A2);
      for (int i=0 ; i<10 ; i++) {temp += T4[i];}
      break;
    default:
      lcd.print("sensor pb");
      break;
  }

  if (val == 1023)
  {
    lcd.setCursor(14,1);
    lcd.print("NC");
  }
  else if (val == 0)
  {
    lcd.setCursor(14,1);
    lcd.print("CC");
  }
  else
  {
    lcd.print(' ');
    temp /= 10.f;
    lcd.print(temp);
    lcd.write((char)223); // ° character
    lcd.print("C");
  }
}

void edit_save_step_time_function()
{
  static int nmin=0, nsec=0;
  int old_sst = save_step_time;
  delay(200);
  while (digitalRead(clearButton)==HIGH)
  {
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
    if (save_step_time >= 0)
    {
      lcd.print("step: ");
      nmin = int(save_step_time/6);
      lcd.print(nmin,DEC);
      lcd.print(':');
      nsec = int(save_step_time - 6*nmin);
      lcd.print(nsec,DEC);
      lcd.print('0');
    }
    else
    {
      if (save_step_time == -1) {lcd.print("pump test");}
      else if (save_step_time == -2) {lcd.print("reboot");}
      else {lcd.print("not implemented");}
    }
    print_time();
    wdt_reset(); // if we take time while changing time step, we don't want a reboot
    delay(100);
  }
  
  if (rtc_connected)
  {
    adt = rtc.now();
  }
  else
  {
    adt = DateTime(uint32_t(millis()/1000));
  }
  
  if (save_step_time == -2)
  {
    wdt_enable(WDTO_1S);
    delay(2000);
  }
  else if (save_step_time == -1)
  {
    save_step_time = old_sst;
    next_save_timestamp = adt.secondstime() + uint32_t(10*save_step_time);
    analogWrite(pump_pin, 250);
  }
  else
  {
    next_save_timestamp = adt.secondstime();
  }
  delay(200);
}

void file_print_sensor()
{
  float temp = 0.f;
  myFile.print(adt.timestamp(DateTime::timestampOpt(1))); // time format hh:mm:ss
  myFile.print(' ');
  for (int i=0 ; i<10 ; i++) {temp += T1[i];}
  temp /= 10.f;
  myFile.print(temp);
  myFile.print(' ');
  temp = 0.f;
  for (int i=0 ; i<10 ; i++) {temp += T2[i];}
  temp /= 10.f;
  myFile.print(temp);
  myFile.print(' ');
  temp = 0.f;
  for (int i=0 ; i<10 ; i++) {temp += T3[i];}
  temp /= 10.f;
  myFile.print(temp);
  myFile.print(' ');
  temp = 0.f;
  for (int i=0 ; i<10 ; i++) {temp += T4[i];}
  temp /= 10.f;
  myFile.println(temp);
  myFile.close();
}

void edit_filename()
{
  static int ld = 1;
  static int lm = 1;
  static int ly = 2000;
  if (int(adt.day()) != last_day)
  {
    int logday = 0;
    logday = int(adt.year());
    itoa(logday,&filename[0],10);
    logday = int(adt.day());
    logday += 100*int(adt.month());
    if (logday < 1000)
    {
      filename[4] = '0';
      itoa(logday,&filename[5],10);
    }
    else {itoa(logday,&filename[4],10);}
    filename[8] = '.';
    filename[9] = 't';
    filename[10] = 'x';
    filename[11] = 't';
    filename[12] = '\0';
  }
}

void loop() 
{
  wdt_reset();
  T1[loopid%10]=temperature(analogRead(A7),1);
  T2[loopid%10]=temperature(analogRead(A6),2);
  T3[loopid%10]=temperature(analogRead(A3),3);
  T4[loopid%10]=temperature(analogRead(A2),4);
  print_time();
  print_sensor();

  if (digitalRead(clearButton)==LOW)
  {
    edit_save_step_time = true;
    edit_save_step_time_function();
    edit_save_step_time = false;
  }

  if (!card_inserted)
  {
    if (card_inserted = SD.begin(chipSelect))
    {
      lcd.setCursor(11,0);
      lcd.print("SD OK");
      myFile = SD.open("info.log", FILE_WRITE);
      if (rtc_connected)
      {
        adt = rtc.now();
      }
      else
      {
        adt = DateTime(uint32_t(millis()/1000));
      }
      myFile.print(adt.timestamp(DateTime::timestampOpt(2))); // date format YYYY-MM-DD
      myFile.print(" ");
      myFile.print(adt.timestamp(DateTime::timestampOpt(1))); // time format hh:mm:ss
      myFile.println(" sd card reinserted");
      myFile.close();
    }
  }
  else // check if card is inserted each loop
  {
    myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile) 
    {
      noInterrupts();
      myFile.close();
      interrupts();
    }
    else 
    {
      lcd.setCursor(11,0);
      lcd.print("NO SD");
      card_inserted = false;
      SD.end(); //for sd.h
    }
  }
  if (card_inserted) // we write in folder only if card is inserted
  {
    if (rtc_connected)
    {
      adt = rtc.now();
    }
    else
    {
      adt = DateTime(uint32_t(millis()/1000));
    }
    if (adt.secondstime() >= next_save_timestamp)
    {
      float temp = 0.f;
      for (int i=0 ; i<10 ; i++) {temp += T1[i];}
      temp /= 10;
      if (temp <= 55.f) {analogWrite(pump_pin,0);}
      else if (temp >= 60.f) {analogWrite(pump_pin,250);}
      
      edit_filename();
      bool exist_file = SD.exists(filename);
      myFile = SD.open(filename, FILE_WRITE);
      if (myFile)
      {
        noInterrupts();
        if (!exist_file) {myFile.println("TIME T1 T2 T3 T4");}
        file_print_sensor();
        myFile.close();
        interrupts();
        next_save_timestamp += uint32_t(10*save_step_time);
        if (rtc_connected)
        {
          adt = rtc.now();
        }
        else
        {
          adt = DateTime(uint32_t(millis()/1000));
        }
        if (next_save_timestamp <= adt.secondstime()) 
        {next_save_timestamp = adt.secondstime() + save_step_time;}
      }
      else
      {
        card_inserted = false;
        SD.end(); //for sd.h
        lcd.setCursor(11,0);
        lcd.print("NO SD");  
      }
    }
  }
  delay(100);
  loopid++;
}
