/****************************************
* Name: Austin Finch
* Assignment: Final Project
* Date: 12/15/2023
****************************************/

#include <LiquidCrystal.h>
#include <Stepper.h>
#include "DHT.h"
#include <DS3231.h>

#define DHTPIN 23
#define DHTTYPE DHT11

//DHT-PA0, Water Sensor-PA1, Blue LED-PA2, Green LED-PA4, Yellow LED-PA6
volatile unsigned char* port_a  = (unsigned char*) 0x22;
volatile unsigned char* ddr_a   = (unsigned char*) 0x21;
volatile unsigned char* pin_a   = (unsigned char*) 0x20;


//Start/Stop-PK0, Reset-PK1
volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* pin_k = (unsigned char*) 0x106;
//Start/Stop-PCINT16, Reset-PCINT17
// volatile unsigned char *myPCIMSK = (unsigned char*) 0x6D;
// volatile unsigned char *myPCICR = (unsigned char*) 0x68;

//Red LED-PC7
volatile unsigned char* port_c  = (unsigned char*) 0x28;
volatile unsigned char* ddr_c   = (unsigned char*) 0x27;
volatile unsigned char* pin_c   = (unsigned char*) 0x26;

#define RDA 0x80
#define TBE 0x20
volatile unsigned char *myUCSR0A = (unsigned char *)0xC0;
volatile unsigned char *myUCSR0B = (unsigned char *)0xC1;
volatile unsigned char *myUCSR0C = (unsigned char *)0xC2;
volatile unsigned int *myUBRR0 = (unsigned int *) 0xC4;
volatile unsigned char *myUDR0 = (unsigned char *)0xC6;
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Stepper motor, pins 8-11
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

//DC motor, pins 2,3
int speedPin=3;
int dir1=2;

//LCD, pins 4-7, 12, 13
const int RS = 13, EN = 12, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//DHT, pin 23
DHT dht(DHTPIN, DHTTYPE);

//RTC, pins SDA and SCL
DS3231 rtc(SDA, SCL);

int state;

void setup() 
{
  //PA2 Output
  *ddr_a |= (0x01 << 2);
  //PA4 Output
  *ddr_a |= (0x01 << 4);
  //PA6 Output
  *ddr_a |= (0x01 << 6);
  //PC7 Output
  *ddr_c |= (0x01 << 7);
  //PK0 Input
  *ddr_k &= ~(0x01 << 0);
  //Enable pullup resistor on PK0
  *port_k |= (0x01 << 0);
  //PK1 Input
  *ddr_k &= ~(0x01 << 1);
  //Enable pullup resistor on PK1
  *port_k |= (0x01 << 1);
  // Enable PCIE2 
  //*myPCICR |= (0x01 << 2);

  lcd.begin(16,2);
  pinMode(22, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(speedPin, OUTPUT);

  // setup the UART
  U0init(9600);
  // setup the ADC
  adc_init();
  dht.begin();
  rtc.begin();
  lcd.clear();
  state = 0;
}

void loop() 
{
  switch (state) {
    case 0:
      Running();
      break;
    case 1:
      Idle();
      break;
    case 2:
      Error();
      break;
    case 3:
      Disabled();
      break;
  }
}

void Running()
{
  //Blue LED ON
  *port_a |= (0x01 << 2);
  //Green LED OFF
  *port_a &= ~(0x01 << 4);
  //Yellow LED OFF
  *port_a &= ~(0x01 << 6);
  //Red LED OFF
  *port_c &= ~(0x01 << 7);
  motorRun(255);

  float tempC = displayHumidTemp();
  adjustStepper();

  //Change to Idle state when temperature falls below threshold
  if (tempC <= 20)
  {
    state = 1;
    motorRun(0);
    String message = "State change to: Idle";
    log(message);
  }

  //Change to Error state when water level falls below threshold
  unsigned int level = readWaterLevel();
  if(level < 513)
  {
    state = 2;
    String message = "State change to: Error";
    log(message);
  }

  // Enable Interrupt (PCINT16)
  //*myPCIMSK |= 0x01
  // Run ISR if Stop pressed
  bool stopPressed = false;
  if (stopPressed)
  {
    StartStop();
  }
}

void Idle()
{
  //Blue LED OFF
  *port_a &= ~(0x01 << 2);
  //Green LED ON
  *port_a |= (0x01 << 4);
  //Yellow LED OFF
  *port_a &= ~(0x01 << 6);
  //Red LED OFF
  *port_c &= ~(0x01 << 7);
  

  //Change to Error state when water level falls below threshold
  unsigned int level = readWaterLevel();
  if(level < 513)
  {
    state = 2;
    String message = "State change to: Error";
    log(message);
  }

  float tempC = displayHumidTemp();
  adjustStepper();

  //Change to Running state when temperature rises above threshold
  if (tempC > 20)
  {
    state = 0;
    String message = "State change to: Running";
    log(message);
  }
  
  // Enable Interrupt (PCINT16)
  //*myPCIMSK |= 0x01
  // Run ISR if Stop pressed
  bool stopPressed = false;
  if (stopPressed)
  {
    StartStop();
  }
}

void Disabled()
{
  //Blue LED OFF
  *port_a &= ~(0x01 << 2);
  //Green LED OFF
  *port_a &= ~(0x01 << 4);
  //Yellow LED ON
  *port_a |= (0x01 << 6);
  //Red LED OFF
  *port_c &= ~(0x01 << 7);

  lcd.clear();

  // Enable Interrupt (PCINT16)
  //*myPCIMSK |= 0x01
  // Run ISR if Start pressed
  bool startPressed = false;
  if (startPressed)
  {
    StartStop();
  }
}

void Error()
{
  //Blue LED OFF
  *port_a &= ~(0x01 << 2);
  //Green LED OFF
  *port_a &= ~(0x01 << 4);
  //Yellow LED OFF
  *port_a &= ~(0x01 << 6);
  //Red LED ON
  *port_c |= (0x01 << 7);

  lcd.clear();

  // Enable Interrupt (PCINT16)
  //*myPCIMSK |= 0x01
  // Run ISR if Stop pressed
  bool stopPressed = false;
  if (stopPressed) 
  {
    StartStop();
  }

  // Enable Interrupt (PCINT17)
  //*myPCIMSK |= (0x01 << 1)
  bool resetPressed = true;
  //If reset pressed, run the ISR
  if (resetPressed)
  {
    reset();
  }
}

void motorRun(int mSpeed)
{
  digitalWrite(dir1, HIGH);
  analogWrite(speedPin, mSpeed);
}

// Should actually be ISR(PCINT16_vect){}
void StartStop()
{
  if(state == 3)
  {
    state = 1;
    String message = "State change to: Idle";
    log(message);
  }
  else
  {
    motorRun(0);
    state = 3;
    String message =  "State change to: Disabled";
    log(message);
  }
}

//Should be ISR(PCINT17_vect){}
void reset()
{
  unsigned int level = readWaterLevel();
  if (level >= 513)
  {
    state = 1;
    motorRun(0);
    String message = "State change to: Idle";
    log(message);
  }
}

float displayHumidTemp()
{
  //Get Temperature and Humidity Data
  float humi = dht.readHumidity();
  float tempC = dht.readTemperature();

  //  Humidity: XX.XX
  //  Temp: XX.XX
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Humidity: ");
  lcd.print(humi);
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(tempC);
  return tempC;
}

unsigned int readWaterLevel()
{
  // Enable Sensor
  digitalWrite(22, HIGH);
  // Read Water Level
  unsigned int level = adc_read(1);
  // Disable Sensor
  digitalWrite(22, LOW);
  return level;
}

void adjustStepper()
{
  //Get potentiometer value
  int in = adc_read(0);
  //Map value -180 to 180
  int angle = map(in, 0, 1023, -180, 180);
  //Set zero movement threshold -10 to 10 degrees
  if ((angle >= 10)||(angle <= -10))
  {
    //Move stepper motor by the defined angle
    myStepper.setSpeed(10);
    myStepper.step((stepsPerRevolution/180)*angle);
    String message = "Angle adjusted by: " + angle;
    log(message);
  }
}

void log(String message)
{
  //Get date and time
  String date = rtc.getDateStr();
  String time = rtc.getTimeStr();

  //Create char* of date and time + message
  int len = date.length()+time.length()+message.length();
  len = len+(sizeof('  '));
  char* msg = new char[len];
  for (int i = 0; i < date.length(); i++)
  {
    msg[i] = date.charAt(i);
  }
  msg[date.length()] = ' ';
  for (int i = 0; i < time.length(); i++)
  {
    msg[i] = time.charAt(i);
  }
  msg[date.length()+time.length()+1] = ' ';
  for (int i = 0; i < message.length(); i++)
  {
    msg[i] = message.charAt(i);
  }
  
  //Print the message 1 char at a time
  for(int i = 0; i < len; i++)
  {
    U0putchar(msg[i]);
  }
  U0putchar('\n');
  free(msg);
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit 7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow readin
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000; // set bit 6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void U0print(int in)
{
  if (in == '0')
  {
    U0putchar('0');
  }
  else
  {
    if (in < 0)
    {
      U0putchar('-');
      in = -in;
    }

    if(in >= 1000)
    {
      U0putchar(in / 1000 + '0');
      in = in % 1000;
    }

    if(in >= 100)
    {
      U0putchar(in / 100 + '0');
      in = in % 100;
    }

    if(in >= 10)
    {
      U0putchar(in / 10 + '0');
      in = in % 10;
    }

      U0putchar(in + '0');
  }
}
