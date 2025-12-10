//Colby Jones
//11-16-2025
//CPE 301, Section 1001
//Group 16
//Final project: Create a simulation of a swamp cooler
//Allowed to use Arduino libraries for the stepper motor,
//LCD, clock and temp/humidity sensor.
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <dht.h>
#include <RTClib.h>
#define RDA 0x80
#define TBE 0x20 

//counter
volatile unsigned char *myTCCR1A = (unsigned char *)0x80;
volatile unsigned char *myTCCR1B = (unsigned char *)0x81;
volatile unsigned char *myTCCR1C = (unsigned char *)0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *)0x6F;
volatile unsigned int *myTCNT1 = (unsigned int *)0x84;
volatile unsigned char *myTIFR1 = (unsigned char *)0x36;

//USB communication
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//ADC Registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Ports
unsigned char* ddr_b = (unsigned char*) 0x24;
unsigned char* port_b = (unsigned char*) 0x25;
unsigned char* pin_b = (unsigned char*) 0x23;
unsigned char* ddr_h = (unsigned char*) 0x101;
unsigned char* port_h = (unsigned char*) 0x102;
unsigned char* pin_a = (unsigned char*) 0x20;
unsigned char* ddr_a = (unsigned char*) 0x21;
unsigned char* port_a = (unsigned char*) 0x22;
unsigned char* pin_c = (unsigned char*) 0x26;
unsigned char* ddr_c = (unsigned char*) 0x27;
unsigned char* port_c = (unsigned char*) 0x28;
unsigned char* pin_d = (unsigned char*) 0x29;
unsigned char* ddr_d = (unsigned char*) 0x30;
unsigned char* port_d = (unsigned char*) 0x31;

//assigns ports for LCD
const int RS = 12, EN = 11, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//assigns port for DHT
dht DHT;
#define DHT11_PIN 24

//defines step motor steps, defines pins for step motor
const int stepsPerRevolution = 2038;
Stepper myStepper= Stepper(stepsPerRevolution, 6, 8, 7, 9);

//RTC clock
RTC_DS3231 rtc;

void setup() {
  U0init(9600);
  //sets up the timer
  setup_timer_regs();
  adc_init();
  //RTC clock
  rtc.begin();
  //recalibates clock
  //rtc.adjust(DateTime(__DATE__, __TIME__));
  attachInterrupt(digitalPinToInterrupt(18), pressChk, RISING);

  //LCD
  lcd.begin(16,2);

  //adc
  //port Section
  //DC motor ports
  DDRB |= (1<< PB4);//ENA
  DDRA |= (1<< PA0);//IN2
  DDRA |= (1<< PA1);//IN1

  // start/stop button
  DDRD &= ~(1<< PD3);
  //reset button
  DDRA &= ~(1<< PA3);
  PORTA |=(1 << PA3);

  //tilt up/down buttons
  //dwn
  DDRC &= ~(1<< PC5);
  //up
  DDRC &= ~(1<< PC4);

  //LEDs
  DDRA |= (1<< PA5);//redLED
  DDRA |= (1<< PA6);//yellowLED
  DDRA |= (1<< PA7);//greenLED
  DDRC |= (1<< PC7);//blueLED

}

int speedPin= 10;
//int to track on/off button
volatile int buttonTrk= 0;
//variables to keep track of cooler on/ off status. Outside of loop to prevent overwriting variables
int coolStatus= -1;
bool repeat= false;
bool errorLoop;

//reports humidity and temperature every minute, will not activate until interval value is reached
long interval= 60000;
//2 seciond delay for DHT(bug)
long dhtInterval=2000;
unsigned long prevMillis= 0;
unsigned long dhtmillis=0;
int temp=0;
void loop(){
  // main sturcture of system
  //four main states
  //states can be switched depending on temp, humidity, threshold, etc.
  unsigned short wtrLvl;

  //integer variable to calculate the value that is considered "low voltage/low water level"
  //This is converted to a value between 0 and 1023.
  unsigned int levelThreshold= (1.25/5)*1023;
  
  unsigned long currentMillis = millis();
  if (currentMillis - dhtmillis >= dhtInterval) {
    dhtmillis = currentMillis;
    
    int chk = DHT.read11(DHT11_PIN); 
  }

  //for all states except disabled
  //determines if user input is start/stop using int buttonTrk
  if(buttonTrk % 2 !=0){
    //if the reset button is pressed, the loop is stopped. It will return to this loop of the water level is still too low.
    wtrLvl= adc_read(0);
    //State 3, Error
    if(wtrLvl<= levelThreshold){
      //motor is off, regardless of water temp reading.
      //press a reset button to return to Idle is the water level is high enough.
      //checks if message was already sent
      if(coolStatus==0){
        repeat= true;
      }
      else{
        coolStatus= 0;
        repeat= false;
        lcd.clear();
      }
      
      //Error message on LCD, Red LED on(others off).
      errorLoop= true;
      
      //DC motor off
      PORTA &= ~(1<< PA0);
      PORTA &= ~(1<< PA1);
      analogWrite(speedPin, 0);

      //generates error message
      lcd.setCursor(0,0);
      lcd.print("Error: Water");
      lcd.setCursor(0,1);
      lcd.print("Level Low");
        
      //Red LED activates, others off
      PORTA |= (1<< PA5);
      PORTA &= ~(1<< PA6);
      PORTA &= ~(1<< PA7);
      PORTC &= ~(1<< PC7);
    }
    //state 4, Running
    else if((int)DHT.temperature>= 23&& !errorLoop){
      //checks if message was already sent
      if(coolStatus==1){
        repeat= true;
      }
      else{
        coolStatus= 1;
        repeat= false;
      }
    
      //fan motor turns on. If temp> threshold, move to Idle state.
      //DC motor off
      PORTA |= (1<< PA0);
      PORTA &= ~(1<< PA1);
      analogWrite(speedPin, 255);
      
      //If the water level< threshold, move to Error state.
      //Blue LED on.
      PORTA &= ~(1<< PA5);
      PORTA &= ~(1<< PA6);
      PORTA &= ~(1<< PA7);
      PORTC |= (1<< PC7);
    }
    //state 2, Idle
    //use the clock to create time stamps to Serial Monitor.
    //water level is continuously monitored. If the water level is too low, move to Error state.
    //Green LED is on.
    else if(!errorLoop){
      //activates if system not in error, considered idle.
      PORTA |= (1<< PA7);
      PORTC &= ~(1<< PC7);
      //DC motor off
      PORTA &= ~(1<< PA0);
      PORTA &= ~(1<< PA1);
      analogWrite(speedPin, 0);
      
      //checks if message was already sent
      if(coolStatus==0){
        repeat= true;
      }
      else{
        coolStatus= 0;
        repeat=false;
      }
      
      PORTA &= ~(1<< PA5);
      PORTA &= ~(1<< PA6);
      PORTA |= (1<< PA7);
      PORTC &= ~(1<< PC7);
    }
    //if up button is pressed, the stepper motor moves ccw. If both are pressed, inputs cancel out
    //stepper motor wil rotate until buttons are not pressed.
    if(PINC & (1<< PC5)){
      myStepper.setSpeed(5);
      myStepper.step(-5);
      temp= 2;
    }
    //if down button is pressed,the stepper motor moves cw
    else if(PINC & (1<< PC4)){
      myStepper.setSpeed(5);
      myStepper.step(5);
      temp= 1;
    }
    else{
      sendMotorMsg(temp);// sends to function that displays direction the stepper motor moved in. 1= ccw, 2= cw, 0= not sending message
      temp=0;
    }

    unsigned long currentMillis2= millis();
    if(!errorLoop){
      if(currentMillis2-prevMillis >= interval){
        prevMillis=currentMillis2;
        //function to report temp/humidity to LCD
        displayHumTemp();
      }
    }
    //reset button implementation
    if(PINA & (1 << PA3)){
      buttonTrk++;
      errorLoop= false;
      coolStatus= -1;
      lcd.clear();
    }
    
  }
  //State 1, Disabled
  //Initial conditons, Fan is off, Yellow LED is on
  //water temp not recorded.
  else{
    lcd.clear();
    if(coolStatus==0){
      repeat= true;
    }
    else{
      coolStatus= 0;
      repeat= false;
    }
    //DC motor off
    PORTA &= ~(1<< PA0);
    PORTA &= ~(1<< PA1);
    analogWrite(speedPin, 0);

    PORTA &= ~(1<< PA5);
    PORTA |= (1<< PA6);
    PORTA &= ~(1<< PA7);
    PORTC &= ~(1<< PC7);
  }

  //checks if repeat= true, prevents messages sent every loop
  if(repeat!=true){
    switch(coolStatus){
      case 0:
        U0putChar('C'); U0putChar('o'); U0putChar('o'); U0putChar('l'); U0putChar('e'); U0putChar('r');
        U0putChar(' '); U0putChar('o'); U0putChar('f'); U0putChar('f'); U0putChar(':'); U0putChar('\n');
        serialMessageDate();
        break;
      case 1:
        U0putChar('C'); U0putChar('o'); U0putChar('o'); U0putChar('l'); U0putChar('e'); U0putChar('r');
        U0putChar(' '); U0putChar('o'); U0putChar('n'); U0putChar(':'); U0putChar('\n');
        serialMessageDate();
        break;
    }
  }
}
void sendMotorMsg(int di){
  if(di==1){
    U0putChar('V'); U0putChar('e'); U0putChar('n'); U0putChar('t'); U0putChar(' '); 
    U0putChar('R'); U0putChar('o'); U0putChar('t'); U0putChar('a'); U0putChar('t'); U0putChar('e'); U0putChar('d'); U0putChar(' ');
    U0putChar('C'); U0putChar('W'); U0putChar('\n');
    serialMessageDate();
  }
  else if(di==2){
    U0putChar('V'); U0putChar('e'); U0putChar('n'); U0putChar('t'); U0putChar(' '); 
    U0putChar('R'); U0putChar('o'); U0putChar('t'); U0putChar('a'); U0putChar('t'); U0putChar('e'); U0putChar('d'); U0putChar(' ');
    U0putChar('C');  U0putChar('C'); U0putChar('W'); U0putChar('\n');
    serialMessageDate();
  }
}
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  if(!(*myUCSR0A & (1<< 7))){
    return false;
  }
  return true;
}

unsigned char U0getChar()
{
  return *myUDR0;
}

void U0putChar(unsigned char U0pdata)
{
  while(!(*myUCSR0A & (1<< 5)));
  *myUDR0= U0pdata;
}

void adc_init()
{
  // Configure ADCSRA:
  // ADEN: Enable ADC
  // ADPS2, ADPS1, ADPS0: Set prescaler to 128 (for 16MHz clock, 125kHz ADC clock)
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // Configure ADMUX:
  // REFS0: Set AVCC as reference (with external capacitor at AREF)
  // MUX bits: Select ADC channel A0 (all MUX bits 0)
  ADMUX = (1 << REFS0); // A0 is default if MUX bits are 0
}
unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
  // Start ADC conversion
  ADCSRA |= (1 << ADSC);

  // Wait for conversion to complete
  while (ADCSRA & (1 << ADSC));

  // Read the 10-bit result
  // Read ADCL first, then ADCH
  uint16_t adc_value = ADCL;
  adc_value |= (ADCH << 8);
  return adc_value;
}
void displayHumTemp(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Humidity: ");
  lcd.setCursor(12, 0);
  lcd.print((int)DHT.humidity);
  lcd.setCursor(15, 0);
  lcd.print("%");
  lcd.setCursor(0,1);
  lcd.print("Air temp: ");
  lcd.setCursor(12, 1);
  lcd.print((int)DHT.temperature);
  lcd.setCursor(15,1);
  lcd.print("C");
}

void serialMessageDate(){
  DateTime now= rtc.now();

  //displays year, month, day, hour, minute, and second approximation using RTC clock
  intToDivChar(now.month(),2); U0putChar('/'); intToDivChar(now.day(),2); U0putChar('/'); intToDivChar(now.year(),4);
  U0putChar(' '); U0putChar('@'); U0putChar(' ');
  intToDivChar(now.hour(),2); U0putChar(':'); intToDivChar(now.minute(),2); 
  U0putChar('.'); intToDivChar(now.second(),2); U0putChar('\n');
}

void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= (1<< TOV1);
  
  // enable the TOV interrupt
  *myTIMSK1 |=TOIE1;
}

void pressChk(){
  if(!errorLoop){
    buttonTrk++;
  }
}
//divides each number into its possible digits, then sends each character one at a time through the USB
void intToDivChar(int num, int digTotal){
  unsigned char d[digTotal];
  int i, j= 0;
  while(num > 0){
    int digit = num % 10;
    d[i] = digit + '0';
    num /= 10;
    i++;
  }
  for (int j = i - 1; j >= 0; j--) {
        U0putChar(d[j]);
    }
}