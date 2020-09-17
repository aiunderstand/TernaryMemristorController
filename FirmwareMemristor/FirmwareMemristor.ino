/*
  Bluetooth Memristor Robot
  created 27 May 2020
  by Steven Bos

  Firmware for Arduino Mega

  Changelog
  v0.1 - initial firmware for MVP (replication of 2019 paper Radakovits and Taherinejad)
  v0.2 - added 3 state operation (write logical 0, 1, 2) based on (Bos et al., 2020). Needs further tuning when read operation allow ternary reads
       - added pulse train for writing and created delays between read/write operations to mimic results of 2019 paper better (but instead of 0 and 1 we use 0 and 2) in prep for ternary.
  v0.3 - refactored read/write ops.
       - added 3 state read operation
  v0.4 - added inverters for both the read and negative bridge gate as well as connected every cmos bridge so that for 3 write bridges and 1 read bridge we need just 4 output pins. All defaulted to be off (0 volt) to protect the memristor when booting up. 
       - added forming routine for new memristor with button
  2do: 
      - add (parallel) read/write circuit for second memristor.
      - refactor to complete embedded c. 
      - add analog in at read/write junction and do a self test for correct voltages for each button before "arming" the memristor (so add another mosfet and leave selftest resistor).
      - add negative write1 when switching from logical 2 to logical 1 (currently two operations)
      - add controller algorithm that checks if more pulses are needed vs fixed 4 pulses
      - replace delay.h using busy-wait loops with native timer/interupt based delays. Timing with read pulse is off. Arduino ADC?
      - hook up to umemristortoolbox to perform longer experiment especially with non-volatility and confusion matrix
      - test longer pulse vs shorter pulses, lower voltage of logical 0 and 2 because we drive it really hard eg. -.4, +.4, +.8 (it is not linear)
      - how important is high impedence (pmos-nmos off to decouple) vs convencience with toggle on/off (pmosOn-nmosOff or viceversa). Spike of logical 2 due to this?
*/
#include <Servo.h>
Servo servo;

#define   F_CPU   16000000UL
#include <util/delay.h>

//PINS
const int READ_PIN      = PL7;
const int WRITE0_PIN    = PL6;
const int WRITE1_PIN    = PL5;
const int WRITE2_PIN    = PL4;

const int SERVO_PIN     = 6;  //PH3 (needs PWM pin)
const int OPAMP_PIN     = A0; //PF0 (needs Analog pin)
const int BUTTON_FORM   = 9;  //PH6 
const int BUTTON_WRITE0 = 10; //PB4
const int BUTTON_WRITE1 = 11; //PB5
const int BUTTON_WRITE2 = 12; //PB6
const int BUTTON_READ   = 13; //PB7
const int RED_LED_PIN   = 14; //PJ1
const int GREEN_LED_PIN = 15; //PJ0
const int WHITE_LED_PIN = 16; //PH1

//SETTINGS
const int readPeriod  = 300; //300 us (note that ADC requires 100us to stabilize)
const int writePeriod = 100; //100 us
const int readDelay  = 200; //200us
const int readAfterWriteDelay = 400; //400us
const double VCC_MAX = 5;
const double res = VCC_MAX / 1024;
const int debounceDelay = 333; //ms (for buttons)
const bool withLeds = true;
const bool withServo = true;

//VARIABLES
int currentState = -1; //not initialized
bool isOn;
int FormBtnPrevious;
int FormBtn;
int ReadBtnPrevious;
int ReadBtn;
int Write0BtnPrevious;
int Write0Btn;
int Write1BtnPrevious;
int Write1Btn;
int Write2BtnPrevious;
int Write2Btn;

void setup() {
  InitMemristorController();

  pinMode(BUTTON_FORM, INPUT_PULLUP); 
  pinMode(BUTTON_READ, INPUT_PULLUP); 
  pinMode(BUTTON_WRITE0, INPUT_PULLUP); 
  pinMode(BUTTON_WRITE1, INPUT_PULLUP); 
  pinMode(BUTTON_WRITE2, INPUT_PULLUP); 

  FormBtn   = digitalRead(BUTTON_FORM);
  ReadBtn   = digitalRead(BUTTON_READ);
  Write0Btn = digitalRead(BUTTON_WRITE0);
  Write1Btn = digitalRead(BUTTON_WRITE1);
  Write2Btn = digitalRead(BUTTON_WRITE2);

  if (withServo)
    servo.attach(SERVO_PIN);

  if (withLeds)
    SetLed(currentState, true);
 
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //indicate ready for use
  Serial.println("Ready");
}

void InitMemristorController() {
   DDRL = 0xFF; //SET PORT L TO OUTPUT
   PORTL = 0x00; //SET ALL BITS OFF
}
  
void FormNewMemristor() {
  int maxRepeat= 20;
 
  for (int i = 0; i <= maxRepeat; i++) {
   digitalWrite(WRITE2_PIN, HIGH);
   delay(5); //ms
   digitalWrite(WRITE2_PIN, LOW);
   delay(5); //ms
    
   digitalWrite(WRITE0_PIN, HIGH);
   delay(5); //ms
   digitalWrite(WRITE0_PIN, LOW);
   delay(5); //ms
  }
}

void SetServo(int state){
  switch (state)
  {
    case 0:
      servo.write(90); //degrees
    break;
    case 1:
      servo.write(45); //degrees
    break;
    case 2:
      servo.write(0); //degrees
    break;
    default: 
    break;
  }
}

void SetLed(int led, bool isOn){
  switch(led)
  {
    case -1:
    {
      digitalWrite(RED_LED_PIN, isOn); 
      digitalWrite(GREEN_LED_PIN, isOn); 
      digitalWrite(WHITE_LED_PIN, isOn); 
    }
    break;
    case 0:
    {
      digitalWrite(RED_LED_PIN, isOn); 
    }
    break;
    case 1:
    {
      digitalWrite(GREEN_LED_PIN, isOn); 
    }
    break;
    case 2:
    {
      digitalWrite(WHITE_LED_PIN, isOn); 
    }
    break;
    default:
    break;
  }  
}

void loop() {
  ScanButtons();
}

void ScanButtons() {
 
  FormBtnPrevious = FormBtn;
  FormBtn = digitalRead(BUTTON_FORM);
  
  ReadBtnPrevious = ReadBtn;
  ReadBtn = digitalRead(BUTTON_READ);
  
  Write0BtnPrevious = Write0Btn;
  Write0Btn = digitalRead(BUTTON_WRITE0);
  
  Write1BtnPrevious = Write1Btn;
  Write1Btn = digitalRead(BUTTON_WRITE1);
  
  Write2BtnPrevious = Write2Btn;
  Write2Btn = digitalRead(BUTTON_WRITE2);

  if (FormBtnPrevious == HIGH && FormBtn == LOW) {
    delay(debounceDelay);

    //TIP: wire scope 3 and 4 to cmos bridge logical 0 and 2 for triggers. Leave scope 1 and 2 to compute current 
    Serial.println("Now forming the new memristor");
    FormNewMemristor();

    //fake 3 button presses to see result of forming. 
    //NOTE: scope 3 (op amp output) and scope 4 (cmos bridge read) might be used for other signals (see tip).
    Write0Btn = LOW;
    Write1Btn = LOW;
    Write2Btn = LOW;
    }
    
  if (ReadBtnPrevious == HIGH && ReadBtn == LOW) {
    delay(debounceDelay);

    Read();
    }

  if (Write0BtnPrevious == HIGH && Write0Btn == LOW) {
    delay(debounceDelay);
  
    Serial.println("Write 0 (4 pulses 0)");
    WriteMemristor(0);
    WriteMemristor(0);
    WriteMemristor(0);
    WriteMemristor(0);
    currentState = 0;
    
    delayMicroseconds(readAfterWriteDelay); //replace with timer
    Read();
    Read();
    Read();
  }

  if (Write1BtnPrevious == HIGH && Write1Btn == LOW) {
    delay(debounceDelay);
    
    Serial.println("Write 1 (write 4 pulses 0, then 4 pulses 1");
    WriteMemristor(0);
    WriteMemristor(0);
    WriteMemristor(0);
    WriteMemristor(0);
    
    WriteMemristor(1);
    WriteMemristor(1);
    WriteMemristor(1);
    WriteMemristor(1);
    currentState = 1;
  
    delayMicroseconds(readAfterWriteDelay); //replace with timer
    Read();
    Read();
    Read();
  }

  if (Write2BtnPrevious == HIGH && Write2Btn == LOW) {
    delay(debounceDelay);
    
    Serial.println("Write 2 (write 4 pulses 2)");
    WriteMemristor(2);
    WriteMemristor(2);
    WriteMemristor(2);
    WriteMemristor(2);
    currentState = 2;
 
    delayMicroseconds(readAfterWriteDelay); //replace with timer
    Read();
    Read();
    Read();
  }
}

void Read()
{
  ReadPulse(true);
}

void ReadPulse(bool computeOutput)
{
  
  PORTL |= _BV(READ_PIN);   // HIGH, so set bit
   _delay_us(readPeriod); //replace with timer

  PORTL &= ~_BV(READ_PIN);   // LOW, so clear bit
   _delay_us(readDelay); //replace with timer

  if (computeOutput)
  {
    double val = analogRead(OPAMP_PIN);
    double voltage = val  * res;

    int state = -1;
    if (voltage < 0.8) //actually op amp threshold is 1.4
    {
      state = 0;
    }
    else if (voltage < 3)
    {
      state = 1;
    }
    else  //voltage > 3v, op amp high is 5v 
    {
      state = 2;
    }

    if (state == currentState)
      Serial.print("[CORRECT] ");
    else
      Serial.print("[FAIL] ");

    Serial.print("Read logical: ");
    Serial.print(state);
    Serial.print(" ( ");
    Serial.print(voltage, 5); //5 decimals precise
    Serial.print(" V.) Groundtruth = ");
    Serial.println(currentState);

    if (withLeds)
    {
      SetLed(-1, false);
      SetLed(state,true);
    }

    if (withServo)
      SetServo(state);
  }
}

void WriteMemristor(int state) {
  switch (state)
  {
    case 0:  //write 0 (erase)
      {
        PORTL |= _BV(WRITE0_PIN);   // HIGH, so set bit
        _delay_us(writePeriod);
        PORTL &= ~_BV(WRITE0_PIN);   // LOW, so clear bit
        _delay_us(writePeriod);
      }
      break;
    case 1:  //write 1
      {
        //2do: make a switch-case here: depending on previous state
        // from state 2 : apply negative pulse
        // from state 1  : apply positive pulse
        // from state -1 (uninitialized) : first apply erase, the positive

        PORTL |= _BV(WRITE1_PIN);   // HIGH, so set bit
        _delay_us(writePeriod);
        PORTL &= ~_BV(WRITE1_PIN);   // LOW, so clear bit
        _delay_us(writePeriod);
      }
      break;
    case 2: //positive voltage bridge for logical 2 (off is LOW)
      {
        PORTL |= _BV(WRITE2_PIN);   // HIGH, so set bit
        _delay_us(writePeriod);
        PORTL &= ~_BV(WRITE2_PIN);   // LOW, so clear bit
        _delay_us(writePeriod);
      }
      break;
    default:
      break;
  }
}
