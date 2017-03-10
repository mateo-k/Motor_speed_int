#include <limits.h>

//Stale ADC
const long AREF = 5000;
const float ADC_MAXVAL = 1023.0;
const int ANALOG_INPUTS = 8;

const int PROCENT = 100;

//Podlaczenie pinow
const int LED = 13;



const int POT1_PIN = A0;
const int POT2_PIN = A7;
const int POT1_ADC_INPUT = 0;
const int POT2_ADC_INPUT = 7;
const int POT1_VCC_PIN = 5;
const int POT2_VCC_PIN = 4;
const int POT1_MAX_VAL = 4990;
const int POT2_MAX_VAL = 5000;

const int Imcu_PIN = A1;
const int Idrv_PIN = A6;
const int Imcu_ADC_INPUT = 1;
const int Idrv_ADC_INPUT = 6;

const int Vcc_PIN = A2;
const int Vbatt_PIN = A3;
const int Vcc_ADC_INPUT = 2;
const int Vbatt_ADC_INPUT = 3;

//Podlaczenie sterownika silnika
const int PWM_MAX_PIN = 9;
const int DIR_MAX_PIN = 8;
const int nFAULT_MAX_PIN = 7;
const int nEN_MAX_PIN = 6;

//Podlaczenie enkodera silnika
const int ENC_A_PIN = 2;
const int ENC_B_PIN = 3;

volatile bool last_enc_a;
volatile bool last_enc_b;
volatile bool fault;

volatile long last_change_tp_a;
volatile long last_change_tp_b;

volatile long last_change_a_per;
volatile long last_change_b_per;
volatile long lowest_change_a_per = LONG_MAX;
volatile long lowest_change_b_per = LONG_MAX;

volatile long position;

const long USEC_IN_SEC = 1000000;



volatile int speed;

void setup() {
  pinMode(LED,OUTPUT);
  
  //Configure pins for potentiometers
  pinMode(POT1_VCC_PIN, OUTPUT);
  digitalWrite(POT1_VCC_PIN, HIGH);
  pinMode(POT2_VCC_PIN, OUTPUT);
  digitalWrite(POT2_VCC_PIN, HIGH);

  //Configure pins for motor drv
  pinMode(PWM_MAX_PIN, OUTPUT);
  pinMode(DIR_MAX_PIN, OUTPUT);
  //pinMode(eEN_MAX_PIN, OUTPUT);

  //Configure motor encoders
  last_enc_a = digitalRead(ENC_A_PIN);
  last_enc_b = digitalRead(ENC_B_PIN);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), Enc_a_ISR, CHANGE );
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), Enc_b_ISR, CHANGE );

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  static long loc_lowest_change_a_per;
  static long loc_lowest_change_b_per;
  static long printable_lowest_change_a_per = LONG_MAX;
  static long printable_lowest_change_b_per = LONG_MAX;
  static long loc_position;
  static long loc_last_change_a_per;
  static long loc_last_change_b_per;
  
const long PROCENT = 100L;
  int pot1 = (PROCENT * AREF * analogRead(POT1_ADC_INPUT) ) / (POT1_MAX_VAL * (long)ADC_MAXVAL );
const long MAX_PWM = 255L;
  int pwm = ((pot1 - PROCENT/2) * MAX_PWM) / (PROCENT/2);
  bool dir = pwm>0 ? 1 : 0 ;
  pwm = pwm>0 ? pwm : -pwm;

  digitalWrite(DIR_MAX_PIN, dir);
  if(pwm == 255) digitalWrite(PWM_MAX_PIN, HIGH);
  else if(pwm >64) analogWrite(PWM_MAX_PIN, pwm);
  else digitalWrite(PWM_MAX_PIN, LOW);
  
  unsigned long millisec = millis();
  static unsigned long last_millis;
  if(millisec - last_millis > 250)
  {
    last_millis = millisec;

    if(fault)
      digitalWrite(LED,HIGH);
    else
      digitalWrite(LED,LOW);
    fault = false;

    noInterrupts();
    loc_lowest_change_a_per = lowest_change_a_per;
    lowest_change_a_per = LONG_MAX;
    interrupts();
  
    if(loc_lowest_change_a_per < LONG_MAX)
      printable_lowest_change_a_per = loc_lowest_change_a_per;
  
    noInterrupts();
    loc_lowest_change_b_per = lowest_change_b_per;
    lowest_change_b_per = LONG_MAX;
    interrupts();
  
    if(loc_lowest_change_b_per < LONG_MAX)
      printable_lowest_change_b_per = loc_lowest_change_b_per;
  
    noInterrupts();
    loc_position = position;
    interrupts();

    noInterrupts();
    loc_last_change_a_per = last_change_a_per;
    interrupts();
    
    noInterrupts();
    loc_last_change_b_per = last_change_b_per;
    interrupts();

    unsigned int freq_a = USEC_IN_SEC / loc_last_change_a_per;
    unsigned int freq_b = USEC_IN_SEC / loc_last_change_b_per; 

    unsigned int max_freq_a = USEC_IN_SEC / loc_lowest_change_a_per; 
    unsigned int max_freq_b = USEC_IN_SEC / loc_lowest_change_a_per; 

    Serial.print(analogRead(POT1_ADC_INPUT));
    Serial.print("\t");
    Serial.print(pot1);
    Serial.print("\t");
    Serial.print(loc_position);
    Serial.print("\t");
    Serial.print(freq_a);
    Serial.print("\t");
    Serial.print(freq_b);    
    Serial.print("\t");
    Serial.print(max_freq_a);
    Serial.print("\t");
    Serial.print(max_freq_b);    
    Serial.print("\t");
    Serial.print(printable_lowest_change_a_per);
    Serial.print("\t");
    Serial.print(printable_lowest_change_b_per);
    Serial.print("\n");
  }
}

void Enc_a_ISR()
{
  static bool enc_a;
  enc_a = digitalRead(ENC_A_PIN);
  if (enc_a and last_enc_a) fault = true; //Nic sie nie zmienilo, stoi w miejscu i swieci dioda
  else if(enc_a xor last_enc_b) position++;
  else position--;
  last_enc_a = enc_a;
  
  static long change_tp_a;
  change_tp_a = micros();
  static long change_a_per;
  change_a_per = (change_tp_a - last_change_tp_a);
  last_change_tp_a = change_tp_a;
  last_change_a_per = change_a_per;
  if( change_a_per<lowest_change_a_per ) 
    lowest_change_a_per = change_a_per;  
}

void Enc_b_ISR()
{
  static bool enc_b;
  enc_b = digitalRead(ENC_B_PIN);
  if (enc_b and last_enc_b) fault = true; //Nic sie nie zmienilo, stoi w miejscu i swieci dioda
  else if(enc_b xor last_enc_a) position--;
  else position++;
  last_enc_b = enc_b;
    
  static long change_tp_b;
  change_tp_b = micros();
  static long change_b_per;
  change_b_per = (change_tp_b - last_change_tp_b);
  last_change_tp_b = change_tp_b;
  last_change_b_per = change_b_per;
  if( change_b_per<lowest_change_b_per ) 
    lowest_change_b_per = change_b_per;
}
