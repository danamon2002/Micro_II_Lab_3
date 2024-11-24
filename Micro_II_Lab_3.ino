/*  Dana Maloney, Thomas Moeller
    Micro II Lab 3 */

#include <arduinoFFT.h>
#include <LiquidCrystal.h>
#include <uRTCLib.h>

#define SAMPLES 256
#define NYQUIST_FREQ 2400
#define C_4 262 // C4 Note
#define A_4 440 // A4 Note
#define MOTOR_L 2 // motor control L
#define MOTOR_R 3 // motor control R
#define BUTTON 4  // button to control motor direction.

//Initialize data memory and constants
//FFT Vars
int usec_per_sample = (1.0 / NYQUIST_FREQ) * 1000000;  //Calculate usec per sample
int micPin = A0;
double samples[SAMPLES];
double imaginary[SAMPLES] = {0}; // needed for FFT but doesn't do anything.
ArduinoFFT<double> FFT = ArduinoFFT<double>(samples, imaginary, SAMPLES, NYQUIST_FREQ);
//Clock Vars
uRTCLib rtc(0x68);
int currentSecond = 0;
//LCD Vars
LiquidCrystal lcd(53, 52, 51, 50, 49, 48);
//Motor Vars
int currentSpeed = 0;
enum direction {
  LEFT = 2,
  RIGHT
};
direction motorDirection = LEFT;
int userControlled = 0;


void setup(){
  Serial.begin(115200);
  lcd.begin(16, 2);
  URTCLIB_WIRE.begin();
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  //Motor off.
  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_R, 0);
  userControlled = 0;

  cli(); // clr interrupts

  //config timer interrupt for 1Hz updates.
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  //allow interrupts
  sei();
}

ISR(TIMER1_COMPA_vect){ //timer1 interrupt: Update Display.
  //Update LCD once every second
  //Clock:
  lcd.setCursor(0,0);
  lcd.print(rtc.hour());
  lcd.print(":");
  lcd.print(rtc.minute());
  lcd.print(":");
  lcd.print(rtc.second());
  lcd.print("         ");
  //Motor Speed/Direction:
  lcd.setCursor(0,1);
  lcd.print("Fan: ");
  switch(motorDirection){
    case LEFT:
      lcd.print("CC ");
      break;
    case RIGHT:
      lcd.print("C  ");
      break;
  }
  lcd.print("Spd: ");
  lcd.print(currentSpeed);
  lcd.print("  ");
}

double doFFT(){
  /* Sample audio and return dominant frequency. */

  //Sampling code:
  unsigned long startTime = micros();

  for(int i = 0; i < SAMPLES; i++){
    while(micros() - startTime < usec_per_sample){
      /* do nothing */
    }
    startTime += usec_per_sample;
    samples[i] = analogRead(micPin);
    imaginary[i] = 0;
  }

  //Level out data
  long avg = 0;
  for(int i = 0; i < SAMPLES; i++){
    avg += samples[i];
  }
  avg = avg / SAMPLES;
  for(int i = 0; i < SAMPLES; i++){
    samples[i] -= avg;
    //Serial.println(samples[i]);
  }

  //Process FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  //Find peak frequency and print peak
  double domFreq = FFT.majorPeak();
  return domFreq;
}

void runMotor(){
  switch (currentSpeed) {
      analogWrite(LEFT, 0);
      analogWrite(RIGHT, 0);
    case 0:
      analogWrite(motorDirection, 0);
      break;
    case 1:
      //write max speed for a second to get motor to start moving
      digitalWrite(motorDirection, HIGH);
      delay(300);
      analogWrite(motorDirection, 190);
      break;
    case 2:
      analogWrite(motorDirection, 212);
      break;
    case 3:
      analogWrite(motorDirection, 234);
      break;
    case 4:
      analogWrite(motorDirection, 255);
      break;
  }
  delay(300);
}

void speedUp(){
  /*If possible, increase the fan speed. */
  int newSpeed = currentSpeed + 1;
  if (newSpeed > 4) {currentSpeed = 4;}
  else {currentSpeed = newSpeed;}
  runMotor();
  // lcd.setCursor(0, 1);
  // lcd.print("Speed: ");
  // lcd.print(currentSpeed);
  // lcd.print("          ");
  
}

void speedDown(){
  /*If possible, decrease the fan speed. */
  int newSpeed = currentSpeed - 1;
  if (newSpeed < 0) {currentSpeed = 0;}
  else {currentSpeed = newSpeed;}
  runMotor();
  // lcd.setCursor(0, 1);
  // lcd.print("Speed: ");
  // lcd.print(currentSpeed);
  // lcd.print("          ");
}

void loop(){

  //Update Clock
  rtc.refresh();

  //Press button to change direction.
  if(digitalRead(BUTTON) == LOW){
    switch (motorDirection){
      case LEFT:
        motorDirection = RIGHT;
        break;
      case RIGHT:
        motorDirection = LEFT;
        break;
    }
    Serial.println(motorDirection);
    analogWrite(LEFT, 0);
    analogWrite(RIGHT, 0);
    delay(50);
    runMotor();
  }


  //TODO: implement clock fan enable/disable.
  if(rtc.second() == 0 and !userControlled){
    for(int i = 0; i < 4; i++){
      speedUp();
    }
  }else if (rtc.second() == 30 and !userControlled){
    for(int i = 0; i < 4; i++){
      speedDown();
    }
  }

  // Do Audio Sampling -> FFT
  int audioFrequency = (int)doFFT();
  if (fabs(audioFrequency - A_4) <= 0.02 * A_4){ //Check if A4 was played (+/- 2%);
    speedUp();
    userControlled = 1;
  } else if (fabs(audioFrequency - C_4) <= 0.02 * C_4){ //Check if AC was played (+/- 2%)
    speedDown();
    userControlled = 1;
  }
}

