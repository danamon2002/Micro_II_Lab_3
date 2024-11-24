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
int direction = 0;


void setup(){
  Serial.begin(115200);
  lcd.begin(16, 2);
  URTCLIB_WIRE.begin();
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);

  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_R, 0);
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

void speedUp(){
  /*If possible, increase the fan speed. */
  int newSpeed = currentSpeed + 1;
  if (newSpeed > 4) {currentSpeed = 4;}
  else {currentSpeed = newSpeed;}
  switch (currentSpeed) {
    case 0:
      analogWrite(MOTOR_L, 0);
      analogWrite(MOTOR_R, 0);
      break;
    case 1:
      //write max speed for a second to get motor to start moving
      digitalWrite(MOTOR_L, HIGH);
      delay(300);
      analogWrite(MOTOR_L, 190);
      analogWrite(MOTOR_R, 0);
      break;
    case 2:
      analogWrite(MOTOR_L, 212);
      analogWrite(MOTOR_R, 0);
      break;
    case 3:
      analogWrite(MOTOR_L, 234);
      analogWrite(MOTOR_R, 0);
      break;
    case 4:
      analogWrite(MOTOR_L, 255);
      analogWrite(MOTOR_R, 0);
      break;
  }
  lcd.setCursor(0, 1);
  lcd.print("Speed: ");
  lcd.print(currentSpeed);
  lcd.print("          ");
  
}

void speedDown(){
  /*If possible, decrease the fan speed. */
  int newSpeed = currentSpeed - 1;
  if (newSpeed < 0) {currentSpeed = 0;}
  else {currentSpeed = newSpeed;}
  switch (currentSpeed) {
    case 0:
      analogWrite(MOTOR_L, 0);
      analogWrite(MOTOR_R, 0);
      break;
    case 1:
      analogWrite(MOTOR_L, 190);
      analogWrite(MOTOR_R, 0);
      break;
    case 2:
      analogWrite(MOTOR_L, 212);
      analogWrite(MOTOR_R, 0);
      break;
    case 3:
      analogWrite(MOTOR_L, 234);
      analogWrite(MOTOR_R, 0);
      break;
    case 4:
      analogWrite(MOTOR_L, 255);
      analogWrite(MOTOR_R, 0);
      break;
  }
  lcd.setCursor(0, 1);
  lcd.print("Speed: ");
  lcd.print(currentSpeed);
  lcd.print("          ");
}

void loop(){

  //Update Clock
  rtc.refresh();
  lcd.setCursor(0,0);
  lcd.print(rtc.hour());
  lcd.print(":");
  lcd.print(rtc.minute());
  lcd.print(":");
  lcd.print(rtc.second());
  lcd.print("         ");

  // Do Audio Sampling -> FFT
  int audioFrequency = (int)doFFT();
  if (fabs(audioFrequency - A_4) <= 0.02 * A_4){ //Check if A4 was played (+/- 2%)
    //lcd.setCursor(0,1);
    //lcd.print("A4 DETECTED!!");
    speedUp();
  } else if (fabs(audioFrequency - C_4) <= 0.02 * C_4){ //Check if AC was played (+/- 2%)
    //lcd.setCursor(0,1);
    //lcd.print("C4 DETECTED!!");
    speedDown();
  }
}

