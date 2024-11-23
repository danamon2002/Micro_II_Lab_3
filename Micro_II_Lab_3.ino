/*  Dana Maloney, Thomas Moeller
    Micro II Lab 3 */

#include <arduinoFFT.h>

#define SAMPLES 256
#define NYQUIST_FREQ 2400
#define C_4 262 // C4 Note
#define A_4 440 // A4 Note

int usec_per_sample = (1.0 / NYQUIST_FREQ) * 1000000;  //Calculate usec per sample
int micPin = A0;
double samples[SAMPLES];
double imaginary[SAMPLES] = {0}; // needed for FFT but doesn't do anything.
ArduinoFFT<double> FFT = ArduinoFFT<double>(samples, imaginary, SAMPLES, NYQUIST_FREQ);


void setup(){
  Serial.begin(115200);
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

void loop(){
  int audioFrequency = (int)doFFT();
  if (fabs(audioFrequency - A_4) <= 0.02 * A_4){ //Check if A4 was played (+/- 2%)
    Serial.println("A4 DETECTED!!");
  } else if (fabs(audioFrequency - C_4) <= 0.02 * C_4){ //Check if AC was played (+/- 2%)
    Serial.println("C4 DETECTED!!");
  }
}

