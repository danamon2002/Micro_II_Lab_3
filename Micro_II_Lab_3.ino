/*  Dana Maloney, Thomas Moeller
    Micro II Lab 3 */

#include <arduinoFFT.h>

#define SAMPLES 256
#define NYQUIST_FREQ 2400

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
  double audioFrequency = doFFT();
  Serial.print("Dominant Frequency (Hz) is ");
  Serial.println(doFFT());
}

