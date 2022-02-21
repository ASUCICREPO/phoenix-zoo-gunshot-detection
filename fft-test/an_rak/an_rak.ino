
#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include "arduinoFFT.h"

struct HEADER {
  unsigned char riff[4];            // RIFF string
  unsigned int overall_size ;       // overall size of file in bytes
  unsigned char wave[4];            // WAVE string
  unsigned char fmt_chunk_marker[4];      // fmt string with trailing null char
  unsigned int length_of_fmt;         // length of the format data
  unsigned int format_type;         // format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law
  unsigned int channels;            // no.of channels
  unsigned int sample_rate;         // sampling rate (blocks per second)
  unsigned int byterate;            // SampleRate * NumChannels * BitsPerSample/8
  unsigned int block_align;         // NumChannels * BitsPerSample/8
  unsigned int bits_per_sample;       // bits per sample, 8- 8bits, 16- 16 bits etc
  unsigned char data_chunk_header [4];    // DATA string or FLLR string
  unsigned int data_size;           // NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read
};

unsigned char buffer4[4];
unsigned char buffer2[2];

struct HEADER header;

const uint16_t samples = 1024;
double vReal[samples];
double vImag[samples];

int myBins[20] = {
  23, 69, 115, 184, 276, 368, 460, 621, 851, 1196, 1656, 2116,
                  3496, 5796, 8096, 10396, 12696, 14996, 17296, 21223
                 };

double level[20];
double lastLevel[20];
double diffLevel[20];
double diffLevelSum;
double prevDiffLevelSum;

double SpectralCentroid;
double SpectralCentroidPrev;
double den;
double num;

int specDiff[3];

int count=0;

long st;

void detectShot(){
//   long millisecs=millis();
   arduinoFFT FFT = arduinoFFT();
   FFT.Windowing(vReal, samples, FFT_WIN_TYP_HANN, FFT_FORWARD);
   FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
   FFT.ComplexToMagnitude(vReal, vImag, samples);

   for( int z=0; z<20; z++ ){
    level[z]=0;
  }
  // i < (samples >> 1)
  for (int i = 0; i<512 ; i++) {
    vReal[i] = vReal[i] / samples;
    if (i == 0)
      level[0] = vReal[i];
    if (i == 1)
      level[1] = vReal[i];
    if (i == 2)
      level[2] = vReal[i];
    if ( i >= 3 && i <= 4)
      level[3] += vReal[i];
    if ( i >= 5 && i <= 6)
      level[4] += vReal[i];
    if ( i >= 7 && i <= 8)
      level[5] += vReal[i];
    if ( i >= 9 && i <= 10)
      level[6] += vReal[i];
    if ( i >= 11 && i <= 15)
      level[7] += vReal[i];
    if ( i >= 16 && i <= 20)
      level[8] += vReal[i];
    if ( i >= 21 && i <= 30)
      level[9] += vReal[i];
    if ( i >= 31 && i <= 40)
      level[10] += vReal[i];
    if ( i >= 41 && i <= 50)
      level[11] += vReal[i];
    if ( i >= 51 && i <= 100)
      level[12] += vReal[i];
    if ( i >= 101 && i <= 150)
      level[13] += vReal[i];
    if ( i >= 151 && i <= 200)
      level[14] += vReal[i];
    if ( i >= 201 && i <= 250)
      level[15] += vReal[i];
    if ( i >= 251 && i <= 300)
      level[16] += vReal[i];
    if ( i >= 301 && i <= 350)
      level[17] += vReal[i];
    if ( i >= 351 && i <= 400)
      level[18] += vReal[i];
    if ( i >= 401 && i <= 511)
      level[19] += vReal[i];
  }

  SpectralCentroidPrev = SpectralCentroid;
  num=0;
  den=0;
  
  for (int i = 0; i < 20; i++) {
    num += (level[i]) * myBins[i];
    den += (level[i]);
    
  }
  
  if(den==0) den=1;

  SpectralCentroid = (num / den);

  prevDiffLevelSum=diffLevelSum;
  diffLevelSum=0;
  
  for (int i = 0; i < 3; i++) {
    diffLevel[i] = (level[i] - lastLevel[i]);
    diffLevelSum += (diffLevel[i]);
    lastLevel[i] = level[i];
  }

  long amp = diffLevelSum-prevDiffLevelSum;
  long cen = SpectralCentroid;

  specDiff[0]=specDiff[1];
  specDiff[1]=specDiff[2];
  specDiff[2]=SpectralCentroid-SpectralCentroidPrev;

  count++;

//  Serial.print(amp);
//  Serial.print(" ");
  Serial.println(cen);
//  Serial.println(specDiff[2]);

//  if(abs(amp)>7000 && (abs(specDiff[0])>80 || abs(specDiff[1])>80 || abs(specDiff[2])>80) ){
//    Serial.print(count);
//    Serial.print(" ");
//    Serial.println("Gunshot Detected");
//  }

//  long mils=millis()-millisecs;
//  Serial.println(mils);

//  delay(10000);

}


void setup() {
  Serial.begin(115200);
}

void loop() {
//  
//  long mill=millis();
//
  double digVol;
  for (int i = 0; i < 1024; i++){
//      analogReadResolution(10);
      digVol=analogRead(A0);

      vReal[i] = (digVol * (5.0/1023.0));
//      Serial.println(String(vReal[i]*10000) + " 2400000");
//      Serial.println(vReal[i]);
//      Serial.print(" ");
//      Serial.println(digVol);
//      if(vReal[i]*10000>2050000){
//        delay(1000);
//      }
      
      vImag[i]=0;
  }
//  delay(2000);

  detectShot();
//
//  long endt=millis()-mill;
//  Serial.println("test ");
//  Serial.println(endt);
  
  
//  int i=0;
//  long mill=millis();
//  long temp=0;
//  double sensorValue;
//  while(millis()-mill<1000){
//      sensorValue = analogRead(A0);
//    
////      Serial.println(sensorValue);
//      if(sensorValue>210.0){
//        delay(10000);
//        
//      }
////      while(temp--){
////        long tesensorValue = analogRead(A0);
////        Serial.println(tesensorValue);
////        if(temp<=10){
////          delay(10000);
////        }
////      }
//      i++;
//    }
//    Serial.println(i);

//  delay(10000);
  

}
