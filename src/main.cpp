#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <NeoPatterns.h>
#include <arduinoFFT.h>

#define PIN_NEO_PIXEL  5   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     5  // The number of LEDs (pixels) on NeoPixel
#define SAMPLES 64        // Must be a power of 2
#define MIC_IN A1
#define SAMPLING_FREQUENCY 4000 //Hz, must be less than 10000 due to ADC
 

double vReal[SAMPLES];
double vImag[SAMPLES];

unsigned int sampling_period_us;
unsigned long microseconds;

arduinoFFT FFT = arduinoFFT(); 


void StripComplete();
void getSamples();

NeoPatterns strip = NeoPatterns(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800, &StripComplete);

uint32_t ledState = 8;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
   strip.begin();
  // put your setup code here, to run once:
  //for (uint32_t i = 0; i < NUM_PIXELS; i++) {
  //  strip.setPixelColor(i, strip.Color(128, 255, 128));
 // }
 //strip.RainbowCycle(2);
  strip.show();
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
   pinMode(MIC_IN, INPUT);
analogReadResolution(12);
 
}

void loop() {
  // put your main code here, to run repeatedly:
//strip.Update();
// int sensorValue; 
  // sensorValue = analogRead(A1);
   //Serial.println(sensorValue);
   
//Collect Samples
  getSamples();
  
  
}


void getSamples(){
  for(int i = 0; i < SAMPLES; i++){
    microseconds = micros();    //Overflows after around 70 minutes!
    vReal[i] = analogRead(MIC_IN);
    //Serial.println(vReal[i]);
    vImag[i] = 0;
    //this ensures it doesn't read too fast. 
    while(micros() < (microseconds + sampling_period_us)){
        }
  }

  //FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
 
    /*PRINT RESULTS*/
    Serial.println("peak");     //Print out what frequency is the most dominant.
    Serial.println(peak);     //Print out what frequency is the most dominant.
 
    for(int i=0; i<(SAMPLES/2); i++)
    {
        /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
         
        Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
        Serial.print(" ");
        Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
    }
      u_int16_t total=0;
      u_int16_t avg=0;
      for(int i=2; i<8; i++){
        total += vReal[i];
      }
      avg = total/6;
      if(avg>1200){
        strip.setPixelColor(0, strip.Color(255, 0, 0));
      }else{
        strip.setPixelColor(0, strip.Color(0, 0, 0));      
      }

      total=0;
      avg=0;
      for(int i=8; i<14; i++){
        total += vReal[i];
      }
      avg = total/6;
      if(avg>600){
        strip.setPixelColor(1, strip.Color(255, 0, 255));
      }else{
        strip.setPixelColor(1, strip.Color(0, 0, 0));      
      }

      total=0;
      avg=0;
      for(int i=14; i<20; i++){
        total += vReal[i];
      }
      avg = total/6;
      if(avg>600){
        strip.setPixelColor(2, strip.Color(0, 0, 255));
      }else{
        strip.setPixelColor(2, strip.Color(0, 0, 0));      
      }

      total=0;
      avg=0;
      for(int i=20; i<26; i++){
        total += vReal[i];
      }
      avg = total/6;
      if(avg>600){
        strip.setPixelColor(3, strip.Color(255, 255, 0)); 
      }else{
        strip.setPixelColor(3, strip.Color(0, 0, 0));      
      }
      total=0;
      avg=0;
      for(int i=26; i<32; i++){
        total += vReal[i];
      }
      avg = total/6;
      if(avg>600){
         strip.setPixelColor(4, strip.Color(0, 255, 0)); 
      }else{
        strip.setPixelColor(4, strip.Color(0, 0, 0));      
      }
      
      strip.show();

  //Update Intensity Array
  //for(int i = 2; i < (xres*Displacement)+2; i+=Displacement){
 //   vReal[i] = constrain(vReal[i],0 ,2047);            // set max value for input data
 //   vReal[i] = map(vReal[i], 0, 2047, 0, yres);        // map data to fit our display

 //   Intensity[(i/Displacement)-2] --;                      // Decrease displayed value
 //   if (vReal[i] > Intensity[(i/Displacement)-2])          // Match displayed value to measured value
 //     Intensity[(i/Displacement)-2] = vReal[i];
 // }
}


// Strip Completion Callback
void StripComplete()
{
  //this starts the rainbow cycle over if the ledState is set to rainbow cycle
  if(ledState==8){
      strip.RainbowCycle(10);
    }  
}
