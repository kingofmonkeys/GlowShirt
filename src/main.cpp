#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <NeoPatterns.h>
#include <arduinoFFT.h>
#include "hardware/pio.h"
#include "quadrature.pio.h"

#define PIN_NEO_PIXEL_FRONT 5 // Arduino pin that connects to NeoPixel
#define PIN_NEO_PIXEL_BACK 4  // Arduino pin that connects to NeoPixel

#define NUM_PIXELS_FRONT 50 // The number of LEDs (pixels) on NeoPixel
#define NUM_PIXELS_BACK 50  // The number of LEDs (pixels) on NeoPixel

#define SAMPLES 64 // Must be a power of 2
#define MIC_IN A1
#define SAMPLING_FREQUENCY 30000 // Hz, must be less than 10000 due to ADC
#define INTENSITY_BINS 5
#define CYCLE_TIME 4

// #define QUADRATURE_A_PIN 10
// #define QUADRATURE_B_PIN 11
#define CLK 10
#define DATA 11
#define BUTTON 12

int counter = 0;
int priorCounter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir = "";

unsigned int brightnessPotPin = A0;
unsigned int brightnessPotVal = 0; // Variable to store the input from the potentiometer

unsigned int senPotPin = A2;
unsigned int senPotVal = 0; // Variable to store the input from the potentiometer

double vReal[SAMPLES];
double vImag[SAMPLES];

double intensity[INTENSITY_BINS];

unsigned int sampling_period_us;
unsigned long startMicros;

// Tracks the time since last event fired
unsigned long buttonPreviousMillis = 0;
unsigned long brightnessPreviousMillis = 0;
unsigned long brightnessPoll = 500;
unsigned long senPreviousMillis = 0;
unsigned long senPoll = 500;

// This is used to change the led mode
u_int8_t ledState = 0;

arduinoFFT FFT = arduinoFFT();

void StripComplete();
void getSamples();
void getIntensities();
void setLEDS();
void processButtonPush();
void updateEncoder();

NeoPatterns frontStrip = NeoPatterns(NUM_PIXELS_FRONT, PIN_NEO_PIXEL_FRONT, NEO_GRB + NEO_KHZ800, &StripComplete);
NeoPatterns backStrip = NeoPatterns(NUM_PIXELS_BACK, PIN_NEO_PIXEL_BACK, NEO_GRB + NEO_KHZ800, &StripComplete);

void setup()
{ // put your setup code here, to run once:
  Serial.begin(115200);
    // turn up the analog resolution becuase i'm using a pico
  analogReadResolution(12);

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DATA, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(MIC_IN, INPUT);  
  // set mic pin to input (not sure if this is even needed)
  pinMode(brightnessPotPin,INPUT);
  pinMode(senPotPin,INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON), processButtonPush, LOW);
//disable encoder code since i'm not using it

 // attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(DATA), updateEncoder, CHANGE);

  // tone down the brightness
  int brightnessCurrentPotVal = analogRead(brightnessPotPin);
  // do a map
  int brightnessValue = map(brightnessCurrentPotVal, 0, 4095, 0, 255);
  // set the brightness
  frontStrip.setBrightness(brightnessValue);
  backStrip.setBrightness(brightnessValue);
  frontStrip.begin();
  backStrip.begin();
  setLEDS();
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  // tone down the brightness
  int senPotVal = analogRead(brightnessPotPin);

}

void loop()
{
  unsigned long currentMillis = millis();

  if ((unsigned long)(currentMillis - brightnessPreviousMillis) >= brightnessPoll)
  {
    int currentPotVal = analogRead(brightnessPotPin);
    // do a map
    int brightnessValue = map(currentPotVal, 0, 4095, 0, 255);
    // set the brightness
    frontStrip.setBrightness(brightnessValue);
    backStrip.setBrightness(brightnessValue);    

        brightnessPreviousMillis = currentMillis;
  }


  if ((unsigned long)(currentMillis - senPreviousMillis) >= senPoll)
  {
    senPotVal = analogRead(senPotPin);
    Serial.println(senPotVal);
    senPreviousMillis = currentMillis;
  }



  if (priorCounter != counter)
  {
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
    priorCounter = counter;
  }

  //  int sensorValue;
  //  sensorValue = analogRead(A1);
  // Serial.println(sensorValue);
  // Collect Samples
  getSamples();
  //should check the ledstate here so we call the right thing...
 if (ledState == 0)
  {
    // find intensities
    getIntensities();
  frontStrip.Update(intensity);
  backStrip.Update(intensity);
  }
  else if (ledState == 1)
  {
  frontStrip.Update();
  backStrip.Update();
  }
  
  
}

void updateEncoder()
{
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1)
  {

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DATA) != currentStateCLK)
    {
      counter--;
      currentDir = "CCW";
    }
    else
    {
      // Encoder is rotating CW so increment
      counter++;
      currentDir = "CW";
    }
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}

void processButtonPush()
{
  unsigned long currentMillis = millis();
  unsigned long debounce = 1000;

  if ((unsigned long)(currentMillis - buttonPreviousMillis) >= debounce)
  {
    // button pushed
    // set the led state/increment
    if (ledState >= 1)
    {
      ledState = 0;
    }
    else
    {
      ledState++;
    }

    setLEDS();

    // make sure to reset this time for the debounce code
    buttonPreviousMillis = currentMillis;
  }
}

void getSamples()
{
  for (int i = 0; i < SAMPLES; i++)
  {
    startMicros = micros();
    vReal[i] = analogRead(MIC_IN);
    // Serial.println(vReal[i]);
    vImag[i] = 0;
    // this ensures it doesn't read too fast.

    while ((unsigned long)(micros() - startMicros) < sampling_period_us)
    {
     // Serial.println("waiting..");
    }
  }
}

void getIntensities()
{
  // FFT
  FFT.DCRemoval(vReal, SAMPLES);
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  /*PRINT RESULTS*/
  // Serial.println("peak"); // Print out what frequency is the most dominant.
   //Serial.println(peak);   // Print out what frequency is the most dominant.

  // for (int i = 0; i < (SAMPLES / 2); i++)
  // {
  //   /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/

  //   // Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
  //   // Serial.print(" ");
  //   // Serial.println(vReal[i], 1); // View only this line in serial plotter to visualize the bins
  // }


  // gets the size of the array to use for finding intensity bins.  We remove the first two since they are bad values.
  u_int8_t arraySize = (SAMPLES / 2) - 2;
  u_int8_t feqPerBin = arraySize / INTENSITY_BINS;

  // test code here.
  // should be 30
  // Serial.print("arraySize: ");
  // Serial.println(arraySize);
  // should be 6
  // Serial.print("feqPerBin: ");
  // Serial.println(feqPerBin);

  // need to setup intensity.

  u_int8_t kin = 2;
  u_int16_t sum = 0;
  u_int8_t count = 0;
  u_int16_t total = 0;
  u_int16_t avg = 0;
  u_int16_t highest = 0;
  u_int8_t highestIndex = 0;

  for (u_int8_t ii = 0; ii < INTENSITY_BINS; ii++)
  {
    for (u_int8_t i = kin; i < kin + feqPerBin; i++)
    {
      // Serial.print("vReal[");
      // Serial.print(i);
      //  Serial.print("]: ");
      // Serial.println(vReal[i]);
      total += vReal[i];
      sum += vReal[i];
      count++;
    }
    kin = kin + feqPerBin;
    // Serial.print("kin: ");
    // Serial.println(kin);
    avg = total / feqPerBin;   

    // adjust the base down.  these offsets might be an issue.
    if (ii == 0)
    {        
      //just take 2 and 3 
      // total = 0;
      // avg = 0;
      // total += vReal[2];
      // total += vReal[3];
      // avg = total / 2;
      avg = avg*.9;
      // if (avg > 1000)
      //   {
      //     avg = avg - 1000;
      //   } else {
      //     avg = avg/2;
      //   }
    } if (ii == 1)
    { 
        avg = avg*1.4;
    }if (ii == 2)
    { 
        avg = avg*3;
    }
    
    else{
      avg = avg *3.5;
      
    }
    if(avg>4095){
        avg = 4095;
      }
    

    // Serial.print("avg: ");
    // Serial.println(avg);
    // this sets the highest value in this loop.


    if (avg > highest)
    {
      highest = avg;
      highestIndex = ii;
    }
    intensity[ii] = avg;
    // reset the average and total
    total = 0;
    avg = 0;
  }
  
  u_int8_t sumAvg = sum / count;
//Serial.println(highest);

  // for (u_int8_t ak=0; ak < INTENSITY_BINS; ak++)
  //{
  //  Serial.println(intensity[ak]);
  // }
  if (highest > senPotVal )
  {
    for (u_int8_t ak = 0; ak < INTENSITY_BINS; ak++)
    {
      intensity[ak] = intensity[ak] / highest;
    }
  }
  else
  {
    for (u_int8_t ak = 0; ak < INTENSITY_BINS; ak++)
    {
      intensity[ak] = 0;
    }
    
  }
}

// Strip Completion Callback
void StripComplete()
{
  setLEDS();
}

void setLEDS()
{
  if (ledState == 0)
  {
    frontStrip.RainbowCycleReact(CYCLE_TIME, INTENSITY_BINS);
    backStrip.RainbowCycleReact(CYCLE_TIME, INTENSITY_BINS);
  }
  else if (ledState == 1)
  {
    frontStrip.RainbowCycle(CYCLE_TIME+6);
    backStrip.RainbowCycle(CYCLE_TIME+6);
  }
}
