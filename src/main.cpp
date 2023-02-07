#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <NeoPatterns.h>
#include <arduinoFFT.h>
#include "hardware/pio.h"
#include "quadrature.pio.h"

#define QUADRATURE_A_PIN 10
#define QUADRATURE_B_PIN 11

#define PIN_NEO_PIXEL 5 // Arduino pin that connects to NeoPixel
#define Clock 9         // Clock pin connected to D9
#define Data 8          // Data pin connected to D8
#define BUTTON 12

#define NUM_PIXELS 50 // The number of LEDs (pixels) on NeoPixel
#define SAMPLES 64    // Must be a power of 2
#define MIC_IN A1
#define SAMPLING_FREQUENCY 4000 // Hz, must be less than 10000 due to ADC
#define INTENSITY_BINS 5
#define CYCLE_TIME 4


PIO pio = pio0;
uint offset, sm;

double vReal[SAMPLES];
double vImag[SAMPLES];

double intensity[INTENSITY_BINS];

unsigned int sampling_period_us;
unsigned long startMicros;

// Tracks the time since last event fired
unsigned long buttonPreviousMillis = 0;

u_int8_t ledState = 0;

arduinoFFT FFT = arduinoFFT();

void StripComplete();
void getSamples();
void getIntensities();
void setLEDS();
void processButtonPush();

NeoPatterns strip = NeoPatterns(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800, &StripComplete);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  pinMode(BUTTON, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON), processButtonPush, LOW);

  // tone down the brightness
  strip.setBrightness(40);
  strip.begin();
  //strip.RainbowCycleReact(CYCLE_TIME, INTENSITY_BINS);
  setLEDS();
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  // set mic pin to input (not sure if this is even needed)
  pinMode(MIC_IN, INPUT);
  // turn up the analog resolution becuase i'm using a pico
  analogReadResolution(12);  

  offset = pio_add_program(pio, &quadrature_program);
  sm = pio_claim_unused_sm(pio, true);
  quadrature_program_init(pio, sm, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);
}

void loop()
{ 
  // put your main code here, to run repeatedly:
  //pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
  //uint encoder_value = pio_sm_get_blocking(pio, sm);
  //Serial.println(encoder_value);

  Serial.println(offset);
  delay(1000);



  // strip.Update();
  //  int sensorValue;
  //  sensorValue = analogRead(A1);
  // Serial.println(sensorValue);
  // Collect Samples
  //getSamples();
  // find intensities
  //getIntensities();
  //strip.Update(intensity);
  //setLEDS();

  
  //Serial.println(ledState);
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
  // Serial.println(peak);   // Print out what frequency is the most dominant.

  for (int i = 0; i < (SAMPLES / 2); i++)
  {
    /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/

    // Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
    // Serial.print(" ");
    // Serial.println(vReal[i], 1); // View only this line in serial plotter to visualize the bins
  }

  /* // TEST data
  vReal[0] = 500;
  vReal[1] = 500;
  vReal[2] = 10;
  vReal[3] = 10;
  vReal[4] = 10;
  vReal[5] = 10;
  vReal[6] = 10;
  vReal[7] = 10;
  vReal[8] = 20;
  vReal[9] = 20;
  vReal[10] = 20;
  vReal[11] = 20;
  vReal[12] = 20;
  vReal[13] = 20;
  vReal[14] = 30;
  vReal[15] = 30;
  vReal[16] = 30;
  vReal[17] = 30;
  vReal[18] = 30;
  vReal[19] = 30;
  vReal[20] = 40;
  vReal[21] = 40;
  vReal[22] = 40;
  vReal[23] = 40;
  vReal[24] = 40;
  vReal[25] = 40;
  vReal[26] = 50;
  vReal[27] = 50;
  vReal[28] = 50;
  vReal[29] = 50;
  vReal[30] = 50;
  vReal[31] = 50; */

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
  u_int16_t total = 0;
  u_int16_t avg = 0;
  u_int16_t highest = 0;

  for (u_int8_t ii = 0; ii < INTENSITY_BINS; ii++)
  {
    for (u_int8_t i = kin; i < kin + feqPerBin; i++)
    {
      // Serial.print("vReal[");
      // Serial.print(i);
      //  Serial.print("]: ");
      // Serial.println(vReal[i]);
      total += vReal[i];
    }
    kin = kin + feqPerBin;
    // Serial.print("kin: ");
    // Serial.println(kin);
    avg = total / feqPerBin;
    // adjust the base down.  these offsets might be an issue.
    if (ii == 0)
    {
      if (avg > 300)
      {
        avg = avg - 300;
      }
    }
    if (ii == 1)
    {
      if (avg > 100)
      {
        avg = avg - 100;
      }
    }
    if (ii == 3)
    {
      avg = avg + 200;
    }
    if (ii == 4)
    {
      avg = avg + 300;
    }
    // Serial.print("avg: ");
    // Serial.println(avg);
    // this sets the highest value in this loop.
    if (avg > highest)
    {
      highest = avg;
    }
    intensity[ii] = avg;
    // reset the average and total
    total = 0;
    avg = 0;
  }

  // for (u_int8_t ak=0; ak < INTENSITY_BINS; ak++)
  //{
  //  Serial.println(intensity[ak]);
  // }
  // this 800 needs to be in a sensitivy control.
  if (highest < 700)
  {
    for (u_int8_t ak = 0; ak < INTENSITY_BINS; ak++)
    {
      intensity[ak] = 0;
    }
  }
  else
  {
    for (u_int8_t ak = 0; ak < INTENSITY_BINS; ak++)
    {
      intensity[ak] = intensity[ak] / highest;
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
    strip.RainbowCycleReact(CYCLE_TIME, INTENSITY_BINS);
  }
  else if (ledState == 1)
  {
    strip.RainbowCycle(CYCLE_TIME);
  }
}

// void setLEDS()
// {

//   u_int8_t totLEDS = 50;
//   u_int8_t bars = 5;
//   u_int8_t perBar = totLEDS / bars;

//   u_int8_t cIndex = 0;

//   for (u_int8_t ak = 0; ak < INTENSITY_BINS; ak++)
//   {
//     //Serial.println(intensity[ak]);
//     u_int8_t toLight = intensity[ak] * perBar;

//     //Serial.print("tolight: ");
//     //Serial.println(toLight);

//     for (u_int8_t i = cIndex; i < cIndex + perBar; i++)
//     {
//       // this should light up to the right level.
//       if (i < cIndex + toLight)
//       {
//         if (i < 10)
//         {
//           strip.setPixelColor(i, strip.Color(255, 0, 0));
//         }
//         else if (i < 20)
//         {
//           strip.setPixelColor(i, strip.Color(255, 0, 255));
//         }
//         else if (i < 30)
//         {
//           strip.setPixelColor(i, strip.Color(0, 0, 255));
//         }
//         else if (i < 40)
//         {
//           strip.setPixelColor(i, strip.Color(255, 255, 0));
//         }
//         else if (i < 50)
//         {
//           strip.setPixelColor(i, strip.Color(0, 255, 0));
//         }
//       }
//       else
//       {
//         if (i < 10)
//         {
//           strip.setPixelColor(i, strip.Color(20, 0, 0));
//         }
//         else if (i < 20)
//         {
//           strip.setPixelColor(i, strip.Color(20, 0, 20));
//         }
//         else if (i < 30)
//         {
//           strip.setPixelColor(i, strip.Color(0, 0, 20));
//         }
//         else if (i < 40)
//         {
//           strip.setPixelColor(i, strip.Color(20, 20, 0));
//         }
//         else if (i < 50)
//         {
//           strip.setPixelColor(i, strip.Color(0, 20, 0));
//         }
//       }
//     }

//     cIndex = cIndex + perBar;
//   }

//   strip.show();
// }