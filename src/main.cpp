#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <NeoPatterns.h>
#include <arduinoFFT.h>

#define PIN_NEO_PIXEL 5 // Arduino pin that connects to NeoPixel
#define NUM_PIXELS 50   // The number of LEDs (pixels) on NeoPixel
#define SAMPLES 64      // Must be a power of 2
#define MIC_IN A1
#define SAMPLING_FREQUENCY 4000 // Hz, must be less than 10000 due to ADC
#define INTENSITY_BINS 5

double vReal[SAMPLES];
double vImag[SAMPLES];

double intensity[INTENSITY_BINS];

unsigned int sampling_period_us;
unsigned long startMicros;

arduinoFFT FFT = arduinoFFT();

void StripComplete();
void getSamples();
void getIntensities();
void setLEDS();

NeoPatterns strip = NeoPatterns(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800, &StripComplete);

uint32_t ledState = 8;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  strip.setBrightness(70);
  strip.begin();
  // put your setup code here, to run once:
  // for (uint32_t i = 0; i < NUM_PIXELS; i++) {
  //  strip.setPixelColor(i, strip.Color(128, 255, 128));
  // }
  // strip.RainbowCycle(2);
  strip.show();
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  // set mic pin to input (not sure if this is even needed)
  pinMode(MIC_IN, INPUT);
  // turn up the analog resolution becuase i'm using a pico
  analogReadResolution(12);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // strip.Update();
  //  int sensorValue;
  //  sensorValue = analogRead(A1);
  // Serial.println(sensorValue);

  // Collect Samples
  getSamples();
  // find intensities
  getIntensities();

  setLEDS();
  //delay(10000);
 
}

void setLEDS()
{

  u_int8_t totLEDS = 50;
  u_int8_t bars = 5;
  u_int8_t perBar = totLEDS / bars;

  u_int8_t cIndex = 0;

  for (u_int8_t ak=0; ak < INTENSITY_BINS; ak++)
  {
    Serial.println(intensity[ak]);
    u_int8_t toLight = intensity[ak] * perBar;
    boolean invert = false;

    Serial.print("tolight: ");
    Serial.println(toLight);

    for (u_int8_t i = cIndex; i < cIndex + perBar; i++)
    {
      // this should light up to the right level.
      if (i < cIndex + toLight)
      {
        if (i < 10)
        {
          strip.setPixelColor(i, strip.Color(255, 0, 0));
        }
        else if (i < 20)
        {
          strip.setPixelColor(i, strip.Color(255, 0, 255));
        }
        else if (i < 30)
        {
          strip.setPixelColor(i, strip.Color(0, 0, 255));
        }
        else if (i < 40)
        {
          strip.setPixelColor(i, strip.Color(255, 255, 0));
        }
        else if (i < 50)
        {
          strip.setPixelColor(i, strip.Color(0, 255, 0));
        }
      }
      else
      {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
    }

    cIndex = cIndex + perBar;
  }
  
  strip.show();

  // // cut out the
  // for (int i = 2; i < 8; i++)
  // {
  //   total += vReal[i];
  // }
  // lowAvg = total / 6;
  // if (lowAvg > 1200)
  // {
  //   strip.setPixelColor(0, strip.Color(255, 0, 0));
  // }
  // else
  // {
  //   strip.setPixelColor(0, strip.Color(0, 0, 0));
  // }

  // total = 0;
  // avg = 0;
  // for (int i = 8; i < 14; i++)
  // {
  //   total += vReal[i];
  // }
  // avg = total / 6;
  // if (avg > 600)
  // {
  //   strip.setPixelColor(1, strip.Color(255, 0, 255));
  // }
  // else
  // {
  //   strip.setPixelColor(1, strip.Color(0, 0, 0));
  // }

  // total = 0;
  // avg = 0;
  // for (int i = 14; i < 20; i++)
  // {
  //   total += vReal[i];
  // }
  // avg = total / 6;
  // if (avg > 600)
  // {
  //   strip.setPixelColor(2, strip.Color(0, 0, 255));
  // }
  // else
  // {
  //   strip.setPixelColor(2, strip.Color(0, 0, 0));
  // }

  // total = 0;
  // avg = 0;
  // for (int i = 20; i < 26; i++)
  // {
  //   total += vReal[i];
  // }
  // avg = total / 6;
  // if (avg > 600)
  // {
  //   strip.setPixelColor(3, strip.Color(255, 255, 0));
  // }
  // else
  // {
  //   strip.setPixelColor(3, strip.Color(0, 0, 0));
  // }
  // total = 0;
  // avg = 0;
  // for (int i = 26; i < 32; i++)
  // {
  //   total += vReal[i];
  // }
  // avg = total / 6;
  // if (avg > 600)
  // {
  //   strip.setPixelColor(4, strip.Color(0, 255, 0));
  // }
  // else
  // {
  //   strip.setPixelColor(4, strip.Color(0, 0, 0));
  // }

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
    // while(micros() < (microseconds + sampling_period_us)){
    //    }
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
  Serial.println("peak"); // Print out what frequency is the most dominant.
  Serial.println(peak);   // Print out what frequency is the most dominant.

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
  //Serial.print("arraySize: ");
  //Serial.println(arraySize);
  // should be 6
  //Serial.print("feqPerBin: ");
  //Serial.println(feqPerBin);

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
    if(ii==0){
      if(avg>300){
        avg=avg-300;
      }
    }
    if(ii==1){
      if(avg>100){
        avg=avg-100;
      }
    }
    if(ii==3){
        avg=avg+200;
    }
    if(ii==4){
        avg=avg+300;
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

  //for (u_int8_t ak=0; ak < INTENSITY_BINS; ak++)
  //{
  // Serial.println(intensity[ak]);
 // }
  //this 800 needs to be in a sensitivy control.
if(highest<700){
 for (u_int8_t ak=0; ak < INTENSITY_BINS; ak++)
  {
    intensity[ak] = 0;
  }
  
}else{
  for (u_int8_t ak=0; ak < INTENSITY_BINS; ak++)
  {
    intensity[ak] = intensity[ak] / highest;
  }
}

  //for (u_int8_t ak=0; ak < INTENSITY_BINS; ak++)
 // {
 //   Serial.println(intensity[ak]);

  // Update Intensity Array
  // for(int i = 2; i < (xres*Displacement)+2; i+=Displacement){
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
  // this starts the rainbow cycle over if the ledState is set to rainbow cycle
  if (ledState == 8)
  {
    strip.RainbowCycle(10);
  }
}
