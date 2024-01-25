/*
MIT License

Copyright (c) 2021 Brett Elliott

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Keyboard.h>

const int LedPin = 13;
bool led = true;

#define BUTTON_DEBUG 1
//#define THRESH_DEBUG

#define N_DELAY 16

/*
 * A Delay-line class. Feed samples in, and it comes back out N_DELAY samples later.
 */
struct DelayLine
{
public:
  DelayLine()
  {
    // Initialize the buffer and index to 0.
    for (unsigned int i=0; i<N_DELAY; i++)
      buff[i] = 0;
    idx = 0;
  }

  // Disallow copying and assignment. Prevents accidental oopsies.
  DelayLine(const DelayLine&) = delete;
  DelayLine & operator=(const DelayLine&) = delete;

  // Pop a value off the delay line, and push a new one one:
  unsigned int next(unsigned int n)
  {
    // Grab the next value off the buffer:
    unsigned int ret = buff[idx];

    // Put the new value into the buffer:
    buff[idx] = n;

    // Increment and wrap the index:
    idx++;
    if (idx >= N_DELAY)
      idx = 0;
    
    return ret;
  }
private:
  // Index to read and write next value:
  unsigned int idx;

  // Buffer to hold our circular buffer of data:
  unsigned int buff[N_DELAY];
  
};

struct Thresher
{
public:
  /*
     Thresher constructor.

     absThresh - Static threshold. If the ADC is below this value. Call it on.

     relThresh -  Relative threshold. If we see the signal drop by this much over
                  N_DELAY samples, we also declare the button on. Helps with light
                  taps.
  */
  Thresher(unsigned int absThresh, long relThresh) :
    absThresh(absThresh), relThresh(relThresh)
  {}

  // Disallow copying and assignment. Prevents accidental oopsies.
  Thresher(const Thresher&) = delete;
  Thresher & operator=(const Thresher&) = delete;

  bool next(unsigned int x)
  {
    // Get the same from N_DELAY samples ago, and push the current sample into the
    // delay line:
    long delayed = delayLine.next(x);

    // Convert to signed long, so we can subtract and get a happily signed result:
    long longX = x;

    // Different between same from N_DELAY ago and current sample:
    long diff = delayed - longX;

    // Check if the button is one:
    bool on = false;
    if (x < absThresh)
    {
#ifdef THRESH_DEBUG
      Serial.printf("%lu ABS THRESH ON\n", millis());
#endif
      on = true;
    }
    if (diff > relThresh)
    {
#ifdef THRESH_DEBUG
      Serial.printf("%lu REL THRESH ON\n", millis());
#endif
      on = true;
    }

    return on;
  }

private:

  // Static threshold. If the ADC is below this value. Call it on:
  unsigned int absThresh;
  
  // Realtive threshold. If we see the signal drop by this much over N_DELAY
  // samples, we also declare the button on. Helps with light taps.
  long relThresh;
  DelayLine delayLine;
};

/*
   Button class. Handles updating buttons.
*/
struct Button
{
public:
  /*
     Button constructor.

     adcPin - analog to digital convertor pin number
     absThresh - see Thresher
     relThresh - see Thresher.
     letter - Which letter to press on the keyboard when activated.
  */
  Button(int adcPin, unsigned int absThresh, long relThresh, char letter) :
    adcPin(adcPin), letter(letter), buttabsThresher(absThresh, relThresh)
  {}

  // Disallow copying and assignment. Prevents accidental oopsies.
  Button(const Button&) = delete;
  Button & operator=(const Button&) = delete;

  // LED upkeep
  static void preUpdate()
  {
    // Turn off the LED:
    led = false;
    //Serial.printf("%lu,", millis());
  }

  /*
     Update method. Check the ADC, and press/unpress the key as appropirate.
  */
  void update()
  {
    
    // Read the ADC:
    int adc = analogRead(adcPin);

    // Check the thresh, to see if we should turn the button on:
    bool on = buttabsThresher.next(adc);

    Serial.printf("%d,", adc);
    // Send the keypress if within threshold
    if (on) {
      Keyboard.press(letter);
    }
    else {
      Keyboard.release(letter);
    }
    // Update the LED state:
    led |= on;
  }

  // LED upkeep
  static void postUpdate()
  {
    // Update the LED. On if something is pressed, off otherwise:
    digitalWrite(LedPin, led);
    Serial.print("\n");
  }

  // Output pin
  int getPin() {
    return adcPin;
  }

  // Output key
  char getKey() {
    return letter;
  }

private:

  // ADC pin number:
  int adcPin;

  // Keyboard key to send through USB
  int letter;

  // Object which does the 
  Thresher buttabsThresher;
};

// Define the buttons (analog input pin, abs thresh, rel thresh, keyboard key):
const int numButtons = 5;
Button buttons[numButtons] =
{
  {A1, 750, 90, 'd'}, // todo: need to test thresholds
  {A2, 750, 90, 'f'},
  {A3, 850, 70, ' '},
  {A4, 750, 90, 'j'},
  {A5, 750, 90, 'k'},
};

// Arduino setup function
void setup()
{
  // Setup the LedPin to be an output, so we can turn the LED on/off:
  pinMode(LedPin, OUTPUT);

  // Set analog pins to input pullup resistor mode
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttons[i].getPin(), INPUT);
  }

  // Enable the keyboard usage over USB
  Keyboard.begin();

  // Enable serial diagnostics
  #ifdef BUTTON_DEBUG
    // Print the ADC value:
    Serial.begin(9600);
  #endif
}

void loop()
{
  // Do the pre-update step:
  Button::preUpdate();

  // Loop over the buttons, and update each one. Make sure to pass buttons by reference.
  for (auto& button : buttons) {
    Serial.print(button.getKey());
    button.update();
    //delay(1000); debug delay
  }
  // Post-update step:
  Button::postUpdate();
}
