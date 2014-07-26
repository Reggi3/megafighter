#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

/*
 * This is a moco-lufa/arduino mega2560 firmware based around a midi fighter controller hardware layout with a few additions :-)
 * the first addition is that instead of 16 buttons it's got 32 in 2 lots of 16 + there is scope to add more
 * It also has 16 faders or pots and it also has addressable ws2812b RGB LED support, one led for each of the 32 buttons
 * Each led is programmable via midi with support for saving LED colours in eeprom.
 * Lastly I have added rotary encoder support, it will require 2 digital pins per encoder
 * at the moment I have only used the encoders with rotary knobs but I suspect that it should be simple to setup
 * a platter of some description and connect it up to a pair of encoder pins for a complete dj controller :)
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 
 */


// Defines
#define USE_LEDS 1

#ifdef USE_LEDS
void blinkLed(uint8_t r,  uint8_t g,  uint8_t b, uint8_t led_num, uint8_t blinkdelay, uint8_t howmany);
#define NUM_LEDS 16 // if you change this you'll need to add/remove a/some line(s) from buttonColours array below
#define DATA_PIN 46
#define BASE_ADDRESS 0 // the base address of our eeprom data, this is where we start, we can move it later if we want to
#define RESERVED_BYTES 4 // first 4 bytes in eeprom are reserved for system usage for us
#define EEPROM_COLOR_OFFSET BASE_ADDRESS+RESERVED_BYTES
#define RED_OFFSET EEPROM_COLOR_OFFSET
#define GREEN_OFFSET EEPROM_COLOR_OFFSET+NUM_LEDS
#define BLUE_OFFSET EEPROM_COLOR_OFFSET+(NUM_LEDS*2)

uint8_t value = 0;

volatile char buttonsRed[NUM_LEDS] = {
  0xFF,
  0x00,
  0x00,
  0xFF,
  0x00,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0xFF,
  0x00,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0xFF,
};

volatile char buttonsGreen[NUM_LEDS] = {
  0x00,
  0xFF,
  0x00,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0xFF,
  0x00,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0xFF,
  0x00,
  0xFF,
};

volatile char buttonsBlue[NUM_LEDS] = {
  0x00,
  0x00,
  0xFF,
  0x00,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0xFF,
  0x00,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0xFF,
  0x00,
};

Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);//WS2812B

#endif // USE_LEDS endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define ARDUINO_MEGA
#endif

// Uncomment this line to enable outputs corresponding to the MIDI Fighter so MF mappings can be used in Traktor.
//#define MIDI_FIGHTER

//#define FASTADC
// defines for setting and clearing register bits
#ifdef FASTADC
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#endif

// MIDI mapping taken from http://www.nortonmusic.com/midi_cc.html
#define MIDI_CC_MODULATION 0x01
#define MIDI_CC_BREATH 0x02
#define MIDI_CC_VOLUME 0x07
#define MIDI_CC_BALANCE 0x08
#define MIDI_CC_PAN 0x0A
#define MIDI_CC_EXPRESSION 0x0B
#define MIDI_CC_EFFECT1 0x0C
#define MIDI_CC_EFFECT2 0x0D

#define MIDI_CC_GENERAL1 0x0E
#define MIDI_CC_GENERAL2 0x0F
#define MIDI_CC_GENERAL3 0x10
#define MIDI_CC_GENERAL4 0x11
#define MIDI_CC_GENERAL5 0x12
#define MIDI_CC_GENERAL6 0x13
#define MIDI_CC_GENERAL7 0x14
#define MIDI_CC_GENERAL8 0x15
#define MIDI_CC_GENERAL9 0x16
#define MIDI_CC_GENERAL10 0x17
#define MIDI_CC_GENERAL11 0x18
#define MIDI_CC_GENERAL12 0x19
#define MIDI_CC_GENERAL13 0x1A
#define MIDI_CC_GENERAL14 0x1B
#define MIDI_CC_GENERAL15 0x1C
#define MIDI_CC_GENERAL16 0x1D
#define MIDI_CC_GENERAL17 0x1E
#define MIDI_CC_GENERAL18 0x1F

#define MIDI_CC_GENERAL1_FINE 0x2E
#define MIDI_CC_GENERAL2_FINE 0x2F
#define MIDI_CC_GENERAL3_FINE 0x30
#define MIDI_CC_GENERAL4_FINE 0x31
#define MIDI_CC_GENERAL5_FINE 0x32
#define MIDI_CC_GENERAL6_FINE 0x33
#define MIDI_CC_GENERAL7_FINE 0x34
#define MIDI_CC_GENERAL8_FINE 0x35
#define MIDI_CC_GENERAL9_FINE 0x36
#define MIDI_CC_GENERAL10_FINE 0x37
#define MIDI_CC_GENERAL11_FINE 0x38
#define MIDI_CC_GENERAL12_FINE 0x39
#define MIDI_CC_GENERAL13_FINE 0x3A
#define MIDI_CC_GENERAL14_FINE 0x3B
#define MIDI_CC_GENERAL15_FINE 0x3C
#define MIDI_CC_GENERAL16_FINE 0x3D
#define MIDI_CC_GENERAL17_FINE 0x3E
#define MIDI_CC_GENERAL18_FINE 0x3F

#define MIDI_CC_SUSTAIN 0x40
#define MIDI_CC_REVERB 0x5B
#define MIDI_CC_CHORUS 0x5D
#define MIDI_CC_CONTROL_OFF 0x79
#define MIDI_CC_NOTES_OFF 0x78
#define MIDI_NOTE_ON 0x90
#define MIDI_NOTE_OFF 0x80
#define NOTE_C0 0x00 // 0
#define NOTE_C1 0x12 // 18
#define NOTE_C2 0x24 // 36

// Number of digital inputs. Can be anywhere from 0 to 68.
#define NUM_DI 32
// Number of analogue inputs. Can be anywhere from 0 to 16.
#define NUM_AI 8


#define MIDI_CHANNEL 1
// First note, starting from upper left button
#define NOTE NOTE_C0
// This pin order corresponds to the top left button being zero, increasing by one as we move from left to right, top to bottom
// 0  1  2  3
// 4  5  6  7
// 8  9  10 11
// This array size must match NUM_DI above.
#define DIGITAL_PIN_ORDER 2,3,4,5,6,7,8,9,10,11,12,13,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41
// This array size must match NUM_AI above.
#define ANALOGUE_PIN_ORDER A0, A1, A2, A3, A4, A5, A6, A7
// NUM_UNUSED must = 16 - NUM_AI
#define NUM_UNUSED 8
// All unused analog pins must be defined here, this is so that we can pull them all low at startup
// to keep the noise down.  clearly I could just set all the analog pins low then initialise just the ones
// we need but what the hell, it's a mega :D
#define UNUSED_ANALOG_PINS A8, A9, A10, A11, A12, A13, A14, A15
#define LED_PIN 13


#define MIDI_CC MIDI_CC_GENERAL1  // Base CC for the analog pins
// Reggie Added, CC for the encoders, lets keep it way from the analog CCs
#define EN_CC MIDI_CC_GENERAL17

// Comment this line out to disable button debounce logic.
// See http://arduino.cc/en/Tutorial/Debounce on what debouncing is used for.
#define DEBOUNCE
// Debounce time length in milliseconds
#define DEBOUNCE_LENGTH 2

// Array containing a mapping of digital pins to channel index.
byte digitalInputMapping[NUM_DI] = {DIGITAL_PIN_ORDER};

// Array containing a mapping of analogue pins to channel index. This array size must match NUM_AI above.
byte analogueInputMapping[NUM_AI] = {ANALOGUE_PIN_ORDER};
byte unusedAnalogPins[NUM_UNUSED] = {UNUSED_ANALOG_PINS};

void analog_init() {

  for (int i = 0; i < NUM_UNUSED; i++) {
    pinMode(unusedAnalogPins[i], OUTPUT);
#ifdef ENABLE_PULLUPS
    digitalWrite(unusedAnalogPins[i], HIGH);

#endif
  }

}

// Contains the current state of the digital inputs.
byte digitalInputs[NUM_DI];
// Contains the current value of the analogue inputs.
byte analogueInputs[NUM_AI];

// Variable to hold temporary digital reads, used for debounce logic.
byte tempDigitalInput;
// Variable to hold temporary analogue values, used for analogue filtering logic.
// Reggie Changed, got to use a signed int if you want to use the abs function
int16_t tempAnalogueInput;

// Preallocate the for loop index so we don't keep reallocating it for every program iteration.
byte i = 0;
// digitalOffset is used to offset for the midifighter '4 bank' mode, will look at implementing it properly
// as there are more than enough digital pins to provide the 'external' 4 bank mode.
byte digitalOffset = 0;

// Reggie added, midi thru support, uncomment to enable pseudo-midi input via Serial1 on the leonardo
// attach another microcontroller to the serial RX pin of the leonardo to expand
// the number of buttons, pots and encoders you can use
// serial port is running at 115200
// #define ENABLE_THRU

// variable to hold the last value for each analog input, used to compare against our hysterysis value
// The original analog code didn't work very well, firing at random, out of sequence and multiple times,
// because the hysterisis check was applied to the converted 7-bit value instead of the raw 10-bit adc value
// we store the 10bit value
int lastVal[NUM_AI];

// Reggie added, rotary encoder setup code
// This allows you to add rotary encoders to the midi controller
// they take 2 digital pins per encoder and do not require an interrupt
//#define ENABLE_ROTARY

#ifdef ENABLE_ROTARY
// define to enable weak pullups.
#define ENABLE_PULLUPS
#define NUM_EN 1 // number of encoders

// Use the full-step state table (emits a code at 00 only)
const char ttable[7][4] = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x40},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x80},
  {0x6, 0x5, 0x4,  0x0},
};

const byte ENC_PINS[NUM_EN * 2] = {6, 7}; //2 pins per encoder
volatile char state[NUM_EN] = {0};

/* Call this once in setup(). */
void rotary_init() {
  for (int i = 0; i < NUM_EN; i++) {
    pinMode(ENC_PINS[i * 2], INPUT);
    pinMode(ENC_PINS[(i * 2) + 1], INPUT);
#ifdef ENABLE_PULLUPS
    digitalWrite(ENC_PINS[i * 2], HIGH);
    digitalWrite(ENC_PINS[(i * 2) + 1], HIGH);
#endif
  }

}

/* Read input pins and process for rotary events. Call this either from a
 * loop or an interrupt (eg pin change or timer).
 *
 * Returns 0 on no event, otherwise 0x80 or 0x40 depending on the direction.
 */
char rotary_process(byte pin1, byte pin2, byte encnum) {
  char pinstate = (digitalRead(pin2) << 1) | digitalRead(pin1);
  state[encnum] = ttable[state[encnum] & 0xf][pinstate];
  return (state[encnum] & 0xc0);
}
#endif // ENABLE_ROTARY endif

void setup() {
  // Taken from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
#ifdef FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2) ;
  cbi(ADCSRA, ADPS1) ;
  cbi(ADCSRA, ADPS0) ;
#endif

#ifdef USE_LEDS
  leds.begin();
  leds.show(); // Initialize all pixels to 'off'

  for (int j = 0; j < NUM_LEDS; j++) {
    leds.setPixelColor(0, buttonsRed[j], buttonsGreen[j], buttonsBlue[j]);
    leds.show(); // show we're alive
    delay(250);
    leds.setPixelColor(j, 0x000000);
    leds.show();
  }

  //blinkLed(0x7F,0x40,0x00,0,250,5); // lets us know we're allive, 5 yellow blinks
  //wipeEeprom();
  value = checkEeprom();
  if (value > 0) { // our register is valid
    readEepromContents(); // read out the saved colour data from eeprom into our arrays
    blinkLed(0x00, 0xFF, 0x00, 0, 250, 9); // flash green 9 times to let them know the eeprom has data in it
  }
  else if (value == 0) {
    blinkLed(0xFF, 0x00, 0x00, 0, 250, 9); // flash red 9 times to let them know that the eeprom is empty
    writeEepromContents();
    blinkLed(0x00, 0xFF, 0x00, 0, 250, 9); // flash green 9 times to let them know the eeprom has data in it
  }

  leds.setPixelColor(0, buttonsRed[0], buttonsGreen[0], buttonsBlue[0]);
  leds.show();
#endif
  // Reggie changed, moved to arduino midi lib for the mega, it's serial based but it handles input/output, thru and sysex way better than I can.
  MIDI.begin(MIDI_CHANNEL_OMNI);

#ifdef ENABLE_THRU
  // added pseudo midi thru, just read in some bytes from serial1 and pass them back out
  // simple way to add more buttons/analog inputs, hook up an atmega328 chip with the original code on it.
  Serial1.begin(115200);
#endif
#ifdef ENABLE_ROTARY
  // Regggie Added, code to use initialise rotary encoders
  rotary_init();
#endif
analog_init();
  // Initialise each digital input channel.
  for (i = 0; i < NUM_DI; i++)
  {
    // Set the pin direction to input.
    pinMode(digitalInputMapping[i], INPUT);

    // Don't enable pullup resistor on LED_PIN, as the LED and resistor will always pull it low, meaning the input won't work.
    // Instead an external pulldown resistor must be used on LED_PIN.
    // NOTE: This will cause all of the high/low logic for LED_PIN to be inverted.
    if (digitalInputMapping[i] != LED_PIN)
    {
      // Enable the pull-up resistor. This call must come after the above pinMode call.
      digitalWrite(digitalInputMapping[i], HIGH);
    }

    // Initialise the digital state with a read to the input pin.
    digitalInputs[i] = digitalRead(digitalInputMapping[i]);
  }

  // Initialise each analogue input channel.
  for (i = 0; i < NUM_AI; i++)
  {
    // Set the pin direction to input.
    pinMode(analogueInputMapping[i], INPUT);

    // Initialise the analogue value with a read to the input pin.
    analogueInputs[i] = analogRead(analogueInputMapping[i]);

  }

}

void loop() {

#ifdef USE_LEDS
  updateLeds();
#endif
  checkButtons();
  checkEncoders();
  checkPots();

}

void checkEncoders(){
  // Reggie Added, code for rotary encoder
#ifdef ENABLE_ROTARY
  for (int i = 0; i < NUM_EN; i++) {
    char result = rotary_process(ENC_PINS[i * 2], ENC_PINS[i * 2 + 1], i);
    // EN_CC
    if (result) {
      if (result == 0x40) { // increment
        MIDI.sendControlChange(EN_CC + i, 0x3F, MIDI_CHANNEL); // binary offset - serato - traktor
//        MIDI.sendControlChange(EN_CC + i, 0x41, MIDI_CHANNEL); // signed bit - serato
//        MIDI.sendControlChange(EN_CC + i, 0x01, MIDI_CHANNEL); // traktor
//        MIDI.sendControlChange(EN_CC + i, 0x00, MIDI_CHANNEL); // relative - on/off - serato
//        MIDI.sendControlChange(0x60, 0x01, MIDI_CHANNEL); // fruityloops, increment rpn/nrpn, should probably use i to offset the channel here or it's useless?
      }
      else { // decrement
        MIDI.sendControlChange(EN_CC + i, 0x41, MIDI_CHANNEL); // binary offset - serato - traktor
//        MIDI.sendControlChange(EN_CC + i, 0x01, MIDI_CHANNEL); // signed bit - serato
//        MIDI.sendControlChange(EN_CC + i, 0x7F, MIDI_CHANNEL); // traktor
//        MIDI.sendControlChange(EN_CC + i, 0x01, MIDI_CHANNEL); // relative - on/off - serato
//        MIDI.sendControlChange(0x61, 0x01, MIDI_CHANNEL); // fruityloops, decrement rpn/nrpn, should probably use i to offset the channel here or it's useless?
      }

    }
  }
#endif
  


}


  /*
   * Analogue input logic:
   * The Arduino uses a 10-bit (0-1023) analogue to digital converter (ADC) on each of its analogue inputs.
   * The ADC isn't very high resolution, so if a pot is in a position such that the output voltage is 'between'
   * what it can detect (say 2.505V or about 512.5 on a scale of 0-1023) then the value read will constantly
   * fluctuate between two integers (in this case 512 and 513).
   *
   * If we're simply looking for a change in the analogue input value like in the digital case above, then
   * there will be cases where the value is always changing, even though the physical input isn't being moved.
   * This will in turn send out a constant stream of MIDI messages to the connected software which may be problematic.
   *
   * To combat this, we require that the analogue input value must change by a certain threshold amount before
   * we register that it is actually changing. This is good in avoiding a constantly fluctuating value, but has
   * the negative effect of a reduced input resolution. For example if the threshold amount was 2 and we slowly moved
   * a slider through it's full range, we would only detect every second value as a change, in effect reducing the
   * already small 7-bit MIDI value to a 6-bit MIDI value.
   *
   * To get around this problem but still use the threshold logic, a timer is used. Initially the analogue input
   * must exceed the threshold to be detected as an input. Once this occurs, we then read every value coming from the
   * analogue input (not just those exceeding a threshold) giving us full 7-bit resolution. At the same time the
   * timer is started. This timer is used to keep track of whether an input hasn't been moved for a certain time
   * period. If it has been moved, the timer is restarted. If no movement occurs the timer is just left to run. When
   * the timer expires the analogue input is assumed to be no longer moving. Subsequent movements must exceed the
   * threshold amount.
   */


  // Reggie changed, I culled all the old analog code, it's unecessary and it didn't work at low speeds
  // I was getting readouts of 0,1,0,1,0,1,0,1,2,1,2,1,2,3,2,3,2,3 etc. from the old code
  // I've replaced it with this neat and simple bit of code, doesn't require much, just a decent hysterisis value
  // and the hysterisis value checked againstthe 10bit abs value, not the 7bit value!  Doing it to the 7bit value
  // thins the resolution of the data out and no one likes thin resolution data!!
  // it turns out if you're going to measure abs you need signed ints :-D

void checkPots(){
  for (i = 0; i < NUM_AI; i++)
  {
    // Read the analogue input pin, check hysterisis, if it's real, shift 3bits right so the 10-bit ADC value (0-1023) is converted to a 7-bit MIDI value (0-127).
    tempAnalogueInput = analogRead(analogueInputMapping[i]); // read the analog pin into a temp variable
    if (abs(tempAnalogueInput - lastVal[i]) >= 8) { // is it noise?
      lastVal[i] = tempAnalogueInput; // nope, it's real, lets store that temp value as the last valid value for this ADC pin
      MIDI.sendControlChange(MIDI_CC + i, tempAnalogueInput >> 3, MIDI_CHANNEL); // send the CC message on it's way.
    }

  }


}

void checkButtons() {
  for (i = 0; i < NUM_DI; i++)
  {
#ifdef MIDI_FIGHTER
    if (i >= SKIP_ROW * 4)
    {
      digitalOffset = i + 4;
    }
    else
    {
#endif

      digitalOffset = i;

#ifdef MIDI_FIGHTER
    }
#endif

    // Read the current state of the digital input and store it temporarily.
    tempDigitalInput = digitalRead(digitalInputMapping[i]);

    // Check if the last state is different to the current state.
    if (digitalInputs[i] != tempDigitalInput)
    {
#ifdef DEBOUNCE
      // Wait for a short period of time, and then take a second reading from the input pin.
      delay(DEBOUNCE_LENGTH);
      // If the second reading is the same as the initial reading, assume it must be true.
      if (tempDigitalInput == digitalRead(digitalInputMapping[i]))
      {
#endif
        // Record the new digital input state.
        digitalInputs[i] = tempDigitalInput;

        // Moved from HIGH to LOW (button pressed)
        if (digitalInputs[i] == 0)
        {
          // All the digital inputs use pullup resistors, except LED_PIN so the logic is inverted
          if (digitalInputMapping[i] != LED_PIN)
          {
            MIDI.sendNoteOn(NOTE + digitalOffset, 0x7F, MIDI_CHANNEL); // middle C, maximum velocity, Channel 1
            leds.setBrightness(128);
            leds.setPixelColor(0, 0x7F, 00, 00);
            leds.show();
          }
          else
          {
            MIDI.sendNoteOff(NOTE + digitalOffset, 0x00, MIDI_CHANNEL); // Channel 1
          }
        }
        // Moved from LOW to HIGH (button released)
        else
        {
          // All the digital inputs use pullup resistors, except LED_PIN so the logic is inverted
          if (digitalInputMapping[i] != LED_PIN)
          {
            MIDI.sendNoteOff(NOTE + digitalOffset, 0x00, MIDI_CHANNEL); // Channel 1
            leds.setBrightness(128);
            leds.setPixelColor(0, 00, 00, 00);
            leds.show();
          }
          else
          {
            MIDI.sendNoteOn(NOTE + digitalOffset, 0x7F, MIDI_CHANNEL); // maximum velocity, Channel 1
          }
        }
#ifdef DEBOUNCE
      }
#endif
    }
  }



}

#ifdef USE_LEDS
void updateLeds() {
  if (MIDI.read()) {
    uint8_t channel = MIDI.getChannel();
    uint8_t type = MIDI.getType() + channel;
    uint8_t data1 = MIDI.getData1();
    uint8_t data2 = MIDI.getData2();


    if (type == 0x80 || type == 0x81) { // a real note off
        if (data1 < NUM_LEDS) { // check whether we're within our array boundaries, if we go outside this, there be magicke and dragons!!
          // It's safe, rhe led exists \o/, turn off the leds
          leds.setPixelColor(data1, 0x00);
          leds.show();
        }
      }
    
    if (type == 0x90 && data2 == 0x00) { // a note on with velocity zero is considered a note off
      // It's safe, rhe led exists \o/, turn off the leds
      if (data1 < NUM_LEDS) { // check whether we're within our array boundaries, if we go outside this, there be magicke and dragons!!
        leds.setPixelColor(data1, 0x00); // a pixel with zero brightness is off, don't need to worry about setting brightness, just turn it off
        leds.show();
      }
    }

    if (type == 0x90 && data2 > 0x00) { // Velocity greater than 0 is a note on
      if (data1 < NUM_LEDS) { // check whether we're within our array boundaries, if we go outside this, there be magicke and dragons!!
        // It's safe, rhe led exists \o/, turn on the leds
        // allow our brightness to be adjusted via velocity, 7bits is good as it sets a maximum of 1/2 brightness otherwise it'll probably
        // draw too much current and do silly things
        // multiply by 2 to get the full brightness but 1/2 the resolution but make sure you've got plenty of power
        // I think it's about 60ma per led total, so 16x60= 960ma at full tilt!!!!
        leds.setBrightness(data2); // change the brightness first, then the pixel colour
        leds.setPixelColor(data1, buttonsRed[data1], buttonsGreen[data1], buttonsBlue[data1]); // update the pixel after the brightness
        leds.show();
      }
    }

    /* this is where we allow the user to change the colour of the leds for each button using note on messages and velocity
     * we take messages in on channels 2 and 3, this allows us to do 7 and 8bit colour values respectively without compromising
     * or having to do unecessary calculations.
     *
     * The note on values we're expecting are laid out like this, Red * NUM_LEDS, Green * NUM_LEDS and Blue * NUM_LEDS
     * notes coming in on channel 2 are 7bit values (0-127) and notes coming in on channel 3 are considered 8bits
     * we achieve this by simply adding 0x80 (128) to the velocity byte for any channel 3 messages.
     * I'm probably going to change this, give a channel to reds, a channel to greens and a channel to blues twice
     * this will give access to upto 128 leds in total with 2 colours per led stored.  Although 128 leds is fine for ram
     * there's only 1KB of eeprom, no way to fit 2 colours into that space and have any spare for anything else.
     * 64 leds would be a better option.
     *
     *
    */


    if (type == 0x91 || type == 0x92) { // these are messages from the user to change buttons
      int buttonNum = 0;
      if (type == 0x92) { // if it came in on channel 3 then it's an 8bit colour value
        data2 = data2 + 0x80; // if it's 8bit then we add 128
      }

      if (data1 < NUM_LEDS) { // it's a red note
        buttonsRed[data1] = data2;  // update the appropriate button in the red array
        leds.setPixelColor(data1, buttonsRed[data1], buttonsGreen[data1], buttonsBlue[data1]); // get ready for the show
        leds.show(); // allow the user to perform photon abuse and show them what they did, it's their own fault if it doesn't match the curtains
        return;
      }
      else if (data1 < NUM_LEDS * 2 && data1 >= NUM_LEDS) { // it's a green note
        buttonNum = data1 - NUM_LEDS; // nicer than using the math in the array brackets to get the correct button number.
        buttonsGreen[buttonNum] = data2;
        leds.setPixelColor(buttonNum, buttonsRed[buttonNum], buttonsGreen[buttonNum], buttonsBlue[buttonNum]);
        leds.show();
        return;
      }
      else if (data1 < NUM_LEDS * 3 && data1 >= NUM_LEDS * 2) { // it's a blue note
        buttonNum = data1 - (NUM_LEDS * 2);
        buttonsBlue[buttonNum] = data2;
        leds.setPixelColor(buttonNum, buttonsRed[buttonNum], buttonsGreen[buttonNum], buttonsBlue[buttonNum]);
        leds.show();
        return;
      }
      // if we made it here and did nothing, it was a brown note, no one likes the brown note, the note on value we were sent was out of range for the number of leds
    }

    if (type == 0x9F && data1 == 0x40 && data2 == 0x46) { // this is the magic note for saving the settings registers and colour data to eeprom
      blinkLed(0xFF, 0xFF, 0x00, 0, 250, 3); // flash yellow to let them know we're about to start updating
      writeEepromContents();
      blinkLed(0x00, 0xFF, 0x00, 0, 250, 3); // flash green to let them know we're done.
    }

    if (type == 0x9F && data1 == 0x40 && data2 == 0x48) { // this is the magic note for erasing to factory defaults, must reboot to perform the reset
      blinkLed(0xFF, 0x00, 0x00, 0, 250, 6); // 6 red flashes, we're starting to wipe
      wipeEeprom();
      blinkLed(0x00, 0x00, 0xFF, 0, 250, 6); // 6 blue flashes, time for the user to powercycle.
    }
  }
}

#if 0
void updateLeds() {
  while (MIDIUSB.available() > 0) {
    MIDIEvent e;
    e = MIDIUSB.read();
    if (e.m1 == 0x80 || e.m1 == 0x81) { // a real note off
      if (e.m2 < NUM_LEDS) { // check whether we're within our array boundaries, if we go outside this, there be magicke and dragons!!
        // It's safe, rhe led exists \o/, turn off the leds
        leds.setPixelColor(e.m2, 0x00);
        leds.show();
      }
    }

    if (e.m1 == 0x90 && e.m3 == 0x00) { // a note on with velocity zero is considered a note off
      // It's safe, rhe led exists \o/, turn off the leds
      if (e.m2 < NUM_LEDS) { // check whether we're within our array boundaries, if we go outside this, there be magicke and dragons!!
        leds.setPixelColor(e.m2, 0x00); // a pixel with zero brightness is off, don't need to worry about setting brightness, just turn it off
        leds.show();
      }
    }

    if (e.m1 == 0x90 && e.m3 > 0x00) { // Velocity greater than 0 is a note on
      if (e.m2 < NUM_LEDS) { // check whether we're within our array boundaries, if we go outside this, there be magicke and dragons!!
        // It's safe, rhe led exists \o/, turn on the leds
        // allow our brightness to be adjusted via velocity, 7bits is good as it sets a maximum of 1/2 brightness otherwise it'll probably
        // draw too much current and do silly things
        // multiply by 2 to get the full brightness but 1/2 the resolution but make sure you've got plenty of power
        // I think it's about 60ma per led total, so 16x60= 960ma at full tilt!!!!
        leds.setBrightness(e.m3); // change the brightness first, then the pixel colour
        leds.setPixelColor(e.m2, buttonsRed[e.m2], buttonsGreen[e.m2], buttonsBlue[e.m2]); // update the pixel after the brightness
        leds.show();
      }
    }

    /* this is where we allow the user to change the colour of the leds for each button using note on messages and velocity
     * we take messages in on channels 2 and 3, this allows us to do 7 and 8bit colour values respectively without compromising
     * or having to do unecessary calculations.
     *
     * The note on values we're expecting are laid out like this, Red * NUM_LEDS, Green * NUM_LEDS and Blue * NUM_LEDS
     * notes coming in on channel 2 are 7bit values (0-127) and notes coming in on channel 3 are considered 8bits
     * we achieve this by simply adding 0x80 (128) to the velocity byte for any channel 3 messages.
     * I'm probably going to change this, give a channel to reds, a channel to greens and a channel to blues twice
     * this will give access to upto 128 leds in total with 2 colours per led stored.  Although 128 leds is fine for ram
     * there's only 1KB of eeprom, no way to fit 2 colours into that space and have any spare for anything else.
     * 64 leds would be a better option.
     *
     *
    */


    if (e.m1 == 0x91 || e.m1 == 0x92) { // these are messages from the user to change buttons
      int buttonNum = 0;
      if (e.m1 == 0x92) { // if it came in on channel 3 then it's an 8bit colour value
        e.m3 = e.m3 + 0x80; // if it's 8bit then we add 128
      }

      if (e.m2 < NUM_LEDS) { // it's a red note
        buttonsRed[e.m2] = e.m3;  // update the appropriate button in the red array
        leds.setPixelColor(e.m2, buttonsRed[e.m2], buttonsGreen[e.m2], buttonsBlue[e.m2]); // get ready for the show
        leds.show(); // allow the user to perform photon abuse and show them what they did, it's their own fault if it doesn't match the curtains
        break;
      }
      else if (e.m2 < NUM_LEDS * 2 && e.m2 >= NUM_LEDS) { // it's a green note
        buttonNum = e.m2 - NUM_LEDS; // nicer than using the math in the array brackets to get the correct button number.
        buttonsGreen[buttonNum] = e.m3;
        leds.setPixelColor(buttonNum, buttonsRed[buttonNum], buttonsGreen[buttonNum], buttonsBlue[buttonNum]);
        leds.show();
        break;
      }
      else if (e.m2 < NUM_LEDS * 3 && e.m2 >= NUM_LEDS * 2) { // it's a blue note
        buttonNum = e.m2 - (NUM_LEDS * 2);
        buttonsBlue[buttonNum] = e.m3;
        leds.setPixelColor(buttonNum, buttonsRed[buttonNum], buttonsGreen[buttonNum], buttonsBlue[buttonNum]);
        leds.show();
        break;
      }
      // if we made it here and did nothing, it was a brown note, no one likes the brown note, the note on value we were sent was out of range for the number of leds
    }

    if (e.m1 == 0x9F && e.m2 == 0x40 && e.m3 == 0x46) { // this is the magic note for saving the settings registers and colour data to eeprom
      blinkLed(0xFF, 0xFF, 0x00, 0, 250, 3); // flash yellow to let them know we're about to start updating
      writeEepromContents();
      blinkLed(0x00, 0xFF, 0x00, 0, 250, 3); // flash green to let them know we're done.
    }

    if (e.m1 == 0x9F && e.m2 == 0x40 && e.m3 == 0x48) { // this is the magic note for erasing to factory defaults, must reboot to perform the reset
      blinkLed(0xFF, 0x00, 0x00, 0, 250, 6); // 6 red flashes, we're starting to wipe
      wipeEeprom();
      blinkLed(0x00, 0x00, 0xFF, 0, 250, 6); // 6 blue flashes, time for the user to powercycle.
    }
  }
}
#endif // if0 endif

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return leds.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return leds.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return leds.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void blinkLed(uint8_t r,  uint8_t g,  uint8_t b, uint8_t led_num, uint8_t blinkdelay, uint8_t howmany) {
  for (int i = 0; i < howmany; i++) {
    leds.setPixelColor(led_num, r, g, b);
    leds.show();
    delay(blinkdelay);
    leds.setPixelColor(led_num, 0x00);
    leds.show();
    delay(blinkdelay);
  }
}

uint8_t checkEeprom() {
  if (EEPROM.read(BASE_ADDRESS) == 0) { // it's empty as far as we're concerned
    blinkLed(0x00, 0x00, 0xFF, 0, 250, 3); // blue // these are here for debug purposes and will be standardised
    return 0;
  }
  // there must be something in it, return a 1
  blinkLed(0x7F, 0x00, 0xFF, 0, 250, 3); // pinky
  return 1;
}

void wipeEeprom() {
  // write a 0 to all 1024 bytes of the EEPROM
  for (int i = 0; i < 1024; i++)
    EEPROM.write(i, 0x00);
}


void writeEepromContents() {

  for (int i = 0; i < NUM_LEDS; i++) { // run through 
    EEPROM.write(i + RED_OFFSET, buttonsRed[i]);
    EEPROM.write(i + GREEN_OFFSET, buttonsGreen[i]);
    EEPROM.write(i + BLUE_OFFSET, buttonsBlue[i]);
  }
  // now we've written the array defaults to eeprom, we can set the flag
  // that lets us know we've got valid colour information in eeprom
  // we will expand that byte and possibly the next 3 to give us a range
  // of settings we can use, for now we will write a 1 here but
  // we will need to be careful in the future once we have some other
  // settings to set!!
  EEPROM.write(BASE_ADDRESS, 0x01);
}

// we need to read the Eeprom data into our array on first run
void readEepromContents() {
  for (int i = 0; i < NUM_LEDS; i++) {
    buttonsRed[i] = EEPROM.read(i + RED_OFFSET);
    buttonsGreen[i] = EEPROM.read(i + GREEN_OFFSET);
    buttonsBlue[i] = EEPROM.read(i + BLUE_OFFSET);
    Serial.print(i);
    Serial.print("R: ");
    Serial.print(buttonsRed[i], HEX);
    Serial.print(" G: ");
    Serial.print(buttonsGreen[i], HEX);
    Serial.print(" B: ");
    Serial.println(buttonsBlue[i], HEX);
  }
}
#endif
