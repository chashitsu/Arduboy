/** 
 * @file ArduboyCore.cpp
 * \brief A class implementing the core functionality of an Arduboy.
 *
 */

#include "ArduboyCore.h"

#ifdef ARDUINO_ARCH_AVR
// need to redeclare these here since we declare them static in .h
volatile uint8_t *ArduboyCore::csport, *ArduboyCore::dcport;
uint8_t ArduboyCore::cspinmask, ArduboyCore::dcpinmask;
#endif

const uint8_t PROGMEM pinBootProgram[] = {
  // buttons
  PIN_LEFT_BUTTON, INPUT_PULLUP,
  PIN_RIGHT_BUTTON, INPUT_PULLUP,
  PIN_UP_BUTTON, INPUT_PULLUP,
  PIN_DOWN_BUTTON, INPUT_PULLUP,
  PIN_A_BUTTON, INPUT_PULLUP,
  PIN_B_BUTTON, INPUT_PULLUP,

  // RGB LED (or single blue LED on the DevKit)
  // TODO For ArduboyZ check if RGB pins are set to high drive current (DRVSTR). Add code to to set DRVSTR if not.
  // Will probably have to do all digital LED control directly to keep DRVSTR active.
  // e.g.: PORT->Group[g_APinDescription[RED_LED].ulPort].PINCFG[g_APinDescription[RED_LED].ulPin].reg =
  //  (uint8_t)(PORT_PINCFG_DRVSTR | PORT_PINCFG_PULLEN);
#ifndef AB_DEVKIT
  RED_LED, INPUT_PULLUP,  // set INPUT_PULLUP to make the pin high when
  RED_LED, OUTPUT,        //   set to OUTPUT
  GREEN_LED, INPUT_PULLUP,
  GREEN_LED, OUTPUT,
#endif
  BLUE_LED, INPUT_PULLUP,
  BLUE_LED, OUTPUT,

  // audio is specifically not included here as those pins are handled
  // separately by `audio.begin()`, `audio.on()` and `audio.off()` in order
  // to respect the EEPROM audio settings

  // OLED SPI
  DISPLAY_DC, OUTPUT,
  DISPLAY_CS, OUTPUT,
  DISPLAY_RST, OUTPUT,
  0xFF
};

const uint8_t PROGMEM displayBootProgram[] = {
  // boot defaults are commented out but left here incase they
  // might prove useful for reference
  //
  // Further reading: https://www.adafruit.com/datasheets/SSD1306.pdf
  //
  // Display Off
  // 0xAE,

  // Set Display Clock Divisor v = 0xF0
  // default is 0x80
  0xD5, 0xF0,

  // Set Multiplex Ratio v = 0x3F
  // 0xA8, 0x3F,

  // Set Display Offset v = 0
  // 0xD3, 0x00,

  // Set Start Line (0)
  // 0x40,

  // Charge Pump Setting v = enable (0x14)
  // default is disabled
  0x8D, 0x14,

  // Set Segment Re-map (A0) | (b0001)
  // default is (b0000)
  0xA1,

  // Set COM Output Scan Direction
  0xC8,

  // Set COM Pins v
  // 0xDA, 0x12,

  // Set Contrast v = 0xCF
  0x81, 0xCF,

  // Set Precharge = 0xF1
  0xD9, 0xF1,

  // Set VCom Detect
  // 0xDB, 0x40,

  // Entire Display ON
  // 0xA4,

  // Set normal/inverse display
  // 0xA6,

  // Display On
  0xAF,

  // set display mode = horizontal addressing mode (0x00)
  0x20, 0x00,

  // set col address range
  // 0x21, 0x00, COLUMN_ADDRESS_END,

  // set page address range
  // 0x22, 0x00, PAGE_ADDRESS_END
};

ArduboyCore::ArduboyCore() {}

void ArduboyCore::boot()
{
  #ifdef ARDUBOY_SET_CPU_8MHZ
  // ARDUBOY_SET_CPU_8MHZ will be set by the IDE using boards.txt
  setCPUSpeed8MHz();
  #endif

  SPI.begin();
  bootPins();
  bootOLED();

  #ifdef SAFE_MODE
  if (buttonsState() == (LEFT_BUTTON | UP_BUTTON))
    safeMode();
  #endif

  bootPowerSaving();
}

#ifdef ARDUBOY_SET_CPU_8MHZ
// If we're compiling for 8MHz we need to slow the CPU down because the
// hardware clock on the Arduboy is 16MHz.
// We also need to readjust the PLL prescaler because the Arduino USB code
// likely will have incorrectly set it for an 8MHz hardware clock.
void ArduboyCore::setCPUSpeed8MHz()
{
  uint8_t oldSREG = SREG;
  cli();                // suspend interrupts
  PLLCSR = bit(PINDIV); // dissable the PLL and set prescale for 16MHz)
  CLKPR = bit(CLKPCE);  // allow reprogramming clock
  CLKPR = 1;            // set clock divisor to 2 (0b0001)
  PLLCSR = bit(PLLE) | bit(PINDIV); // enable the PLL (with 16MHz prescale)
  SREG = oldSREG;       // restore interrupts
}
#endif

void ArduboyCore::bootPins()
{
  uint8_t pin, mode;
  const uint8_t *i = pinBootProgram;

  while (true)
  {
    pin = pgm_read_byte(i++);
    mode = pgm_read_byte(i++);
    if (pin == 0xFF)
      break;
    pinMode(pin, mode);
  }

  digitalWrite(DISPLAY_RST, HIGH);
  delay(1);           // VDD (3.3V) goes high at start, lets just chill for a ms
  digitalWrite(DISPLAY_RST, LOW);   // bring reset low
  delay(10);          // wait 10ms
  digitalWrite(DISPLAY_RST, HIGH);  // bring out of reset
}

void ArduboyCore::bootOLED()
{
#ifdef ARDUINO_ARCH_AVR
  // setup the ports we need to talk to the OLED
  csport = portOutputRegister(digitalPinToPort(DISPLAY_CS));
  cspinmask = digitalPinToBitMask(DISPLAY_CS);
  dcport = portOutputRegister(digitalPinToPort(DISPLAY_DC));
  dcpinmask = digitalPinToBitMask(DISPLAY_DC);
#endif

  displaySPIbegin();

  displayCommandMode();
  // run our customized boot-up command sequence against the
  // OLED to initialize it properly for Arduboy
  for (int8_t i = 0; i < sizeof(displayBootProgram); i++)
    SPI.transfer(pgm_read_byte(displayBootProgram + i));

  displayDataMode();
}

void ArduboyCore::displaySPIbegin()
{
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  displayDataMode();
}

void ArduboyCore::displaySPIend()
{
  // de-assert the display chip select
#ifdef ARDUINO_ARCH_SAMD
  PORT->Group[g_APinDescription[DISPLAY_CS].ulPort].OUTSET.reg =
   bit(g_APinDescription[DISPLAY_CS].ulPin);
#else
  *csport |= cspinmask;
#endif

  SPI.endTransaction();
}

void ArduboyCore::displayDataMode()
{
#ifdef ARDUINO_ARCH_SAMD
  PORT->Group[g_APinDescription[DISPLAY_DC].ulPort].OUTSET.reg =
   bit(g_APinDescription[DISPLAY_DC].ulPin);
  PORT->Group[g_APinDescription[DISPLAY_CS].ulPort].OUTCLR.reg =
   bit(g_APinDescription[DISPLAY_CS].ulPin);
#else
  *dcport |= dcpinmask;
  *csport &= ~cspinmask;
#endif
}

void ArduboyCore::displayCommandMode()
{
#ifdef ARDUINO_ARCH_SAMD
  PORT->Group[g_APinDescription[DISPLAY_CS].ulPort].OUTSET.reg =
   bit(g_APinDescription[DISPLAY_CS].ulPin);
  PORT->Group[g_APinDescription[DISPLAY_DC].ulPort].OUTCLR.reg =
   bit(g_APinDescription[DISPLAY_DC].ulPin);
  PORT->Group[g_APinDescription[DISPLAY_CS].ulPort].OUTCLR.reg =
   bit(g_APinDescription[DISPLAY_CS].ulPin);
#else
  *csport |= cspinmask;
  *dcport &= ~dcpinmask;
  *csport &= ~cspinmask;
#endif
}

void ArduboyCore::safeMode()
{
  blank(); // clear screen to avoid writing random bytes in image buffer
  while (true) {
    asm volatile("nop \n");
  }
}

/* Power Management */

void ArduboyCore::idle()
{
// TODO Add equivalent for SAMD
#ifdef ARDUINO_ARCH_AVR
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();
#endif
}

void ArduboyCore::bootPowerSaving()
{
// TODO Add equivalent for SAMD
#ifdef ARDUINO_ARCH_AVR
  power_adc_disable();
  power_usart0_disable();
  power_twi_disable();
  // timer 0 is for millis()
  // timers 1 and 3 are for music and sounds
  power_timer2_disable();
  power_usart1_disable();
  // we need USB, for now (to allow triggered reboots to reprogram)
  // power_usb_disable()
#endif
}

uint8_t ArduboyCore::width() { return WIDTH; }

uint8_t ArduboyCore::height() { return HEIGHT; }

/* Drawing */

void ArduboyCore::paint8Pixels(uint8_t pixels)
{
  SPI.transfer(pixels);
}

void ArduboyCore::paintScreen(const uint8_t *image)
{
  for (int i = 0; i < (HEIGHT*WIDTH)/8; i++)
    SPI.transfer(pgm_read_byte(image + i));
}

// paint from a memory buffer, this should be FAST as it's likely what
// will be used by any buffer based subclass.
// if this function is changed, make sure corresponding changes
// are made to paintScreenAndClearImage()
void ArduboyCore::paintScreen(uint8_t image[])
{
#ifdef ARDUBOY_Z
  // TODO optimise like AVR version
  for (int i = 0; i < (HEIGHT*WIDTH)/8; i++)
  {
    SPI.transfer(image[i]);
  }
#else
  uint8_t c;
  int i = 0;

  SPDR = image[i++]; // set the first SPI data byte to get things started

  // the code to iterate the loop and get the next byte from the buffer is
  // executed while the previous byte is being sent out by the SPI controller
  while (i < (HEIGHT * WIDTH) / 8)
  {
    // get the next byte. It's put in a local variable so it can be sent as
    // as soon as possible after the sending of the previous byte has completed
    c = image[i++];

    while (!(SPSR & bit(SPIF))) { } // wait for the previous byte to be sent

    // put the next byte in the SPI data register. The SPI controller will
    // clock it out while the loop continues and gets the next byte ready
    SPDR = c;
  }
  while (!(SPSR & bit(SPIF))) { } // wait for the last byte to be sent
#endif
}

// this function is the same as paintScreen() except it also zeros the image
// buffer.
// it's kept separate from paintScreen() for speed.
// if paintScreen() is changed this fuction likely should also be changed
// to match.
void ArduboyCore::paintScreenAndClearImage(uint8_t image[])
{
#ifdef ARDUBOY_Z
  // TODO optimise like AVR version
  for (int i = 0; i < (HEIGHT * WIDTH) / 8; i++)
  {
    SPI.transfer(image[i]);
    image[i] = 0;
  }
#else
  uint8_t c;
  int i = 0;

  SPDR = image[i]; // set the first SPI data byte to get things started
  image[i++] = 0;  // clear the first image byte

  // the code to iterate the loop and get the next byte from the buffer is
  // executed while the previous byte is being sent out by the SPI controller
  while (i < (HEIGHT * WIDTH) / 8)
  {
    // get the next byte. It's put in a local variable so it can be sent as
    // as soon as possible after the sending of the previous byte has completed
    c = image[i];
    // clear the byte in the image buffer
    image[i++] = 0;

    while (!(SPSR & bit(SPIF))) { } // wait for the previous byte to be sent

    // put the next byte in the SPI data register. The SPI controller will
    // clock it out while the loop continues and gets the next byte ready
    SPDR = c;
  }
  while (!(SPSR & bit(SPIF))) { } // wait for the last byte to be sent
#endif
}

void ArduboyCore::blank()
{
  for (int i = 0; i < (HEIGHT * WIDTH) / 8; i++)
    SPI.transfer(0x00);
}

void ArduboyCore::sendDisplayCommand(uint8_t command)
{
  displayCommandMode();
  SPI.transfer(command);
  displayDataMode();
}

// invert the display or set to normal
// when inverted, a pixel set to 0 will be on
void ArduboyCore::invert(bool inverse)
{
  sendDisplayCommand(inverse ? OLED_PIXELS_INVERTED : OLED_PIXELS_NORMAL);
}

// turn all display pixels on, ignoring buffer contents
// or set to normal buffer display
void ArduboyCore::allPixelsOn(bool on)
{
  sendDisplayCommand(on ? OLED_ALL_PIXELS_ON : OLED_PIXELS_FROM_RAM);
}

// flip the display vertically or set to normal
void ArduboyCore::flipVertical(bool flipped)
{
  sendDisplayCommand(flipped ? OLED_VERTICAL_FLIPPED : OLED_VERTICAL_NORMAL);
}

// flip the display horizontally or set to normal
void ArduboyCore::flipHorizontal(bool flipped)
{
  sendDisplayCommand(flipped ? OLED_HORIZ_FLIPPED : OLED_HORIZ_NORMAL);
}

/* RGB LED */

void ArduboyCore::setRGBled(uint8_t red, uint8_t green, uint8_t blue)
{
#ifndef AB_DEVKIT
  // RGB, all the pretty colors
  // inversion is necessary because these are common annode LEDs
  analogWrite(RED_LED, 255 - red);
  analogWrite(GREEN_LED, 255 - green);
  analogWrite(BLUE_LED, 255 - blue);
#else
  // only blue on devkit
  digitalWrite(BLUE_LED, ~blue);
#endif
}

void ArduboyCore::digitalWriteRGB(uint8_t red, uint8_t green, uint8_t blue)
{
#ifndef AB_DEVKIT
  digitalWrite(RED_LED, red);
  digitalWrite(GREEN_LED, green);
  digitalWrite(BLUE_LED, blue);
#else
  digitalWrite(BLUE_LED, blue);
#endif
}

/* Buttons */

uint8_t ArduboyCore::buttonsState()
{
  uint8_t buttons;

#if defined ARDUBOY_Z
  // buttons: L R x U x B A D
  // PORT A bits: left 11, right 10, up 8, down 4
  buttons = (~PORT->Group[PORTA].IN.reg & (bit(11) | bit(10) | bit(8) | bit(4)))
            >> 4;
  // PORT B bits: B 9, A 8
  buttons |= (~PORT->Group[PORTB].IN.reg & (bit(9) | bit(8))) >> 7;
#elif defined ARDUBOY_10
  // buttons: D U L R A B x x
  // down, up, left right
  buttons = (~PINF & B11110000);
  // A (left)
  buttons |= (~PINE & B01000000) >> 3;
  // B (right)
  buttons |= (~PINB & B00010000) >> 2;
#elif defined AB_DEVKIT
  // buttons: x D L U x A B
  // down, left, up
  buttons = (~PINB & B01110000);
  // right button
  buttons |= (~PINC & B01000000) >> 4;
  // A and B
  buttons |= (~PINF & B11000000) >> 6;
#endif

  return buttons;
}

uint8_t ArduboyCore::getInput() // deprecated
{
  return buttonsState();
}
