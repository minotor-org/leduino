// Minotor FastSPI library for Arduino compatible device
#include <inttypes.h>

#include "FastSPI_LED2.h"

// Gamma correction LUT
// Please note this gamma correction lookup table have been established using a
// cheap luxmeter in front of a 384 leds matrix using WS2801-based strip.
// This gamma coorection allow us to have a more linear brightness on each
// color and try to have a similar lux level between colors.
#include "gamma.h"

//Change this to YOUR matrix size!!
#define MATRIX_PANEL_X         3
#define MATRIX_PANEL_Y         2
#define MATRIX_PANEL_LEDS_X    8
#define MATRIX_PANEL_LEDS_Y    8
#define MATRIX_PANEL_LEDS      (MATRIX_PANEL_LEDS_X*MATRIX_PANEL_LEDS_Y)
#define MATRIX_LEDS_X          (MATRIX_PANEL_X*MATRIX_PANEL_LEDS_X)
#define MATRIX_LEDS_Y          (MATRIX_PANEL_Y*MATRIX_PANEL_LEDS_Y)
#define MATRIX_LEDS            (MATRIX_LEDS_X*MATRIX_LEDS_Y)

#define CMD_NEW_DATA 0x01

volatile uint8_t framebuffer[MATRIX_LEDS * 3];

// framebuffer-related variables used during UART interrupt 
static uint8_t volatile*ptr;
static unsigned int pos = 0;
// Flag to allow loop() to display current framebuffer
volatile bool framebuffer_ready = false;

// Data pin that led data will be written out over
//#define DATA_PIN 7
// Clock pin only needed for SPI based chipsets when not using hardware SPI
//#define CLOCK_PIN 8

CRGB leds[MATRIX_LEDS];

void setup() {
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);

  // Uncomment one of the following lines for your leds arrangement.
  // Don't forget to change RGB order depending on your pixel modules
  // FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<TM1804, DATA_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<TM1809, DATA_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<UCS1903, DATA_PIN, RGB>(leds, MATRIX_LEDS);
  FastLED.addLeds<WS2801, BRG>(leds, MATRIX_LEDS);
  // FastLED.addLeds<SM16716, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<LPD8806, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(leds, MATRIX_LEDS);
  // FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, MATRIX_LEDS);
      
  //Disable global interrupts
  cli();
 
  // UART Initialisation
  UCSR0A |= _BV(U2X0);
  UCSR0B |= _BV(RXEN0)  | _BV(TXEN0) | _BV(RXCIE0);
  // Set baudrate to 1 Mbps
  UBRR0H = 0;
  UBRR0L = 1;
  // Enable UART interrupt
  UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00); 
  
  // Reset ptr to framebuffer address
  ptr=framebuffer;

  // This display a 4 fade-in bars of colors (White, red, green, blue)
  // Please note that test pattern only works for a specific leds order:
  //    snake from top left to bottom right on each panel, the same order for panel within screen)
  // and a specific leds color order:
  //   Green, Red, Blue
  display_test_pattern();

  //Enable global interrupts
  sei();
}

void display_test_pattern()
{
  for (uint8_t y=0;y<MATRIX_LEDS_Y;y++)
  {
    for (uint8_t x=0;x<MATRIX_LEDS_X;x++) {
      uint16_t id;

      uint16_t x_panel_id = (y%2)
                       ?((MATRIX_PANEL_LEDS_X-1)-(x%MATRIX_PANEL_LEDS_X))
                       :(x%MATRIX_PANEL_LEDS_X);
      uint16_t y_panel_id = (y%MATRIX_PANEL_LEDS_Y);
      uint16_t panel_id = ((y/MATRIX_PANEL_LEDS_Y)%2)
                     ?((MATRIX_PANEL_X-1)-(x/MATRIX_PANEL_LEDS_X)) + ((y/MATRIX_PANEL_LEDS_Y) * MATRIX_PANEL_X)
                     :(x/MATRIX_PANEL_LEDS_X) + ((y/MATRIX_PANEL_LEDS_Y) * MATRIX_PANEL_X);
      
      id = x_panel_id + (y_panel_id*MATRIX_PANEL_LEDS_X) + (panel_id*MATRIX_PANEL_LEDS);

      //const int id = y;
      const uint16_t r_id = (id * 3) + 1;
      const uint16_t g_id = (id * 3) + 2;
      const uint16_t b_id = (id * 3) + 0;

      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      
      if (x<3) {
        // White
        r = y << 4;
        g = y << 4;
        b = y << 4;
      } else if (x<7) {
      } else if (x<10) {
        // Red
        r = y << 4;
      } else if (x<14) {
      } else if (x<17) {
        // Green
        g = y << 4;
      } else if (x<21) {
      } else if (x<24) {
        // Blue
        b = y << 4;
      }
      
      framebuffer[r_id] = r;
      framebuffer[g_id] = g;
      framebuffer[b_id] = b;
    }
  }
  framebuffer_ready=true;
}

void framerate_test()
{
  //Framerate test
  //Count the number of flashs on a 10sec period, multiply by 10 and you have your framerate
  //Mesured arond 75fps with ArduinoMega2560 connected to 384 ws2801 pixels using fastSPI 
  static int cpt=0;
  if ((cpt%100)==0)
  {
    for (int i=0; i<MATRIX_LEDS; i++)
    {
      framebuffer[i*3+0]=0xff;
      framebuffer[i*3+1]=0xff;
      framebuffer[i*3+2]=0xff;
    }
    cpt=0;
  }
  else
  {
    for (int i=0; i<MATRIX_LEDS; i++)
    {
      framebuffer[i*3+0]=0x00;
      framebuffer[i*3+1]=0x00;
      framebuffer[i*3+2]=0x00;
    }
  }
  cpt++;
  
  framebuffer_ready = true;
}

void loop() 
{
  //framerate_test();

  if (framebuffer_ready) {
    framebuffer_ready=false;
    shift_out_data();
  }
}

static uint8_t escape_command_step = 0;
static bool gamma_correction = true;

ISR(USART0_RX_vect) 
{
  unsigned char b;
  
  // Retreive UART byte
  b=UDR0;
  
  if (escape_command_step>3) {
    switch (b)
    {
      case 'g': // Toggle gamma correction
        gamma_correction = !gamma_correction;
      break;
    }
    escape_command_step = 0;
  } else {
    if(!framebuffer_ready) {
      if (b == CMD_NEW_DATA)  {
        pos=0; ptr=framebuffer; return;
        escape_command_step++;
      } else {
        escape_command_step = 0;
      }
      if (pos == (MATRIX_LEDS*3)) {} else {*ptr=b; ptr++; pos++;}  
      if (pos == ((MATRIX_LEDS*3)-1)) {framebuffer_ready = true;}
    }
  }
}

void shift_out_data()
{

  for (int i=0; i<MATRIX_LEDS; i++)
  {
    const uint16_t id = i*3;
    uint8_t r, g, b;
    if(gamma_correction) {
      r = gammaRed[(framebuffer[(id)+1])>>2];
      g = gammaGreen[(framebuffer[(id)+2])>>2];
      b = gammaBlue[(framebuffer[(id)+0])>>2];
    } else {
      r = framebuffer[(id)+1];
      g = framebuffer[(id)+2];
      b = framebuffer[(id)+0];
    }

    leds[i].r = r;
    leds[i].g = g;
    leds[i].b = b;
  }
  FastLED.show();
  delayMicroseconds(1200); //Latch Data
}

