#include <Arduino.h>
#include <malloc.h>
#include <SPI.h>
#include <TNTSChar.h>
#include <TNTSCAnsi.h>
#include <PS2Keyboard.h>

#include <libmaple/libmaple.h>
#include <libmaple/gpio.h>
#include <libmaple/timer.h>
#include <libmaple/usart.h>

#include "consoleio.h"
#include "debugmsg.h"
#include "mini-printf.h"
#include "editor.h"
#include "flashstruct.h"

#define RTS_OUTPUT PB12
#define CTS_INPUT PA8

#ifdef SERIAL_USB
#define SerPort Serial1
#else
#define SerPort Serial
#endif

extern PS2Keyboard kbd;
extern TNTSChar_class TNTSChar;

#define MAGIC_DEFAULT 0xA901BEEF

#define SERFIFOSIZE 64

struct serfifo
{
  unsigned char buf[SERFIFOSIZE];
  int fifohead;
  int fifotail;
};

struct serfifo inputfifo;
struct serfifo outputfifo;

static void initserfifo(struct serfifo *fifo)
{
  fifo->fifohead = fifo->fifotail = 0;
}


static int fifoused(struct serfifo *fifo)
{
  if (fifo->fifotail > fifo->fifohead)
      return (SERFIFOSIZE + fifo->fifohead - fifo->fifotail);
  else
      return (fifo->fifohead - fifo->fifotail); 
}

static int getserfifo(struct serfifo *fifo)
{ 
  int ch;
  int newpos;
  if (fifo->fifotail == fifo->fifohead)
    return -1;
  ch = fifo->buf[fifo->fifotail];
  newpos = fifo->fifotail+1;
  if (newpos >= SERFIFOSIZE) newpos = 0;
  fifo->fifotail = newpos;
  return ch;
}

static void putserfifo(struct serfifo *fifo, int ch)
{
  int newpos = fifo->fifohead + 1;
  if (newpos >= SERFIFOSIZE) newpos = 0; 
  if (newpos == fifo->fifotail)
    return;
  fifo->buf[fifo->fifohead] = ch;
  fifo->fifohead = newpos;
}

const char intro[] =
  "One Chip Terminal by D.L. Marks\r\n"
  "zLib license for software, CC-BY-SA 4.0 for hardware\r\n"
  "\r\n\r\nPress ALT-H for help\r\n\r\n";

typedef struct _serial_state
{
  uint32_t magic;
  uint32_t baud;
  uint8_t stopbits;
  uint8_t parity;
  uint8_t localecho;
  uint8_t rtscts;
  uint8_t crlf;
} serial_state;

serial_state ser_state;

#define CONFIGURATION_STORAGE_ADDRESS 0x0800F800u

void configuration_read_storage(void)
{
  void *vp[1];
  int b[1];

  vp[0] = (void *)&ser_state;
  b[0] = sizeof(ser_state);
  readflashstruct((void *)CONFIGURATION_STORAGE_ADDRESS, 1, vp, b);
}

void configuration_write_storage(void)
{
  void *vp[1];
  int b[1];

  vp[0] = (void *)&ser_state;
  b[0] = sizeof(ser_state);
  writeflashstruct((void *)CONFIGURATION_STORAGE_ADDRESS, 1, vp, b);
}

void setup_serial_port(struct _serial_state *ser_state)
{
  uint32_t parity = (ser_state->parity == 'E') || (ser_state->parity == 'O');
  usart_set_baud_rate(USART1, USART_USE_PCLK, ser_state->baud);
  USART1->regs->CR1  = (USART1->regs->CR1 & 0B1110000111111111) | (parity << 10) | ((uint32_t)(ser_state->parity == 'O') << 9);
  USART1->regs->CR2  = (USART1->regs->CR2 & 0B1100111111111111) | ((uint32_t)(ser_state->stopbits == 2) << 13);
  initserfifo(&inputfifo);
  initserfifo(&outputfifo);
}

void setup() {
  configuration_read_storage();
  if (ser_state.magic != MAGIC_DEFAULT)
  {
    ser_state.magic = MAGIC_DEFAULT;
    ser_state.baud = 19200;
    ser_state.stopbits = 1;
    ser_state.parity = 'N';
    ser_state.localecho = 0;
    ser_state.rtscts = 0;
    ser_state.crlf = 0;
  }
  SerPort.begin(19200);
  pinMode(PC13, OUTPUT);
  pinMode(CTS_INPUT,INPUT_PULLUP);
  pinMode(RTS_OUTPUT,OUTPUT);
  digitalWrite(RTS_OUTPUT,HIGH);
  setup_serial_port(&ser_state);
  nvic_irq_set_priority(NVIC_USART1,1);
  console_init();
  console_puts(intro);
}

extern "C" {
void flicker(void)
{
  digitalWrite(PC13,!digitalRead(PC13));
}
}

void write_message(uint8_t row, uint8_t col, const char *c)
{
  uint16_t cols = TNTSChar.cols();
  uint8_t *screendata = TNTSChar.screendata();
  uint8_t *ch = screendata + (cols * row) + col;
  while (*c != 0) *ch++ = *c++;
}

void do_set_baud_rate()
{
  int ch, baud = -1;
  write_message(0,0,"Baud Rate: 0=300,1=1200,2=2400,3=4800,4=9600,5=19200");
  write_message(1,11,"6=38400,7=57600,8=115200,9=230400");
  while ((ch=console_inchar()) < 0);
  switch (ch)
  {
    case '0': baud = 300; break;
    case '1': baud = 1200; break;
    case '2': baud = 2400; break;
    case '3': baud = 4800; break;
    case '4': baud = 9600; break;
    case '5': baud = 19200; break;
    case '6': baud = 38400; break;
    case '7': baud = 57600; break;
    case '8': baud = 115200; break;
    case '9': baud = 230400; break;
  }
  if (baud >= 0)
  {
    char s[20];
    write_message(2,0,"Baud rate set to ");
    myltoa(s, baud);
    write_message(2,18, s);
    ser_state.baud = baud;
    setup_serial_port(&ser_state);
  } else
    write_message(2,0,"Baud rate not selected");
  delay(1000); 
}

#define CLEAR_ROWS 6

void write_pause_message(const char *c)
{
  write_message(0,0,c);
  delay(1000);
}

void do_set_no_parity(void)
{
  ser_state.parity = 'N';
  setup_serial_port(&ser_state);
  write_pause_message("No parity set");
}

void do_set_even_parity(void)
{
  ser_state.parity = 'E';
  setup_serial_port(&ser_state);
  write_pause_message("Even parity set");
}

void do_set_odd_parity(void)
{
  ser_state.parity = 'O';
  setup_serial_port(&ser_state);
  write_pause_message("Odd parity set");
}

void do_set_one_stop_bit(void)
{
  ser_state.stopbits = 1;
  setup_serial_port(&ser_state);
  write_pause_message("One stop bit set");
}

void do_set_two_stop_bits(void)
{
  ser_state.stopbits = 2;
  setup_serial_port(&ser_state);
  write_pause_message("Two stop bits set");
}

void do_local_echo(void)
{
  ser_state.localecho = !ser_state.localecho;
  write_pause_message(ser_state.localecho ? "Local echo on" : "Local echo off");  
}

void do_flow_control(void)
{
  ser_state.rtscts = !ser_state.rtscts;
  write_pause_message(ser_state.rtscts ? "Hardware flow control on" : "Hardware flow control off");  
}

void do_crlf(uint8_t ch, const char *c)
{
  ser_state.crlf = ch;
  write_message(0,16,c);
  write_pause_message("CRLF sequence: ");
}

void do_write_storage(void)
{
  configuration_write_storage();
  write_pause_message("Configuration written to flash");  
}

void do_show_help(void)
{
  write_message(0,0,"Alt-H Help, Alt-B Baud Rate, Alt-N No Parity, Alt-E Even Parity");
  write_message(1,0,"Alt-O Odd Parity, Alt-1 One Stop Bit, Alt-2 Two Stop Bits,");
  write_message(2,0,"Alt-L Toggle Local Echo, Alt-F Toggle Hardware Flow Control,");
  write_message(3,0,"Alt-J LF is CRLF, Alt-M CR is CRLF, Alt-R none is CRLF,");
  write_message(4,0,"Alt-W Write Configuration To Flash");  
  while (console_inchar() < 0);
}

void do_altkey(void)
{
  uint16_t rows = TNTSChar.rows();
  uint16_t cols = TNTSChar.cols();
  uint16_t tchars = rows*cols;
  uint8_t *screendata = TNTSChar.screendata(), *save_screen;
  volatile uint16_t *xpos, *ypos;
  uint16_t save_xpos, save_ypos;

  TNTSChar.get_cursor_ptr(&xpos,&ypos);
  if ((save_screen = (uint8_t *)malloc(tchars)) == NULL) return;
  memcpy(save_screen, screendata, tchars);
  save_xpos = *xpos;
  save_ypos = *ypos;

  while (kbd.altkey())
  {
    int ch = console_inchar();
    if (ch >= 0)
    {
      memset(screendata,' ', cols*(CLEAR_ROWS-1));
      memset(screendata+(cols*(CLEAR_ROWS-1)),' ' | 0x80,cols);  
      ch = toupper(ch);
      switch (ch)
      {
         case 'H':  do_show_help();
                    break;
         case 'B' : do_set_baud_rate();
                    break;
         case 'N':  do_set_no_parity();
                    break;
         case 'E':  do_set_even_parity();
                    break;
         case 'O':  do_set_odd_parity();
                    break;
         case '1':  do_set_one_stop_bit();
                    break;
         case '2':  do_set_two_stop_bits();
                    break;
         case 'L':  do_local_echo();
                    break;
         case 'F':  do_flow_control();
                    break;
         case 'J':  do_crlf('\n', "LF");
                    break;
         case 'M':  do_crlf('\r', "CR");
                    break;
         case 'R':  do_crlf('\000', "none");
                    break;
         case 'W':  do_write_storage();
                    break;
      }
    }
  }
  
  *xpos = save_xpos;
  *ypos = save_ypos;
  memcpy(screendata, save_screen, tchars);
  free(save_screen);
}


void loop()
{
  int ch;
  if (kbd.altkey())
    do_altkey();
  while ((ch = console_inchar()) >= 0)
    putserfifo(&inputfifo, ch);
  while ((!ser_state.rtscts) || (digitalRead(CTS_INPUT)))
  {
    if ((ch = getserfifo(&inputfifo)) >= 0)
    {
      Serial.write(ch);
      if (ser_state.localecho) 
      {
        if ((ch == ser_state.crlf) && (ser_state.crlf))
        {
          console_putch('\r');
          console_putch('\n');
        } else console_putch(ch);
      }
    } else break;
  }
  while ((ch = SerPort.read()) >= 0)
  {
    putserfifo(&outputfifo, ch);
    if ((ser_state.rtscts) && (fifoused(&outputfifo) > (SERFIFOSIZE / 2)))
       digitalWrite(RTS_OUTPUT,LOW);
  }
  while (((ch = getserfifo(&outputfifo)) >= 0))
  {
     if ((ch == ser_state.crlf) && (ser_state.crlf))
     {
       console_putch('\r');
       console_putch('\n');
     } else console_putch(ch);
     if ((ser_state.rtscts) && (fifoused(&outputfifo) <= (SERFIFOSIZE / 2)))
       digitalWrite(RTS_OUTPUT,HIGH);
  }

}
