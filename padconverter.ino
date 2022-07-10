// Multi-way pad converter
// for Arduino ATmega328p 5V 16MHz 
// by takuya matsubara
// (from DualShock2 to FC/SFC/MD/PCE/X68K/MSX)

#define USE_LCD   0      // LCD(AQM1602XA):0=not use / 1=use
#define USE_OLED  1      // OLED(SSD1306) :0=not use / 1=use
#define USE_CYBERSTICK 0 // cyberstick    :0=not use / 1=use
#define USE_SERIAL 0     // serial port   :0=not use / 1=use
#define EEPROMADDR 0     // address of mode number

#if USE_LCD+USE_OLED
#include <Wire.h>
#endif
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "8x8font.h"      // font data

#define MENU_MAX  18
#define MENU_LEN  (16+1)
PROGMEM const char str_mode[MENU_MAX][MENU_LEN]={
  "PAD-TEST        ",
  "MDRIVE  -3BUTTON",
  "MDRIVE  -6BUTTON",
  "MDRIVE  -ANALOG ",
  "MDRIVE  -MOUSE  ",
  "PCENG   -DIGITAL",
  "PCENG   -ANALOG ",
  "PCENG   -MOUSE  ",
  "FAMI    -DIGITAL",
  "FAMI    -PADDLE ",
  "FAMI    -CRAZYCL",
  "SFC     -DIGITAL",
  "SFC     -MOUSE  ",
  "X68K    -DIGITAL",
  "X68K    -ANALOG ",
  "MSX     -PADDLE ",
  "MSX-PADDLE-VOL  ",
  "FAMI-PADDLE-VS. "
};

PROGMEM const char str_howto[]="M E N U";
PROGMEM const char str_select[]="SELECT";
PROGMEM const char str_saved[]="SAVED";
PROGMEM const char str_ok[]="OK";
PROGMEM const char str_running[]="RUNNING";
PROGMEM const char str_cyberstick[] ="CYBERSTICK";
PROGMEM const char str_adctest[]  ="ADC-TEST";
PROGMEM const char str_padtest[]  ="PAD-TEST";
PROGMEM const char str_timertest[]="TIMER-TEST";
PROGMEM const char str_reqtest[]  ="REQ-TEST";

#define CPUHZ 16000000  // CPU frequency[Hz]
#define PRS1  1         // timer1 prescaler
#define PRS2  128       // timer2 prescaler
#define T1HZ  (CPUHZ/PRS1)  // timer1 freq.[Hz]
#define T2HZ  (CPUHZ/PRS2)  // timer2 freq.[Hz]

#define TCNT2_1MSEC     (unsigned int)(T2HZ/1000)
#define TCNT2_1100USEC  (unsigned int)(T2HZ/909)
#define TIMERTEMP      TCNT2
#define TIMERLIMIT     TCNT2_1MSEC

#define TIMER_250NSEC  (unsigned int)(0x10000-(T1HZ/4000000))
#define TIMER_500NSEC  (unsigned int)(0x10000-(T1HZ/2000000))
#define TIMER_1USEC    (unsigned int)(0x10000-(T1HZ/1000000))
#define TIMER_2USEC    (unsigned int)(0x10000-(T1HZ/500000))
#define TIMER_4USEC    (unsigned int)(0x10000-(T1HZ/250000))
#define TIMER_6USEC    (unsigned int)(0x10000-(T1HZ/166666))
#define TIMER_8USEC    (unsigned int)(0x10000-(T1HZ/125000))
#define TIMER_10USEC   (unsigned int)(0x10000-(T1HZ/100000))
#define TIMER_12USEC   (unsigned int)(0x10000-(T1HZ/83333))
#define TIMER_22USEC   (unsigned int)(0x10000-(T1HZ/45454))
#define TIMER_26USEC   (unsigned int)(0x10000-(T1HZ/38461))
#define TIMER_40USEC   (unsigned int)(0x10000-(T1HZ/25000))
#define TIMER_50USEC   (unsigned int)(0x10000-(T1HZ/20000))
#define TIMER_64USEC   (unsigned int)(0x10000-(T1HZ/15625))
#define TIMER_74USEC   (unsigned int)(0x10000-(T1HZ/13513))
#define TIMER_88USEC   (unsigned int)(0x10000-(T1HZ/11363))
#define TIMER_100USEC  (unsigned int)(0x10000-(T1HZ/10000))
#define TIMER_1MSEC    (unsigned int)(0x10000-(T1HZ/1000))

// timer initialize
void timer_init(void)
{
  // timer1 prescaler
  TCCR1A = 0;
  TCCR1B = 1;
  // 0: No clock source (Timer/Counter stopped).
  // 1: clock /1 (No prescaling)
  // 2: clock /8 (From prescaler)
  // 3: clock /64 (From prescaler)
  // 4: clock /256 (From prescaler)
  // 5: clock /1024 (From prescaler)

  // timer2 prescaler
  TCCR2A = 0;
  TCCR2B = 5;
  // 1: clock /(No prescaling)
  // 2: clock /8 (From prescaler)
  // 3: clock /32 (From prescaler)
  // 4: clock /64 (From prescaler)
  // 5: clock /128 (From prescaler)
  // 6: clock /256 (From prescaler)
  // 7: clock /1024 (From prescaler)
}

//----- wait micro second
void timer_uswait(unsigned int limitcnt)
{
  TCNT1 = limitcnt;
  TIFR1 |= (1 << TOV1);  // clear TOV1
  while(!(TIFR1 & (1 << TOV1)));
}

//----- wait mili second
void timer_delay(int milisec)
{
  while(milisec--){
    TCNT2 = 0;
    while(TCNT2 < TCNT2_1MSEC);
  }
}

//---- ADC
//---- AD init
void adc_init(void)
{
//#define ADCLOCK  0 // clock 1/2 
//#define ADCLOCK  1 // clock 1/2 
#define ADCLOCK  2 // clock 1/4 
//#define ADCLOCK  3 // clock 1/8 
//#define ADCLOCK  4 // clock 1/16 
//#define ADCLOCK  5 // clock 1/32 
//#define ADCLOCK  6 // clock 1/64 
//#define ADCLOCK  7 // clock 1/128     
  DDRC &= ~((1<<1)|(1<<0));   // input
  PORTC &= ~((1<<1)|(1<<0));  // no pull up
  ADCSRA = (1<<ADEN)|(0<<ADIE)|(1<<ADIF) | ADCLOCK;
// ADEN: ADC enable
// ADIF: ADC interrupt flag
// ADIE: ADC interrupt enable 1=enable/0=disable
  ADCSRB = 0;
}

//---- AD start
unsigned char adc_get(char adchan)
{
  int tempcnt;
  ADMUX = (1<<REFS0) | adchan; // select AD channel
  timer_uswait(TIMER_10USEC);
  ADCSRA |= (1<<ADSC);  // AD start
  while((ADCSRA & (1<<ADIF))==0){
  }
  tempcnt = (int)ADCL; 
  tempcnt += ((int)ADCH) << 8;
  return((unsigned char)(tempcnt >> 2));
}

//----VRAM
#define VRAMW 128
#define VRAMH 64
#define VRAMSIZE (VRAMW*VRAMH/8)
unsigned char vram[VRAMSIZE];
#define TEXTZOOMX 2
#define TEXTZOOMY 2
#define FONTW (TEXTZOOMX*8)
#define FONTH (TEXTZOOMY*8)

//---- vram all clear
void vram_clear(void)
{
  int vsize;
  unsigned char *p;
  p = &vram[0];
  vsize = VRAMSIZE;
  while(vsize--){
    *p++ = 0;
  }
}

//---- vram get
char vram_pget(unsigned char x,unsigned char y){
  int adr;
  unsigned char mask;
  if((x>=VRAMW)||(y>=VRAMH))return(0);
  adr = x+(VRAMW*(y/8));
  mask = (1<<(y % 8));
  if(vram[adr] & mask){
    return(1);
  }else{
    return(0);
  }
}

//---- vram put 
void vram_pset(unsigned char x,unsigned char y,char color){
  int adr;
  unsigned char mask;
  if((x>=VRAMW)||(y>=VRAMH))return;
  adr = x+(VRAMW*(y/8));
  mask = (1<<(y % 8));
  if(color==1){
    vram[adr] |= mask;
  }else if(color==0){
    vram[adr] &= ~mask;
  }else{
    vram[adr] ^= mask;
  }
}

//---- put chara( x,y,ch)
void vram_putch(unsigned char textx,unsigned char texty, unsigned char ch)
{
  char color;
  unsigned char i,j,bitdata;
  unsigned char x,y,xd,yd;
  PGM_P p;

  if(ch < 0x20)return;
  ch -= 0x20;
  p = (PGM_P)font;
  p += ((int)ch * 8);

  for(i=0 ;i<8 ;i++) {
    bitdata = pgm_read_byte(p++);
    for(j=0; j<8; j++){
      if(bitdata & ((1<<7)>>j)){
        color=1;
      }else{
        color = 0;
      }
      x = textx+(j*TEXTZOOMX);
      y = texty+(i*TEXTZOOMY);
      for(xd=0;xd<TEXTZOOMX;xd++){
        for(yd=0;yd<TEXTZOOMY;yd++){
          vram_pset(x+xd,y+yd,color);
        }
      }
    }
  }
}

//---- print HEX
void vram_puthex(unsigned char x,unsigned char y,char num)
{
  char temp;
  temp = ((num >> 4) & 0x0f)+'0';
  if(temp > '9')temp += ('A'-('9'+1));
  vram_putch(x,y,temp);
  x += FONTW;
  
  temp = (num & 0x0f)+'0';
  if(temp > '9')temp += ('A'-('9'+1));
  vram_putch(x,y,temp);
}

//---- print string on PROGMEM
void vram_putstr_pgm(unsigned char x,unsigned char y,PGM_P p)
{
  char temp;
  while(1){
    temp = pgm_read_byte(p++);
    if(temp==0)break;    
    vram_putch(x,y,temp);
    x += FONTW;
    if(x+(FONTW-1) >= VRAMW){
      x=0;
      y+=FONTH;
    }
  }
}

//---- print string
void vram_putstr(unsigned char x,unsigned char y,char *p)
{
  while(*p!=0){
    vram_putch(x,y,*p++);
    x += FONTW;
    if(x+(FONTW-1) >= VRAMW){
      x=0;
      y+=FONTH;
    }
  }
}

//---- box fill(X1,Y1,X2,Y2,color)
void vram_fill(int x1 ,int y1 ,int x2 ,int y2 ,char color)
{
  int x,y;

  for(y=y1; y<=y2; y++){
    for(x=x1; x<=x2; x++){
      vram_pset(x, y ,color); //ドット描画
    }
  }
}

//---- draw line(X1,Y1,X2,Y2,color)
void vram_line(int x1 ,int y1 ,int x2 ,int y2 ,char color)
{
  int xd;    // X2-X1座標の距離
  int yd;    // Y2-Y1座標の距離
  int xs=1;  // X方向の1pixel移動量
  int ys=1;  // Y方向の1pixel移動量
  int e;

  xd = x2 - x1;  // X2-X1座標の距離
  if(xd < 0){
    xd = -xd;  // X2-X1座標の絶対値
    xs = -1;    // X方向の1pixel移動量
  }
  yd = y2 - y1;  // Y2-Y1座標の距離
  if(yd < 0){
    yd = -yd;  // Y2-Y1座標の絶対値
    ys = -1;    // Y方向の1pixel移動量
  }
  vram_pset(x1, y1 ,color); //ドット描画
  e = 0;
  if( yd < xd ) {
    while( x1 != x2) {
      x1 += xs;
      e += (2 * yd);
      if(e >= xd) {
        y1 += ys;
        e -= (2 * xd);
      }
      vram_pset(x1, y1 ,color); //ドット描画
    }
  }else{
    while( y1 != y2) {
      y1 += ys;
      e += (2 * xd);
      if(e >= yd) {
        x1 += xs;
        e -= (2 * yd);
      }
      vram_pset(x1, y1 ,color); //ドット描画
    }
  }
}

// scroll
void vram_scroll(char x1,char y1){
  unsigned char x,y;
  char color;
  for(y=0;y<VRAMH;y++){
    for(x=0;x<VRAMW;x++){
      color = vram_pget(x+x1, y+y1);
      vram_pset(x,y,color);
    }
  }
}

//----OLED(SD1306)
#if USE_OLED

#define OLEDADDR 0x78 // SSD1306 slave address

// OLED(SSD1306)
#define SET_CONTRAST_CONTROL  0x81
#define SET_CHARGE_PUMP       0x8D 
#define SET_ADDRESSING_MODE   0x20
#define SET_DISPLAY_STARTLINE 0x40
#define SET_SEGMENT_REMAP     0xA1
#define SET_ENTIRE_DISPLAY    0xA4  
#define SET_DISPLAY_NORMAL    0xA6
#define SET_MULTIPLEX_RATIO   0xA8
#define SET_DISPLAY_ON        0xAF
#define SET_COM_OUTPUT_SCAN   0xC8
#define SET_DISPLAY_OFFSET    0xD3
#define SET_OSCILLATOR_FREQ   0xD5
#define SET_COM_PINS_HARDWARE 0xDA
#define SET_COLUMN_ADDRESS    0x21   //start,end
#define SET_PAGE_ADDRESS      0x22   //start,end

// SSD1306:
void oled_command(unsigned char data)
{
  Wire.beginTransmission(OLEDADDR >> 1);
  Wire.write(0b10000000); // control(single + command)
  Wire.write(data);             
  Wire.endTransmission();
}

//SSD1306:
void oled_command2(unsigned char data1,unsigned char data2)
{
  Wire.beginTransmission(OLEDADDR >> 1);
  Wire.write(0b00000000); // control(Continuation + command)
  Wire.write(data1);             
  Wire.write(data2);             
  Wire.endTransmission();
}

// SSD1306: initialize
void oled_init(void)
{
  Wire.setClock(400000);  
  timer_delay(50);
  oled_command2(SET_MULTIPLEX_RATIO , 0x3F);  // multiplex ratio
  oled_command2(SET_DISPLAY_OFFSET,0);
  oled_command(SET_DISPLAY_STARTLINE);  // starting address of display RAM
  oled_command(SET_COM_OUTPUT_SCAN);
  oled_command(SET_SEGMENT_REMAP);  // column address and the segment driver
  oled_command2(SET_COM_PINS_HARDWARE, 0x12);
  oled_command2(SET_CONTRAST_CONTROL , 0x80);
  oled_command(SET_ENTIRE_DISPLAY); // entire display “ON” stage
  oled_command(SET_DISPLAY_NORMAL);
  oled_command2(SET_OSCILLATOR_FREQ  , 0x80);  
  oled_command2(SET_ADDRESSING_MODE  ,0); 
  oled_command2(SET_CHARGE_PUMP , 0x14);  // Enable charge pump
  oled_command(SET_DISPLAY_ON);
  timer_delay(1);
  vram_line(0,0,VRAMW-1,VRAMH-1,1);
  vram_line(VRAMW-1,0,0,VRAMH-1,1);
  oled_redraw();
}

// SSD1306:screen update
void oled_redraw(void){
  int i,addr;

  Wire.beginTransmission(OLEDADDR >> 1);
  Wire.write(0b00000000); // control(Continuation + command)
  Wire.write(SET_COLUMN_ADDRESS);
  Wire.write(0);       // start column
  Wire.write(VRAMW-1); // end column
  Wire.write(SET_PAGE_ADDRESS);
  Wire.write(0);           // start page
  Wire.write((VRAMH/8)-1); // end page
  Wire.endTransmission();

  addr =0;
  while(addr < VRAMSIZE){  
    Wire.beginTransmission(OLEDADDR >> 1);
    Wire.write(0b01000000); //control(Continuation + data)
    for(i=0; i<8; i++){  
     Wire.write(vram[addr++]);
    }
    Wire.endTransmission();
  }
}
#endif // USE_OLED

#if USE_LCD
// AQM1602XA: initialize
void lcd_init(void)
{
  #define LCDCNTR 12   //LCD contrast(0-63)
  timer_delay(50);
  lcd_cmdwrite(0x38);  //function set(normal instruction)
  lcd_cmdwrite(0x39);  //function set(extension instruction)
  lcd_cmdwrite(0x14);  //internal osc freq.
  lcd_cmdwrite(0x70+(LCDCNTR & 0xF)); //contrast set
  lcd_cmdwrite(0x54+(LCDCNTR >> 4));  //power/icon/contrast control
  lcd_cmdwrite(0x6C);  //follower control
  lcd_cmdwrite(0x38);  //function set(normal instruction)
  lcd_cmdwrite(0x01);  //clear display
  lcd_cmdwrite(0x08+4+2+1);  //display control(display=on/cursor=on/blink=on)
}

//AQM1602XA:write
void lcd_write(unsigned char temp1,unsigned char temp2)
{
  #define LCDADDR  0x7C  //slave address
  Wire.beginTransmission(LCDADDR >> 1);
  Wire.write(temp1);
  Wire.write(temp2);
  Wire.endTransmission();
}

//AQM1602XA:locate
void lcd_locate(char x,char y)
{
  unsigned char cmd=0x80;
  if((x>=16)||(y>=2))return;
  cmd += x;
  cmd += (0x40*y);
  lcd_cmdwrite(cmd);
}

// AQM1602XA:write command
void lcd_cmdwrite(unsigned char command)
{
  lcd_write(0x00,command);
  timer_delay(1);
}

// AQM1602XA:write character
void lcd_datawrite(unsigned char data)
{
  lcd_write(0x40,data);
  timer_delay(1);
}

// AQM1602XA:clear
void lcd_cls(void)
{
  char i;
  lcd_locate(0,0);
  for(i=0;i<16;i++){
    lcd_datawrite(' ');
  }
  lcd_locate(0,1);
  for(i=0;i<16;i++){
    lcd_datawrite(' ');
  }
}

// AQM1602XA:print string on PROGMEM
void lcd_putstr_pgm(char x,char y,PGM_P p)
{
  char temp;
  lcd_locate(x,y);
  while(1){
    pgm_read_byte(p++);
    if(temp==0)break;
    lcd_datawrite(temp);
  }
}

// AQM1602XA:print string
void lcd_putstr(char x,char y,char *p)
{
  lcd_locate(x,y);
  while(*p!=0){
    lcd_datawrite(*p++);
  }
}

// AQM1602XA:decimal number 5chara.
void lcd_putnum(char x,char y,unsigned int num)
{
  unsigned int keta=10000;
  char temp;
  lcd_locate(x,y);
  while(keta>0){
    temp = (num/keta)% 10;
    lcd_datawrite('0'+temp);
    keta /= 10;
  }
}

//AQM1602XA:HEX number 2 character
void lcd_puthex(char x,char y,char num)
{
  char temp;
  lcd_locate(x,y);
  temp = ((num >> 4) & 0x0f)+'0';
  if(temp > '9')temp += ('A'-('9'+1));
  lcd_datawrite(temp);
  
  temp = (num & 0x0f)+'0';
  if(temp > '9')temp += ('A'-('9'+1));
  lcd_datawrite(temp);
}
#endif // USE_LCD

//----
#define MOUSE_SPEED  5  // mouse speed
#define MOUSE_THRESHOLD 24  // mouse threshold

// FC/SFC
#define FC_PORT PORTD     // Data/Clock port
#define FC_PIN  PIND      // Data/Clock port
#define FC_DDR  DDRD      // Data/Clock direction
#define FC_BITDAT  (1<<2) // P1 data mask
#define FC_BITDAT2 (1<<3) // P2 data mask
#define FC_BITCLK  (1<<4) // P1 clock mask
#define FC_BITCLK2 (1<<5) // P2 clock mask
#define FC_BITD2B4 (1<<6) // P2.bit4 data mask
#define FC_BITD2B3 (1<<7) // P2.bit3 data mask
#define UNTIL_FCCLK_H  while((FC_PIN&FC_BITCLK)==0)
#define UNTIL_FCCLK_L  while((FC_PIN&FC_BITCLK)!=0)
#define UNTIL_FCCLK2_H while((FC_PIN&FC_BITCLK2)==0)
#define UNTIL_FCCLK2_L while((FC_PIN&FC_BITCLK2)!=0)
#define FC_DAT_L   FC_PORT&=~FC_BITDAT
#define FC_DAT_H   FC_PORT|=FC_BITDAT
#define FC_DAT2_L  FC_PORT&=~FC_BITDAT2
#define FC_DAT2_H  FC_PORT|=FC_BITDAT2
#define FC_D2B3_L  FC_PORT&=~FC_BITD2B3
#define FC_D2B3_H  FC_PORT|=FC_BITD2B3
#define FC_D2B4_L  FC_PORT&=~FC_BITD2B4
#define FC_D2B4_H  FC_PORT|=FC_BITD2B4

#define LATCH_PORT PORTB  // LATCH port
#define LATCH_PIN PINB    // LATCH pin
#define LATCH_DDR DDRB    // LATCH direction
#define LATCH_BIT (1<<0)  // LATCH mask
#define UNTIL_LATCH_H while((LATCH_PIN&LATCH_BIT)==0)
#define UNTIL_LATCH_L while((LATCH_PIN&LATCH_BIT)!=0)

// MD/X68K/PCE
#define BITTXRX      0b11
#define MD_PORT      PORTD   // Data/LH/ACK port
#define MD_DDR       DDRD    // Data/LH/ACK direction
#define MD_PIN       PIND    // Data/LH/ACK Pin
#define MD_DAT_SHIFT 2       // Data bit shift count
#define MD_BITUP     (1<<2)  // up mask
#define MD_BITDOWN   (1<<3)  // down mask
#define MD_BITLEFT   (1<<4)  // left mask
#define MD_BITRIGHT  (1<<5)  // right mask
#define MD_BITA      (1<<6)  // A mask
#define MD_BITB      (1<<6)  // B mask
#define MD_BITC      (1<<7)  // C mask
#define MD_BITMODE   (1<<5)  // MODE mask
#define MD_BITX      (1<<4)  // X mask
#define MD_BITY      (1<<3)  // Y mask
#define MD_BITZ      (1<<2)  // Z mask
#define MD_BITSTART  (1<<7)  // Start mask
#define MD_BITLH     (1<<6)  // LH(XE-1AP)  mask
#define MD_BITACK    (1<<7)  // ACK(XE-1AP) mask
#define MD_BITLR   (MD_BITLEFT|MD_BITRIGHT)
#define MD_BITBC   (MD_BITB|MD_BITC)
#define MD_BITUDLR (MD_BITUP|MD_BITDOWN|MD_BITLEFT|MD_BITRIGHT)
#define MD_BITALL  (MD_BITBC|MD_BITUDLR)
#define MD_LH_H   MD_PORT|=MD_BITLH    // LH=H(XE-1AP)
#define MD_LH_L   MD_PORT&=~MD_BITLH   // LH=L(XE-1AP)
#define MD_LH_INVERT   MD_PORT^=MD_BITLH   // LH=L<->H(XE-1AP)
#define MD_ACK_H  MD_PORT|=MD_BITACK   // ACK=H(XE-1AP)
#define MD_ACK_L  MD_PORT&=~MD_BITACK  // ACK=L(XE-1AP)

#define X68K_BITUP    (1<<2)  // up mask
#define X68K_BITDOWN  (1<<3)  // down mask
#define X68K_BITLEFT  (1<<4)  // left mask
#define X68K_BITRIGHT (1<<5)  // right mask
#define X68K_BITA     (1<<6)  // a mask
#define X68K_BITB     (1<<7)  // b mask
#define X68K_BITALL   (0b111111<<2)  // ALL mask

#define REQ_PORT PORTB  // REQ port
#define REQ_PIN PINB    // REQ pin
#define REQ_DDR DDRB    // REQ direction
#define REQ_BIT (1<<0)  // REQ mask
#define UNTIL_REQ_H while((REQ_PIN&REQ_BIT)==0)
#define UNTIL_REQ_L while((REQ_PIN&REQ_BIT)!=0)

// PS Game pad
#define PAD_PORT  PORTB
#define PAD_DDR   DDRB
#define PAD_PIN   PINB

#define PAD_DATBIT  (1<<1)
#define PAD_CMDBIT  (1<<2)
#define PAD_SELBIT  (1<<3)
#define PAD_CLKBIT  (1<<4)

#define PAD_DISABLE PAD_PORT|=PAD_SELBIT
#define PAD_ENABLE  PAD_PORT&=~PAD_SELBIT
#define PAD_CLK_H PAD_PORT|=PAD_CLKBIT
#define PAD_CLK_L PAD_PORT&=~PAD_CLKBIT
#define PAD_CMD_H PAD_PORT|=PAD_CMDBIT
#define PAD_CMD_L PAD_PORT&=~PAD_CMDBIT

const unsigned char config_enter[]  ={0x01,0x43,0x00,0x01,0x00,0x00,0x00,0x00,0x00};
const unsigned char config_exit[]   ={0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const unsigned char get_querymodel[]={0x01,0x45,0x00,0x5a,0x5a,0x5a,0x5a,0x5a,0x5a};
const unsigned char set_analogmode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
const unsigned char get_paddata[]   ={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char padinput[30]; // PAD buffer

//               b7 |  b6 |  b5 |  b4 |  b3 |  b2 |  b1 |  b0
//             -----+-----+-----+-----+-----+-----+-----+------
// padinput[3]:LEFT | DOWN|RIGHT|UP   |START|   L3|   R3|SELECT
// padinput[4]:4KAKU| BATU| MARU|3KAKU|   R1|   L1|   R2|    L2
#define PAD_LEFT    ((padinput[3]&(1<<7))==0)
#define PAD_DOWN    ((padinput[3]&(1<<6))==0)
#define PAD_RIGHT   ((padinput[3]&(1<<5))==0)
#define PAD_UP      ((padinput[3]&(1<<4))==0)
#define PAD_START   ((padinput[3]&(1<<3))==0)
#define PAD_SELECT  ((padinput[3]&(1<<0))==0)
#define PAD_4KAKU   ((padinput[4]&(1<<7))==0)
#define PAD_BATU    ((padinput[4]&(1<<6))==0)
#define PAD_MARU    ((padinput[4]&(1<<5))==0)
#define PAD_3KAKU   ((padinput[4]&(1<<4))==0)
#define PAD_R1      ((padinput[4]&(1<<3))==0)
#define PAD_L1      ((padinput[4]&(1<<2))==0)
#define PAD_R2      ((padinput[4]&(1<<1))==0)
#define PAD_L2      ((padinput[4]&(1<<0))==0)

#define PAD_RX padinput[5]
#define PAD_RY padinput[6]
#define PAD_LX padinput[7]
#define PAD_LY padinput[8]

//------ PS pad init
void pad_init(void)
{
  PAD_DDR |= (PAD_CLKBIT + PAD_SELBIT + PAD_CMDBIT);    // direction output
  PAD_PORT |= (PAD_CLKBIT + PAD_SELBIT + PAD_CMDBIT);   // output High
  PAD_DDR &= ~PAD_DATBIT;   // direction input
  PAD_PORT &= ~PAD_DATBIT;  // no pull up

  padinput[3] = 0xff;
  padinput[4] = 0xff;
  padinput[5] = 0x80;
  padinput[6] = 0x80;
  padinput[7] = 0x80;
  padinput[8] = 0x80;

  timer_delay(60);
  pad_read(); // set analog mode
  timer_delay(20);
}

//------ (PS pad)send/recv 1Byte
unsigned char pad_send1byte(unsigned char sendbyte)
{
  unsigned char recvbyte,bitmask;

  recvbyte = 0;
  PAD_CLK_H;
  PAD_CMD_H;
  timer_uswait(TIMER_2USEC);

  bitmask = 0x01;
  while(bitmask){
    if(sendbyte & bitmask){
      PAD_CMD_H;
    }else{
      PAD_CMD_L;
    }
    PAD_CLK_L;
    timer_uswait(TIMER_2USEC);

    if(PAD_PIN & PAD_DATBIT) recvbyte |= bitmask;
    PAD_CLK_H;
    bitmask <<= 1;
  }
  PAD_CMD_H;
  return recvbyte;
}

//------ (PS pad)send/recv command
void pad_sendcommand(unsigned char *cmd,char sendcnt)
{
  unsigned char i;

  PAD_ENABLE;
  timer_uswait(TIMER_4USEC);
  for(i=0; i < sendcnt; i++){
    timer_uswait(TIMER_4USEC);
    padinput[i] = pad_send1byte(*(cmd+i));
  }
  PAD_DISABLE;
}

//------ (PS pad)change analog mode
void pad_analogmode(void)
{
  timer_uswait(TIMER_4USEC);
  pad_sendcommand((unsigned char *)config_enter,sizeof(config_enter));
  timer_uswait(TIMER_4USEC);
  pad_sendcommand((unsigned char *)config_enter,sizeof(config_enter));
  timer_uswait(TIMER_4USEC);

  //query
  pad_sendcommand((unsigned char *)get_querymodel,sizeof(get_querymodel));
  timer_uswait(TIMER_4USEC);
  pad_sendcommand((unsigned char *)get_querymodel,sizeof(get_querymodel));
  timer_uswait(TIMER_4USEC);
  
  //analog mode
  pad_sendcommand((unsigned char *)set_analogmode,sizeof(set_analogmode));
  timer_uswait(TIMER_4USEC);
  pad_sendcommand((unsigned char *)set_analogmode,sizeof(set_analogmode));
  timer_uswait(TIMER_4USEC);

  //exit
  pad_sendcommand((unsigned char *)config_exit,sizeof(config_exit));
  timer_uswait(TIMER_4USEC);
  pad_sendcommand((unsigned char *)config_exit,sizeof(config_exit));
}

//------ (PS pad)read data
void pad_read(void)
{
  pad_sendcommand((unsigned char *)get_paddata,sizeof(get_paddata));
  if(padinput[2] == 0x5a){
    if(padinput[1]==0x73){
      return; //正常
    }
    //analog modeではない場合、
    pad_analogmode(); // change analog mode
  }else{
    // fatal error
  }
}

//------ FC port initialize
void fcport_init(void)
{
  LATCH_DDR &= ~LATCH_BIT;  // direction input
  LATCH_PORT |= LATCH_BIT;  // pull up
  FC_DDR |=  (FC_BITDAT | FC_BITDAT2 | FC_BITD2B3 | FC_BITD2B4);  // output
  FC_DDR &= ~(FC_BITCLK | FC_BITCLK2);  // input
  FC_PORT |= (FC_BITDAT | FC_BITDAT2 | FC_BITD2B3 | FC_BITD2B4);  // high
  FC_PORT |= (FC_BITCLK | FC_BITCLK2);  // pullup
}

//------ MD port initialize
void mdport_init(void)
{
  REQ_DDR &= ~REQ_BIT;  // direction input
  REQ_PORT |= REQ_BIT;  // pull up
  MD_DDR |= MD_BITALL;  // direction output
  MD_PORT |= MD_BITALL; // output high
}

// SFC connector
//  ---------+------------+
// |(7)(6)(5)|(4)(3)(2)(1)|
//  ---------+------------+
// (1)VCC : 5V
// (2)CLK : clock (rising edge=change data)
// (3)P/S : latch (low=latch / high=loading)
// (4)DAT : data
// (7)GND : ground

// super famicom mouse
// DAT
//     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
//   +-----------------------+--+--+-----+--------+  +-----------------------+-----------------------+
//   | 0  0  0  0  0  0  0  0| R| L|SPEED| 0  0  0| 1|YD Y6 Y5 Y4 Y3 Y2 Y1 Y0|XD X6 X5 X4 X3 X2 X1 X0|
// --+                       +--+--+-----+        +--+-----------------------+-----------------------+---
void sfc_mouse(void)
{
  unsigned char bitcnt;
  unsigned long work;
  int movecnt,centerx,centery;

  fcport_init();
  cli();
  pad_read();
  centerx = PAD_LX;
  centery = PAD_LY;
  timer_delay(16);
  
  while(1){
    pad_read();
    work = 0x00010000;  //mouse data
    movecnt = (PAD_LX - centerx) >> MOUSE_SPEED;
    if(movecnt < 0){
      movecnt = -movecnt;
      work |= 0x80+movecnt; //left
    }else if(movecnt > 0){
      work |= movecnt; //right
    }
    movecnt = (PAD_LY - centery) >> MOUSE_SPEED;
    if(movecnt < 0){
      movecnt = -movecnt;
      work |= 0x8000+(movecnt<<8); //up
    }else if(movecnt > 0){
      work |= movecnt<<8; //down
    }

    if(PAD_BATU ){ // L button
      work |= 0x00400000;
    }
    if(PAD_MARU ){ // R button
      work |= 0x00800000;
    }
    UNTIL_LATCH_H;
    UNTIL_LATCH_L;

    bitcnt = 32;
    while(bitcnt--){
      if(work & 0x80000000){
        FC_DAT_L; // DAT=1
      }else{
        FC_DAT_H; // DAT=0
      }
      work <<= 1;
      UNTIL_FCCLK_L;
      UNTIL_FCCLK_H;
      if((LATCH_PIN & LATCH_BIT)!=0)break; // end of latch
    }
    FC_DAT_H;  // DAT=0
  }
}

// super famicom digital
//  DAT
//    15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
// --+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--
//   | B  Y SL ST UP DW LT RT  A  X  L  R|           |
//   +--+--+--+--+--+--+--+--+--+--+--+--+           +
#define SFC_BITB      (1<<15)
#define SFC_BITY      (1<<14)
#define SFC_BITSELECT (1<<13)
#define SFC_BITSTART  (1<<12)
#define SFC_BITUP     (1<<11)
#define SFC_BITDOWN   (1<<10)
#define SFC_BITLEFT   (1<<9)
#define SFC_BITRIGHT  (1<<8)
#define SFC_BITA      (1<<7)
#define SFC_BITX      (1<<6)
#define SFC_BITL      (1<<5)
#define SFC_BITR      (1<<4)
#define SFC_BITALL    0xffff
void sfc_digital(void)
{
  unsigned char loopcnt;
  unsigned int senddata,temp;

  fcport_init();
  cli();
  senddata = SFC_BITALL;
  while(1){
    TIMERTEMP = 0;
    while((LATCH_PIN&LATCH_BIT)==0){ //until LATCH=H
      if(TIMERTEMP < TIMERLIMIT)continue;

      pad_read();
      senddata = SFC_BITALL;
      if(PAD_BATU  ) senddata &= ~SFC_BITB;
      if(PAD_4KAKU ) senddata &= ~SFC_BITY;
      if(PAD_SELECT) senddata &= ~SFC_BITSELECT;
      if(PAD_START ) senddata &= ~SFC_BITSTART;
      if(PAD_UP    ) senddata &= ~SFC_BITUP;
      if(PAD_DOWN  ) senddata &= ~SFC_BITDOWN;
      if(PAD_LEFT  ) senddata &= ~SFC_BITLEFT;
      if(PAD_RIGHT ) senddata &= ~SFC_BITRIGHT;
      if(PAD_MARU  ) senddata &= ~SFC_BITA;
      if(PAD_3KAKU ) senddata &= ~SFC_BITX;
      if(PAD_L1    ) senddata &= ~SFC_BITL;
      if(PAD_R1    ) senddata &= ~SFC_BITR;

      UNTIL_LATCH_H;
      break;
    }
    temp = senddata;
    loopcnt = 16;

    UNTIL_LATCH_L;
    while(loopcnt--){
      if(0x8000 & temp){
        FC_DAT_H;
      }else{
        FC_DAT_L;
      }
      UNTIL_FCCLK_L;
      temp <<= 1;
      UNTIL_FCCLK_H;
    }
  }
}

// FC Ext connector
// +--------------------------------+
// | (8) (7) (6) (5) (4) (3) (2) (1)|
//  |                              |
//   |(15)(14)(13)(12)(11)(10)( 9)|
//   +----------------------------+

// normal
// (1)GND : ground
// (7)P2_DAT:player2 data(8bit)
// (9)P2_CLK:player2 clock
// (12)P/S : latch (low=latch / high=load)
// (13)P1_DAT :player1 data(8bit)
// (14)P1_CLK :player1 clock
// (15)VCC : 5V

// arkanoid
// (1)GND : ground
// (4):P2_D2B4:player2 paddle data(8bit)
// (5):P2_D2B3:player2 button(low=on)
// (7)P2_DAT:player1 paddle data(8bit)
// (9)P2_CLK:player1/2 paddle clock
// (12)P/S : latch (low=latch / high=load)
// (13)P1_DAT :player1 button(low=on)
// (15)VCC : 5V

// famicom digital
// CLK
// -----+--+--+--+--+--+--+--+--
//      |  |  |  |  |  |  |  |  
//
// DAT
//    07 06 05 04 03 02 01 00
//   +--+--+--+--+--+--+--+--+
//   | A  B SL ST UP DW LT RT|
// --+--+--+--+--+--+--+--+--+--

#define FC_BITALL    0xff
#define FC_BITA      (1<<7)
#define FC_BITB      (1<<6)
#define FC_BITSELECT (1<<5)
#define FC_BITSTART  (1<<4)
#define FC_BITUP     (1<<3)
#define FC_BITDOWN   (1<<2)
#define FC_BITLEFT   (1<<1)
#define FC_BITRIGHT  (1<<0)
void fc_digital(void)
{
  unsigned char temp,loopcnt,senddata;

  senddata = 0xff;
  fcport_init();
  cli();
  while(1){
    TIMERTEMP = 0;
    while((LATCH_PIN&LATCH_BIT)==0){ //until LATCH=H
      if(TIMERTEMP < TIMERLIMIT)continue;

      pad_read();
      senddata = FC_BITALL;
      if(PAD_MARU  ) senddata &= ~FC_BITA;
      if(PAD_BATU  ) senddata &= ~FC_BITB;
      if(PAD_SELECT) senddata &= ~FC_BITSELECT;
      if(PAD_START ) senddata &= ~FC_BITSTART;
      if(PAD_UP    ) senddata &= ~FC_BITUP;
      if(PAD_DOWN  ) senddata &= ~FC_BITDOWN;
      if(PAD_LEFT  ) senddata &= ~FC_BITLEFT;
      if(PAD_RIGHT ) senddata &= ~FC_BITRIGHT;
      UNTIL_LATCH_H;
      break;
    }
    temp = senddata;
    loopcnt = 8;
    UNTIL_LATCH_L;
    while(loopcnt--){
      if(0x80 & temp){
        FC_DAT_H;
      }else{
        FC_DAT_L;
      }
      UNTIL_FCCLK_L;
      temp <<= 1;
      UNTIL_FCCLK_H;
    }
    FC_DAT_H;
  }
}

// famicom arkanoid
// P/S
//    +-+
//    | |  LATCH
// ---+ +---------------------------------

// CLK2
// ---------+--+--+--+--+--+--+--+-------
//          |  |  |  |  |  |  |  |    

// P2DAT
//       +-----------------------+
//       |D7 D6 D5 D4 D3 D2 D1 D0|
// ------+--+--+--+--+--+--+--+--+----

void fc_arkanoid(void)
{
  int centerx,x,posx;
  unsigned char loopcnt;
  unsigned char senddata;

  fcport_init();
  pad_read();
  timer_delay(16);
  posx = 128;   // software position
  centerx = PAD_LX;
  
  while(1){
    sei();
    pad_read();
    if(PAD_MARU){
      FC_DAT_L; // button ON
    }else{
      FC_DAT_H; // button OFF 
    }
    x = PAD_LX - centerx;

    // ドリフト防止
    if((x < MOUSE_THRESHOLD)&&(x > -MOUSE_THRESHOLD))x=0;

    x >>= MOUSE_SPEED;
    posx += x;
    if(posx < 1 )posx=1;
    if(posx >254)posx=254;
    senddata = (unsigned char)posx;
    if(0x80 & senddata){
      FC_DAT2_H;
    }else{
      FC_DAT2_L;
    }
    cli();
    UNTIL_LATCH_H;
    UNTIL_LATCH_L;
    loopcnt = 8;
    while(loopcnt--){
      UNTIL_FCCLK2_L;
      senddata <<= 1;
      UNTIL_FCCLK2_H;
      if(0x80 & senddata){
        FC_DAT2_H;
      }else{
        FC_DAT2_L;
      }
    }
    FC_DAT2_L;
  }
}

// ----appendix: arkanoid II VS mode
// please add this parts
//  PORTC0:P1 Volume(100k ohm B curve)
//  PORTC1:P2 Volume(100k ohm B curve)
//  PORTC2:P1 button
//  PORTC3:P2 button
void fc_arkanoid_vs(void)
{
#define VSMODE_PORT PORTC
#define VSMODE_DDR  DDRC
#define VSMODE_PIN  PINC
#define VSMODE_BITBTN1	(1<<2) // player1 button
#define VSMODE_BITBTN2	(1<<3) // player2 button

  unsigned char loopcnt;
  unsigned char senddata1,senddata2;

  VSMODE_DDR &= ~(VSMODE_BITBTN1|VSMODE_BITBTN2);   // direction input
  VSMODE_PORT |= (VSMODE_BITBTN1|VSMODE_BITBTN2);   // pull up

  fcport_init();
  while(1){
    sei();
    timer_delay(3);
    if((VSMODE_PIN & VSMODE_BITBTN1)==0){
      FC_DAT_L; // button ON
    }else{
      FC_DAT_H; // button OFF
    }
    if((VSMODE_PIN & VSMODE_BITBTN2)==0){
      FC_D2B3_L; // button ON
    }else{
      FC_D2B3_H; // button OFF
    }
    senddata1 = adc_get(0); // player1 Volume
    senddata2 = adc_get(1); // player2 Volume
    if(0x80 & senddata1){
      FC_DAT2_H;
    }else{
      FC_DAT2_L;
    }
    if(0x80 & senddata2){
      FC_D2B4_H;
    }else{
      FC_D2B4_L;
    }
    cli();
    UNTIL_LATCH_H;
    UNTIL_LATCH_L;
    loopcnt = 8;
    while(loopcnt--){
      UNTIL_FCCLK2_L;
      senddata1 <<= 1;
      senddata2 <<= 1;
      UNTIL_FCCLK2_H;
      if(0x80 & senddata1){
        FC_DAT2_H;
      }else{
        FC_DAT2_L;
      }
      if(0x80 & senddata2){
        FC_D2B4_H;
      }else{
        FC_D2B4_L;
      }
    }
    FC_DAT2_L;
    FC_D2B4_L;
  }
}

// ---- famicom crazy climber
// [left hand][right hand]
// UP          3KAKU      --turn--> LEFT  button
// DOWN        BATU       --turn--> RIGHT button
// LEFT        4KAKU      --turn--> DOWN  button
// RIGHT       MARU       --turn--> UP    button
void fc_crazyclimber(void)
{
  unsigned char loopcnt;
  unsigned char senddata1,senddata2;

  fcport_init();
  cli();
  while(1){
    pad_read();
    senddata1 = FC_BITALL;  // gamepad1
    senddata2 = FC_BITALL;  // gamepad2
    if(PAD_SELECT) senddata1 &= ~FC_BITSELECT;
    if(PAD_START ) senddata1 &= ~FC_BITSTART;
    if(PAD_RIGHT ) senddata1 &= ~FC_BITUP;
    if(PAD_LEFT  ) senddata1 &= ~FC_BITDOWN;
    if(PAD_UP    ) senddata1 &= ~FC_BITLEFT;
    if(PAD_DOWN  ) senddata1 &= ~FC_BITRIGHT;
    if(PAD_MARU  ) senddata2 &= ~FC_BITUP;
    if(PAD_4KAKU ) senddata2 &= ~FC_BITDOWN;
    if(PAD_3KAKU ) senddata2 &= ~FC_BITLEFT;
    if(PAD_BATU  ) senddata2 &= ~FC_BITRIGHT;

    UNTIL_LATCH_H;
    UNTIL_LATCH_L;
    if(0x80 & senddata1){
      FC_DAT_H;
    }else{
      FC_DAT_L;
    }
    if(0x80 & senddata2){
      FC_DAT2_H;
    }else{
      FC_DAT2_L;
    }
    loopcnt = 8;
    while(loopcnt--){
      UNTIL_FCCLK_L;
      senddata1 <<= 1;
      UNTIL_FCCLK_H;
      if(0x80 & senddata1){
        FC_DAT_H;
      }else{
        FC_DAT_L;
      }
      UNTIL_FCCLK2_L;
      senddata2 <<= 1;
      UNTIL_FCCLK2_H;
      if(0x80 & senddata2){
        FC_DAT2_H;
      }else{
        FC_DAT2_L;
      }
    }
    FC_DAT_H;
    FC_DAT2_H;
  }
}

//------ X68K digital/PCE digital
void x68k_digital(void)
{
  unsigned char temp,milisec;
  milisec = 0;
  mdport_init();
  cli();
  while(1){
    if((REQ_PIN&REQ_BIT)==0){ // L = enable
      timer_delay(1);
      if(milisec == 0){
        milisec = 16;
        pad_read();
        temp = X68K_BITALL;
        if(PAD_MARU ) temp &= ~X68K_BITA;
        if(PAD_BATU ) temp &= ~X68K_BITB;
        if(PAD_RIGHT) temp &= ~X68K_BITRIGHT;
        if(PAD_LEFT ) temp &= ~X68K_BITLEFT;
        if(PAD_DOWN ) temp &= ~X68K_BITDOWN;
        if(PAD_UP   ) temp &= ~X68K_BITUP;
        MD_PORT = temp;
      }
      milisec--;
    }else{ // H = disable
      MD_PORT = X68K_BITALL;
      milisec = 0;
    }  
  }
}

//------ analog pad(XE-1AP)
// +-------------------+  Megadrive connector
// |(5) (4) (3) (2) (1)|
//  |                 |
//   |(9) (8) (7) (6)|
//   +---------------+
// (1)DATA0:port D2
// (2)DATA1:port D3
// (3)DATA2:port D4
// (4)DATA3:port D5
// (5)VCC+5V
// (6)LH   :port D6
// (7)REQ  :port B0
// (8)GND
// (9)ACK  :port D7

// +-------------------+  X68K connector
// |(5) (4) (3) (2) (1)|
//  |                 |
//   |(9) (8) (7) (6)|
//   +---------------+
// (1)DATA0:port D2
// (2)DATA1:port D3
// (3)DATA2:port D4
// (4)DATA3:port D5
// (5)VCC+5V
// (6)LH   :port D6
// (7)ACK  :port D7
// (8)REQ  :port B0
// (9)GND

// REQ
// ---+   +--------------------------------------------------------------------
//    |   |
//    +---+

// PC mode
// DATA   +0    +1    +2    +3    +4    +5    +6    +7    +8    +9    +10
// -----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+----
//      |E*FG |ABCD |ch0H |ch1H |ch2H |ch3H |ch0L |ch1L |ch2L |ch3L |ABA'B'
//      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

// MD mode
// DATA   +0    +1    +2    +3    +4    +5    +6    +7    +8    +9    +10
// -----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+----
//      |E*FG |ABCD |ch1H |ch0H |     |ch2H |ch1L |ch0L |     |ch2L | ?   |
//      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
//      <16us><34us>
//      <  50usec  >
//      <a><b><c><d>
// fast 12 +4 +12 +22=50micro sec
// 1/2  26 +8 +40 +22=96micro sec
// 1/3  50 +8 +64 +22=144micro sec
// 1/4  74 +8 +88 +22=192micro sec
// REQ_____-------------------------------------------------------(fast)
// REQ_________________-------------------------------------------( 1/2 )  
// REQ_____________________________-------------------------------( 1/3 ) 
// REQ_________________________________________-------------------( 1/4 )

// ACK
// loop < 0  ><  1 ><  2 ><  3 ><  4 ><  5 ><  6 ><  7 ><  8 ><  9 >< 10 >< 11 >
// -----+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +---
//      |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
//      +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+  +--+

// LH
//         +-----+     +-----+     +-----+     +-----+     +-----+     +-----+
//         |     |     |     |     |     |     |     |     |     |     |     |
// --------+     +-----+     +-----+     +-----+     +-----+     +-----+     +---

#define CYBER_BITA      (1<<3)
#define CYBER_BITB      (1<<2)
#define CYBER_BITC      (1<<1)
#define CYBER_BITD      (1<<0)
#define CYBER_BITE1     (1<<3)
#define CYBER_BITE2     (1<<2)
#define CYBER_BITSTART  (1<<1)
#define CYBER_BITSELECT (1<<0)

void x68k_analog(void)
{
  unsigned char sendbuf[12];
  int datanum;
  unsigned char ch0,ch1,ch2,temp,speedmode,newspeed;
  unsigned int timea,timeb,timec,timed;

  speedmode = 0;  // fast
  mdport_init();
  MD_LH_L;
  while(1){
    cli();
    pad_read();
    temp = 0x0f;
    if(PAD_R1) temp &= ~CYBER_BITA;
    if(PAD_R2) temp &= ~CYBER_BITB;
    if(PAD_L1) temp &= ~CYBER_BITC;
    if(PAD_L2) temp &= ~CYBER_BITD;
    sendbuf[0] = temp;
    sendbuf[10] = temp; //

    temp = 0x0f;
    if(PAD_MARU  ) temp &= ~CYBER_BITE1;
    if(PAD_BATU  ) temp &= ~CYBER_BITE2;
    if(PAD_START ) temp &= ~CYBER_BITSTART;
    if(PAD_SELECT) temp &= ~CYBER_BITSELECT;
    sendbuf[1] = temp;
    sendbuf[11] = temp;

    ch0 = PAD_LY;  
    ch1 = PAD_LX;  
    ch2 = PAD_RY;  
    sendbuf[2] = ch0 >> 4;    // CH0 H
    sendbuf[3] = ch1 >> 4;    // CH1 H
    sendbuf[4] = ch2 >> 4;    // CH2 H
    sendbuf[5] = 0;           // CH3 H
    sendbuf[6] = ch0 & 0x0f;  // CH0 L
    sendbuf[7] = ch1 & 0x0f;  // CH1 L
    sendbuf[8] = ch2 & 0x0f;  // CH2 L
    sendbuf[9] = 0;           // CH3 L

    if(speedmode == 0){
      timea=TIMER_12USEC;    
      timeb=TIMER_4USEC;
      timec=TIMER_12USEC;
      timed=TIMER_22USEC;
    }else if(speedmode == 1){
      timea=TIMER_26USEC;    
      timeb=TIMER_8USEC;
      timec=TIMER_40USEC;
      timed=TIMER_22USEC;
    }else if(speedmode == 2){
      timea=TIMER_50USEC;    
      timeb=TIMER_8USEC;
      timec=TIMER_64USEC;
      timed=TIMER_22USEC;
    }else{
      timea=TIMER_74USEC;    
      timeb=TIMER_8USEC;
      timec=TIMER_88USEC;
      timed=TIMER_22USEC;
    }
    newspeed = 0xff;
    UNTIL_REQ_H;
    UNTIL_REQ_L;
    timer_uswait(TIMER_4USEC);

    for(datanum=0 ;datanum<12; datanum++){
      temp = (MD_PORT & ~MD_BITUDLR);
      temp |= (sendbuf[datanum] << MD_DAT_SHIFT);
      MD_PORT = temp;
  
      if((datanum & 1)==0){
        MD_ACK_L;
        timer_uswait(timea);
        MD_ACK_H;
        MD_LH_INVERT;
        timer_uswait(timeb);
      }else{
        MD_ACK_L;
        timer_uswait(timec);
        if(newspeed == 0xff){ // speed setting
          if(REQ_PIN & REQ_BIT){
            newspeed = datanum/2;
          }
        }
        MD_ACK_H;
        MD_LH_INVERT;
        timer_uswait(timed);
      }
    }
    MD_PORT |= MD_BITUDLR;
    MD_LH_L;
    if(newspeed > 3)newspeed = 3;
    speedmode = newspeed;
    sei();
  }
}

//------ Mega Drive digital 3-button pad
// 7pin  | 9pin | 6pin | 4pin | 3pin | 2pin | 1pin |
//-------+------+------+------+------+------+------+-------
// H     | C    | B    | RIGHT| LEFT | DOWN | UP   |
// L     | START| A    | L    | L    | DOWN | UP   |
void md_3button(void)
{
  unsigned char senddata1,senddata2;

  senddata1 = MD_BITALL;
  senddata2 = MD_BITALL & ~MD_BITLR;
  mdport_init();
  cli();
  while(1){
    TIMERTEMP = 0;
    while((REQ_PIN&REQ_BIT)==0){ // until REQ=H
      /*
      if(TIMERTEMP < TIMERLIMIT)continue;
      pad_read();
      senddata1 = MD_BITALL;
      senddata2 = MD_BITALL & ~MD_BITLR;
      if(PAD_MARU ) senddata1 &= ~MD_BITC;    
      if(PAD_BATU ) senddata1 &= ~MD_BITB;    
      if(PAD_RIGHT) senddata1 &= ~MD_BITRIGHT;
      if(PAD_LEFT ) senddata1 &= ~MD_BITLEFT; 
      if(PAD_DOWN ) senddata1 &= ~MD_BITDOWN; 
      if(PAD_UP   ) senddata1 &= ~MD_BITUP;   
      if(PAD_START) senddata2 &= ~MD_BITSTART;
      if(PAD_4KAKU) senddata2 &= ~MD_BITA;    
      if(PAD_DOWN ) senddata2 &= ~MD_BITDOWN; 
      if(PAD_UP   ) senddata2 &= ~MD_BITUP;   

      UNTIL_REQ_H;
      break;
      */
    }
    MD_PORT = senddata1;

    TIMERTEMP = 0;
    while((REQ_PIN&REQ_BIT)!=0){ // until REQ=L
      if(TIMERTEMP < TIMERLIMIT)continue;
      pad_read();
      senddata1 = MD_BITALL;
      senddata2 = MD_BITALL & ~MD_BITLR;
      if(PAD_MARU ) senddata1 &= ~MD_BITC;    
      if(PAD_BATU ) senddata1 &= ~MD_BITB;    
      if(PAD_RIGHT) senddata1 &= ~MD_BITRIGHT;
      if(PAD_LEFT ) senddata1 &= ~MD_BITLEFT; 
      if(PAD_DOWN ) senddata1 &= ~MD_BITDOWN; 
      if(PAD_UP   ) senddata1 &= ~MD_BITUP;   
      if(PAD_START) senddata2 &= ~MD_BITSTART;
      if(PAD_4KAKU) senddata2 &= ~MD_BITA;    
      if(PAD_DOWN ) senddata2 &= ~MD_BITDOWN; 
      if(PAD_UP   ) senddata2 &= ~MD_BITUP;   

      UNTIL_REQ_L;
      break;
    }
    MD_PORT = senddata2;
  }
}

//------ Mega Drive digital 6-button pad
// 7pin  | 9pin | 6pin | 4pin | 3pin | 2pin | 1pin |
//-------+------+------+------+------+------+------+-------
// H     | C    | B    | RIGHT| LEFT | DOWN | UP   |CYCLE1
// L     | START| A    | L    | L    | DOWN | UP   |CYCLE1
// H     | C    | B    | RIGHT| LEFT | DOWN | UP   |CYCLE2
// L     | START| A    | L    | L    | L    | L    |CYCLE2
// H     | H    | H    | MODE | X    | Y    | Z    |CYCLE3
// L     | START| A    | H    | H    | H    | H    |CYCLE3
// H     | C    | B    | RIGHT| LEFT | DOWN | UP   |CYCLE4
// L     | START| A    | L    | L    | DOWN | UP   |CYCLE4
void md_6button(void) 
{
  unsigned char paddata1,paddata2,paddata3;

  paddata1 = MD_BITALL +BITTXRX;
  paddata2 = (MD_BITALL & ~MD_BITLR) +BITTXRX;
  paddata3 = MD_BITALL +BITTXRX;
  mdport_init();
  cli();
  UNTIL_REQ_L;
  while(1){
    UNTIL_REQ_H;
    while(1){
    //----CYCLE1
    // REQ=H   | C    | B    | RIGHT| LEFT | DOWN | UP
      MD_PORT = paddata1;
      TIMERTEMP = 0;
      UNTIL_REQ_L;
    // REQ=L   | START| A    | L    | L    | DOWN | UP
      MD_PORT = paddata2;

      UNTIL_REQ_H;
      if(TIMERTEMP < TIMERLIMIT){
      //under 1.1mSec = 6 button mode
      // over 1.1mSec = 3 button mode
        break;
      }     
    }
    //----CYCLE2
    // REQ=H   | C    | B    | RIGHT| LEFT | DOWN | UP
    MD_PORT = paddata1;
    UNTIL_REQ_L;
    // REQ=L   | START| A    | L    | L    | L    | L
    MD_PORT = paddata2 & MD_BITBC;

    UNTIL_REQ_H;
    //----CYCLE3
    // REQ=H   | H    | H    | MODE | X    | Y    | Z
    MD_PORT = paddata3 | MD_BITBC;
    UNTIL_REQ_L;
    // REQ=L   | START| A    | H    | H    | H    | H
    MD_PORT = paddata2 | MD_BITUDLR;

    UNTIL_REQ_H;
    //----CYCLE4
    // REQ=H   | C    | B    | RIGHT| LEFT | DOWN | UP
    MD_PORT = paddata1;

    TIMERTEMP = 0;
    while((REQ_PIN&REQ_BIT)!=0){ // until REQ=L
      if(TIMERTEMP < TIMERLIMIT)continue;

      pad_read();
      paddata1 = MD_BITALL +BITTXRX;
      paddata2 = (MD_BITALL & ~MD_BITLR)+BITTXRX;
      paddata3 = MD_BITALL +BITTXRX;
      if(PAD_UP){
        paddata1 &= ~MD_BITUP;
        paddata2 &= ~MD_BITUP;
      }
      if(PAD_DOWN){
        paddata1 &= ~MD_BITDOWN;
        paddata2 &= ~MD_BITDOWN;
      }
      if(PAD_MARU  ) paddata1 &= ~MD_BITC;
      if(PAD_BATU  ) paddata1 &= ~MD_BITB;
      if(PAD_RIGHT ) paddata1 &= ~MD_BITRIGHT;
      if(PAD_LEFT  ) paddata1 &= ~MD_BITLEFT;
      if(PAD_START ) paddata2 &= ~MD_BITSTART;
      if(PAD_4KAKU ) paddata2 &= ~MD_BITA;
      if(PAD_SELECT) paddata3 &= ~MD_BITMODE;
      if(PAD_3KAKU ) paddata3 &= ~MD_BITX;
      if(PAD_L1    ) paddata3 &= ~MD_BITY;
      if(PAD_R1    ) paddata3 &= ~MD_BITZ;

      UNTIL_REQ_L;
      break;
    }

    // REQ=L   | START| A    | L    | L    | DOWN | UP
    MD_PORT = paddata2;
  }
}

//------ analog pad(XE-1AP) MD mode
void md_analog(void)
{
  unsigned char sendbuf[12];
  int datanum;
  unsigned char ch0,ch1,ch2,temp;

  sendbuf[11] = 0xf;        //未調査
  mdport_init();
  MD_LH_L;
  while(1){
    cli();
    pad_read();
    temp = 0x0f;
    if(PAD_MARU  ) temp &= ~CYBER_BITE1;
    if(PAD_BATU  ) temp &= ~CYBER_BITE2;
    if(PAD_START ) temp &= ~CYBER_BITSTART;
    if(PAD_SELECT) temp &= ~CYBER_BITSELECT;
    sendbuf[0] = temp;
    sendbuf[10] = temp;

    temp = 0x0f;
    if(PAD_R1 ) temp &= ~CYBER_BITA;
    if(PAD_R2 ) temp &= ~CYBER_BITB;
    if(PAD_L1 ) temp &= ~CYBER_BITC;
    if(PAD_L2 ) temp &= ~CYBER_BITD;
    sendbuf[1] = temp;

    ch1 = PAD_LX;  //
    ch0 = PAD_LY;  //
    ch2 = PAD_RY;  //
    sendbuf[2] = ch1 >> 4;    // CH1 H
    sendbuf[3] = ch0 >> 4;    // CH0 H
    sendbuf[4] = 0;           // CH3 H
    sendbuf[5] = ch2 >> 4;    // CH2 H
    sendbuf[6] = ch1 & 0x0f;  // CH1 L
    sendbuf[7] = ch0 & 0x0f;  // CH0 L
    sendbuf[8] = 0;           // CH3 L
    sendbuf[9] = ch2 & 0x0f;  // CH2 L

    UNTIL_REQ_H;
    UNTIL_REQ_L;
    timer_uswait(TIMER_4USEC);

    for(datanum=0 ;datanum<12; datanum++){
      temp = (MD_PORT & ~MD_BITUDLR);
      temp |= (sendbuf[datanum] << MD_DAT_SHIFT);
      MD_PORT = temp;
      if((datanum & 1)==0){
        MD_ACK_L;
        timer_uswait(TIMER_12USEC);
        MD_ACK_H;
        MD_LH_INVERT;
        timer_uswait(TIMER_4USEC);
      }else{
        MD_ACK_L;
        timer_uswait(TIMER_12USEC);
        MD_ACK_H;
        MD_LH_INVERT;
        timer_uswait(TIMER_22USEC);
      }
    }
    MD_PORT |= MD_BITALL;
    MD_LH_L;
    sei();
  }
}

char timeout_ack(char state)
{
  TIMERTEMP = 0;
  while(1){
    if(state){
      if((MD_PIN & MD_BITACK)!=0)break;
    }else{
      if((MD_PIN & MD_BITACK)==0)break;
    }
    if(TIMERTEMP > TIMERLIMIT)  return(1);  // error
  }
  return(0);
}

char timeout_reqest(char state)
{
  TIMERTEMP = 0;
  while(1){
    if(state){
      if((REQ_PIN&REQ_BIT)!=0)break;
    }else{
      if((REQ_PIN&REQ_BIT)==0)break;
    }
    if(TIMERTEMP > TIMERLIMIT)  return(1);  // error
  }
  return(0);
}

//------ Mega Drive sega mouse
void md_segamouse(void)
{
  unsigned char sendbuf[10];
  int datanum,x,y,centerx,centery;
  unsigned char temp;
  char timeout;

  mdport_init();
  cli();
  MD_DDR &= ~MD_BITACK; // direction input

  pad_read();
  centerx = PAD_LX;
  centery = PAD_LY;
  timer_delay(16);

  sendbuf[0] = 0x00;    // header
  sendbuf[1] = 0x0b;    // header
  sendbuf[2] = 0x0f;    // header
  sendbuf[3] = 0x0f;    // header
  sendbuf[4] = 0x00;    // b3=Y over|b2=X over|b1=Y sign|b0=X sign
  sendbuf[5] = 0x00;    // b3=start |b2=M     |b1=R     |b0=L(1=ON)
  sendbuf[6] = 0x00;    // X  bit7～4
  sendbuf[7] = 0x00;    // X  bit3～0
  sendbuf[8] = 0x00;    // Y  bit7～4
  sendbuf[9] = 0x00;    // Y  bit3～0
  
  while(1){
    UNTIL_REQ_H;
    temp = (MD_PORT & ~MD_BITUDLR);
    temp |= (sendbuf[0] << MD_DAT_SHIFT);
    MD_PORT = temp;
    MD_PORT |= MD_BITLH;

    TIMERTEMP = 0;
    while((REQ_PIN&REQ_BIT)!=0){ // until REQ=L
      if(TIMERTEMP < TIMERLIMIT)continue;

      pad_read();
      x = PAD_LX - centerx;
      y = centery - PAD_LY;

      // ドリフト防止
      if((x < MOUSE_THRESHOLD)&&(x > -MOUSE_THRESHOLD))x=0;
      if((y < MOUSE_THRESHOLD)&&(y > -MOUSE_THRESHOLD))y=0;

      x >>= MOUSE_SPEED;
      y >>= MOUSE_SPEED;
      temp = 0;
      if(x < 0)temp |= (1<<0);  // minus
      if(y < 0)temp |= (1<<1);  // minus
      sendbuf[4] = temp;     

      temp = 0;
      if(PAD_START ) temp |= (1<<3); // START
      if(PAD_4KAKU ) temp |= (1<<2); // BALL
      if(PAD_BATU  ) temp |= (1<<1); // L
      if(PAD_MARU  ) temp |= (1<<0); // R
      sendbuf[5] = temp;            //

      sendbuf[6] = (unsigned char)((x >> 4) & 0x0f);  
      sendbuf[7] = (unsigned char)(x & 0x0f);         
      sendbuf[8] = (unsigned char)((y >> 4)& 0x0f);   
      sendbuf[9] = (unsigned char)(y & 0x0f);  
      UNTIL_REQ_L;
      break;
    }

    for(datanum=1;datanum<10;datanum++){
      timeout = timeout_reqest(0);
      if(timeout)break;
      if((datanum & 1)==0){
        timeout = timeout_ack(0);
          // until ACK=L
      }else{
        timeout = timeout_ack(1);
          // until ACK=H
      }
      if(timeout)break;

      temp = (MD_PORT & ~MD_BITUDLR);
      temp |= (sendbuf[datanum] << MD_DAT_SHIFT);
      MD_PORT = temp;

      timer_uswait(TIMER_1USEC);
      if((MD_PIN & MD_BITACK)!=0){
        MD_PORT |= MD_BITLH;
      }else{
        MD_PORT &= ~MD_BITLH;
      }
      timer_uswait(TIMER_1USEC);
    }
    timer_uswait(TIMER_12USEC);
    UNTIL_REQ_H;
    MD_PORT |= MD_BITALL;
  }
}

// ---- appendix:cyberstick to megadrive
// please add this parts
// PORTC0:dsub9pin 1
// PORTC1:dsub9pin 2
// PORTC2:dsub9pin 3
// PORTC3:dsub9pin 4
// VCC   :dsub9pin 5
// PORTB5:dsub9pin 7(ACK)
// PORTB3:dsub9pin 8(REQ)
// GND   :dsub9pin 9
void cyber_to_megadrive(void)
{
  #define CYBERDAT_PORT PORTC  // output data
  #define CYBERDAT_DDR  DDRC   // direction
  #define CYBERDAT_PIN  PINC   // input
  #define CYBERDAT_MASK 0xf
  #define CYBER_PORT PORTB     // output data
  #define CYBER_DDR  DDRB      // direction
  #define CYBER_PIN  PINB      // input
  #define CYBER_BITACK (1<<5)  // ACK mask
  #define CYBER_BITREQ (1<<3)  // REQ mask
  #define CYBER_REQ_H  CYBER_PORT|=CYBER_BITREQ
  #define CYBER_REQ_L  CYBER_PORT&=~CYBER_BITREQ
  #define UNTIL_CYBER_ACK_H  while((CYBER_PIN&CYBER_BITACK)==0)
  #define UNTIL_CYBER_ACK_L  while(CYBER_PIN&CYBER_BITACK)
// not use
//  #define CYBER_BITLH  (1<<1)  // LH mask
//  #define UNTIL_CYBER_LH_H   while((CYBER_PIN&CYBER_BITLH)==0)
//  #define UNTIL_CYBER_LH_L   while(CYBER_PIN&CYBER_BITLH)

  int datanum;
  unsigned char temp,x,y;
  unsigned char cyberbuff[11];
  unsigned char sendbuf[11];

  CYBERDAT_DDR  &= ~CYBERDAT_MASK;    // direction input
  CYBERDAT_PORT |= CYBERDAT_MASK;     // pull up
  CYBER_DDR |= CYBER_BITREQ;          // direction output
  CYBER_DDR &= ~CYBER_BITACK;  // direction input
  CYBER_PORT |= CYBER_BITACK;  // pull up
  CYBER_REQ_H;
// not use  
//  CYBER_DDR &= ~CYBER_BITLH;  // direction input
//  CYBER_PORT |= CYBER_BITLH;  // pull up
  mdport_init();
  timer_delay(10);
  while(1){
    cli();
    CYBER_REQ_L;  // request start
    timer_uswait(TIMER_1USEC); // high speed mode
    for(datanum=0 ;datanum<11; datanum++){
// not use
//      if((datanum & 1)==0){
//        UNTIL_CYBER_LH_L;
//      }else{
//        UNTIL_CYBER_LH_H;
//      }
      UNTIL_CYBER_ACK_L;
      CYBER_REQ_H;
      cyberbuff[datanum] = CYBERDAT_PIN & CYBERDAT_MASK;
      UNTIL_CYBER_ACK_H;
    }
    sei();

// self test mode
#if 0
    vram_clear();
    temp = (cyberbuff[1]<<4) | cyberbuff[0];
    vram_puthex(64,FONTH*0,temp);
    temp = (cyberbuff[3]<<4) | cyberbuff[7];
    vram_puthex(64,FONTH*1,temp);
    x = temp/(256/VRAMH);
    temp = (cyberbuff[2]<<4) | cyberbuff[6];   
    vram_puthex(64,FONTH*2,temp);
    y = temp/(256/VRAMH);
    vram_line(x-16,y,x+16,y,1);
    vram_line(x,y-16,x,y+16,1);
    x = 32;
    temp = (cyberbuff[4]<<4) | cyberbuff[8];
    vram_puthex(64,FONTH*3,temp);
    y = temp/(256/VRAMH);;
    vram_line(x-16,y,x+16,y,1);
    vram_line(x,y-16,x,y+16,1);
    oled_redraw();
    timer_delay(16);
    continue;
#endif

    sendbuf[0] = cyberbuff[1];   // E1/ E2/STA/SEL
    sendbuf[1] = cyberbuff[0];   //  A/  B/  C/  D
    sendbuf[2] = cyberbuff[3];   // CH1 H
    sendbuf[3] = cyberbuff[2];   // CH0 H
    sendbuf[4] = cyberbuff[5];   // CH3 H
    sendbuf[5] = cyberbuff[4];   // CH2 H
    sendbuf[6] = cyberbuff[7];   // CH1 L
    sendbuf[7] = cyberbuff[6];   // CH0 L
    sendbuf[8] = cyberbuff[9];   // CH3 L
    sendbuf[9] = cyberbuff[8];   // CH2 L
    sendbuf[10] = 0xf;           // 未調査

    cli();
    UNTIL_REQ_H;
    UNTIL_REQ_L;
    timer_uswait(TIMER_4USEC);

    for(datanum=0 ;datanum<11; datanum++){
      if((datanum & 1)==0){
        MD_LH_L;
      }else{
        MD_LH_H;
      }
      temp = (MD_PORT & ~MD_BITUDLR);
      temp |= (sendbuf[datanum] << MD_DAT_SHIFT);
      MD_PORT = temp;
      MD_ACK_L;
      timer_uswait(TIMER_12USEC);
      MD_ACK_H;
      if((datanum & 1)==0){
        timer_uswait(TIMER_4USEC);
      }else{
        timer_uswait(TIMER_22USEC);
      }
    }
    MD_PORT |= MD_BITALL;
    sei();
  }
}

//------ PC Engine mouse
void pce_mouse(void)
{
  unsigned char sendbuf[4];
  int datanum,x,y,centerx,centery;
  unsigned char temp,button;
  char timeout;

  mdport_init();
  pad_read();
  centerx = PAD_LX;
  centery = PAD_LY;
  timer_delay(16);
  sendbuf[0] = 0x00;    // X  bit7～4
  sendbuf[1] = 0x00;    // X  bit3～0
  sendbuf[2] = 0x00;    // Y  bit7～4
  sendbuf[3] = 0x00;    // Y  bit3～0
  
  while(1){    
    cli();
    UNTIL_REQ_L;
    TIMERTEMP = 0;
    while((REQ_PIN&REQ_BIT)==0){ // until REQ=H
      if(TIMERTEMP < TIMERLIMIT)continue;
      pad_read();

      x = centerx - PAD_LX;
      y = centery - PAD_LY;
      // ドリフト防止
      if((x < MOUSE_THRESHOLD)&&(x > -MOUSE_THRESHOLD))x=0;
      if((y < MOUSE_THRESHOLD)&&(y > -MOUSE_THRESHOLD))y=0;
      x >>= MOUSE_SPEED;
      y >>= MOUSE_SPEED;
      sendbuf[0] = (unsigned char)((x >> 4) & 0x0f);  
      sendbuf[1] = (unsigned char)(x & 0x0f);         
      sendbuf[2] = (unsigned char)((y >> 4)& 0x0f);   
      sendbuf[3] = (unsigned char)(y & 0x0f);  

      temp = MD_PORT | (X68K_BITA | X68K_BITB);
      if(PAD_MARU) temp &= ~X68K_BITA;
      if(PAD_BATU) temp &= ~X68K_BITB;
      MD_PORT = temp;

      UNTIL_REQ_H;
      break;
    }

    for(datanum=0;datanum<4;datanum++){
      UNTIL_REQ_L;
      temp = (MD_PORT & ~MD_BITUDLR);
      temp |= (sendbuf[datanum] << MD_DAT_SHIFT);
      MD_PORT = temp;
      
      if(datanum < 3){       
        timeout = timeout_reqest(1);
        if(timeout)break;
      }
    }
    UNTIL_REQ_L;
    sei();
    timer_uswait(TIMER_4USEC);
//    MD_PORT |= MD_BITALL;
  }
}

//---- MSX arkanoid
void msx_arkanoid(void)
{
  int centerx,x,posx;
  unsigned char loopcnt;
  unsigned char senddata;
#define ARKA_BITDAT   X68K_BITUP  
#define ARKA_BITBTN   X68K_BITDOWN
#define ARKA_BITCLK   X68K_BITA

  mdport_init();
  MD_DDR &= ~ARKA_BITCLK; // direction input
  pad_read();
  timer_delay(16);
  posx = 128;   // software position
  centerx = PAD_LX;
  cli();
  while(1){
    UNTIL_REQ_L;
    UNTIL_REQ_H;
    pad_read();
    if(PAD_MARU ){ // button ON
      MD_PORT &= ~ARKA_BITBTN;
    }else{
      MD_PORT |= ARKA_BITBTN;
    }
    x = PAD_LX - centerx;

    // ドリフト防止
    if((x < MOUSE_THRESHOLD)&&(x > -MOUSE_THRESHOLD))x=0;

    x >>= MOUSE_SPEED;
    posx += x;
    if(posx < 1 )posx=1;
    if(posx >254)posx=254;
    senddata = (unsigned char)posx;    
    loopcnt = 8;
    while(loopcnt--){
      if(0x80 & senddata){
        MD_PORT |= ARKA_BITDAT;
      }else{
        MD_PORT &= ~ARKA_BITDAT;
      }
      // until CLK=low
      while((MD_PIN & ARKA_BITCLK)!=0){
        if((REQ_PIN & REQ_BIT)==0)break;
      }
      if((REQ_PIN & REQ_BIT)==0)break;
      while((MD_PIN & ARKA_BITCLK)==0); // until CLK=high
      senddata <<= 1;
    }
    MD_PORT &= ~ARKA_BITDAT;
  }
}

//---- appendix:MSX arkanoid vol edition
// please add this parts
//  PORTC0:Player Volume(100k ohm B curve)
//  PORTC2:Player button
void msx_arkanoid_vol(void)
{
#define VOL_PORT PORTC
#define VOL_DDR  DDRC
#define VOL_PIN  PINC
#define VOL_BITBTN  (1<<2) // player button
#define ARKA_BITDAT   X68K_BITUP  
#define ARKA_BITBTN   X68K_BITDOWN
#define ARKA_BITCLK   X68K_BITA
  unsigned char loopcnt;
  unsigned char senddata;

  mdport_init();
  MD_DDR &= ~ARKA_BITCLK;   // direction input
  VOL_DDR &= ~VOL_BITBTN;   // direction input
  VOL_PORT |= VOL_BITBTN;   // pull up
  cli();
  while(1){
    UNTIL_REQ_L;
    UNTIL_REQ_H;
    if((VOL_PIN & VOL_BITBTN)==0){  // button ON
      MD_PORT &= ~ARKA_BITBTN;
    }else{
      MD_PORT |= ARKA_BITBTN;
    }
    senddata = adc_get(0); // player1 Volume
    loopcnt = 8;
    while(loopcnt--){
      if(0x80 & senddata){
        MD_PORT |= ARKA_BITDAT;
      }else{
        MD_PORT &= ~ARKA_BITDAT;
      }
      // until CLK=low
      while((MD_PIN & ARKA_BITCLK)!=0){
        if((REQ_PIN & REQ_BIT)==0)break;
      }
      if((REQ_PIN & REQ_BIT)==0)break;
      while((MD_PIN & ARKA_BITCLK)==0); // until CLK=high
      senddata <<= 1;
    }
    MD_PORT &= ~ARKA_BITDAT;
  }
}

//------ (debug)PlayStation pad input test
void pad_test(void)
{
  char i,x,y;
#if USE_LCD      
  while(1){
    pad_read();
    for(i=0; i<8; i++){
      x = i*2;
      y = 1;
      lcd_puthex(x,y,padinput[i+3]);
    }
    timer_delay(16);
  }
#endif
#if USE_OLED
  while(1){
    pad_read();
    vram_clear();
    for(i=0; i<6; i++){
      x = (i % 2)*(2*FONTW)+64;
      y = (i/2)*FONTH;
      vram_puthex(x,y,padinput[i+3]);
    }
    x=PAD_RX/(256/VRAMH);
    y=PAD_RY/(256/VRAMH);
    vram_line(x-16,y,x+16,y,1);
    vram_line(x,y-16,x,y+16,1);

    x=PAD_LX/(256/VRAMH);
    y=PAD_LY/(256/VRAMH);
    vram_line(x-16,y,x+16,y,1);
    vram_line(x,y-16,x,y+16,1);
    oled_redraw();
    timer_delay(16);
  }
#endif
}

void req_test2(void)
{
  unsigned char temp;
  unsigned char i,x,y;
  unsigned char buff[128];

  MD_DDR &= ~MD_BITALL;
  MD_PORT |= MD_BITALL;
  REQ_DDR &= ~REQ_BIT;
  REQ_PORT |= REQ_BIT;
  while(1){
    cli();
    UNTIL_REQ_H;
    UNTIL_REQ_L;
//    while((MD_PIN & X68K_BITA)!=0);

    for(x=0; x<128; x++){
      temp = 0xff;      
      for(i=0; i<6; i++){
        temp &= 0xC0 + (MD_PIN >> 2);
        if((REQ_PIN&REQ_BIT)==0) temp &= ~(1<<6);
      }
      buff[x] = temp;
    }
    vram_clear();
    for(x=0; x<128; x++){
      for(i=0; i<7; i++){
        y = (i*8)+4;
        if(buff[x] & (1<<i)) y = y-3;
        vram_pset(x,y,1);
      }
    }
    sei();
    oled_redraw();
  }  
}

// debug
void trig_test(void)
{
  unsigned char edgecnt,edgedir,edge;
  char milisec;
  
  mdport_init();
  MD_DDR &= ~(MD_BITA);
  vram_clear();      
  vram_putstr(0,0,"TRIG");
  oled_redraw();
  while(1){
    cli();
    edgedir=0;
    edgecnt=0;
    milisec=0;
    TCNT2=0;
    while(1){
      if(TCNT2 >= TCNT2_1MSEC){
        TCNT2=0;
        milisec++;
        if(milisec >= 16)break;
      }  
//      if((REQ_PIN & REQ_BIT)!=0){
      if((MD_PIN & X68K_BITA)!=0){
        edge = 1;
      }else{
        edge = 0;
      }
      if(edgedir==edge){
        edgedir ^= 1;
        if(edge)edgecnt++;
      }
    }
    vram_puthex(0,16,edgecnt);
    sei();
    oled_redraw();
  }
}

//------ (debug)input request
void req_test(void)
{
  unsigned char edgecnt;
  edgecnt=0;
  mdport_init();
#if USE_LCD      
  lcd_putstr_pgm(0,0,str_reqtest);
#endif
#if USE_OLED
  vram_clear();      
  vram_putstr_pgm(0,0,str_reqtest);
  oled_redraw();
#endif
  while(1){
    cli();
    UNTIL_REQ_H;
    UNTIL_REQ_L;
    edgecnt++;
#if USE_LCD      
    lcd_puthex(0,1, (char)edgecnt);
#endif
#if USE_OLED
    vram_puthex(0,16,edgecnt);
    sei();
    oled_redraw();
#endif
  }
}
//----- (debug)adc test
void adc_test(void)
{
  unsigned char temp;

#if USE_LCD      
  lcd_putstr_pgm(0,0,str_adctest);
  while (1){
    temp = adc_get(0);
    lcd_puthex(0,1,temp);
    timer_delay(16);
  }
#endif
#if USE_OLED
  vram_clear();      
  vram_putstr_pgm(0,0,str_adctest);
  vram_putstr(0,16,"ch0");
  vram_putstr(0,32,"ch1");
  while(1){
    temp = adc_get(0);
    vram_puthex(64,16,temp);
    temp = adc_get(1);
    vram_puthex(64,32,temp);
    oled_redraw();
    timer_delay(16);
  }
#endif
}

//----- (debug)timer test
void timer_test(void)
{
  unsigned char num=0;

#if USE_LCD      
  lcd_putstr_pgm(0,0,str_timertest);
  while (1)
  {
    lcd_puthex(0,1,num);
    num++;
    timer_delay(1000);
  }
#endif
#if USE_OLED
  vram_clear();      
  vram_putstr_pgm(0,0,str_timertest);
  while(1){
    vram_puthex(0,16,num);
    oled_redraw();
    num++;
    timer_delay(1000);
  }
#endif
}

//---- push any button
void pad_wait(char blinkflag)
{
  #define BLINK_START 160
  #define BLINK_END   180
  unsigned char blinktimer=0;
  
  while(1){
    pad_read();
    if((padinput[3]==0xFF)&&(padinput[4]==0xFF))break;
    timer_delay(16);
  }
  while(1){
    if(blinkflag){
      blinktimer = (blinktimer+1) % BLINK_END;
      if((blinktimer == 0)||(blinktimer == BLINK_START)){
#if USE_OLED
        vram_fill(0,VRAMH-FONTH,127,63,2);
        oled_redraw();
#endif
      }
    }
    pad_read();
    if((padinput[3]!=0xFF)||(padinput[4]!=0xFF))break;
    timer_delay(16);
  }
} 

//----select menu
char menu(void)
{
  char modenum;
  modenum = EEPROM.read(EEPROMADDR);
  if((modenum < 0)||(modenum > (MENU_MAX-1))){
    modenum = 0;  // data broken
                  // goto menu
  }else{
    pad_read();
    timer_delay(16);    
    if((padinput[3]==0xFF)&&(padinput[4]==0xFF)){
      // no button
      return(modenum);
    }
  }
#if USE_OLED
  vram_clear();
  vram_putstr_pgm(FONTW*0,FONTH*1,str_howto);
  oled_redraw();
#endif
  timer_delay(3000);
  
  while(1){
#if USE_LCD
    lcd_puthex(0,0,modenum);
    lcd_putstr_pgm(0,1,str_mode[modenum]);
    pad_wait(1);
#endif
#if USE_OLED
    vram_clear();
    vram_putstr_pgm(FONTW*1,FONTH*3,str_select);
    vram_putstr_pgm(FONTW*0,FONTH*0,str_mode[modenum]);
    oled_redraw();
    pad_wait(1);
    vram_clear();
    oled_redraw();
#endif    
    timer_delay(100);
    if((PAD_UP)||(PAD_LEFT))modenum--;
    if((PAD_DOWN)||(PAD_RIGHT))modenum++;
    if(modenum < 0)modenum = (MENU_MAX-1);
    if(modenum >(MENU_MAX-1))modenum = 0;
    if(PAD_MARU) break;
  }
  // save eeprom
  EEPROM.write(EEPROMADDR, modenum);
#if USE_OLED
  vram_putstr_pgm(FONTW*1,FONTH*1,str_saved);
  vram_putstr_pgm(FONTW*3,FONTH*3,str_ok);
  oled_redraw();
#endif
  pad_wait(1);
  return(modenum);
}

//----
void launch(char modenum)
{ 
  if((modenum < 0)||(modenum > (MENU_MAX-1))){
    modenum = 0;  // data broken
  }
#if USE_LCD
  lcd_puthex(0,0,modenum);
  lcd_putstr_pgm(0,1,str_mode[modenum]);
#endif
#if USE_OLED
  vram_clear();
  vram_putstr_pgm(FONTW*0,FONTH*0,str_mode[modenum]);
  vram_putstr_pgm(FONTW*0,FONTH*3,str_running);
  oled_redraw();
#endif
  switch(modenum)
  {
  case 1:// MD 3-BUTTON
    md_3button();
    break;
  case 2:// MD 6-BUTTON
    md_6button();
    break;
  case 3:// MD ANALOG
    md_analog();
    break;
  case 4:// MD SEGA MOUSE
    md_segamouse(); //
    break;
  case 5:// PCE DIGITAL
    x68k_digital();
    break;
  case 6:// PCE ANALOG
    x68k_analog();  //
    break;
  case 7:// PCE MOUSE
    pce_mouse();
    break;
  case 8:// FC DIGITAL
    fc_digital();
    break;
  case 9:// FC ARKANOID
    fc_arkanoid();  //
    break;
  case 10://  FC CrazyClimb
    fc_crazyclimber();
    break;
  case 11://  SFC DIGITAL,
    sfc_digital();
    break;
  case 12://  SFC MOUSE
    sfc_mouse();  //
    break;
  case 13://  X68 DIGITAL,
    x68k_digital();
    break;
  case 14://  X68 ANALOG,
    x68k_analog();  //
    break;
  case 15://  MSX ARKANOID
    msx_arkanoid();
    break;
  case 16://  MSX ARKANOID vol edition
    msx_arkanoid_vol();
    break;
  case 17:// FC ARKANOID VS.
    fc_arkanoid_vs();  //
    break;
  default:
    pad_test(); //debug
    break;
  }
}

//----
void setup()
{
  char x,y,modenum;
  timer_init();

#if USE_LCD+USE_OLED
  Wire.begin();
#endif
#if USE_OLED
  oled_init();  // SSD1306
#endif
#if USE_LCD
  lcd_init(); // AQM1602XA
#endif

#if USE_SERIAL
  Serial.begin(115200);
  Serial.print("hello\n");
//  if (Serial.available() > 0) {
//    ret = Serial.read();
//  }
#endif

#if USE_CYBERSTICK
  cyber_to_megadrive();
#endif

  adc_init();
  pad_init();

  modenum = menu();
  launch(modenum);

//  adc_test(); // debug
//  pad_test(); //debug
//  timer_test(); //debug
//  trig_test(); //debug
//  req_test(); //debug
//  req_test2();  //debug
}

void loop() {
}
