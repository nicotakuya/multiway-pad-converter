// Spinner Decorder(Arcade Spinner for Arcade1Up)
// for Arduino Pro Mini(ATmega328p,5V,16MHz) 
// by takuya matsubara

#define SPINNER_SPEED 2
#define USE_OLED 0        // 1=USE /0=NOT USE

#if USE_OLED
#include <Wire.h>
#include "8x8font.h"      // font data
#endif
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

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

#if USE_OLED
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

//---- spinner
#define SIO_BAUDRATE 125000
#define TCNT_ONEBIT (T1HZ/SIO_BAUDRATE)
#define SIO_PORT PORTB   // port
#define SIO_PIN  PINB    // pin
#define SIO_DDR  DDRB    // direction
#define SIO_BITRX   (1<<5) // bit mask
unsigned char spinner_pos;
unsigned char spinner_milisec;

// spinner's dip switch
// SW1=OFF/SW2=OFF : 16 pulse/rotation
// SW1=ON /SW2=OFF : 64 pulse/rotation
// SW1=OFF/SW2=ON  : 512 pulse/rotation *best
// SW1=ON /SW2=ON  : 1024 pulse/rotation

// SW3=ON  : signal duration 0.5ms
// SW3=OFF : signal duration 8ms *best

void spinner(void)
{
  unsigned int timing;
  unsigned char rxdat,mask;

  SIO_DDR &= ~SIO_BITRX;  // direction
  SIO_PORT |= SIO_BITRX;  // pull up
  spinner_pos = 128;  // software position
  spinner_milisec = 0;

  Serial.begin(115200);
  while (!Serial) {
  }
  cli();
  while(1){
    while(1){
      if((SIO_PIN & SIO_BITRX)==0)break;
      if(TCNT2 >= TCNT2_1MSEC){
        TCNT2 = 0;
        spinner_milisec++;
        if(spinner_milisec >= 16){
          spinner_milisec = 0;
          sei();
          Serial.write(spinner_pos);
#if USE_OLED
//        vram_puthex( 0,16,spinner_pos); // debug
//        oled_redraw();                  // debug
#endif
          cli();
        }
      }
    }
    TCNT1 = 0;
    mask = 0x01;
    rxdat = 0;
    timing = TCNT_ONEBIT+(TCNT_ONEBIT/2);
    while(mask < 0x80){ 
      while(TCNT1 < timing);
      if((SIO_PIN & SIO_BITRX)==0)
        rxdat |= mask;

      timing += TCNT_ONEBIT;
      mask <<= 1;
    }
    if(rxdat == 0x2f)  spinner_pos += SPINNER_SPEED;
    if(rxdat == 0x6f)  spinner_pos -= SPINNER_SPEED;
  }
}

// debug
void req_test3(void)
{
  unsigned char temp,x,y;
  int i;
  unsigned char buff[128];

  DDRB &= ~SIO_BITRX;
  PORTB |= SIO_BITRX;
  while(1){
    cli();
    while((PINB & SIO_BITRX)==0);
    while((PINB & SIO_BITRX)!=0);  // edge

    for(x=0; x<128; x++){
      temp = 1;      
      for(i=0; i<1000; i++){
        if((PINB&SIO_BITRX)==0) temp = 0;
      }
      buff[x] = temp;
    }
#if USE_OLED    
    vram_clear();
    for(x=0; x<128; x++){
      y = 16;
      if(buff[x]) y -= 8;
      vram_pset(x,y,1);
    }
    sei();
    oled_redraw();
#endif
  }  
}

//----
void setup()
{
  timer_init();

#if USE_OLED
  Wire.begin();
  oled_init();  // SSD1306
  vram_clear();      
  oled_redraw();
#endif

  spinner(); //
}

void loop() {
}
