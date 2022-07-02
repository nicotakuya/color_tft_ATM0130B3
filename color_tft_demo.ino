// Color TFT test program by takuya matsubara
// for Arduino Pro Mini + AE-ATM0130B3

// Akizuki AE-ATM0130B3
// Reference :AE-ATM0130_sample(AE-ATM0310B3 with Arduino UNO R3)
// https://akizukidenshi.com/catalog/g/gK-15560/
// https://akizukidenshi.com/download/ds/akizuki/AE-ATM0130_sample.zip

// AE-ATM0130B3 pin assign
//1: VDD
//2: VDDIO
//3: GND
//4: SCK
//5: MOSI
//6: ~CS(L=Active)
//7: D/C(H=Data/L=Command)
//8: ~RES

// Arduino Pro Mini
// CPU：ATmega328P 5V 16HMz
// Flash:32 KB 
// SRAM:2 KB

#define SOFT_SPI  0 // software SPI(0=fast / 1=slow)

#if SOFT_SPI==0
#include <SPI.h>
#endif
#include <avr/pgmspace.h>
#include "8x8font.h"

#define TFTCS    10   // (SS) chip select
#define TFTCD     9   // command/data
#define SPICLK   13   // (SCK) clock
#define SPIDAT   11   // (MOSI) data
#define TFTRST    8   // reset

PROGMEM const unsigned char gamma1_table[] = {
  0xf0,0x0b,0x11,0x0e,0x0d,0x19,0x36,0x33,0x4b,0x07,0x14,0x14,0x2c,0x2e
};

PROGMEM const unsigned char gamma2_table[] = {
  0xf0,0x0d,0x12,0x0b,0x09,0x03,0x32,0x44,0x48,0x39,0x16,0x16,0x2d,0x30
};

PROGMEM const unsigned char framerate_table[] = {
  0x0c,0x0c,0x00,0x33,0x33
};

#define VRAMWIDTH        60  // width[pixel]
#define VRAMHEIGHT       60  // height[pixel]
#define VRAMDIV      4  // 2bit color(1BYTE/2bits=4pixel)
#define VRAMSIZE   (VRAMWIDTH*VRAMHEIGHT/VRAMDIV)

#define ROTATE  0 //TURN SCREEN(0=normal / 1=turn90 / 2=turn180 / 3=turn270)
#if ROTATE==0
#define VRAMXMAX  (VRAMWIDTH-1)
#define VRAMYMAX  (VRAMHEIGHT-1)
#endif
#if ROTATE==1
#define VRAMYMAX  (VRAMWIDTH-1)
#define VRAMXMAX  (VRAMHEIGHT-1)
#endif
#if ROTATE==2
#define VRAMXMAX  (VRAMWIDTH-1)
#define VRAMYMAX  (VRAMHEIGHT-1)
#endif
#if ROTATE==3
#define VRAMYMAX  (VRAMWIDTH-1)
#define VRAMXMAX  (VRAMHEIGHT-1)
#endif

unsigned char vram[VRAMSIZE];
int def_posx = 0;
int def_posy = 0;
unsigned char def_tcolor = 3;

// sprite pattern
PROGMEM const unsigned char sp_pattern[] = {
  0b0010100,
  0b0010100,
  0b0011100,
  0b0011100,
  0b1111111,
  0b1111111,
  0b1011101,

  0b0111110,
  0b1111111,
  0b1000001,
  0b1111111,
  0b1111111,
  0b1100011,
  0b0110110,

  0b0111110,
  0b1111111,
  0b1100011,
  0b1111111,
  0b1111111,
  0b0110110,
  0b1100011,

  0b0110110,
  0b0011100,
  0b0011100,
  0b0011100,
  0b0011100,
  0b0011100,
  0b0001000,

  0b1001001,
  0b0101010,
  0b0000000,
  0b1100011,
  0b0000000,
  0b0101010,
  0b1001001,

  0b0001000,
  0b0011100,
  0b0011100,
  0b0011100,
  0b0011100,
  0b0011100,
  0b0110110,
  
  0b0011100,
  0b0111110,
  0b1110111,
  0b1100011,
  0b1110111,
  0b0111110,
  0b0011100
};

// vector
PROGMEM const unsigned char vect_table[]={
    0,/* 0 degrees*/
    4,/* 1 degrees*/
    9,/* 2 degrees*/
   13,/* 3 degrees*/
   18,/* 4 degrees*/
   22,/* 5 degrees*/
   27,/* 6 degrees*/
   31,/* 7 degrees*/
   35,/* 8 degrees*/
   40,/* 9 degrees*/
   44,/* 10 degrees*/
   48,/* 11 degrees*/
   53,/* 12 degrees*/
   57,/* 13 degrees*/
   61,/* 14 degrees*/
   66,/* 15 degrees*/
   70,/* 16 degrees*/
   74,/* 17 degrees*/
   78,/* 18 degrees*/
   83,/* 19 degrees*/
   87,/* 20 degrees*/
   91,/* 21 degrees*/
   95,/* 22 degrees*/
   99,/* 23 degrees*/
  103,/* 24 degrees*/
  107,/* 25 degrees*/
  111,/* 26 degrees*/
  115,/* 27 degrees*/
  119,/* 28 degrees*/
  123,/* 29 degrees*/
  127,/* 30 degrees*/
  131,/* 31 degrees*/
  135,/* 32 degrees*/
  138,/* 33 degrees*/
  142,/* 34 degrees*/
  146,/* 35 degrees*/
  149,/* 36 degrees*/
  153,/* 37 degrees*/
  156,/* 38 degrees*/
  160,/* 39 degrees*/
  163,/* 40 degrees*/
  167,/* 41 degrees*/
  170,/* 42 degrees*/
  173,/* 43 degrees*/
  176,/* 44 degrees*/
  180,/* 45 degrees*/
  183,/* 46 degrees*/
  186,/* 47 degrees*/
  189,/* 48 degrees*/
  192,/* 49 degrees*/
  195,/* 50 degrees*/
  197,/* 51 degrees*/
  200,/* 52 degrees*/
  203,/* 53 degrees*/
  205,/* 54 degrees*/
  208,/* 55 degrees*/
  211,/* 56 degrees*/
  213,/* 57 degrees*/
  215,/* 58 degrees*/
  218,/* 59 degrees*/
  220,/* 60 degrees*/
  222,/* 61 degrees*/
  224,/* 62 degrees*/
  226,/* 63 degrees*/
  228,/* 64 degrees*/
  230,/* 65 degrees*/
  232,/* 66 degrees*/
  234,/* 67 degrees*/
  236,/* 68 degrees*/
  237,/* 69 degrees*/
  239,/* 70 degrees*/
  240,/* 71 degrees*/
  242,/* 72 degrees*/
  243,/* 73 degrees*/
  244,/* 74 degrees*/
  245,/* 75 degrees*/
  246,/* 76 degrees*/
  247,/* 77 degrees*/
  248,/* 78 degrees*/
  249,/* 79 degrees*/
  250,/* 80 degrees*/
  251,/* 81 degrees*/
  252,/* 82 degrees*/
  252,/* 83 degrees*/
  253,/* 84 degrees*/
  253,/* 85 degrees*/
  253,/* 86 degrees*/
  254,/* 87 degrees*/
  254,/* 88 degrees*/
  254,/* 89 degrees*/
  255 /* 90 degrees*/
};

unsigned int colortable[4];

typedef struct vecline{
  int ang;
  int len;
} VECLINE,*PVECLINE;

void tft_init(void);
void tft_from_vram(void);
void tft_sendcmd_n_byte(unsigned char cmd ,PGM_P p ,char cnt);
void vram_cls(void);
unsigned char vram_point(int x,int y);
void vram_pset(int x,int y,unsigned char color);
void vram_line(int x1 ,int y1 ,int x2 ,int y2 ,char color);
void vram_fill(int x1 ,int y1 ,int x2 ,int y2 ,char color);
void vram_locate(int textx, int texty);
void vram_textcolor(unsigned char color);
void vram_putch(unsigned char ch);
void vram_putstr(unsigned char *p);
void vram_putdec(unsigned int num);
void vram_puthex(unsigned int num);
void vram_scroll(int xd,int yd);
void vram_spput(char x,char y, int num,unsigned char color);
void vram_spclr(int x,int y);
void spi_sendbyte(unsigned char data);
int vect_x1(int a);
int vect_y1(int a);
void vect_put(PVECLINE pV,int vx,int vy,int angle,int zoom,unsigned char color);
unsigned int color16bit(int r,int g,int b);

// near sin(degree)
int vect_y1(int a)
{
  int ret;
  while(a < 0){
    a+=360;
  }
  while(a >= 360){
    a-=360;
  }

  if(a >= 270){              //270～359degrees
    a-=270;
    ret = (int)pgm_read_byte(vect_table + a);
    return(-ret);
  }
  if(a >= 180){              //180～269degrees
    a-=180;
    ret = (int)pgm_read_byte(vect_table + 90 - a);
    return(ret);
  }
  if(a >= 90){               //90～179degrees
    a-=90;
    ret = (int)pgm_read_byte(vect_table + a);
    return(ret);
  }
  ret = (int)pgm_read_byte(vect_table + 90 - a);
  return(-ret);  //0～90degrees
}

// near cos(degree)
int vect_x1(int a)
{
  return(vect_y1(a+90));
}

void vect_put(PVECLINE pV,int vx,int vy,int angle,int zoom,unsigned char color)
{
  int i;
  int tempkak;
  int x1,y1,x2=0,y2=0;

  for( i=0; pV->ang != -1; i++){
    tempkak = pV->ang + angle;
    x1 = vect_x1(tempkak)* pV->len;
    y1 = vect_y1(tempkak)* pV->len;
    x1 = (zoom*x1>>10) + vx;
    y1 = (zoom*y1>>10) + vy;
    if(i > 0){
      vram_line(x1,y1,x2,y2,color);
    }
    x2 = x1;
    y2 = y1;
    pV++;
  }
}

void tft_init(void)
{
  pinMode(SPIDAT, OUTPUT);
  pinMode(SPICLK, OUTPUT);
  pinMode(TFTCD, OUTPUT);
  pinMode(TFTCS, OUTPUT);
  pinMode(TFTRST, OUTPUT);
  digitalWrite(SPIDAT, HIGH);
  digitalWrite(SPICLK, LOW);
  digitalWrite(TFTCD, HIGH);
  digitalWrite(TFTCS, HIGH);
  digitalWrite(TFTRST, HIGH);

//  Serial.begin(115200);
//  while (!Serial) {
//  }

#if SOFT_SPI==0
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
//  SPI.setClockDivider(SPI_CLOCK_DIV4); 
  SPI.setClockDivider(SPI_CLOCK_DIV2); 
#endif
  
  delay(20);
  digitalWrite(TFTRST, LOW);  // reset
  delay(20);
  digitalWrite(TFTRST, HIGH);
  delay(500);
  digitalWrite(TFTCS,LOW);
  tft_sendcmd(0x11);
  delay(100);
  tft_sendcmd_byte(0x36, 0x00);  // MADCTL
  tft_sendcmd_byte(0x3A, 0x55);  // 16bit color/pixel
  // Frame rate
  tft_sendcmd_n_byte(0xB2,framerate_table,5);
  tft_sendcmd_byte(0xB7,0x75);
  // power
  tft_sendcmd_byte(0xC2,0x01);
  tft_sendcmd_byte(0xC3,0x10);
  tft_sendcmd_byte(0xC4,0x20);
  tft_sendcmd_byte(0xC6,0x0F);
  tft_sendcmd_word(0xB0,0x00F0); // RRRR RGGGG GGGB BBBB
  tft_sendcmd_word(0xD0,0xA4A1);
  // gamma
  tft_sendcmd(0x21);
  tft_sendcmd_byte(0xBB,0x3B);
  tft_sendcmd_n_byte(0xE0,gamma1_table,14);
  tft_sendcmd_n_byte(0xE1,gamma2_table,14);
  tft_sendcmd_long(0x2A,0x000000EF); 
  tft_sendcmd_long(0x2B,0x000000EF); 
  tft_sendcmd(0x29);    // Display on
  tft_sendcmd(0x2C);
  digitalWrite(TFTCS,HIGH);
  delay(50);
  colortable[0] = color16bit(  0,  0,  0); /*color 0*/
  colortable[1] = color16bit(  0,  0,255); /*color 1*/
  colortable[2] = color16bit(255,  0,  0); /*color 2*/
  colortable[3] = color16bit(255,255,255); /*color 3*/
}

void spi_sendbyte(unsigned char data)
{
#if SOFT_SPI==0
  SPI.transfer(data);
#else
  unsigned char bitmask;
  digitalWrite(SPICLK, LOW);
  bitmask = 0x80; // MSB FIRST
  while(bitmask){
    if(bitmask & data){
      digitalWrite(SPIDAT, HIGH);
    }else{
      digitalWrite(SPIDAT, LOW);
    }    
    digitalWrite(SPICLK, HIGH);
    digitalWrite(SPICLK, LOW);
    bitmask >>= 1;
  }
#endif
}

void tft_sendcmd(unsigned char data)
{
  delay(1);
  digitalWrite(TFTCD,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCD,HIGH);
}

void tft_send_data(unsigned char data)
{
  spi_sendbyte(data);
}

void tft_sendcmd_n_byte(unsigned char cmd ,PGM_P p ,char cnt)
{
  tft_sendcmd(cmd);
  while(cnt--){
    tft_send_data(pgm_read_byte(p++));
  }
}

void tft_sendcmd_byte(unsigned char cmd,unsigned char data)
{
  tft_sendcmd(cmd);
  tft_send_data(data);
}

void tft_sendcmd_word(unsigned char cmd,unsigned int data)
{
  tft_sendcmd(cmd);
  tft_send_data((unsigned char)(data >> 8));
  tft_send_data((unsigned char)(data & 0xff));
}

void tft_sendcmd_long(unsigned char cmd,unsigned long data)
{
  tft_sendcmd(cmd);
  tft_send_data((unsigned char)(data >> 24));
  tft_send_data((unsigned char)((data >> 16)& 0xff));
  tft_send_data((unsigned char)((data >> 8)& 0xff));
  tft_send_data((unsigned char)(data & 0xff));
}

void tft_fill(unsigned char x, unsigned char y, unsigned char width, unsigned char height,unsigned int color) {
  int loopcnt = width * height;

  digitalWrite(TFTCS,LOW);
  tft_sendcmd(0x2A);
  tft_send_data(0x00);
  tft_send_data(x);               // start X
  tft_send_data(0x00);
  tft_send_data(x + width - 1);   // end X

  tft_sendcmd(0x2B);
  tft_send_data(0x00);
  tft_send_data(y);               // start Y
  tft_send_data(0x00);
  tft_send_data(y + height - 1);  // end Y

  tft_sendcmd(0x2C);
  while(loopcnt--) {
    tft_send_data(color >> 8);
    tft_send_data(color & 0xff);
  }
  digitalWrite(TFTCS,HIGH);
}

// send VRAM to TFT 
void tft_from_vram(void)
{
  unsigned char x,y,ym;
  unsigned char shift,colorh,colorl;
  unsigned int color;
  unsigned char *ptr;

  digitalWrite(TFTCS,LOW);
  tft_sendcmd(0x2A);
  tft_send_data(0x00);
  tft_send_data(0);               // start X
  tft_send_data(0x00);
  tft_send_data((VRAMWIDTH*4) - 1);   // end X

  tft_sendcmd(0x2B);
  tft_send_data(0x00);
  tft_send_data(0);               // start Y
  tft_send_data(0x00);
  tft_send_data((VRAMHEIGHT*4) - 1);  // end Y

  tft_sendcmd(0x2C);
  ptr = (unsigned char *)vram;
  for(y=0; y<VRAMHEIGHT; y++){
    for(ym=0; ym<4; ym++){
      for(x=0; x<VRAMWIDTH; x++){
        shift = (x % 4)*2;
        color = (*(ptr+(x/4)) >> shift)& 0b11;
        color = colortable[color];
        colorh = color >> 8;
        colorl = color & 0xff;
//      tft_send_data(color >> 8);
//      tft_send_data(color & 0xff);
        spi_sendbyte(colorh);
        spi_sendbyte(colorl);
        spi_sendbyte(colorh);
        spi_sendbyte(colorl);
        spi_sendbyte(colorh);
        spi_sendbyte(colorl);
        spi_sendbyte(colorh);
        spi_sendbyte(colorl);
      }
    }
    ptr += (VRAMWIDTH/4);
  }
  digitalWrite(TFTCS,HIGH);
  delay(5);
}

unsigned int color16bit(int r,int g,int b)
{
// blue :bit4 -bit0 (0-31)
// green:bit10-bit5 (0-63)
// red  :bit15-bit11(0-31)
  r >>= 3;
  g >>= 2;
  b >>= 3;
  return(((unsigned int)r << 11)+(g << 5)+b);
}

// color test
void tft_colortest(void)
{
  unsigned char width;
  unsigned char height;
  int x,y,pitch,ymod;
  char mode;
  unsigned int r,g,b,color;

  width=240;
  height=240;
  digitalWrite(TFTCS,LOW);
  tft_sendcmd(0x2A);
  tft_send_data(0x00);
  tft_send_data(0);           // start X
  tft_send_data(0x00);
  tft_send_data(width - 1);   // end X

  tft_sendcmd(0x2B);
  tft_send_data(0x00);
  tft_send_data(0);           // start Y
  tft_send_data(0x00);
  tft_send_data(height - 1);  // end Y

  tft_sendcmd(0x2c);
  color = 0;
  pitch = height / 4;
  for(y=0; y<height; y++){
    for(x=0; x<width; x++){
      mode = y / pitch;
      ymod = y % pitch;
      if(mode==0) color = (ymod*32/pitch);     //blue
      if(mode==1) color = (ymod*64/pitch)<<5;  //green
      if(mode==2) color = (ymod*32/pitch)<<11; //red
      if(mode==3){ //grayscale
        r=ymod*32/pitch;
        g=ymod*64/pitch;
        b=r;
        color = (r<<11)+(g<<5)+b; 
      }
      tft_send_data(color >> 8);
      tft_send_data(color & 0xff);
    }
  }
  digitalWrite(TFTCS,HIGH);
  delay(5000);
}

void vram_cls(void)
{
  int i;
  unsigned char *ptr;

  ptr = (unsigned char *)vram;
  i = VRAMSIZE;
  while(i--){
    *ptr = 0;
    ptr++;
  }
}

//get pixel(X座標,Y座標)
unsigned char vram_point(int x,int y)
{
  int i;
  unsigned char color;
  char shift;
  unsigned char *ptr;

#if ROTATE==1
  i=x;
  x=(VRAMWIDTH-1)-y;
  y=i;
#endif
#if ROTATE==2
  x=(VRAMWIDTH-1)-x;
  y=(VRAMHEIGHT-1)-y;
#endif
#if ROTATE==3
  i=x;
  x=y;
  y=(VRAMHEIGHT-1)-i;
#endif

  if(x<0)return(0);
  if(y<0)return(0);
  if(x>=VRAMWIDTH)return(0);
  if(y>=VRAMHEIGHT)return(0);

  shift = (x % 4)*2;
  ptr = &vram[y*(VRAMWIDTH/4)+ (x/4)];
  color = (*ptr >> shift)& 0b11; 
  return(color);
}

//draw pixel(X座標,Y座標,カラーコード)
void vram_pset(int x,int y,unsigned char color)
{
  int i;
  char shift;
  unsigned char *ptr;

#if ROTATE==1
  i=x;
  x=(VRAMWIDTH-1)-y;
  y=i;
#endif
#if ROTATE==2
  x=(VRAMWIDTH-1)-x;
  y=(VRAMHEIGHT-1)-y;
#endif
#if ROTATE==3
  i=x;
  x=y;
  y=(VRAMHEIGHT-1)-i;
#endif

  if(x<0)return;
  if(y<0)return;
  if(x>=VRAMWIDTH)return;
  if(y>=VRAMHEIGHT)return;

  shift = (x % 4)*2;
  ptr = &vram[y*(VRAMWIDTH/4)+ (x/4)];
  *ptr &= ~(0b11 << shift); 
  *ptr |= (color << shift);
}

//box fill(X1,Y1,X2,Y2,color)
void vram_fill(int x1 ,int y1 ,int x2 ,int y2 ,char color)
{
  int x,y;

  for(y=y1; y<=y2; y++){
    for(x=x1; x<=x2; x++){
      vram_pset(x, y ,color); //ドット描画
    }
  }
}

//draw line(X1,Y1,X2,Y2,color)
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
  vram_pset (x1, y1 ,color); //ドット描画
  e = 0;
  if( yd < xd ) {
    while( x1 != x2) {
      x1 += xs;
      e += (2 * yd);
      if(e >= xd) {
        y1 += ys;
        e -= (2 * xd);
      }
      vram_pset (x1, y1 ,color); //ドット描画
    }
  }else{
    while( y1 != y2) {
      y1 += ys;
      e += (2 * xd);
      if(e >= yd) {
        x1 += xs;
        e -= (2 * yd);
      }
      vram_pset (x1, y1 ,color); //ドット描画
    }
  }
}

//locate(x,y)
void vram_locate(int textx, int texty)
{
  def_posx = textx;
  def_posy = texty;
}

//
void vram_textcolor(unsigned char color)
{
  def_tcolor = color;
}

//print character(x,y,character,color)
void vram_putch(unsigned char ch)
{
  char i,j;
  unsigned char bitdata;
  PGM_P p;

  if(ch < 0x20)return;
  if(def_posx > (VRAMXMAX-7)){
    def_posx = 0;
    def_posy += 8;
    if(def_posy > VRAMYMAX){
      def_posy = VRAMYMAX-7;    
      vram_scroll(0,8);
    }
  }  

  ch -= 0x20;
  p = font +((int)ch * 8);
  for(i=0 ;i<8 ;i++) {
    bitdata = pgm_read_byte(p++);
    for(j=0; j<8; j++){
      if(bitdata & 0x80){
        vram_pset(def_posx+j,def_posy+i,def_tcolor);
      }
      bitdata <<= 1;
    }
  }
  def_posx += 8;
}

// put string
void vram_putstr(unsigned char *p)
{
  while(*p != 0){
    vram_putch( *p++ );
  }
}

//
void vram_putdec(unsigned int num)
{
    unsigned char ch;
    unsigned int shift=10000;
  
  while(shift > 0){
    ch = (num / shift) % 10;
    ch += '0';
    vram_putch(ch);
    shift /= 10;
  }
}

// print Hexadecimal number
void vram_puthex(unsigned char num)
{
  unsigned char ch;
  char shift=4;

  while(shift >= 0){
    ch = (num >> shift) & 0xf;
    if(ch < 10){
      ch += '0';
    }else{
      ch += ('A'-10);
    }
    vram_putch( ch);
    shift -= 4;
  }
}

#define VRSPSIZE 7
// print sprite
void vram_spput(char x,char y, int num,unsigned char color)
{
  char i,j;
  unsigned char dat;
  PGM_P p;

  p = sp_pattern + (num * VRSPSIZE);
  x -= (VRSPSIZE/2);
  y -= (VRSPSIZE/2);

  for(j=0; j<VRSPSIZE; j++){
    dat = pgm_read_byte(p++);
    for(i=0; i<VRSPSIZE; i++){
      if(dat & (1<<(VRSPSIZE-1))){
        vram_pset(x+i,y+j,color);
      }
      dat <<= 1;
    }
  }
}

void vram_spclr(int x,int y)
{
  char i,j;

  x -= (VRSPSIZE/2);
  y -= (VRSPSIZE/2);

  for(j=0; j<VRSPSIZE; j++){
    for(i=0; i<VRSPSIZE; i++){
      vram_pset(x+i,y+j,0);
    }
  }
}

//scroll(delta x,delta y)
void vram_scroll(int xd,int yd)
{
  int x,y;
  unsigned char color;

  for(y=0;y<(VRAMYMAX+1);y++){
    for(x=0;x<(VRAMXMAX+1);x++){
      color = vram_point(x+xd, y+yd);
      vram_pset(x,y,color);
    }
  }
}

//----
void setup(void)
{
  tft_init();

//  Serial.begin(115200); //debug
//  while (!Serial) {
//  }
}

void chardemo(void)
{
  int x,y;
  unsigned char chrnum;
  unsigned char color;
  int timeout;

  vram_cls();
  colortable[1] = color16bit(128,255,  0); /*color 1*/
  colortable[2] = color16bit(  0,128,255); /*color 2*/
  chrnum=0x20;
  vram_locate(0, VRAMYMAX-7);
  color = 3;
  vram_textcolor(color);
  timeout=100;
  while(timeout--){
    vram_putch(chrnum);
    color++;
    if(color > 3)color=1;
    vram_textcolor(color);
    chrnum++;
    if(chrnum > 0x7f){
      chrnum=0x20;
    }
    tft_from_vram();
  }
}
  
void linedemo(void)
{
  int xa,ya,xb,yb;
  int xa1,ya1,xb1,yb1;
  unsigned char color;
  int timeout;

  vram_cls();   // clear VRAM
  xa =random(VRAMXMAX-1)+1;
  ya =random(VRAMYMAX-1)+1;
  xb =random(VRAMXMAX-1)+1;
  yb =random(VRAMYMAX-1)+1;
  xa1 =random(5)-2;
  ya1 =random(5)-2;
  xb1 =random(5)-2;
  yb1 =random(5)-2;

  color=0;
  timeout = 100;
  while(timeout--){
    xa += xa1;
    ya += ya1;
    xb += xb1;
    yb += yb1;
    vram_line(xa ,ya ,xb ,yb ,(color/4)& 3);
    if((xa<=0)||(xa>=VRAMXMAX))xa1 = -xa1;
    if((ya<=0)||(ya>=VRAMYMAX))ya1 = -ya1;
    if((xb<=0)||(xb>=VRAMXMAX))xb1 = -xb1;
    if((yb<=0)||(yb>=VRAMYMAX))yb1 = -yb1;
    color++;
    tft_from_vram();
  }
}

int fnc_abs(int a)
{
  if(a<0)a = -a;
  return (a);
}

int fnc_sgn(int a)
{
  if(a<0)return(-1);
  return (1);
}

void balldemo(void)
{
  typedef struct ball{
    int x;
    int y;
    int x1;
    int y1;
  } ST_BALL;   // 構造体

  #define BALLMAX 14
  ST_BALL ball[BALLMAX];

  #define MLT 3

  int x2,y2;
  int xd,yd;
  unsigned char i,j;
  int timeout;

  colortable[1] = color16bit(255,  0,255); /*color 1*/

  for(i=0;i<BALLMAX;i++){
    ball[i].x = random(VRAMXMAX)*MLT;
    ball[i].y = random(VRAMYMAX)*MLT;
    ball[i].x1 = random(MLT*2)-MLT;
    ball[i].y1 = random(MLT*2)-MLT;
  }

  timeout=100;
  while(timeout--){
    vram_cls();
    
    for(i=0; i<BALLMAX; i++){
      x2 = ball[i].x;
      y2 = ball[i].y;

      x2 += ball[i].x1;
      y2 += ball[i].y1;
      for(j=0 ;j<BALLMAX ;j++){
        if(i==j)continue;
        xd = x2-ball[j].x;
        yd = y2-ball[j].y;

        if( (fnc_abs(xd) < 7*MLT)&&(fnc_abs(yd) < 7*MLT)){
          ball[i].x1 = fnc_sgn(xd)*(random(MLT*2)+1);
          ball[i].y1 = fnc_sgn(yd)*(random(MLT*2)+1);
          ball[j].x1 = -ball[i].x1;
          ball[j].y1 = -ball[i].y1;
        }
      }
      ball[i].x = x2;
      ball[i].y = y2;

      x2/=MLT;
      y2/=MLT;
      if(x2 <= 3) ball[i].x1 = (random(MLT*2)+1);
      if(x2 >= (VRAMXMAX-3)) ball[i].x1 = -(random(MLT*2)+1);
      if(y2 <= 3) ball[i].y1 = (random(MLT*2)+1);
      if(y2 >= (VRAMYMAX-3)) ball[i].y1 = -(random(MLT*2)+1);
      
      vram_spput(x2,y2,6,(i % 3)+1);
    }
    tft_from_vram();
  }
}

//vector
void vectordemo(void)
{
  typedef struct inseki{
    int x;
    int y;
    int x1;
    int y1;
    int ang;
    int a1;
    int zoom;
  } INSEKI, *PINSEKI;
  #define OFST 5
  #define INSEKIMAX 10
  INSEKI insk[INSEKIMAX];
  PINSEKI pInsk;
  int i;
  int timeout;
  int tx,ty;
  VECLINE myship[5] = {
    {  40 ,6},  /*  */
    {  320,6},  /*  */
    {  180,6},  /*  */
    {  40 ,6},  /*  */
    {  -1 ,-1}  /*  */
  };

  for(i=0; i<INSEKIMAX; i++){
    insk[i].x=-999;
    insk[i].y=0;
    insk[i].x1=0;
    insk[i].y1=0;
  }

  colortable[2] = color16bit(255,255,  0); /*color 2*/

  timeout=100;
  while(timeout--){
    vram_cls();   // clear VRAM

    pInsk = &insk[0];
    for(i=0; i<INSEKIMAX; i++,pInsk++){
      tx = pInsk->x;
      ty = pInsk->y;
      tx += pInsk->x1;
      ty += pInsk->y1;
      if((ty< -(8<<OFST))||(ty>((VRAMYMAX+8)<<OFST))||(tx < -(8<<OFST))||(tx>((VRAMXMAX+8)<<OFST))){
        if(random(2)){
          pInsk->x = -8<<OFST;
          pInsk->x1 = random(20)+30;
        }else{
          pInsk->x = (VRAMXMAX+8)<<OFST;
          pInsk->x1 = -random(20)-30;
        }
        pInsk->ang = random(36)*10;
        pInsk->y = random(6)*((VRAMYMAX<<OFST)/6);
        pInsk->y1 = random(13)-7;
        pInsk->a1 = random(13)-6;
        pInsk->zoom = random(8)+4;
        continue;
      }
      vect_put((PVECLINE)myship, tx>>OFST, ty>>OFST, pInsk->ang, pInsk->zoom,(i%3)+1);
      pInsk->x = tx;
      pInsk->y = ty;
      pInsk->ang += pInsk->a1;
      if(pInsk->ang >=360) pInsk->ang -= 360;
      if(pInsk->ang <   0) pInsk->ang += 360;
    }
    tft_from_vram();
  }
}

//life game
void lifegame(void)
{
  #define WSIZE (VRAMXMAX/2)

  int x,y;
  int ax,bx;
  unsigned char color;
  char touch;
  int i;
  int timeout;
  int lx1[8]={ 0, 1, 1, 1, 0,-1,-1,-1};
  int ly1[8]={-1,-1, 0, 1, 1, 1, 0,-1};

  vram_cls();
  for(i=0; i<500; i++){
    x= random(WSIZE);
    y= random(WSIZE);
    vram_pset(x,y,3);
  }

  ax=0;
  bx=WSIZE;
  timeout=100;
  while(timeout--){
    for(y=0; y<WSIZE; y++){
      for( x=0; x<WSIZE; x++){
        color=0;
        touch=0;
        for( i=0;i<8;i++){
          if(vram_point(ax+x+lx1[i],y+ly1[i])){
            touch++;
            if(touch>=4){
              break;
            }
          }
        }
        if(touch==2) color=vram_point(ax+x,y);
        if(touch==3) color=3;
        vram_pset(bx+x,y,color);
      }
    }
    x=ax; //swap ax,bx
    ax=bx;
    bx=x;
    tft_from_vram();
  }
}

// space fight
void spacefight(void)
{
#define TEKIMAX 15
#define MOVEPITCH 30  //移動カウント
#define MOVESEQ   (10+10+2+2) //移動シーケンス
const unsigned char strready[]="READY";
const unsigned char strover[]="GAME OVER";

  int tx[TEKIMAX]; //敵座標
  int ty[TEKIMAX];
  int tb[TEKIMAX];
  char ttime; //Enemy Timing
  char tmove; //Enemy Moving
  char gamespeed=0; //game speed
  int score=0;  //Score
  char tnum;  //Enemy Count
  int ax=0,ay; //My Ship X,Y
  int bx,by; //My Ship Beam
  int cx,cy; //Enemy Beam
  int x,y;
  int i;
  char overflag=1;
  int timeout;

  colortable[1] = color16bit(255,128,  0); /*color 1*/
  colortable[2] = color16bit(255,  0,128); /*color 2*/
  timeout=100;
  while(timeout){
    vram_cls();
    if(overflag){
      overflag=0;
      vram_textcolor(1);
      vram_locate(13,VRAMYMAX/2);
      vram_putstr((unsigned char *)strready);
      score=0;
      gamespeed=MOVEPITCH;
      ax=VRAMXMAX/2;  //プレーヤ座標
      ay=VRAMYMAX-5;
    }
    vram_spput(ax, ay, 0,1);    //my ship
    bx=by=-1;   //自弾座標
    cx=cy=-1;   //敵弾座標

    for(i=0;i<TEKIMAX;i++){
      tx[i] = ((i % 5)*(VRAMXMAX/5))+(VRAMXMAX/8);
      ty[i] = ((i / 5)*(VRAMXMAX/5))+(VRAMXMAX/8);
      tb[i] = 0;
    }

    tmove=0;
    ttime=0;
    tft_from_vram();
    delay(1000);

    while(overflag==0){
      vram_cls(); 
//      ax += joy_getx()/60;
      if(ax<8)ax=8;
      if(ax>VRAMXMAX-8)ax=VRAMXMAX-8;
      vram_spput(ax, ay, 0,1);    //my ship

      if(by < -1){
        bx=ax;
        by=ay;
      }else{
        by-=2;
        vram_spput(bx,by,5,3);  //my meam
      }

      //Enemy Beam
      if(cy == -1){
        i=random(TEKIMAX);
        if(ty[i] != -1){
          cx = tx[i];
          cy = ty[i];
        }
      }else{
        cy++;
        if(cy > VRAMYMAX){
          cy = -1;
        }else{
          vram_spput(cx,cy,3,3);
        }
      }

      ttime = (ttime+1)%gamespeed;
      if (ttime==0) {
        tmove=(tmove+1) % MOVESEQ;
      }

      tnum=0;
      for(i=0 ;i<TEKIMAX ;i++) {
        x=tx[i];
        y=ty[i];
        if(y == -1)continue;
        tnum++;

        if(tb[i]){
          tb[i]--;
          vram_spput(x,y,4,1);
          if(tb[i]==0)
            ty[i]=-1;
          continue;
        }
        if((fnc_abs(by-y)<5)&&(fnc_abs(bx-x)<5)){ //命中
          if(score < 9999)score++;
          if(gamespeed > 2)gamespeed-=1;
          tb[i]=15;
          by=-1;
          continue;
        }
        if(ttime==0){
          if(tmove < 10){//01234
            x+=2;
          }else if(tmove < (10+2)){//5
            y+=2;
          }else if(tmove < (10+2+10)){//6789a
            x-=2;
          }else{
            y+=2;
          }
          if(y >= VRAMYMAX)
            overflag=1;
        }
        tx[i]=x;
        ty[i]=y;
        vram_spput(x,y ,1+(tmove & 1),2);
        continue;
      }
      if((fnc_abs(cy-ay)<5)&&(fnc_abs(cx-ax)<5)){
        overflag=1;
      }

      if(tnum==0){  //敵全滅
        gamespeed+=8;
        break;
      }
      tft_from_vram();
      timeout--;
      if(timeout==0)break;
    }
    if(overflag){ //game over
      vram_spclr(ax,ay);
      vram_spput(ax, ay, 4,1);    //------自機をvramに転送
      vram_locate(8,VRAMYMAX/4);
      vram_putstr((unsigned char *)strover);
      vram_locate(8,VRAMYMAX*3/4);
      vram_putdec(score);
      tft_from_vram();
      delay(1000);
    }
  }
}

void randscape(void)
{
#define RANDSCAPEY1  ((VRAMYMAX/2)-3)
#define RANDSCAPEY2 ((VRAMYMAX/2)+3)
#define DEGPITCH  15
  int x1,x2,y,i,deg,iso=0;
  int timeout;

  colortable[1] = color16bit(  0,255,255); /*color 1*/
  colortable[2] = color16bit(  0,255,  0); /*color 2*/
  timeout=100;
  while(timeout--){
    iso += 3;
    if(iso< 0       )iso += DEGPITCH;
    if(iso>=DEGPITCH)iso -= DEGPITCH;

    vram_cls();
    for(i=-10; i<10; i++){
      x1=i*6;
      x2=i*25;
      vram_line((VRAMXMAX/2)+x1,RANDSCAPEY1,(VRAMXMAX/2)+x2, 0,1);
      vram_line((VRAMXMAX/2)+x1,RANDSCAPEY2,(VRAMXMAX/2)+x2,VRAMYMAX,2);
    }

    for(deg=0; deg<90; deg+=DEGPITCH){
      y=(255+vect_y1(deg+iso))*RANDSCAPEY1/255;
      vram_line(0,RANDSCAPEY1-y,VRAMXMAX,RANDSCAPEY1-y,1);
      vram_line(0,RANDSCAPEY2+y,VRAMXMAX,RANDSCAPEY2+y,2);
    }
    tft_from_vram();
  }
}

void vmeter(void)
{
  int x,y;
  int adc;
  int deg;
  int timeout;

  vram_cls();
  colortable[1] = color16bit(  0,  0,255); /*color 1*/
  colortable[2] = color16bit(255,  0,  0); /*color 2*/
  deg=0;
  timeout=100;
  while(timeout--)
  {
    adc = vect_x1(deg);
    vram_scroll(2,0);
    x = VRAMXMAX;
    for(y=0; y<VRAMYMAX+1; y+=10){
        vram_pset(x, y ,1);
    }
    y = (VRAMYMAX/2)-(adc*(VRAMYMAX/2)/255); 
    vram_pset(x, y,2);
    if((deg % 90) == 0){
      for(y=0; y<VRAMYMAX+1; y+=2){
        vram_pset(x, y ,3);
      }
    }
    deg = (deg+5)% 360;
    tft_from_vram();
  }
}

void loop()
{
  tft_colortest();
  vmeter();
  randscape();
  spacefight();
  vectordemo();
  lifegame();
  balldemo();
  chardemo();
  linedemo();
}
