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

// test pattern
void tft_display_test( void ){
  int x,y,pitch,ymod;
  char mode;
  unsigned int r,g,b,color;
  unsigned char width,height;

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
// blue :bit4 -bit0 (0-31)
// green:bit10-bit5 (0-63)
// red  :bit15-bit11(0-31)
    }
  }
  digitalWrite(TFTCS,HIGH);
}

void setup(void)
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
  tft_display_test();
}
 
void loop()
{
}
