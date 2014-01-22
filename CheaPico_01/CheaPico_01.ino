#include <string.h>
#include <util/crc16.h>

#include <Wire.h>
#include <stdlib.h>

#include <JeeLib.h>
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

#include <SPI.h>
#include <RFM22.h>
 
#define RADIO_FREQUENCY 434.408
#define RADIO_CALLSIGN "BEACON"

#define RFM22B_SDN 3
#define RFM22B_PIN 10
rfm22 radio1(RFM22B_PIN);

 
#include <PString.h>
static char datastring[110];
PString str(datastring, sizeof(datastring));

int cnt = 0;
int rfm22_temp = 0;

#define ALT_CORRECTION 130


void setup() {
  //Serial.begin(9600);

  setupRadio();
  //rtty_txstring("\n$$booting..\n");

  Wire.begin();
  bmp085Calibration();

  pinMode(A2, INPUT);
  digitalWrite(A2, HIGH);
}
 
void loop() {
  
  //if (cnt % 2 == 0) {
  //if (true) {
  if (cnt > 0) {
    for (byte i = 0; i < 10; i++) {
      // radio on
      setupRadio();
      if (i > 6)
        Sleepy::loseSomeTime(50); // minimum watchdog granularity is 16 ms
      //if (i > 4)
        rtty_txbyte(0x80);
      if (i > 6)
        rtty_txbyte(0x80);
  
      // radio off
      radio1.write(0x07, 0x01);
      digitalWrite(RFM22B_SDN, HIGH);
      
      Sleepy::loseSomeTime(1000);
      if (i < 7)
        Sleepy::loseSomeTime(1000);

    }
    // radio on
    setupRadio();
    // radio warm up
    Sleepy::loseSomeTime(1000);
  }

   
  float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  long pressure = bmp085GetPressure(bmp085ReadUP());
  //float atm = pressure / 101325; // "standard atmosphere"
  long altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 
  altitude += ALT_CORRECTION;
 
  // tsrange = 0x00 = –64 C .. 64 C (full operating range), with 0.5 C resolution (1 LSB in the 8-bit ADC)
  // tvoffs = 0 = zero offset
  rfm22_temp = (temperatureRead(0x00, 0) / 2) - 64;  //get temp from RFM22b, 0.5 deg C per 
  rfm22_temp -= 10; // Sensor has at least a 10 degree positive offset.
 
  analogRead(A2);
  Sleepy::loseSomeTime(200);
  unsigned int rawvcc = analogRead(A2);
  //float voltage = rawvcc / 1023.0 * 3.3; // in millivolts
  unsigned int voltage = map(rawvcc, 0, 1023, 0, 3300);
 
  str.begin();
  str.print(F("$$"));
  str.print(F(RADIO_CALLSIGN));
  str.print(",");
  str.print(cnt);
  str.print(",");
  str.print(altitude);
  str.print(",");
  str.print(rfm22_temp);
  str.print(",");
  str.print(temperature, 0);
  //str.print(",");
  //str.print(pressure);
  str.print(",");
  str.print(voltage);
  //str.print(",");
  //str.print(rawvcc);

  /*for (char i = 0; datastring[i] != 0; i++){
    char info = datastring[i];*/


  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);


  for (byte i = 0; i < 10; i++)
    rtty_txbyte(0x80);

//  if (cnt % 2 == 0)
    rtty_txstring("\n");
  rtty_txstring("$$");
  rtty_txstring(datastring);

  //if (cnt % 2 == 1) {
  if (true) {
    // turn off
    radio1.write(0x07, 0x01);
    digitalWrite(RFM22B_SDN, HIGH);
  }

  Sleepy::loseSomeTime(2000);

  cnt++;
}

void setupRadio() {
  pinMode(RFM22B_SDN, OUTPUT);    // RFM22B SDN is on ARDUINO A3
  digitalWrite(RFM22B_SDN, LOW);
  Sleepy::loseSomeTime(1000);

  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  // Upu's board
  //radio1.write(0x0b,0x15);
  //radio1.write(0x0c,0x12);

  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.write(0x6D, 0x04);// turn tx low power 11db
  radio1.write(0x07, 0x08);

  //Sleepy::loseSomeTime(500);
}

//----------------------------------

void rtty_txstring (char * string) { 
  /* Simple function to sent a char at a time to 
   	** rtty_txbyte function. 
   	** NB Each char is one byte (8 Bits)
   	*/
 
  char c;
 
  c = *string++;
 
  while ( c != '\0')
  {
    rtty_txbyte(c);
    c = *string++;
  }
}
 
 
void rtty_txbyte (char c) {
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	**
   	*/

  //Serial.print(c);

  int i;
 
  rtty_txbit(0); // Start bit
 
  // Send bits for for char LSB first	
 
  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0);	
 
    c = c >> 1;
 
  }
 
  rtty_txbit(1); // Stop bit
  rtty_txbit(1); // Stop bit
}

void rtty_txbit (int bit) {
  if (bit)
  {
    radio1.write(0x74,0x00); // High  - Need to set high as zero and low as less than zero due to resetting radio every time around loop.x
    radio1.write(0x73,0x00); // High
  }
  else
  {
    radio1.write(0x74,0xFF); // 10 bit Low 2's compliment  http://www.mutter.in/binary-twos-complement-calculator
    radio1.write(0x73,0xFD); // 10 bit Low 2's compliment  Section 3.5.5 http://www.sparkfun.com/datasheets/Wireless/General/RFM22B.pdf
  }
  //delayMicroseconds(19500); // 50 baud - 20150
  delayMicroseconds(20150);
  //delayMicroseconds(10000); // 100 baud 
  //delayMicroseconds(3370); // 300 baud
}
 
uint16_t gps_CRC16_checksum(char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}


// https://github.com/jamescoxon/PicoAtlas/blob/master/Pico7/Pico7.ino
//Taken from RFM22 library + navrac
uint8_t adcRead(uint8_t adcsel) {
    uint8_t configuration = adcsel;
    radio1.write(0x0f, configuration | 0x80);
    radio1.write(0x10, 0x00);

    // Conversion time is nominally 305usec
    // Wait for the DONE bit
    while (!(radio1.read(0x0f) & 0x80))
	;
    // Return the value  
    return radio1.read(0x11);
}

uint8_t temperatureRead(uint8_t tsrange, uint8_t tvoffs) {
    radio1.write(0x12, tsrange | 0x20);
    radio1.write(0x13, tvoffs);
    return adcRead(0x00 | 0x00); 
}


//------------------------------------
/*Based largely on code by  Jim Lindblom

  Get pressure, altitude, and temperature from the BMP085.
  Serial.print it out at 9600 baud to serial monitor.
*/


#define BMP085_ADDRESS 0x77  // I2C address of BMP085

const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 


// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

float calcAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}
