#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#define CS_MCP3208  6        // BCM_GPIO 25

#define SPI_CHANNEL 0
#define SPI_SPEED   1000000  // 1MHz


int read_mcp3208_adc(unsigned char adcChannel)
{
  unsigned char buff[3];
  int adcValue = 0;

  buff[0] = 0x06 | ((adcChannel & 0x07) >> 2);
  buff[1] = ((adcChannel & 0x07) << 6);
  buff[2] = 0x00;

  digitalWrite(CS_MCP3208, 0);  // Low : CS Active

  wiringPiSPIDataRW(SPI_CHANNEL, buff, 3);

  buff[1] = 0x0F & buff[1];
  adcValue = ( buff[1] << 8) | buff[2];

  digitalWrite(CS_MCP3208, 1);  // High : CS Inactive
  adcValue = adcValue>>2;
  //adcValue = (adcValue^(-1.15))*12343;
  return adcValue;
}


int main (void)
{
  int adcChannel = 0;
  int adcValue   = 0;
  float distance=0;
  if(wiringPiSetup() == -1)
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror(errno));
    return 1 ;
  }

  if(wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1)
  {
    fprintf (stdout, "wiringPiSPISetup Failed: %s\n", strerror(errno));
    return 1 ;
  }

  pinMode(CS_MCP3208, OUTPUT);

  while(1)
  {
    adcValue = read_mcp3208_adc(adcChannel);
	//distance = (6787/(adcValue*5/1023 -3))-4;
	//printf("adc0 Value = %f\n",distance );
    printf("adc0 Value = %u\n",adcValue );
	//printf ("7 ^ 3 = %f\n", pow (adcValue,-1.15)*12343 );
	delay(200);
  }

  return 0;
}
