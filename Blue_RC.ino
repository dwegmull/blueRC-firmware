
// Blue RC firmware
// By David Wegmuller
// This code is provided "as is" in the hope it will be helpful.
// You may use it any way you want.
// I'm not responsible if it doesn't work or causes any damages.
// Send your questions to david at wegmuller dot org

#include <EEPROM.h>
#include <Servo.h>
// Pins
#define PIN_THROTTLE  2
#define PIN_REVERSE1  3
#define PIN_REVERSE2  4
#define PIN_DRAIN1    5
#define PIN_DRAIN2    6
#define PIN_WHISTLE   7
#define NUMBER_OF_SERVOS 6

// Timeout related
#define DEFAULT_TIMEOUT      4 * 60 // four Minutes in Seconds

// Serial and protocol definitions
#define SERIAL_BUFF_SIZE      100
#define SERIAL_START_RX       "@"
#define SERIAL_END_RX         '!'
#define SERIAL_START_TX       '#'
#define SERIAL_END_TX         '?'
#define RX_COMMAND_OFFSET     0
#define RX_REGISTER_HI        1
#define RX_REGISTER_LO        2
#define RX_SIZE_HI            3
#define RX_SIZE_LO            4
#define RX_DATA_START         5
#define TX_START              0
#define TX_RESPONSE_LO        1
#define TX_RESPONSE_HI        2
#define TX_DATA_START         3
#define TX_END_BASE           4
#define COM_COMMAND           'C'
#define COM_QUERY             'Q'
#define COM_KEEPALIVE         'K'
#define RESP_LOW_AN           'A'
#define RESP_HIGH_AN          'N'

// Register locations
#define  REG_THROTTLE         0
#define  REG_REVERSE1         1
#define  REG_REVERSE2         2
#define  REG_DRAIN1           3
#define  REG_DRAIN2           4
#define  REG_WHISTLE          5
#define  REG_EEPROM_VALID     6
#define  REG_FEATURES         7
#define  REG_LO_THROTTLE      8
#define  REG_HI_THROTTLE      9
#define  REG_LO_REVERSE1      10
#define  REG_MI_REVERSE1      11
#define  REG_HI_REVERSE1      12
#define  REG_LO_REVERSE2      13
#define  REG_MI_REVERSE2      14
#define  REG_HI_REVERSE2      15
#define  REG_LO_DRAIN1        16
#define  REG_HI_DRAIN1        17
#define  REG_LO_DRAIN2        18
#define  REG_HI_DRAIN2        19
#define  REG_LO_WHISTLE       20
#define  REG_HI_WHISTLE       21
#define  REG_DISC_TIMEOUT     22
#define  REG_VERSION_H        23
#define  REG_VERSION_F        24
#define  REG_VERSION_P        25
#define  REG_SAFE_THROTTLE    26
#define  REG_SAFE_REVERSE1    27
#define  REG_SAFE_REVERSE2    28
#define  REG_SAFE_DRAIN1      29
#define  REG_SAFE_DRAIN2      30
#define  REG_SAFE_WHISTLE     31
#define  REG_DESC_SIZE        32
#define  REG_DESCRIPTION      33
#define  MAX_REGISTERS        255
#define  START_OF_EEPROM      REG_EEPROM_VALID
#define  REG_SAFE_BASE        REG_SAFE_THROTTLE

Servo servos[NUMBER_OF_SERVOS];
char serialBuff[SERIAL_BUFF_SIZE];
char timeout;
unsigned long timeCt;

// Set the servos to a safe position: either the
// one stored in EEPROM, if available or mid point otherwise.
void set_servos_safe()
{
  char loopCt = NUMBER_OF_SERVOS - 1;
  // check the EEPROM for valid data
  if(0x42 == EEPROM.read(REG_EEPROM_VALID - START_OF_EEPROM))
  {
    // Get default servo position from the EEPROM
    do
    {
      servos[loopCt].write(EEPROM.read(REG_SAFE_BASE + loopCt - START_OF_EEPROM));
    }
    while(loopCt--);
    timeout = EEPROM.read(REG_DISC_TIMEOUT - START_OF_EEPROM);
  }
  else
  {
    // No calibration data: use 50% (90 degrees)
    do
    {
      servos[loopCt].write(90);
    }
    while(loopCt--);
    timeout = DEFAULT_TIMEOUT;
  }
}
char digit2char(char digit)
{
  if(('0' <= digit) && ('9' >= digit))
  {
    return (digit - '0');
  }
  else
  {
    if (('A' <= digit) && ('Z' >= digit))
    {
      return (digit - 'A');
    }
  }
  return 0;
}

char ASCII2char(char upper, char lower)
{
  return (digit2char(upper) << 4) + digit2char(lower); 
}

void setup()
{
  Serial.begin(57600);
  servos[REG_THROTTLE].attach(PIN_THROTTLE);
  servos[REG_REVERSE1].attach(PIN_REVERSE1);
  servos[REG_REVERSE2].attach(PIN_REVERSE2);
  servos[REG_DRAIN1].attach(PIN_DRAIN1);
  servos[REG_DRAIN2].attach(PIN_DRAIN2);
  servos[REG_WHISTLE].attach(PIN_WHISTLE);
  set_servos_safe();
}

void loop()
{
  // Is the timeout used?
  if(timeout)
  {
    // check it
    if((timeCt + 1000) < millis())
    {
      timeCt = millis();
      timeout--;
    }
    if(0 == timeout)
    {
      set_servos_safe();
    }
  }
  if(Serial.find(SERIAL_START_RX))
  {
    // Found a start of packet: now look for the rest of it
    if(0 < Serial.readBytesUntil(SERIAL_END_RX, serialBuff, SERIAL_BUFF_SIZE))
    {
      char dataSize = 0;
      char regBase = 0;
      
      // We have a valid packet: reset the keepalive timer and let's process the packet
      if(0x42 == EEPROM.read(REG_EEPROM_VALID - START_OF_EEPROM))
      {
        timeout = EEPROM.read(REG_DISC_TIMEOUT - START_OF_EEPROM);
      }
      else
      {
        timeout = DEFAULT_TIMEOUT;
      }
      
      regBase = ASCII2char(serialBuff[RX_REGISTER_HI], serialBuff[RX_REGISTER_LO]);
      dataSize = ASCII2char(serialBuff[RX_SIZE_HI], serialBuff[RX_SIZE_LO]);
      
      switch(serialBuff[RX_COMMAND_OFFSET])
      {
        default:
          Serial.write("#KO?");
        break;
        case COM_COMMAND:
          if(dataSize)
          {
            char loopCt, bufferOffset;
            bufferOffset = RX_DATA_START;
            for(loopCt = regBase; loopCt < regBase + dataSize + 1; loopCt++)
            {
              if(loopCt < NUMBER_OF_SERVOS)
              {
                // set servo
                servos[loopCt].write(serialBuff[bufferOffset]);
              }
              else
              {
                if((loopCt >= START_OF_EEPROM) && (loopCt <= MAX_REGISTERS))
                {
                  // write to an EEPROM register
                  EEPROM.write(loopCt - START_OF_EEPROM, serialBuff[bufferOffset]);
                }
              }
              bufferOffset++;
            }
            Serial.write("#OK?");
          }
          else
          {
            // We don't know how to write 0 registers
            Serial.write("#KO?");
          }
        break;
        case COM_QUERY:
          if(dataSize)
          {
            char loopCt, bufferOffset;
            bufferOffset = TX_DATA_START;
            for(loopCt = regBase; loopCt < regBase + dataSize + 1; loopCt++)
            {
              if(loopCt < NUMBER_OF_SERVOS)
              {
                // read servo
                serialBuff[bufferOffset] = servos[loopCt].read();
              }
              else
              {
                if((loopCt >= START_OF_EEPROM) && (loopCt <= MAX_REGISTERS))
                {
                  // read from an EEPROM register
                  serialBuff[bufferOffset] = EEPROM.read(loopCt - START_OF_EEPROM);
                }
              }
              bufferOffset++;
            }
            serialBuff[TX_START] = SERIAL_START_TX;
            serialBuff[TX_RESPONSE_LO] = RESP_LOW_AN;
            serialBuff[TX_RESPONSE_HI] = RESP_HIGH_AN;
            serialBuff[TX_END_BASE + bufferOffset] = SERIAL_END_TX;
          }
          else
          {
            // We don't know how to read nothing!
            Serial.write("#KO?");
          }
        break;
        case COM_KEEPALIVE:
          Serial.write("#OK?");
        break;
      }
    }
  }
}

