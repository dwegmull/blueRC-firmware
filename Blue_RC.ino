
// Blue RC firmware
// By David Wegmuller
// This code is provided "as is" in the hope it will be helpful.
// You may use it any way you want.
// I'm not responsible if it doesn't work or causes any damages.
// Send your questions to david at wegmuller dot org

//#define DEBUG

#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// Pins
#define PIN_THROTTLE  3
#define THROTTLE_PORT PORTD
#define THROTTLE_MASK 0x08
#define THROTTLE_DIR DDRD
#define PIN_REVERSE1  4
#define REVERSE1_PORT PORTD
#define REVERSE1_MASK 0x10
#define REVERSE1_DIR DDRD
#define PIN_REVERSE2  5
#define REVERSE2_PORT PORTD
#define REVERSE2_MASK 0x20
#define REVERSE2_DIR DDRD
#define PIN_DRAIN1    6
#define DRAIN1_PORT PORTD
#define DRAIN1_MASK 0x40
#define DRAIN1_DIR DDRD
#define PIN_DRAIN2    7
#define DRAIN2_PORT PORTD
#define DRAIN2_MASK 0x80
#define DRAIN2_DIR DDRD
#define PIN_WHISTLE   8
#define WHISTLE_PORT PORTB
#define WHISTLE_MASK 0x01
#define WHISTLE_DIR DDRB
#define NUMBER_OF_SERVOS 6


// Timeout related
#define DEFAULT_TIMEOUT      4 * 60 // four Minutes in Seconds

// Serial and protocol definitions
#define SERIAL_BUFF_SIZE      100
#define SERIAL_START_RX       '@'
#define SERIAL_END_RX         '!'
#define SERIAL_START_TX       '#'
#define SERIAL_END_TX         '?'
#define RX_COMMAND_OFFSET     0
#define RX_REGISTER_HI        1
#define RX_REGISTER_LO        2
#define RX_SIZE_HI            3
#define RX_SIZE_LO            4
#define RX_DATA_START         5
#define TX_RESPONSE           0
#define TX_REGISTER_HI        1
#define TX_REGISTER_LO        2
#define RX_SIZE_HI            3
#define RX_SIZE_LO            4
#define TX_DATA_START         5
#define TX_END_BASE           6
#define COM_COMMAND           'C'
#define COM_QUERY             'Q'
#define COM_KEEPALIVE         'K'
#define RESP                  'A'

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
#define  REG_MI_THROTTLE      9
#define  REG_HI_THROTTLE      10
#define  REG_LO_REVERSE1      11
#define  REG_MI_REVERSE1      12
#define  REG_HI_REVERSE1      13
#define  REG_LO_REVERSE2      14
#define  REG_MI_REVERSE2      15
#define  REG_HI_REVERSE2      16
#define  REG_LO_DRAIN1        17
#define  REG_HI_DRAIN1        18
#define  REG_LO_DRAIN2        19
#define  REG_HI_DRAIN2        20
#define  REG_LO_WHISTLE       21
#define  REG_HI_WHISTLE       22
#define  REG_DISC_TIMEOUT     23
#define  REG_VERSION_H        24
#define  REG_VERSION_F        25
#define  REG_VERSION_P        26
#define  REG_SAFE_THROTTLE    27
#define  REG_SAFE_REVERSE1    28
#define  REG_SAFE_REVERSE2    29
#define  REG_SAFE_DRAIN1      30
#define  REG_SAFE_DRAIN2      31
#define  REG_SAFE_WHISTLE     32
#define  REG_DESC_SIZE        33
#define  REG_DESCRIPTION      34
#define  MAX_REGISTERS        255
#define  START_OF_EEPROM      REG_EEPROM_VALID
#define  REG_SAFE_BASE        REG_SAFE_THROTTLE

char servos[NUMBER_OF_SERVOS];
#define SERVO_THROTTLE 0
#define SERVO_REVERSER1 1
#define SERVO_REVERSER2 2
#define SERVO_DRAIN1    3
#define SERVO_DRAIN2    4
#define SERVO_WHISTLE   5

char serialBuff[SERIAL_BUFF_SIZE];
volatile unsigned char timeout;
char timeCt;
int dataSize;

char servoState;
union w2b
{
int w;
byte b[2];
} word2bytes;



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
      servos[loopCt] = EEPROM.read(REG_SAFE_BASE + loopCt - START_OF_EEPROM);
    }
    while(loopCt--);
    timeout = EEPROM.read(REG_DISC_TIMEOUT - START_OF_EEPROM);
  }
  else
  {
    // No calibration data: use 50% (90 degrees)
    do
    {
      servos[loopCt] = 127;
    }
    while(loopCt--);
    timeout = DEFAULT_TIMEOUT;
  }
}

char char2digit(char c)
{
  if(c < 0x0A)
  {
    return c + 0x30;
  }
  else
  {
    return c + 0x41 - 0x0A;
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
      return (digit - 'A' + 10);
    }
  }
  return 0;
}

int ASCII2char(char upper, char lower)
{
  return ((int)((digit2char(upper) << 4) + digit2char(lower))) & 0x00FF; 
}

void char2ASCII(char c, char *a)
{
  *a = char2digit((c >> 4) & 0x0F);
  a++;
  *a = char2digit(c & 0x0F);
}

// Receives a packet over the UART
// Returns 0 if a packet arrived, 1 if timeout, 2 if error
char receivePacket(void)
{
  while(timeout != 1)
  {

    if(UCSR0A & (1 << RXC0))
    {

      // A character arrived: check if it's a start of packet
      if(UDR0 == SERIAL_START_RX)
      {
        
        // found a start of packet: read the rest of it
        dataSize = 0;
        while(timeout != 1)
        {
          // Receiving a packet
          if(UCSR0A & (1 << RXC0))
          {
            serialBuff[dataSize] = UDR0;
            if(serialBuff[dataSize] == SERIAL_END_RX)
            {
              dataSize++;
              return 0;
            }
            dataSize++;
            if(dataSize > SERIAL_BUFF_SIZE)
            {
              return 2;
            }
          }
        }
        return 1;
      }
    }
  }
  return 1;  
}

void txString(char* s)
{
  // Loop through the string
  while(*s != 0)
  {
    // Wait for the UART Tx to be ready
    while(0 == (UCSR0A & (1 << UDRE0)));
    
    UDR0 = *s;
    s++;
  }
}

// Wait with timeout for an incoming string on the UART.
// Check if the string matches the one given as an argument.
// Return 0 if a matching string is received.
// Return 1 in case of a timeout.
// Return 2 if the string does not match.
char getString(char* s)
{
  while(*s != 0)
  {
    int timeout = 0;
    while(0 == (UCSR0A & (1 << RXC0)))
    {
      timeout++;
      if(timeout > 20000)
      {
        // Timeout.
        return 1;
      }
    }
    // A character arrived: check if it's the one we expect
    if(UDR0 != *s)
    {
      // String does not match!
      return 2;
    }
    s++;
  }
  // A matching string was received!
  return 0;
}


void stop_UART()
{
  // Wait to make sure the transmission is done before shutting down the UART.
  while(0 == (UCSR0A & TXC0));
// Disable the UART
  UCSR0B = 0;
  UCSR0A = 0;
  UCSR0C = 0;
}

void start_UART_100K()
{
// Setup UART for 115200 bits/S
  UBRR0H = 0;
  UBRR0L = 8;
  
  // Enable receiver and transmitter
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0A = (1<<U2X0);
  
  // Set frame format: 8 data, 1 stop bit
  UCSR0C = (3<<UCSZ00);
}

void start_UART_50K()
{
// Setup the UART for 57600 bits/S
  UBRR0H = 0;
  UBRR0L = 16;
 // Enable receiver and transmitter
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0A = (1<<U2X0);
  UCSR0C = (3<<UCSZ00);
}

void active_wait(int timeCt)
{
  for(timeCt = 0; timeCt < 10000; timeCt++)
  {
   PINB |= 0x04;
  }  
}

void setup()
{
  unsigned int timeCt;
  // All servo signals are output low
  THROTTLE_DIR |= THROTTLE_MASK;
  THROTTLE_PORT &= ~THROTTLE_MASK;
  REVERSE1_DIR |= REVERSE1_MASK;
  REVERSE1_PORT &= ~REVERSE1_MASK;
  REVERSE2_DIR |= REVERSE2_MASK;
  REVERSE2_PORT &= ~REVERSE2_MASK;
  DRAIN1_DIR |= DRAIN1_MASK;
  DRAIN1_PORT &= ~DRAIN1_MASK;
  DRAIN2_DIR |= DRAIN2_MASK;
  DRAIN2_PORT &= ~DRAIN2_MASK;
  WHISTLE_DIR |= WHISTLE_MASK;
  WHISTLE_PORT &= ~WHISTLE_MASK;

  set_servos_safe();
  // Configure timer 1 to generate the servo pulses.
  cli();
 //set timer1 interrupt at 50Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  TIFR1 |= ((1 << OCF1A) | (1 << OCF1B));
  // set compare match register for 20mS period
  OCR1A = 2500;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS12 bits for 64 prescaler
  TCCR1B |= ((1 << CS10) | (1 << CS11));  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  // disable timer 0 to prevent jitter on the PWMs
  TCCR0B = 0;
  
  // In most cases, the BT module is already setup to run at 50Kbits/S
  start_UART_50K();
  
  // Wait to make sure the Bluetooth module is done booting
  // It continuously toggles a GPIO once it is ready.
  while(0 == (PINB & 0x02));
  while(PINB & 0x02);
  
  // It is now safe to talk to the BT module.
  txString("$$$");
  if(0 != getString("CMD\r\n"))
  {
    // Failed to communicate with the BT module at 50K. Try at 100K
    stop_UART();
    start_UART_100K();
    txString("$$$");
    // Wait a bit between commands
    DDRB |= 0x04;
    active_wait(20000);

    // Set the BT module bit bit rate to 57600 bits/s
    txString("U,57.6,N\r");
    stop_UART();
    start_UART_50K();
    txString("$$$");
    while(0 != getString("CMD\r\n"))
    {
      // Try again
      txString("$$$");
    }
    // Permanently set the BT module UART speed to 50K
    txString("SU,57\r");
    while(0 != getString("AOK\r\n"));
  }
  // in command mode at 50K
  if(0x42 == EEPROM.read(REG_EEPROM_VALID - START_OF_EEPROM))
  {
    int nameSize = EEPROM.read(REG_DESC_SIZE - START_OF_EEPROM);
    if(nameSize > 0)
    {
      // The locomotive name stored in the EEPROM is valid
      int cnt;
      if(nameSize > 18)
      {
        nameSize = 18;
      }
      for(cnt = 0; cnt < nameSize; cnt++)
      {
        unsigned char c;
        c = EEPROM.read(cnt + REG_DESCRIPTION - START_OF_EEPROM);
        if(c > 122)
        {
          c = 'X';
        }
        serialBuff[cnt] = c; 
      }
      serialBuff[cnt++] = '\r'; // The BT module returns a CR+ LF at the end of its messages
      serialBuff[cnt++] = '\n';
      serialBuff[cnt] = 0; // zero termination needed by getString.
      // Ask the BT module for its name
      txString("GN\r");
      // Check its answer against the name stored in EEPROM.
      cnt = getString(serialBuff);
      if(2 == cnt)
      {
        // Names don't match: program the new one into the BT module
        active_wait(10000);
        txString("SN,");
        serialBuff[nameSize + 1] = 0;
        txString(serialBuff);
        while(0 != getString("AOK\r\n"));
        // Reset the BT module to use the new name.
        DDRC |= 0x08;
        PORTC &= ~0x08;
        active_wait(10000);
        PORTC |= 0x08;
      }
      else
      {
        // Get out of command mode
        txString("---\r");
      }
    }
  }
  else
  {
    // Get out of command mode
    txString("---\r");
  }
  sei();
}

void loop()
{
  // Is the timeout used?
  if(timeout)
  {
    // check it
    if(timeout == 1)
    {
      set_servos_safe();
    }
  }
 
  if(0 == receivePacket())
  {
    char regBase = 0;
    char stringOffset = 0;
  
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
        txString("#N?");
      break;
      case COM_COMMAND:
        if(dataSize)
        {
          char loopCt, bufferOffset;
          bufferOffset = RX_DATA_START;
          for(loopCt = regBase; loopCt < (regBase + dataSize); loopCt++)
          {
            if(loopCt < NUMBER_OF_SERVOS)
            {
              // set servo
              servos[loopCt] = ASCII2char(serialBuff[bufferOffset], serialBuff[bufferOffset + 1]);
            }
            else
            {
              if((loopCt >= START_OF_EEPROM) && (loopCt <= REG_DESC_SIZE))
              {
                // write to an EEPROM register
                EEPROM.write(loopCt - START_OF_EEPROM, ASCII2char(serialBuff[bufferOffset], serialBuff[bufferOffset + 1]));
              }
              else
              {
                if(loopCt == REG_DESCRIPTION)
                {
                  stringOffset = bufferOffset;
                }
                if(loopCt <= MAX_REGISTERS)
                {
                  EEPROM.write(loopCt - START_OF_EEPROM, serialBuff[stringOffset]);
                  stringOffset++;
                }
              }            
            }
            bufferOffset += 2;
          }
          txString("#P?");
        }
        else
        {
          // We don't know how to write 0 registers
          txString("#N?");
        }
      break;
      case COM_QUERY:
        if(dataSize)
        {
          int loopCt, bufferOffset;
          char tempBuff[3];
          tempBuff[2] = 0;

          bufferOffset = TX_DATA_START;
          for(loopCt = 0; loopCt < dataSize; loopCt++)
          {
            if(regBase < NUMBER_OF_SERVOS)
            {
              // read servo
              char2ASCII(servos[regBase], &serialBuff[bufferOffset]);
              bufferOffset += 2;
            }
            else
            {
              if((regBase >= START_OF_EEPROM) && (regBase <= REG_DESC_SIZE))
              {
                // read from an EEPROM register
                char2ASCII(EEPROM.read(regBase - START_OF_EEPROM), &serialBuff[bufferOffset]);
                bufferOffset += 2;
              }
              else
              {
                if(loopCt <= MAX_REGISTERS)
                {
/*                  char2ASCII(stringOffset, tempBuff);
                  txString(tempBuff);
                  char2ASCII(loopCt, tempBuff);
                  txString(tempBuff);
                  */
                  serialBuff[bufferOffset] = EEPROM.read(regBase - START_OF_EEPROM);
                  bufferOffset++;
                }
              }
            }
            regBase++;
          }
          serialBuff[TX_RESPONSE] = RESP;
          serialBuff[bufferOffset] = SERIAL_END_TX;
          serialBuff[bufferOffset + 1] = 0;
          txString("#");
          txString(serialBuff);
          
        }
        else
        {
          // We don't know how to read nothing!
          txString("#N?");
        }
      break;
      case COM_KEEPALIVE:
        txString("#P?");
      break;
    }
  }
}

// The servo signals are generated by the following two interrupt handlers
// One servo cycle lasts 20mS. Each channel follows the preceeding one, 
// using up to 15mS (2.5 * 6).
ISR(TIMER1_COMPA_vect)
{
  // This ISR occurs at the start of a servo sequence.
  // Set the first channel up
  
  THROTTLE_PORT |= THROTTLE_MASK;
  // Initialize the state machine
  servoState = 0;
  // Set the OC to trigger at the end of the first servo pulse
  word2bytes.b[0] = servos[SERVO_THROTTLE];
  word2bytes.b[1] = 0;
  word2bytes.w += 63;
  OCR1B = word2bytes.w;
  // Enable the OCB interrupt in addition to the OCA one
  TIMSK1 |= (1 << OCIE1B);
  // Soft timer
  if(timeCt)
  {
    timeCt--;
  }
  else
  {
    // One second tick (20mS * 5)
    timeCt = 5;
    if(timeout > 1)
    {
      // Stop at 1 second because a value of 0 means the timout is disabled.
      timeout--;
    }
  }
}

ISR(TIMER1_COMPB_vect)
{
  switch(servoState)
  {
    case 0:
    // End of Throttle pulse, start of Reverser 1
    THROTTLE_PORT &= ~THROTTLE_MASK;
    REVERSE1_PORT |= REVERSE1_MASK;
    word2bytes.b[0] = servos[SERVO_REVERSER1];
    word2bytes.b[1] = 0;
    word2bytes.w += 63;
    word2bytes.w += OCR1B;
    OCR1B = word2bytes.w;
    break;
    
    case 1:
    // End of Reverser1 pulse, start of Reverser2
    REVERSE1_PORT &= ~REVERSE1_MASK;
    REVERSE2_PORT |= REVERSE2_MASK;
    word2bytes.b[0] = servos[SERVO_REVERSER2];
    word2bytes.b[1] = 0;
    word2bytes.w += 63;
    word2bytes.w += OCR1B;
    OCR1B = word2bytes.w;
    break;
    
    case 2:
    // End of Reverser2 pulse, start of Drain1
    REVERSE2_PORT &= ~REVERSE2_MASK;
    DRAIN1_PORT |= DRAIN1_MASK;
    word2bytes.b[0] = servos[SERVO_DRAIN1];
    word2bytes.b[1] = 0;
    word2bytes.w += 63;
    word2bytes.w += OCR1B;
    OCR1B = word2bytes.w;
    break;
    
    case 3:
    // End of Drain1 pulse, start of Drain2
    DRAIN1_PORT &= ~DRAIN1_MASK;
    DRAIN2_PORT |= DRAIN2_MASK;
    word2bytes.b[0] = servos[SERVO_DRAIN2];
    word2bytes.b[1] = 0;
    word2bytes.w += 63;
    word2bytes.w += OCR1B;
    OCR1B = word2bytes.w;
    break;
    
    case 4:
    // End of Drain2 pulse, start of Whistle
    DRAIN2_PORT &= ~DRAIN2_MASK;
    WHISTLE_PORT |= WHISTLE_MASK;
    word2bytes.b[0] = servos[SERVO_WHISTLE];
    word2bytes.b[1] = 0;
    word2bytes.w += 63;
    word2bytes.w += OCR1B;
    OCR1B = word2bytes.w;
    break;    
    
    default:
    // End of Whistle pulse.
    WHISTLE_PORT &= ~WHISTLE_MASK;
  }
  servoState++;
}

