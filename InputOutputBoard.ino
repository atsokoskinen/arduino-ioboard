#include <avr/wdt.h>
#include <util/crc16.h>

#define ID_STRING F("InputOutputBoard 1.0\n")
#define HELP_STRING F("Commands accepted in format '*x' or '*xy' terminated with \\n:\nAy = enable floating analog input 'Ay'\nF  = change analog inputs to LOW outputs (default is pulled-up inputs)\nHy = set digital pin y as output (HIGH)\nLy = set digital pin y as output (LOW)\n")

//#define WATCHDOG_SAFETY_DELAY

#define SERIAL_SPEED_bps 115200
#define LOOP_PERIOD_ms 5u

static const uint8_t analogPins[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
#define NUMBER_OF_ANALOG_PINS sizeof( analogPins )

static const uint8_t digitalPins[] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
#define NUMBER_OF_DIGITAL_PINS sizeof( digitalPins )

typedef enum
{
  CS_NOT_ACTIVE = 0u,
  CS_ENTERED,
  CS_AD_ENABLE,
  CS_AD_FORCE_LOW,
  CS_DOUT_HIGH,
  CS_DOUT_LOW
} E_ConfigState;

void updateLeds( void );
void configProtocol( void );
void printHex8( uint8_t value );
void printHex12( uint16_t value );

void setup() {
  // Initialize watchdog timer
  wdt_disable();
#ifdef WATCHDOG_SAFETY_DELAY
  delay( 3000u ); // safety period for entering bootloader
#endif
  wdt_enable( WDTO_2S );

  // Initialize serial communication
  Serial.begin(SERIAL_SPEED_bps);
  Serial.println( ID_STRING );
  Serial.print( "Cycle time is " );
  Serial.print( LOOP_PERIOD_ms );
  Serial.println( " ms\n" );
  Serial.println( HELP_STRING );

  // Initialize digital IO
  for ( uint8_t pin = 0u; pin < NUMBER_OF_DIGITAL_PINS; pin++ )
  {
    pinMode( digitalPins[ pin ], INPUT_PULLUP );
  }

  // Enable pull-ups for unused AD inputs
  // Note that analog-only pins are not affected
  for( uint8_t pin = 0u; pin < NUMBER_OF_ANALOG_PINS; pin++ )
  {
    pinMode( analogPins[ pin ], INPUT_PULLUP );
  }

  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite( LED_BUILTIN, LOW );

  Serial.println( F("[START]") );
}

void loop() {
  const long microsStart = micros();
  uint8_t checksum = 0u;

  // Print start char
  Serial.print( ":" );

  // Print analog input values
  for ( uint8_t i = 0u; i < NUMBER_OF_ANALOG_PINS; i++ )
  {
    const uint16_t analogValue = analogRead( analogPins[ i ] );
    printHex12( analogValue );
    Serial.print( " " );
    checksum = _crc8_ccitt_update( checksum, lowByte( analogValue ) );
    checksum = _crc8_ccitt_update( checksum, highByte( analogValue ) );
  }

  // Print digital input values
  uint16_t digitalValues = 0u;
  for ( uint8_t pin = 0u; pin < NUMBER_OF_DIGITAL_PINS; pin++ )
  {
    bitWrite( digitalValues, pin, digitalRead( digitalPins[ pin ] ) );
  }
  printHex12( digitalValues );
  Serial.print( " " );
  checksum = _crc8_ccitt_update( checksum, lowByte( digitalValues ) );
  checksum = _crc8_ccitt_update( checksum, highByte( digitalValues ) );

  // Print checksum
  printHex8( checksum );

  // End of line
  Serial.println();

  // Blink the builtin LED for alive indication
  updateLeds();

  // Protocol for disabling analog inputs
  configProtocol();

  // Wait until the end of loop period
  while ( ( micros() - microsStart ) < ( LOOP_PERIOD_ms * 1000u ) )
  {
  }

  // Reset the watchdog timer
  wdt_reset();
}

void updateLeds( void )
{
  static uint8_t blinkCounter = 0u;

  if ( blinkCounter > 0x7Fu )
  {
    digitalWrite( LED_BUILTIN, HIGH );
  }
  else
  {
    digitalWrite( LED_BUILTIN, LOW );
  }
  blinkCounter++;
}

void configProtocol( void )
{
  static E_ConfigState configState = CS_NOT_ACTIVE;

  while ( Serial.available() )
  {
    const char incomingByte = Serial.read();

    switch ( configState )
    {
      case CS_NOT_ACTIVE:
        if ( incomingByte == '*' )
        {
          configState = CS_ENTERED;
        }
        break;

      case CS_ENTERED:
        switch( incomingByte )
        {
          case 'A':
            configState = CS_AD_ENABLE;
            break;

          case 'L':
            configState = CS_DOUT_LOW;
            break;

          case 'H':
            configState = CS_DOUT_HIGH;
            break;

          case 'F':
            configState = CS_AD_FORCE_LOW;
            break;

          default:
            configState = CS_NOT_ACTIVE;
        }
        break;

      case CS_AD_ENABLE:
        if ( ( incomingByte >= '0' ) && ( incomingByte <= ( '0' + NUMBER_OF_ANALOG_PINS - 1 ) ) )
        {
          const uint8_t pin = analogPins[ incomingByte - '0' ];
          pinMode( pin, INPUT );
        }
        configState = CS_NOT_ACTIVE;
        break;

      case CS_DOUT_LOW:
      case CS_DOUT_HIGH:
        if ( isHexadecimalDigit( incomingByte ) )
        {
          uint8_t index = 0u;
          if( isDigit( incomingByte ) )
          {
            index = incomingByte - '0';
          }
          else
          {
            index = incomingByte - 'A' + 0xA;
          }

          if( index < NUMBER_OF_DIGITAL_PINS )
          {
            const uint8_t pin = digitalPins[ index ];
            pinMode( pin, OUTPUT );
            if( configState == CS_DOUT_LOW )
            {
              digitalWrite( pin, LOW );
            }
            else
            {
              digitalWrite( pin, HIGH );
            }
          }
        }
        configState = CS_NOT_ACTIVE;        
        break;

      case CS_AD_FORCE_LOW:
        for( uint8_t pin = 0u; pin < NUMBER_OF_ANALOG_PINS; pin++ )
        {
          pinMode( analogPins[ pin ], OUTPUT );
          digitalWrite( analogPins[ pin ], LOW );
        }
        configState = CS_NOT_ACTIVE;
        break;

      default:
        configState = CS_NOT_ACTIVE;
        break;
    }

    if ( incomingByte == '*' )
    {
      configState = CS_ENTERED;
    }
  }
}

void printHex8( uint8_t value )
{
  if ( value < 0x10u )
  {
    Serial.print( "0" );
  }
  Serial.print( value, HEX );
}

void printHex12( uint16_t value )
{
  uint16_t cmp = 0x0100u;
  for ( uint8_t i = 0; i < 2u; i++ )
  {
    if ( value < cmp )
    {
      Serial.print( "0" );
    }
    cmp >>= 4u;
  }
  Serial.print( value, HEX );
}
