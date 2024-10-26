// The code has been aquired from iforce2d Youtube channel
// Modified by 3DElectroTec Youtube channel

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x17,0xDA, //NAV-SOL on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //NAV-VELNED on

  // Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x78,0x00,0x01,0x00,0x01,0x00,0x8E,0x8A, //(8.33Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x82,0x00,0x01,0x00,0x01,0x00,0x98,0xC6, //(7.69Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x96,0x00,0x01,0x00,0x01,0x00,0xAC,0x3E, //(6.67Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

#define button 3
int state = 0;
int old = 0;
int buttonPoll = 0;

#include <Adafruit_SH1106.h> 
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

//On an arduino UNO: A4(SDA), A5(SCL)
#define OLED_RESET -1 //Reset pin # (or -1 if sharing Arduino reset pin) 
#define SCREEN_ADDRESS 0x3C //See datasheet for Address   
Adafruit_SH1106 display(OLED_RESET); 


#define GPS Serial    

const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };
const unsigned char NAV_VELNED_HEADER[] = { 0x01, 0x12 };
const unsigned char NAV_SOL_HEADER[]    = { 0x01, 0x06 };

enum _ubxMsgType {
  MT_NONE,                    
  MT_NAV_VELNED,              
  MT_NAV_SOL,                 
};

struct NAV_VELNED {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long velN;
  long velE;
  long velD;
  unsigned long speed;
  unsigned long gSpeed;
  long heading;
  unsigned long sAcc;
  unsigned long cAcc;
};

struct NAV_SOL {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long fTOW;
  short week;
  unsigned char gpsFix;
  unsigned char flags;
  long ecefX;
  long ecefY;
  long ecefZ;
  unsigned long pAcc;
  long ecefVX;
  long ecefVY;
  long ecefVZ;
  unsigned long sAcc;
  unsigned short pDOP;
  unsigned char reserved1;
  unsigned char numSV;
  unsigned long reserved2;
};

union UBXMessage {
  NAV_VELNED navvelned;
  NAV_SOL navsol;
};

UBXMessage ubxMessage;

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}

// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}

// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];

  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while ( GPS.available() ) {

    byte c = GPS.read();

    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0; // Reset to beginning state.
    }
    else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&ubxMessage))[fpos-2] = c;

      fpos++;

      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if ( compareMsgHeader(NAV_VELNED_HEADER) ) {
          currentMsgType = MT_NAV_VELNED;
          payloadSize = sizeof(NAV_VELNED);
        }
        else if ( compareMsgHeader(NAV_SOL_HEADER) ) {
          currentMsgType = MT_NAV_SOL;
          payloadSize = sizeof(NAV_SOL);
        }
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0; 
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          // Checksum matches, we have a valid message.
          return currentMsgType; 
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  return MT_NONE;
}

void setup() 
{
  // Serial.begin(9600);
  GPS.begin(9600);
  pinMode(button, INPUT);
  digitalWrite(LED_BUILTIN, LOW);

  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    GPS.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SH1106_SWITCHCAPVCC, SCREEN_ADDRESS);

  display.clearDisplay();
  display.display();
  delay(2000);

}


int numSV = 0;
unsigned long gSpeed = 0;
unsigned long lastScreenUpdate = 0;
int speedCalc = 0;
String speedoText = "N/A";

char* spinner = "/-\\|";
byte screenRefreshSpinnerPos = 0;
byte gpsUpdateSpinnerPos = 0;

void loop() {
  //############################################################################
  Serial.println(speedoText);

  buttonPoll = digitalRead(button);
  if(buttonPoll == 1){
    delay(50);
    buttonPoll = digitalRead(button);
    if(buttonPoll == 0){
      state = old + 1;
    }
  }
  // else{
  //   delay(100);
  // }

  
  //################################################################################# 
  int msgType = processGPS();
  if ( msgType == MT_NAV_VELNED ) {
    gSpeed = ubxMessage.navvelned.gSpeed; 
    gpsUpdateSpinnerPos = (gpsUpdateSpinnerPos + 1) % 4;
  }
  else if (msgType == MT_NAV_SOL) {
    numSV = ubxMessage.navsol.numSV;
    gpsUpdateSpinnerPos = (gpsUpdateSpinnerPos + 1) % 4;
  }
  
  unsigned long now = millis();
  if ( now - lastScreenUpdate > 100 ) {   //................. Previous value was 100 "Screen refresh rate" the higher the slower
    updateScreen();
    lastScreenUpdate = now;
    screenRefreshSpinnerPos = (screenRefreshSpinnerPos + 1) % 4;
  }
}

void updateScreen(){
  // Button state change
  switch (state){
      case 1:
        digitalWrite(LED_BUILTIN, HIGH);
        old = state;
        speedoText = "mph";
        speedCalc = (gSpeed * 0.0223694) + 0.5; // mph
        // speedoText = "Km/h";
        // speedCalc = (gSpeed * 0.036) + 0.5; // kmh
        break;

      default:
        digitalWrite(LED_BUILTIN, LOW);
        old = 0;
        // speedoText = "mph";
        // speedCalc = (gSpeed * 0.0223694) + 0.5; // mph
        speedoText = "Km/h";
        speedCalc = (gSpeed * 0.036) + 0.5; // kmh
      break;
    }

  display.clearDisplay();
  display.setTextColor(WHITE);

  display.setTextSize(2);

  display.setCursor(0, 0);
  display.print(spinner[screenRefreshSpinnerPos]);
  
  display.setCursor(15, 0);
  display.print(spinner[gpsUpdateSpinnerPos]);

  display.setCursor(30, 0);
  display.print(numSV);

  display.setTextSize(5);
  display.setCursor(0, 25);
  display.print(speedCalc); // Km/h or mph, calculation
  
  display.setTextSize(1);
  display.setCursor(100, 55);
  display.print(speedoText); // Km/h or mph, text only

  display.display(); 
}
