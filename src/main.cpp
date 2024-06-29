#include "SoftSerial.h"
#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "innerWdt.h"

softSerial GPS(GPIO5, GPIO7) ;

uint8_t devEui[] = { 0xDA, 0x98, 0x35, 0x50, 0x39, 0x5E, 0x92, 0xA6 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x06, 0xD3, 0x9A, 0x81, 0xFE, 0x19, 0x38, 0xC1, 0xE7, 0x12, 0x97, 0xBF, 0x83, 0x8F, 0xF5, 0x26 };

uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

DeviceClass_t  loraWanClass = LORAWAN_CLASS;

uint32_t appTxDutyCycle = 60000;

bool overTheAirActivation = LORAWAN_NETMODE;

bool loraWanAdr = LORAWAN_ADR;

bool keepNet = LORAWAN_NET_RESERVE;

bool isTxConfirmed = LORAWAN_UPLINKMODE;

uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

int realData[13] ;
int commaIndex[13] ;
int cnt = 0 ;
int counted = 0 ;
String nmea ;
int Hours, Minutes, Seconds ;
int latitude, longitude ;
float speedKph ;
int fulldata, decimal_prt, number_prt ;

void speeddivided()  { 
  fulldata = speedKph * 100 ;
  decimal_prt = fulldata % 100 ; // 50
  number_prt = (fulldata - decimal_prt) / 100 ; // 125
  realData[3] = number_prt ;
  realData[4] = decimal_prt ;
}

void longtitudedivided() {
  fulldata = longitude ; // 100444256
  decimal_prt = fulldata % 1000000 ; // 444256
  number_prt = (fulldata - decimal_prt) / 1000000 ; // 100
  realData[5] = number_prt ;
  // decimal first&second digit 
  realData[6] = int(decimal_prt / 10000) ; // 44
  // decimal third&fourth digit 
  realData[7] = ((decimal_prt - (realData[6] * 10000)) / 100) ; // 42
  // decimal fifth&sixth digit
  realData[8] = decimal_prt % 100 ; // 56
}

void latitudedivided() {
  fulldata = latitude ;
  decimal_prt = fulldata % 1000000 ; // 11|11|11
  number_prt = (fulldata - decimal_prt) / 1000000 ; // 16
  realData[9] = number_prt ;
  // decimal first&second digit 
  realData[10] = int(decimal_prt / 10000) ; // 11
  // decimal third&fourth digit 
  realData[11] = ((decimal_prt - (realData[10] * 10000)) / 100) ; // 11
  // decimal fifth&sixth digit
  realData[12] = decimal_prt % 100 ; // 11
}

void PrepareDataForSending() { //must be transform index 3, 4, 5 
  realData[0] = Hours ;
  realData[1] = Minutes ;
  realData[2] = Seconds ;
  speeddivided() ;
  longtitudedivided() ;
  latitudedivided() ;
}

void nmeadecode() {
  String utcTimeStr = nmea.substring(commaIndex[0] + 1, commaIndex[1]);
  Hours = (utcTimeStr.substring(0, 2).toInt() + 7) % 24;
  Minutes = utcTimeStr.substring(2, 4).toInt();
  Seconds = utcTimeStr.substring(4, 6).toInt();

  String latitudeStr = nmea.substring(commaIndex[2] + 1, commaIndex[3]);
  float latitudeDegrees = latitudeStr.substring(0, 2).toFloat();
  float latitudeMinutes = latitudeStr.substring(2).toFloat();
  latitude = (latitudeDegrees * 1000000) + (latitudeMinutes * 1000000 / 60);

  String longitudeStr = nmea.substring(commaIndex[4] + 1, commaIndex[5]);
  float longitudeDegrees = longitudeStr.substring(0, 3).toFloat();
  float longitudeMinutes = longitudeStr.substring(3).toFloat();
  longitude = (longitudeDegrees * 1000000) + (longitudeMinutes * 1000000 / 60);

  // Extract Speed
  String speedStr = nmea.substring(commaIndex[6] + 1, commaIndex[7]);
  float speedKnots = speedStr.toFloat();
  speedKph = speedKnots * 1.852; // Convert knots to km/h

  PrepareDataForSending() ;
}

void Timer() {
  int cnt = 0 ;
  memset(realData,0,appDataSize) ;
  while (cnt < 15) {
    cnt ++ ; 
    delay(1000) ;
    Serial.print(cnt) ;
  } 
}

int count = 0 ;

void RawData() {
  int commacount = 0 ;
  for (int i = 0 ; i < nmea.length() ; i ++) {
    if (nmea[i] == ',') {
      commaIndex[commacount++] = i ;
    }
  }
  String status = nmea.substring(commaIndex[1] + 1, commaIndex[2]) ;
  if (status == "A") {
    nmeadecode() ;
  }
  else {
    Hours = 0 ;
    Minutes = 0 ;
    Seconds = 0 ;
    latitude = 0 ;
    longitude = 0 ;
    speedKph = 0 ;
    Serial.println("No valid GPS fix.") ;
  }

  Serial.print("count: ") ;
  Serial.println(count) ;
  Serial.print("Time (Thailand): ");
  Serial.println(String(Hours) + ":" + String(Minutes) + ":" + String(Seconds));
  Serial.print("Latitude: "); Serial.println(latitude); // 6 decimal places for precision
  Serial.print("Longitude: "); Serial.println(longitude);
  Serial.print("Speed (km/h): "); Serial.println(speedKph);
  Serial.println() ;
  Serial.println("------------------------------------") ;
  for (int j = 0 ; j < 13 ; j ++) {
    Serial.print(realData[j]) ;
    Serial.print(", ") ;
  }
  Serial.println() ;
}

static void prepareTxFrame (uint8_t port ) {
  appDataSize = 13 ;
  appData[0] = realData[0] ; 
  appData[1] = realData[1] ;
  appData[2] = realData[2] ;
  appData[3] = realData[3] ;
  appData[4] = realData[4] ;
  appData[5] = realData[5] ;
  appData[6] = realData[6] ;
  appData[7] = realData[7] ;
  appData[8] = realData[8] ;
  appData[9] = realData[9] ;
  appData[10] = realData[10] ;
  appData[11] = realData[11] ;
  appData[12] = realData[12] ;
  LoRaWAN.send() ;
}

void setup() {
  pinMode(GPIO5, OUTPUT) ;
  pinMode(GPIO7, INPUT) ;
  Serial.begin(9600) ;
  GPS.begin(9600) ;
  Serial.println("Start") ;
  innerWdtEnable(false);
  #if(AT_SUPPORT)
    enableAt();
  #endif
    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();
}

void loop() {
    CySysWdtEnable();
    Serial.println("On loop") ;
    GPS.begin(9600) ;
    counted = 0 ;
    if (GPS.available() > 0) {
      Serial.print("available") ;
      CySysWdtDisable();
      while (GPS.available() > 0) {
        nmea = GPS.readStringUntil('\n') ;
        Serial.print(nmea) ;
        Serial.println() ;
        if (nmea.startsWith("$GPRMC")) {
          GPS.end() ;
          Serial.println(" GPRMC Found") ;
          count ++ ;
          delay(1000) ;
          Timer() ;
          if (count == 100) {
            CySoftwareReset() ;
          }
          RawData() ;
        }
        GPS.flush() ;
        delay(100) ;
      }
    }

	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			LoRaWAN.setDataRateForNoADR(1); 
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
      prepareTxFrame( appPort );
			deviceState = DEVICE_STATE_CYCLE ;
      LoRaWAN.send() ;
      delay(5000) ;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}
