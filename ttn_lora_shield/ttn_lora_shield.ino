/*******************************************************************************

   Example of using a Lora Shield Uno and DHT11 with a single-channel
   Dragino LG01-P US915 gateway to send data to The Things Network.

   This uses ABP (Activation-by-personalisation), where a DevAddr and
   Session keys are preset (unlike OTAA, where a DevEUI and
   application key is configured, while the DevAddr and session keys are
   assigned/generated in the over-the-air-activation procedure).

   To use this sketch, first register your application and device with
   TTN, to generate a DevAddr, NwkSKey and AppSKey.
   Each device should have their own unique values for these fields.

   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
   Copyright (c) 2018 James Ren

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

 *******************************************************************************/

// include LoraWAN-in-C Library: https://github.com/rqg0717/arduino-lmic
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// include DHT Sensor Library
#include <DHT.h>

// DHT digital pin and sensor type
#define DHTPIN A0
#define DHTTYPE DHT11
#define DATA_LEN 5

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x07, 0x02, 0xD9, 0x76, 0x87, 0x8F, 0x0E, 0x23, 0xDE, 0x28
                                        };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x5F, 0x80, 0x1D, 0xC6, 0xF9, 0x2C, 0xF0, 0xBC, 0x56, 0x54
                                        };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000;

// payload to send to the gateway
static uint8_t payload[DATA_LEN];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping for LoRa Shield Uno
const lmic_pinmap lmic_pins = { .nss = 10, .rxtx = LMIC_UNUSED_PIN, .rst = 9,
                                .dio = { 2, 6, 7 },
                              };

// init. DHT
DHT dht(DHTPIN, DHTTYPE);

void dec_to_hex(unsigned char c) {
  switch (c) {
    case 0:
      Serial.print("0");
      break;
    case 1:
      Serial.print("1");
      break;
    case 2:
      Serial.print("2");
      break;
    case 3:
      Serial.print("3");
      break;
    case 4:
      Serial.print("4");
      break;
    case 5:
      Serial.print("5");
      break;
    case 6:
      Serial.print("6");
      break;
    case 7:
      Serial.print("7");
      break;
    case 8:
      Serial.print("8");
      break;
    case 9:
      Serial.print("9");
      break;
    case 10:
      Serial.print("A");
      break;
    case 11:
      Serial.print("B");
      break;
    case 12:
      Serial.print("C");
      break;
    case 13:
      Serial.print("D");
      break;
    case 14:
      Serial.print("E");
      break;
    case 15:
      Serial.print("F");
      break;
    default:
      Serial.print("?");
      break;
  }
}

void print_ABP_information(void) {
  unsigned char i;
  unsigned char cc;

  Serial.println(F("using ABP mode to join TTN"));
  Serial.println(
    F("DevEui: Can be any value, which length should be 8 numbers"));
  Serial.print("DevAddr: ");
  Serial.println(DEVADDR, HEX);

  Serial.print("NwKSKey: ");
  for (i = 0; i <= 15; i++) {
    chartemp = pgm_read_word_near(NWKSKEY + i);
    dec_to_hex((cc >> 4) & 0xf);
    dec_to_hex(cc & 0xf);
  }

  Serial.println("");
  Serial.print("AppSKey: ");
  for (i = 0; i <= 15; i++) {
    chartemp = pgm_read_word_near(APPSKEY + i);
    dec_to_hex((cc >> 4) & 0xf);
    dec_to_hex(cc & 0xf);
  }
  // add one new line
  Serial.println("");
}
void read_DHT() {
  // read the temperature from the DHT11
  float temperature = dht.readTemperature(true, false);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" F");

  // read the humidity from the DHT11
  float humidity = dht.readHumidity();
  Serial.print("%RH ");
  Serial.println(humidity);

  // float -> int
  uint16_t temp = (uint16_t)(temperature * 100);
  // int -> bytes
  byte tempLow = lowByte(temp);
  byte tempHigh = highByte(temp);
  // place the bytes into the payload
  payload[0] = tempLow;
  payload[1] = tempHigh;

  // float -> int
  uint16_t humid = (uint16_t)(humidity * 100);
  // int -> bytes
  byte humidLow = lowByte(humid);
  byte humidHigh = highByte(humid);
  payload[2] = humidLow;
  payload[3] = humidHigh;
}
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    read_DHT();
    // prepare upstream data transmission at the next possible time.
    // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
    // don't request an ack (the last parameter, if not zero, requests an ack from the network).
    // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
    LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);

    Serial.println("Packet queued");
    Serial.println(LMIC.freq);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.println(ev);
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println("EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      Serial.println("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println("EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println("EV_JOINED");
      break;
    case EV_RFU1:
      Serial.println("EV_RFU1");
      break;
    case EV_JOIN_FAILED:
      Serial.println("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println("EV_REJOIN_FAILED");
      break;
    case EV_TXCOMPLETE:
      Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print("Data Received: ");
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println("EV_LINK_ALIVE");
      break;
    default:
      Serial.println("Unknown event");
      break;
  }
}

void setup() {
  delay(5000);
  while (!Serial);
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  
  // We'll disable all 72 channels
  /*
    channel	0:	902300000	hz	min_dr	0	max_dr	3
    channel	1:	902500000	hz	min_dr	0	max_dr	3
    channel	2:	902700000	hz	min_dr	0	max_dr	3
    channel	3:	902900000	hz	min_dr	0	max_dr	3
    channel	4:	903100000	hz	min_dr	0	max_dr	3
    channel	5:	903300000	hz	min_dr	0	max_dr	3
    channel	6:	903500000	hz	min_dr	0	max_dr	3
    channel	7:	903700000	hz	min_dr	0	max_dr	3
    channel	8:	903900000	hz	min_dr	0	max_dr	3
    channel	9:	904100000	hz	min_dr	0	max_dr	3
    channel	10:	904300000	hz	min_dr	0	max_dr	3
    channel	11:	904500000	hz	min_dr	0	max_dr	3
    channel	12:	904700000	hz	min_dr	0	max_dr	3
    channel	13:	904900000	hz	min_dr	0	max_dr	3
    channel	14:	905100000	hz	min_dr	0	max_dr	3
    channel	15:	905300000	hz	min_dr	0	max_dr	3
    channel	16:	905500000	hz	min_dr	0	max_dr	3
    channel	17:	905700000	hz	min_dr	0	max_dr	3
    channel	18:	905900000	hz	min_dr	0	max_dr	3
    channel	19:	906100000	hz	min_dr	0	max_dr	3
    channel	20:	906300000	hz	min_dr	0	max_dr	3
    channel	21:	906500000	hz	min_dr	0	max_dr	3
    channel	22:	906700000	hz	min_dr	0	max_dr	3
    channel	23:	906900000	hz	min_dr	0	max_dr	3
    channel	24:	907100000	hz	min_dr	0	max_dr	3
    channel	25:	907300000	hz	min_dr	0	max_dr	3
    channel	26:	907500000	hz	min_dr	0	max_dr	3
    channel	27:	907700000	hz	min_dr	0	max_dr	3
    channel	28:	907900000	hz	min_dr	0	max_dr	3
    channel	29:	908100000	hz	min_dr	0	max_dr	3
    channel	30:	908300000	hz	min_dr	0	max_dr	3
    channel	31:	908500000	hz	min_dr	0	max_dr	3
    channel	32:	908700000	hz	min_dr	0	max_dr	3
    channel	33:	908900000	hz	min_dr	0	max_dr	3
    channel	34:	909100000	hz	min_dr	0	max_dr	3
    channel	35:	909300000	hz	min_dr	0	max_dr	3
    channel	36:	909500000	hz	min_dr	0	max_dr	3
    channel	37:	909700000	hz	min_dr	0	max_dr	3
    channel	38:	909900000	hz	min_dr	0	max_dr	3
    channel	39:	910100000	hz	min_dr	0	max_dr	3
    channel	40:	910300000	hz	min_dr	0	max_dr	3
    channel	41:	910500000	hz	min_dr	0	max_dr	3
    channel	42:	910700000	hz	min_dr	0	max_dr	3
    channel	43:	910900000	hz	min_dr	0	max_dr	3
    channel	44:	911100000	hz	min_dr	0	max_dr	3
    channel	45:	911300000	hz	min_dr	0	max_dr	3
    channel	46:	911500000	hz	min_dr	0	max_dr	3
    channel	47:	911700000	hz	min_dr	0	max_dr	3
    channel	48:	911900000	hz	min_dr	0	max_dr	3
    channel	49:	912100000	hz	min_dr	0	max_dr	3
    channel	50:	912300000	hz	min_dr	0	max_dr	3
    channel	51:	912500000	hz	min_dr	0	max_dr	3
    channel	52:	912700000	hz	min_dr	0	max_dr	3
    channel	53:	912900000	hz	min_dr	0	max_dr	3
    channel	54:	913100000	hz	min_dr	0	max_dr	3
    channel	55:	913300000	hz	min_dr	0	max_dr	3
    channel	56:	913500000	hz	min_dr	0	max_dr	3
    channel	57:	913700000	hz	min_dr	0	max_dr	3
    channel	58:	913900000	hz	min_dr	0	max_dr	3
    channel	59:	914100000	hz	min_dr	0	max_dr	3
    channel	60:	914300000	hz	min_dr	0	max_dr	3
    channel	61:	914500000	hz	min_dr	0	max_dr	3
    channel	62:	914700000	hz	min_dr	0	max_dr	3
    channel	63:	914900000	hz	min_dr	0	max_dr	3
    channel	64:	903000000	hz	min_dr	4	max_dr	4
    channel	65:	904600000	hz	min_dr	4	max_dr	4
    channel	66:	906200000	hz	min_dr	4	max_dr	4
    channel	67:	907800000	hz	min_dr	4	max_dr	4
    channel	68:	909400000	hz	min_dr	4	max_dr	4
    channel	69:	911000000	hz	min_dr	4	max_dr	4
    channel	70:	912600000	hz	min_dr	4	max_dr	4
    channel	71:	914200000	hz	min_dr	4	max_dr	4
  */
  for (int c = 0; c < 72; c++) {
    LMIC_disableChannel(c);
  }

  // We'll only enable Channel 8 (903.9Mhz) since we're transmitting on a single-channel
  LMIC_enableChannel(8);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  print_ABP_information();

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
