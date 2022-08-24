#include <ESP8266TimerInterrupt.h>
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// ----- change this (when required) -------

#define DST_CALLSIGN "APRS"
#define SRC_CALLSIGN "PD9FVH-2"
#define TEXT "www.vanheusden.com"

constexpr uint8_t pinNSS = D8, pinRESET = D1, pinDIO0 = D2;

constexpr uint32_t frequency = 433775000ll;

constexpr uint8_t pinRX = D3, pinTX = D4;  // serial pins to the GPS

constexpr uint8_t pinGPSFixLed = D0;

constexpr uint32_t interval = 5000; // transmit every 5 seconds
constexpr uint32_t interval_jitter = 2000; // ... + max. 2s

constexpr uint32_t short_interval = 2500; // transmit 2nd packet (binary) after 2.5 seconds
constexpr uint32_t short_interval_jitter = 5000; // ... + max. 5s

// -----------------------------------------

volatile bool ledStatus = false;

ESP8266Timer ITimer;

SoftwareSerial gpsSer(pinRX, pinTX);

TinyGPSPlus gps;

void IRAM_ATTR TimerHandler() {
	digitalWrite(pinGPSFixLed, ledStatus);
}

void setup() {
	Serial.begin(115200);

	Serial.println(F("LoRa tracker " __DATE__ " " __TIME__));
	Serial.println(F("Written by Folkert van Heusden <mail@vanheusden.com>"));

	Serial.println(F("Callsign: " SRC_CALLSIGN));

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	pinMode(pinGPSFixLed, OUTPUT);
	digitalWrite(pinGPSFixLed, HIGH);

	LoRa.setPins(pinNSS, pinRESET, pinDIO0);

	if (!LoRa.begin(frequency)) {
		Serial.println("Starting LoRa failed!");

		ESP.restart();
	}

	LoRa.setSpreadingFactor(12);
	LoRa.setSignalBandwidth(125000);
	LoRa.setCodingRate4(5);
	LoRa.setPreambleLength(8);
	LoRa.setSyncWord(0x12);
	LoRa.enableCrc();

	LoRa.setTxPower(20);

	gpsSer.begin(9600);

	ITimer.attachInterruptInterval(1000 * 250, TimerHandler);

	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(pinGPSFixLed, LOW);

	Serial.println(F("Go!"));
}

String gps_double_to_aprs(double lat, double lng) {
	double lata = abs(lat);
	double latd = floor(lata);
	double latm = (lata - latd) * 60;
	double lath = (latm - floor(latm)) * 100;
	double lnga = abs(lng);
	double lngd = floor(lnga);
	double lngm = (lnga - lngd) * 60;
	double lngh = (lngm - floor(lngm)) * 100;

	static char buffer[71];

	snprintf(buffer, sizeof buffer, "%02d%02d.%02d%c/%03d%02d.%02d%c",
			int(latd), int(floor(latm)), int(floor(lath)), lat > 0 ? 'N' : 'S',
			int(lngd), int(floor(lngm)), int(floor(lngh)), lng > 0 ? 'E' : 'W');

	return buffer;
}

#define AX25_CALLSIGN_LEN 6

void put_addr(uint8_t *const target, const char *const what, const byte ssid)
{
	byte i   = 0;
	byte len = strlen(what);

	for(i=0; i<len; i++)
		target[i] = what[i] << 1;

	for(; i<AX25_CALLSIGN_LEN; i++)
		target[i] = ' ' << 1;

	target[6] = (('0' + ssid) << 1) | 1;
}

uint16_t make_ax25(uint8_t *const buffer, const String & text, const char *const callsign)
{
	put_addr(&buffer[0], DST_CALLSIGN, 0);

	put_addr(&buffer[7], callsign, 0);

	buffer[14] = 0x03;  // UI frame
	buffer[15] = 0xf0;  // no layer 3

	uint8_t text_len = text.length();

	memcpy(&buffer[16], text.c_str(), text_len);

	return 16 + text_len;
}

uint8_t  tx_buffer[256];

uint32_t last_tx = 0;
uint32_t next_delay = 2500;
bool     mode = false;
int      prev_gps_state = -1;

double   latitude    = 0.;
double   longitude   = 0.;
bool     gps_updated = false;

void loop() {
	uint32_t now = millis();

	while(gpsSer.available())
		gps.encode(gpsSer.read());

	double new_latitude  = gps.location.lat();
	double new_longitude = gps.location.lng();

	gps_updated |= (new_latitude != latitude && new_latitude != 0.) || (new_longitude != longitude && new_longitude != 0.);

	ledStatus = (now & 256) && gps_updated;

	latitude  = new_latitude;
	longitude = new_longitude;

	if (now - last_tx >= next_delay && gps_updated) {
		digitalWrite(LED_BUILTIN, LOW);

		gps_updated = false;

		memset(tx_buffer, 0x00, sizeof tx_buffer);

		Serial.print(F("GPS coordinates: "));
		Serial.print(latitude, 6);
		Serial.print(F(", "));
		Serial.println(longitude, 6);

		String aprs;
		aprs += "!" + gps_double_to_aprs(latitude, longitude);

		aprs += "[" TEXT;

		uint16_t size = 0;

		if (mode) {
			size = make_ax25(tx_buffer, aprs, SRC_CALLSIGN);
			next_delay = interval + (rand() % interval_jitter);
		}
		else {
			size = snprintf(reinterpret_cast<char *>(tx_buffer), sizeof tx_buffer, "\x3c\xff\x01%s>%s:%s", SRC_CALLSIGN, DST_CALLSIGN, aprs.c_str());
			next_delay = short_interval + (rand() % short_interval_jitter);
		}

		mode = !mode;

		ITimer.stopTimer();

		LoRa.beginPacket();
		LoRa.write(tx_buffer, size);
		LoRa.endPacket();

		ITimer.restartTimer();

		Serial.print(millis());
		Serial.print(F(" transmitting: "));
		Serial.println(reinterpret_cast<char *>(tx_buffer));

		last_tx = millis();

		digitalWrite(LED_BUILTIN, HIGH);
	}

	int packetSize = LoRa.parsePacket();

	if (packetSize > 0) {
		Serial.print(F("Received packet with RSSI "));
		Serial.print(LoRa.packetRssi());
		Serial.print(F(": "));

		while(LoRa.available())
			Serial.print(char(LoRa.read()));

		Serial.println(F(""));
	}
}
