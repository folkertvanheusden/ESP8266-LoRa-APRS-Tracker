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

constexpr uint32_t interval = 5000; // transmit every 5 seconds
constexpr uint32_t interval_jitter = 2000; // ... + max. 2s

constexpr uint32_t short_interval = 2500; // transmit 2nd packet (binary) after 2.5 seconds
constexpr uint32_t short_interval_jitter = 5000; // ... + max. 5s

// -----------------------------------------

SoftwareSerial gpsSer(pinRX, pinTX);

TinyGPSPlus gps;

void setup() {
	Serial.begin(115200);

	Serial.println("LoRa Sender");

	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(D0, OUTPUT);

	LoRa.setPins(pinNSS, pinRESET, pinDIO0);

	if (!LoRa.begin(frequency)) {
		Serial.println("Starting LoRa failed!");
		while (1) {
		}
	}

	LoRa.setSpreadingFactor(12);
	LoRa.setSignalBandwidth(125000);
	LoRa.setCodingRate4(5);
	LoRa.setPreambleLength(8);
	LoRa.setSyncWord(0x12);
	LoRa.enableCrc();

	LoRa.setTxPower(20);

	gpsSer.begin(9600);

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

void loop() {
	uint32_t now = millis();

	while(gpsSer.available()) {
		char c = gpsSer.read();

		gps.encode(c);
	}

	bool gps_updated = gps.location.isUpdated();

	bool gps_valid = gps.location.isValid();

	if (gps_valid != prev_gps_state) {
		prev_gps_state = gps_valid;

		if (gps_valid)
			Serial.println(F("GPS state went to FIX"));
		else
			Serial.println(F("GPS state went to NO FIX"));
	}

	if (now - last_tx >= next_delay && gps_valid) {
		digitalWrite(D0, LOW);
		digitalWrite(LED_BUILTIN, LOW);

		memset(tx_buffer, 0x00, sizeof tx_buffer);

		Serial.print(F("GPS updated: "));
		Serial.println(gps_updated);
		Serial.print(F("GPS coordinates: "));
		Serial.print(gps.location.lat());
		Serial.print(',');

		Serial.println(gps.location.lng());

		String aprs;
		aprs += "!" + gps_double_to_aprs(gps.location.lat(), gps.location.lng());

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

		LoRa.beginPacket();
		LoRa.write(tx_buffer, size);
		LoRa.endPacket();

		Serial.print(millis());
		Serial.print(F(" transmitting: "));
		Serial.println(reinterpret_cast<char *>(tx_buffer));

		digitalWrite(LED_BUILTIN, HIGH);

		last_tx = millis();
	}

	digitalWrite(D0, (now & 256) && (gps_valid | gps_updated));

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
