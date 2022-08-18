#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define AX25_CALLSIGN_LEN 6

constexpr uint8_t pinRX = D3, pinTX = D4;
SoftwareSerial gpsSer(pinRX, pinTX);

TinyGPSPlus gps;

void setup() {
	Serial.begin(115200);

	Serial.println("LoRa Sender");

	pinMode(LED_BUILTIN, OUTPUT);

	LoRa.setPins(D8, D1, D2);

	if (!LoRa.begin(433775000ll)) {
		Serial.println("Starting LoRa failed!");
		while (1) {
		}
	}

	LoRa.setSpreadingFactor(12);
	LoRa.setSignalBandwidth(125000);
	LoRa.setCodingRate4(5);
	LoRa.setPreambleLength(8);
	LoRa.setSyncWord(0x34);
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

	char buffer[32];

	// !5201.66N/00441.75E
	// ddmm.hh

	snprintf(buffer, sizeof buffer, "%02d%02d.%02d%c/%03d%02d.%02d%c",
			int(latd), int(floor(latm)), int(floor(lath)), lat > 0 ? 'N' : 'S',
			int(lngd), int(floor(lngm)), int(floor(lngh)), lng > 0 ? 'E' : 'W');

	return buffer;
}

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
	put_addr(&buffer[0], "APLG01", 0);

	put_addr(&buffer[7], callsign, 0);

	buffer[14] = 0x03;  // UI frame
	buffer[15] = 0xf0;  // no layer 3

	uint8_t text_len = text.length();

	memcpy(&buffer[16], text.c_str(), text_len);

	return 16 + text_len;
}

uint8_t ax25_buffer[256];

uint32_t last_tx = 0;

void loop() {
	uint32_t now = millis();

	while(gpsSer.available()) {
		char c = gpsSer.read();

		gps.encode(c);
	}

	if (now - last_tx >= 2500) {
		digitalWrite(LED_BUILTIN, LOW);

		String aprs;

		if (gps.location.isValid())
			aprs = "!" + gps_double_to_aprs(gps.location.lat(), gps.location.lng());

		aprs += "[www.vanheusden.com";

		uint16_t size = make_ax25(ax25_buffer, aprs, "PD9FVH");

		LoRa.beginPacket();
		LoRa.write(ax25_buffer, size);
		LoRa.endPacket();

		Serial.print(millis());
		Serial.print(F(" transmitting: "));
		Serial.println(aprs);

		digitalWrite(LED_BUILTIN, HIGH);

		last_tx = millis();
	}
}
