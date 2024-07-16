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

constexpr uint8_t pin_LED_TX = D0;

constexpr uint32_t interval = 5000; // transmit every 5 seconds
constexpr uint32_t interval_jitter = 2000; // ... + max. 2s

constexpr uint32_t short_interval = 2500; // transmit 2nd packet (binary) after 2.5 seconds
constexpr uint32_t short_interval_jitter = 5000; // ... + max. 5s

// -----------------------------------------

volatile bool ledStatus = false;

bool monitor_gps = false;

ESP8266Timer ITimer;

SoftwareSerial gpsSer(pinRX, pinTX);

TinyGPSPlus gps;

void setup() {
	Serial.begin(115200);

	Serial.println(F("LoRa tracker " __DATE__ " " __TIME__));
	Serial.println(F("Written by Folkert van Heusden <mail@vanheusden.com>"));

	Serial.println(F("Callsign: " SRC_CALLSIGN));

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	pinMode(pin_LED_TX, OUTPUT);
	digitalWrite(pin_LED_TX, HIGH);

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

	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(pin_LED_TX, LOW);

	Serial.println(F("Go!"));

	Serial.print(F("> "));
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

void put_addr(uint8_t *const target, const char *const what, const byte ssid) {
	byte i   = 0;
	byte len = strlen(what);

	for(i=0; i<len; i++)
		target[i] = what[i] << 1;

	for(; i<AX25_CALLSIGN_LEN; i++)
		target[i] = ' ' << 1;

	target[6] = (('0' + ssid) << 1) | 1;
}

uint16_t make_ax25(uint8_t *const buffer, const String & text, const char *const callsign) {
	put_addr(&buffer[0], DST_CALLSIGN, 0);

	put_addr(&buffer[7], callsign, 0);

	buffer[14] = 0x03;  // UI frame
	buffer[15] = 0xf0;  // no layer 3

	uint8_t text_len = text.length();

	memcpy(&buffer[16], text.c_str(), text_len);

	return 16 + text_len;
}

void emit_gps_stats(const bool force) {
	static int num_sat = -1, loc_valid = -1;

	int cur_num_sat = gps.satellites.value();

	if (cur_num_sat != num_sat || force) {
		num_sat = cur_num_sat;

		Serial.print(millis());
		Serial.print(F(" number of satellites seen: "));
		Serial.println(cur_num_sat);
	}

	int cur_loc_valid = gps.location.isValid();

	if (cur_loc_valid != loc_valid || force) {
		loc_valid = cur_loc_valid;

		Serial.print(millis());
		Serial.print(F(" position valid: "));
		Serial.println(gps.location.isValid());
	}

	if (force) {
		double latitude  = gps.location.lat();
		double longitude = gps.location.lng();

		Serial.print(millis());
		Serial.print(F(" position: "));
		Serial.println(gps_double_to_aprs(latitude, longitude));
	}
}

uint8_t  tx_buffer[256];

uint32_t last_tx = 0;
uint32_t next_delay = 2500;
bool     mode = false;
int      prev_gps_state = -1;

double   latitude         = 0.;
double   longitude        = 0.;
bool     gps_updated      = false;
bool     first_state_dump = true;
bool     show_until_fix   = false;
int      p_validness      = -1;

bool     force_send       = false;

char line[128] { 0 };
int  line_pos { 0 };

uint32_t msgs_transmitted = 0;
uint32_t msgs_received    = 0;

double   p_distance       = 0.;

typedef struct {
	uint32_t ts;
	double   lat;
	double   lng;
        bool     fix;
	uint8_t  n_sat;
} history_t;

std::vector<history_t> history;

void emit_history() {
	Serial.println(F("ts\tlat\tlng\tfix\t#sat"));

	for(auto h : history) {
		Serial.print(h.ts);
		Serial.print('\t');
		Serial.print(h.lat);
		Serial.print('\t');
		Serial.print(h.lng);
		Serial.print('\t');
		Serial.print(h.fix);
		Serial.print('\t');
		Serial.print(h.n_sat);
		Serial.println(F(""));
	}

	Serial.println(F(""));
}

void emit_gen_stats() {
	Serial.print(F("Number of messages transmitted: "));
	Serial.println(msgs_transmitted);

	Serial.print(F("Number of messages received: "));
	Serial.println(msgs_received);

	Serial.print(F("Uptime: "));
	Serial.println(millis());
}

void process_command() {
	if (strcmp(line, "stats") == 0) {
		emit_gen_stats();
		emit_gps_stats(true);
	}
	else if (strcmp(line, "force") == 0)
		force_send = true;
	else if (strcmp(line, "history") == 0)
		emit_history();
	else if (strcmp(line, "mon") == 0)
		monitor_gps = !monitor_gps;
	else if (strcmp(line, "reboot") == 0)
		ESP.restart();
	else if (strcmp(line, "help") == 0)
		Serial.println(F("force / history / reboot / stats"));
	else {
		Serial.println(F("?"));
	}
}

void loop() {
	uint32_t now = millis();

	while(Serial.available()) {
		int c = Serial.read();

		Serial.print(char(c));

		if ((c == 8 || c == 127) && line_pos > 0)
			line_pos--;
		else if (c == 13) {
			line[line_pos] = 0x00;
			line_pos = 0;

			Serial.println(F(""));
			
			process_command();

			Serial.println(F(""));
			Serial.print(F("> "));
		}
		else if (c == 10)
			continue;
		else if (line_pos < sizeof(line) - 1) {
			line[line_pos++] = c;
		}
	}

	while(gpsSer.available()) {
		int c = gpsSer.read();
		if (monitor_gps)
			Serial.print(char(c));
		gps.encode(c);
	}

	double new_latitude  = gps.location.lat();
	double new_longitude = gps.location.lng();

	double distance      = TinyGPSPlus::distanceBetween(new_latitude, new_longitude, latitude, longitude);

	// 25: gps resolution
	if (int(distance / 25) != int(p_distance / 25) || gps.location.isValid() != p_validness) {
		p_distance  = distance;

		p_validness = gps.location.isValid();

		history_t h;
		h.ts    = millis();
		h.lat   = new_latitude;
		h.lng   = new_longitude;
        	h.fix   = p_validness;
		h.n_sat = gps.satellites.value();

		history.push_back(h);

		while(history.size() > 10)
			history.erase(history.begin() + 0);
	}

	gps_updated |= (new_latitude != latitude && new_latitude != 0.) || (new_longitude != longitude && new_longitude != 0.);

	ledStatus = (!!(now & 512)) && gps_updated;

	latitude  = new_latitude;
	longitude = new_longitude;

	if ((now - last_tx >= next_delay && gps_updated) || force_send) {
		digitalWrite(pin_LED_TX, HIGH);

		if (show_until_fix)
			emit_gps_stats(false);

		show_until_fix = first_state_dump = false;

		force_send  = false;

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

		msgs_transmitted++;

		LoRa.beginPacket();
		LoRa.write(tx_buffer, size);
		LoRa.endPacket();

		ITimer.restartTimer();

		Serial.print(millis());
		Serial.print(F(" transmitting: "));
		Serial.println(reinterpret_cast<char *>(tx_buffer));

		last_tx = millis();

		digitalWrite(pin_LED_TX, LOW);
	}

	if (now > 60000 && (first_state_dump || show_until_fix)) {
		digitalWrite(LED_BUILTIN, HIGH);

		if (first_state_dump) {
			first_state_dump = false;

			show_until_fix = true;
		}

		emit_gps_stats(false);

		digitalWrite(LED_BUILTIN, LOW);
	}

	int packetSize = LoRa.parsePacket();

	if (packetSize > 0) {
		digitalWrite(pin_LED_TX, !!(millis() & 256));

		Serial.print(millis());
		Serial.print(F(" received packet with RSSI "));
		Serial.print(LoRa.packetRssi());
		Serial.print(F(": "));

		while(LoRa.available())
			Serial.print(char(LoRa.read()));

		Serial.println(F(""));

		msgs_received++;
	}
}
