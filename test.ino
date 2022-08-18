#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
	Serial.begin(115200);

	Serial.println("LoRa Sender");

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
}

void loop() {
	Serial.print(millis());
	Serial.print(" sending packet: ");
	Serial.println(counter);

	// send packet
	LoRa.beginPacket(false);
	LoRa.print("hello ");
	LoRa.print(counter);
	LoRa.endPacket();

	counter++;

	delay(1000);
}
