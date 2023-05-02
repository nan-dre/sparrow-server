#include "Adafruit_BME680.h"
#include "secrets.h"
#include <Adafruit_Sensor.h>
#include <LTR308.h>
#include <SPI.h>
#include <Wifi.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;
LTR308 light;
WiFiServer server(80);
String header;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

unsigned char gain = 0;			   // Gain setting, values = 0-4
unsigned char integrationTime = 0; // Integration ("shutter") time, values 0 - 4
unsigned char measurementRate = 3; // Interval between DATA_REGISTERS update, values 0 - 7, except 4
unsigned char ID;
unsigned char control;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
float hum_reference = 40;

float calculate_iaq(float gas, float humidity)
{
	float hum_score;
	float gas_score;
	if (humidity >= 38 && humidity <= 42)
		hum_score = 0.25 * 100; // Humidity +/-5% around optimum
	else {						// sub-optimal
		if (humidity < 38)
			hum_score = 0.25 / hum_reference * humidity * 100;
		else {
			hum_score = ((-0.25 / (100 - hum_reference) * humidity) + 0.416666) * 100;
		}
	}

	// Calculate gas contribution to IAQ index
	float gas_lower_limit = 5000;  // Bad air quality limit
	float gas_upper_limit = 50000; // Good air quality limit
	if (gas > gas_upper_limit)
		gas = gas_upper_limit;
	if (gas < gas_lower_limit)
		gas = gas_lower_limit;
	gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas -
				 (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) *
				100;

	// Combine results for the final IAQ index value (0-100% where 100% is good quality air)
	float air_quality_score = hum_score + gas_score;
	return air_quality_score;
}

void printError(byte error)
{
	// If there's an I2C error, this function will
	// print out an explanation.

	Serial.print("I2C error: ");
	Serial.print(error, DEC);
	Serial.print(", ");

	switch (error) {
	case 0:
		Serial.println("success");
		break;
	case 1:
		Serial.println("data too long for transmit buffer");
		break;
	case 2:
		Serial.println("received NACK on address (disconnected?)");
		break;
	case 3:
		Serial.println("received NACK on data");
		break;
	case 4:
		Serial.println("other error");
		break;
	default:
		Serial.println("unknown error");
	}
}

void setup()
{
	Serial.begin(9600);
	delay(1000);
	pixel.begin();
	pixel.setBrightness(3);
	pixel.fill(0xFF00FF);
	pixel.show();
	while (!Serial)
		;
	Serial.println(F("Sparrow test"));

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME680 sensor, check wiring!");
		while (1)
			;
	}
	if (!light.begin()) {
		Serial.println("Could not find a valid LTR308 sensor, check wiring!");
		while (1)
			;
	}
	if (!SD.begin(4)) {
		Serial.println("Could not find a valid SD, check wiring!");
		while (1)
			;
	}
	File file = SD.open("/hello.txt", FILE_WRITE);

	if (file) {
		Serial.print("Writing to hello.txt...");
		file.println("hello.txt");
		// close the file:
		file.close();
		Serial.println("done.");
	} else {
		// if the file didn't open, print an error:
		Serial.println("error opening hello.txt");
	}

	// Now read from it
	Serial.println("Read from the file:");
	file = SD.open("/hello.txt", FILE_READ);
	if (file) {
		while (file.available()) {
			Serial.write(file.read());
		}
		file.close();
	} else {
		Serial.println("error opening hello.txt");
	}

	// Set up oversampling and filter initialization
	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150); // 320*C for 150 ms

	if (light.getPartID(ID)) {
		Serial.print("Got Sensor Part ID: 0X");
		Serial.print(ID, HEX);
		Serial.println();
	}
	// Most library commands will return true if communications was successful,
	// and false if there was a problem. You can ignore this returned value,
	// or check whether a command worked correctly and retrieve an error code:
	else {
		byte error = light.getError();
		printError(error);
	}

	// To start taking measurements, power up the sensor

	if (light.setPowerUp()) {
		Serial.print("Powering up...");
		Serial.println();
	} else {
		byte error = light.getError();
		printError(error);
	}

	// Allow for a slight delay in power-up sequence (typ. 5ms from the datasheet)
	delay(10);

	if (light.getPower(control)) {
		Serial.print("Control byte is: 0X");
		Serial.print(control, HEX);
		Serial.println();
	} else {
		byte error = light.getError();
		printError(error);
	}
	// The light sensor has a default integration time of 100ms,
	// and a default gain of low (3X).

	// If you would like to change either of these, you can
	// do so using the setGain() and setMeasurementRate() command.

	Serial.println("Setting Gain...");

	if (light.setGain(gain)) {
		light.getGain(gain);

		Serial.print("Gain Set to 0X");
		Serial.print(gain, HEX);
		Serial.println();
	} else {
		byte error = light.getError();
		printError(error);
	}

	Serial.println("Set timing...");
	if (light.setMeasurementRate(integrationTime, measurementRate)) {
		light.getMeasurementRate(integrationTime, measurementRate);

		Serial.print("Timing Set to ");
		Serial.print(integrationTime, HEX);
		Serial.println();

		Serial.print("Meas Rate Set to ");
		Serial.print(measurementRate, HEX);
		Serial.println();
	} else {
		byte error = light.getError();
		printError(error);
	}

	Serial.print("Connecting to ");
	Serial.println(ssid);
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected.");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());
	server.begin();
}

void read_from_bme()
{
	if (!bme.performReading()) {
		Serial.println("Failed to perform reading :(");
		return;
	}
	Serial.print("Temperature = ");
	Serial.print(bme.temperature);
	Serial.println(" *C");

	Serial.print("Pressure = ");
	Serial.print(bme.pressure / 100.0);
	Serial.println(" hPa");

	Serial.print("Humidity = ");
	Serial.print(bme.humidity);
	Serial.println(" %");

	Serial.print("Gas = ");
	Serial.print(bme.gas_resistance / 1000.0);
	Serial.println(" KOhms");

	Serial.print("Approx. Altitude = ");
	Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
	Serial.println(" m");

	Serial.println();
}

void read_from_ltr()
{
	unsigned long rawData;

	if (light.getData(rawData)) {

		Serial.print("Raw Data: ");
		Serial.println(rawData);

		// To calculate lux, pass all your settings and readings
		// to the getLux() function.

		// The getLux() function will return 1 if the calculation
		// was successful, or 0 if the sensor was
		// saturated (too much light). If this happens, you can
		// reduce the integration time and/or gain.

		double lux;	  // Resulting lux value
		boolean good; // True if sensor is not saturated

		// Perform lux calculation:

		good = light.getLux(gain, integrationTime, rawData, lux);

		// Print out the results:

		Serial.print("Lux: ");
		Serial.print(lux);
		if (good)
			Serial.println(" (valid data)");
		else
			Serial.println(" (BAD)");
	} else {
		byte error = light.getError();
		printError(error);
	}
}

void loop()
{
	WiFiClient client = server.available();
	if (client) { // If a new client connects,
		currentTime = millis();
		previousTime = currentTime;
		Serial.println("New Client."); // print a message out in the serial port
		String currentLine = "";	   // make a String to hold incoming data from the client
		while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
			currentTime = millis();
			if (client.available()) {	// if there's bytes to read from the client,
				char c = client.read(); // read a byte, then
				Serial.write(c);		// print it out the serial monitor
				header += c;
				if (c == '\n') { // if the byte is a newline character
					// if the current line is blank, you got two newline characters in a row.
					// that's the end of the client HTTP request, so send a response:
					if (currentLine.length() == 0) {
						// HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
						// and a content-type so the client knows what's coming, then a blank line:
						client.println("HTTP/1.1 200 OK");
						client.println("Content-type:application/json");
						client.println("Connection: close");
						client.println();

						if (header.indexOf("GET /bme") >= 0) {
							bme.performReading();
							float iaq = calculate_iaq(bme.gas_resistance, bme.humidity);
							client.printf(
								"{\"temperature\":%f,\"pressure\":%f,\"humidity\":%f,\"gas\":%f,\"altitude\":%f,\"iaq\":%f}",
								bme.temperature, bme.pressure / 100.0, bme.humidity, bme.gas_resistance / 1000.0,
								bme.readAltitude(SEALEVELPRESSURE_HPA), iaq);
						} else if (header.indexOf("GET /ltr") >= 0) {
							unsigned long int rawData;
							double lux;
							light.getData(rawData);
							light.getLux(gain, integrationTime, rawData, lux);
							client.printf("{\"lux\":%f}", lux);
						}
						break;
					} else {
						currentLine = "";
					}
				} else if (c != '\r') {
					currentLine += c;
				}
			}
		}
		header = "";
		// Close the connection
		client.stop();
		Serial.println("Client disconnected.");
		Serial.println("");
	}
}