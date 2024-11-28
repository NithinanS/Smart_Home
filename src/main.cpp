// For Blynk 
#define BLYNK_TEMPLATE_ID           "TMPL61lOG-kSF"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "foSfONd_gnxdxbSVeqfF1mIl_n5rxAGR"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>




/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

// Sensor วัดความชิ้น + อุณหภูมิ
#define	DHTPIN 4
#define DHTTYPE DHT22
#define DHT_RESULT_PIN 22 // For test ปล่อยละอองน้ำ (LED)

// Sensor วัดความชิ้น + อุณหภูมิ
#define ULTRA_SONIC_TRIG 18
#define ULTRA_SONIC_ECHO 23

// Sensor วัดควัน
#define SMOKE_DETECTOR 2
#define SPEAKER 19

const int SAFETY_LIMIT = 60;


char ssid[] = "Dami 14";
char password[] = "BigMi1414";



float humidity, temp, heatIndex;

DHT dht_sensor(DHTPIN, DHTTYPE);

// This function is called every time the Virtual Pin 0 state changes
BlynkTimer timer;

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
}

BLYNK_WRITE(V4) // smoke
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}


void setup() {
	Serial.begin(9600);
	Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
	timer.setInterval(1000L, myTimerEvent);
	dht_sensor.begin();

	pinMode(DHT_RESULT_PIN, OUTPUT);

	pinMode(ULTRA_SONIC_TRIG, OUTPUT);
    pinMode(ULTRA_SONIC_ECHO, INPUT);

	pinMode(SMOKE_DETECTOR, INPUT);
	pinMode(SPEAKER, OUTPUT);

}

void loop() {
	Blynk.run();
	startSmokeDetector();
	// DHT 22 วัดความชื้น  (OK)
	// humidity = dht_sensor.readHumidity();
	// temp = dht_sensor.readTemperature(false); // Temp in Celcius
	// heatIndex = dht_sensor.computeHeatIndex(temp, humidity, false);

	// Serial.println("Temperature: " + String(temp) + " Celcius");
	// Serial.println("Humidity: " + String(humidity) + " %");

	// Serial.println("Heat Index: " + String(heatIndex) + " Celcius");

	// if (heatIndex > 28) {
	// 	digitalWrite(DHT_RESULT_PIN, 1);

		
	// }
	// else {
	// 	digitalWrite(DHT_RESULT_PIN, 0);
	// }

	// delay(2000);

	
	
}

void startDHT() {

	// DHT 22 วัดควา?มชื้น  (OK)
	humidity = dht_sensor.readHumidity();
	temp = dht_sensor.readTemperature(false); // Temp in Celcius
	heatIndex = dht_sensor.computeHeatIndex(temp, humidity, false);

	Serial.println("Temperature: " + String(temp) + " Celcius");
	Serial.println("Humidity: " + String(humidity) + " %");

	Serial.println("Heat Index: " + String(heatIndex) + " Celcius");

	if (heatIndex > 28) {
		digitalWrite(DHT_RESULT_PIN, 1);

		
	}
	else {
		digitalWrite(DHT_RESULT_PIN, 0);
	}

	delay(2000);
}

void startUltraSonic() {
	// Ultra Sonic Sensor ชาง (OK)
	digitalWrite(ULTRA_SONIC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(ULTRA_SONIC_TRIG, LOW);

	// measure duration of pulse from ECHO pin
	float duration_us = pulseIn(ULTRA_SONIC_ECHO, HIGH);
	float distance_cm = 0.017 * duration_us;

	Serial.println(duration_us);

	if(distance_cm <= 70) {
		Serial.println("life detected");
	}
	else Serial.println("zzz...");
	
	delay(1000);
}

void startSmokeDetector() {
	/* Smoke Detector */
	int smoke = analogRead(SMOKE_DETECTOR);
	Serial.println("Smoke Density: " + String(smoke));

	if (smoke > SAFETY_LIMIT) {
		// tone(SPEAKER, 500, 1000);
		Serial.println("SMOKE DETECTED!!!!!");
	}
	else {
		// noTone(SPEAKER);
		Serial.println("SMOKE NOT FOUND");
	}

	delay(1000);
}
