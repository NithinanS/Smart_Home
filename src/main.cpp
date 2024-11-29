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
#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

// Sensor วัดความชิ้น + อุณหภูมิ
#define	DHTPIN 4 // เปลี่ยน เลข PIN ด้วย!!!
#define DHTTYPE DHT22 
#define DHT_RESULT_PIN 22 // For test ปล่อยละอองน้ำ (LED)

// Sensor วัดความชิ้น + อุณหภูมิ
#define ULTRA_SONIC_TRIG 18 // เปลี่ยน เลข PIN ด้วย!!!
#define ULTRA_SONIC_ECHO 23 // เปลี่ยน เลข PIN ด้วย!!!

// Sensor วัดควัน
#define SMOKE_DETECTOR 4 // เปลี่ยน เลข PIN ด้วย!!!
#define SPEAKER 19 // ยังไม่เสร็จ

const int SAFETY_LIMIT = 60;

// For PhotoReceptor
int ledPin = 5; // เปลี่ยน เลข PIN ด้วย!!!
int ldrPin = 4;  // เปลี่ยน เลข PIN ด้วย!!!

char ssid[] = "Dami 14";
char password[] = "BigMi1414";

// Keypad
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
//  {'Q','W','#','*'},
 {'1','2','3','U'},
 {'4','5','6','D'},
 {'7','8','9','S'},
 {'L','0','R','N'}
};

byte rowPins[ROWS] = {32,33,25,26}; 
byte colPins[COLS] = {13,12,14,27}; 

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// Declaration for LCD 
LiquidCrystal_I2C lcd(0x27, 16, 2);


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
  int smoke = param.asInt();

  // Update state
//   Blynk.virtualWrite(V1, );
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

void setup() {
	Serial.begin(9600);
	// Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
	timer.setInterval(1000L, myTimerEvent);
	dht_sensor.begin();

	Wire.begin(22,23);
	lcd.begin(16,2,1);

	// // Turn on the blacklight and print a message.
	lcd.backlight();
	lcd.setCursor(0, 0); 
	lcd.print("Hello");

	pinMode(DHT_RESULT_PIN, OUTPUT);

	pinMode(ULTRA_SONIC_TRIG, OUTPUT);
    pinMode(ULTRA_SONIC_ECHO, INPUT);

	pinMode(SMOKE_DETECTOR, INPUT);
	pinMode(SPEAKER, OUTPUT);

	/* Photoreceptor */
	pinMode(ledPin, OUTPUT);
	pinMode(ldrPin, INPUT);
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
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

	// if (smoke > 2200) {
	// 	tone(SPEAKER, 500, 1000);
	// 	Serial.println("SMOKE DETECTED!!!!!");
	// }
	// else {
	// 	noTone(SPEAKER);
	// 	Serial.println("SMOKE NOT FOUND");
	// }
	if (smoke > 2200) {
		digitalWrite(SPEAKER, HIGH);
		Serial.println("SMOKE DETECTED!!!!!");
	}
	else {
		digitalWrite(SPEAKER, LOW);
		Serial.println("SMOKE NOT FOUND");
	}

	delay(1000);
}
/* For krypad */
void scanning() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}
char buf[256];

void startKeyPad() { // กดรหัส ประตูบ้าน

}


void startPhotoReceptor() { // ไฟ auto 
	float  val = 0;
	int val = analogRead(ldrPin);  
  	Serial.print("val = "); 
	Serial.println(val); 
	float MAX = 4095; 
	float MIN = 2500; 
	float c = MAX - val;
	float p = MAX - MIN;
	float Value = (1 - (c / p)) * 255 ;
	Serial.println(Value);

	if (val < MIN ) { 
		analogWrite(ledPin, 0); 
	} 
	else if (val > MAX){
		analogWrite(ledPin, 255);
	}
	else {
		analogWrite(ledPin, Value);
	}
	delay(100);
}

void loop() {
	//Blynk.run();
	startSmokeDetector();
	startPhotoReceptor();
}

