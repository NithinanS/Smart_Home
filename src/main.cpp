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
#include <ArtronShop_LineNotify.h>	

/* RT TX  */
#define TxPin 1
#define RxPin 3

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
#define SMOKE_DETECTOR 36 // เปลี่ยน เลข PIN ด้วย!!!
// #define SPEAKER 19 // ยังไม่เสร็จ

/* Line Notify Token*/
#define LINE_TOKEN "DJTlQcoA6UTw7mkog4TQc43GWsnweROoGCh8VhtoFMF" // Do not DELETE

const int SAFETY_LIMIT = 60;

// For PhotoReceptor
#define LEDPIN 5
#define LDRPIN 4  // เปลี่ยน เลข PIN ด้วย!!!

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

// Declaration for LCD  (เผื่อจะต่อ จอ LCD แต่ตอนนี้ไม้่ได้ใช้)
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
	// timer.setInterval(1000L, myTimerEvent);
	dht_sensor.begin();

	/* ---- KeyPad  & LCD Display ---- */
	Wire.begin();
	// lcd.begin(16,2,1);

	// // Turn on the blacklight and print a message.
	// lcd.backlight();
	// lcd.setCursor(0, 0); 
	// lcd.print("Hello");
	/*  ------------------------------ */

	/* Connecting to WIFI */
	Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password); // เริ่มต้นเชื่อมต่อ WiFi

    while (WiFi.status() != WL_CONNECTED) { // วนลูปหากยังเชื่อมต่อ WiFi ไม่สำเร็จ
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
	/*---------------------------*/

	/* Line Notify Connection */
	LINE.begin(LINE_TOKEN);

	if (LINE.send("Test Nofication")) { 
    	Serial.println("Send notify successful"); // ส่งข้อความ "Send notify successful" ไปที่ Serial Monitor
  	} 
	else { // ถ้าส่งไม่สำเร็จ
    	Serial.printf("Send notify fail. check your token (code: %d)\n", LINE.status_code); // ส่งข้อความ "Send notify fail" ไปที่ Serial Monitor
  	}
	/*---------------------------*/


	pinMode(DHT_RESULT_PIN, OUTPUT);

	pinMode(ULTRA_SONIC_TRIG, OUTPUT);
    pinMode(ULTRA_SONIC_ECHO, INPUT);

	pinMode(SMOKE_DETECTOR, INPUT);
	// pinMode(SPEAKER, OUTPUT);

	/* Photoreceptor */
	pinMode(LEDPIN, OUTPUT);
	pinMode(LDRPIN, INPUT);
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

bool haveNotified = false;

void startUltraSonic() {
	// Ultra Sonic Sensor ชาง (OK)
	digitalWrite(ULTRA_SONIC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(ULTRA_SONIC_TRIG, LOW);

	// measure duration of pulse from ECHO pin
	float duration_us = pulseIn(ULTRA_SONIC_ECHO, HIGH);
	float distance_cm = 0.017 * duration_us;

	Serial.println(distance_cm);

	if(distance_cm <= 70) {
		if (!haveNotified) {
			LINE.send("Visitor Detected");
			haveNotified = true;
		}
		Serial.println("life detected");
	}
	else {
		Serial.println("zzz...");
		haveNotified = false;
	}
	delay(1000);
}

int lastDetectedTime = 0;
void startSmokeDetector() {
	/* Smoke Detector */
	int smoke = analogRead(SMOKE_DETECTOR);
	Serial.println("Smoke Density: " + String(smoke));

	if (smoke > 2200) {
		// digitalWrite(SPEAKER, HIGH);
		if (millis() - lastDetectedTime > 30 * 1000) {
			LINE.send("Smoke Detetected");
			Serial.println("SMOKE DETECTED!!!!!");
		}
	}
	else {
		// digitalWrite(SPEAKER, LOW);
		Serial.println("SMOKE NOT FOUND");
	}

	delay(1000);
}
/* For Keypad */
char buf[256];

String input = "";
boolean isEnter;
void startKeyPad(String password) { // กดรหัส ประตูบ้าน
	char key = keypad.getKey(); 
	if (key != NO_KEY){ 
		if (isEnter || key == 'S') {
			lcd.clear();
			input = "";
			isEnter = 0;
		}
		Serial.println(key);
		// lcd.setCursor(0, 1); 
		// lcd.print(key);
		

		if (key != 'N' && key != 'S' && key != 'L' && key != 'R' && key != 'U' && key != 'D') {
			input += key;
		//   lcd.print(input);
			Serial.println(input);
		}
		else if (key == 'N') {
		// lcd.print(input);
			if (input == password ) {
				// lcd.setCursor(0, 0);
				Serial.println("Door Opened");
				isEnter = 1;
		} 
		else {
			// lcd.setCursor(0, 0);
			Serial.println("Incorrect Password");
			isEnter = 1;
		}
		input = "";
	   
	}
		delay(1);     
	}
}

float val = 100;
void startPhotoReceptor() { // ไฟ auto 
	val = analogRead(LDRPIN);  
  	Serial.print("val = "); 
	Serial.println(val); 
	float MAX = 2000; 
	float MIN = 260; 
	float c = MAX - val;
	float p = MAX - MIN;
	float Value = (1 - (c / p)) * 255 ; // อย่าลืมลบออก เอาแค่ติด/ ไม่ติด
	Serial.println(Value);

	if (val < MIN ) { 
		// analogWrite(ledPin, 0); 
		digitalWrite(LEDPIN, HIGH);
	} 
	else if (val > MAX){
		// analogWrite(LEDPIN, 255);
		digitalWrite(LEDPIN, LOW);
	}
	else {
		analogWrite(LEDPIN, Value);
	}
	delay(1000);
}

void loop() {
	//Blynk.run();
	startSmokeDetector();
	// startPhotoReceptor();
	startUltraSonic();
	startKeyPad("12345"); // Door's Password = 12345
}

