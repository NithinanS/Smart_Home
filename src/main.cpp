#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <ArtronShop_LineNotify.h>	
#include <SoftwareSerial.h>
#include <ESP_Google_Sheet_Client.h>
#include <ESP32Servo.h>

/* RT TX  */
#define TxPin 22
#define RxPin 23

// Sensor วัดความชิ้น + อุณหภูมิ
#define	DHTPIN 4 
#define DHTTYPE DHT22 

// Sensor วัดความชิ้น + อุณหภูมิ
#define ULTRA_SONIC_TRIG 18 
#define ULTRA_SONIC_ECHO 17 

// Sensor วัดควัน
#define SMOKE_DETECTOR 36 // เปลี่ยน เลข PIN ด้วย!!!
// #define SPEAKER 19 // ยังไม่เสร็จ

#define MOTION_SENSOR 21

#define SERVO_PIN 19

Servo myServo;

const int SAFETY_LIMIT = 60;

// For PhotoReceptor
#define LDRPIN 34  

SoftwareSerial anotherSerial(RxPin, TxPin);


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

float humidity, temp, heatIndex;

DHT dht_sensor(DHTPIN, DHTTYPE);

// Google Project ID
#define PROJECT_ID "smart-home-logging"

// Service Account's client email
#define CLIENT_EMAIL "smart-home-data@smart-home-logging.iam.gserviceaccount.com"

// Service Account's private key
const char PRIVATE_KEY[] PROGMEM = "-----BEGIN PRIVATE KEY-----\nMIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQC2bwVe/9gDRzgX\nXhPKYvK+OyljsyF1z4KJP0nhXpjCgi3875qpT2P8b6wfFGO2UUve2gVyXfPjSom3\nONxrQftOYZvwuWf1V2gGkxFLAeHEDfLK9oXS0eAFBbP2RFWB8pXrU454CMx2Mlz2\n7B6Hh8/fSzgUdNDjhdXtk9b9kM9f24x26zc9cyi+tKzWdKGDoFckQ/HmeFaKwxxb\n04nyEwV/yU3v11FslpB7UVqdO7pkMjgPfHGBrchMRWeWgzlgUJ9au6GGiuG4XUu/\np+v95uD1X/GfHxJFP9JOiyU2Cbikd+UzDONzOYEqISshAFMgDUGcU1ht4yBMVEkW\n3U4C9ny5AgMBAAECggEAAIot21HVxTXvVIlefAmhDEKfaW9zggug3ZA6nAzF8q6Z\n8rQl1fvgm4IVBUY+C1albrkum4bdcQtQO5SPQIELSsMBzzt0FtuBFUY63pOfx4DY\nAZHAoJUDTQOdQ4rAvoNrIbf1nIxjyF13MsEbSNnJjzQcwYRnr2zexANdJWhUWIU9\nseRCUkkE5sXAYE4sDGJoeXQlZW2RZGqCAyC2k/8DvpKz5dtnGt/v2/QEtM/NEBIy\nS6p+hVAY8VIIOP5splXtPhmfbbbylShumIpbGIqLX/fytRpGSp55i9JbbqaaBytA\nIy68lAd9yRqJATIJnoYcB1nrI14Sw82POA+ASJgLcQKBgQDw2rloI3mjcBVoXXYs\nw2E9tq1b7SJraekpXecxSLZKx/ET8KPmeLLVzaaiDngF8w0fU7o2tPXZ5UHqQoDo\nyw0yxcSm8AKXWyTI9928XgFy7rq5hgaZZdfKUWd0sjAKsphTZ07igIYsWHESNKSg\nNEu2nH1RT+iUys4VcohXOZm/SQKBgQDB59bxpP6lndrw2uwOPz/kDHuEZrT5sMfH\ny3AUwlMUDJywTOHLvBJvlmmUa4xUEU36+JWW+gmmqtHZme9PyAKloQGmUzdq9LyV\n9dxOYgtZ4AFQA/7P5JL/CLo/a+mnNNHoUudpidm8E5avrWONpxzh6aBjaehA5FED\nrfF0Tgkh8QKBgQDfbkgo7x/cGhIB6xHBnM4MbfaMHIOdmXaOxWm/MnDJFonxsX6a\nQsXDyS8XjsyQ2FQIB6frMBwRsrdfFap4SkVIIGa5ZlVBC3AobqfoIly9vfDCQi7F\nBtEUAkgy3Unr4pmkZWWsgQX3BWR/Ow6s/1ZtZOZsE/DyduCOzNzl7a1O4QKBgCN8\npI/Q3HG+9ATEbBMnbC/2QNy4V0KjIczk5HxIHCA9NqflSiWZI96cLnJlRGdWS2k0\n+VuKIx4HgSRL0cpOFn3te3nPHQNMCv9+4XTO6LjoEBVuxcGSWXynQj69JyPcvNXB\n5mwgOGL9SsSX/PI9tUF7GklXZpQJtPHxcZERfwZhAoGBAMu5mLDVxWT6o0ZQ4bGa\nMDpi94P6rmoHdRuOTjuDh09eSayfAxKK6Hlhp7ZMWSqB1CGh50K2UeT61S5tOBFP\nq/ICEIQuHbrp+ywWT8TICyX95uSlBenAMbrm5QZ6BTJHCFOFhofWEwfMfozgPZwU\n3pq8fYYmg2Vm3O3iCuGfWjo1\n-----END PRIVATE KEY-----\n";

// The ID of the spreadsheet where you'll publish the data
const char spreadsheetId[] = "1tnMCpomUUxd9DjJwiWM8km7IvivF2o9xrALw0bdRbec";

// Token Callback function

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";
 

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

void setup() {
	Serial.begin(9600);
	anotherSerial.begin(19200);

	dht_sensor.begin();

	/* ---- KeyPad  & LCD Display ---- */
	// Wire.begin();
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
	/*---------------------------*/

	pinMode(ULTRA_SONIC_TRIG, OUTPUT);
    pinMode(ULTRA_SONIC_ECHO, INPUT);

	pinMode(SMOKE_DETECTOR, INPUT);

	/* Photoreceptor */
	pinMode(LDRPIN, INPUT);

	pinMode(MOTION_SENSOR, INPUT);

	// Servo
	pinMode(SERVO_PIN, OUTPUT);
	myServo.attach(SERVO_PIN);

	//------Google Sheet Setup------//
    GSheet.printf("ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);
    // Set the callback for Google API access token generation status (for debug only)

    // Set the seconds to refresh the auth token before expire (60 to 3540, default is 300 seconds)
    GSheet.setPrerefreshSeconds(10 * 60);

    // Begin the access token generation for Google API authentication
    GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
    //------Google Sheet Setup------//
}

unsigned long lastTime = 0;
unsigned long timerDelay = 30000;
void saveDataToSheet(){
    // Call ready() repeatedly in loop for authentication checking and processing
    bool ready = GSheet.ready();

    if (ready && millis() - lastTime > timerDelay){
        lastTime = millis();

        FirebaseJson response;

        Serial.println("\nAppend spreadsheet values...");
        Serial.println("----------------------------");

        FirebaseJson valueRange;

        // Get timestamp
        time_t timestamp;
          time(&timestamp);
        timestamp += 7 * 3600;

        valueRange.add("majorDimension", "COLUMNS");
        valueRange.set("values/[0]/[0]", ctime(&timestamp));
        valueRange.set("values/[1]/[0]", humidity);
        valueRange.set("values/[2]/[0]", temp);
        valueRange.set("values/[3]/[0]", heatIndex);
        // For Google Sheet API ref doc, go to https://developers.google.com/sheets/api/reference/rest/v4/spreadsheets.values/append
        // Append values to the spreadsheet
        bool success = GSheet.values.append(&response , spreadsheetId , "Sheet1" , &valueRange);
        if (success){
            response.toString(Serial, true);
            valueRange.clear();
        }
        else{
            Serial.println(GSheet.errorReason());
        }
        Serial.println();
        Serial.println(ESP.getFreeHeap());
    }
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
		Serial.println("Water Dispensed");
	}
	anotherSerial.println("t=" + String(temp)); // Send to gateway board
	delay(100);
	anotherSerial.println("h=" + String(humidity)); // Send to gateway board
	delay(100);
	anotherSerial.println("q=" + String(heatIndex)); // Send to gateway board
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

	Serial.println("Distance:" +  String(distance_cm) + " cm");

	if(distance_cm <= 70) {
		if (!haveNotified) {
			LINE.send("Visitor Detected");
			haveNotified = true;
			anotherSerial.println("d=1"); // Send to gateway board
		}
		Serial.println("life detected");
		
	}
	else {
		Serial.println("zzz...");
		haveNotified = false;
		anotherSerial.println("d=0"); // Send to gateway board

	}
}

int lastDetectedTime = 0;
void startSmokeDetector() {
	/* Smoke Detector */
	int smoke = analogRead(SMOKE_DETECTOR);

	Serial.println("==========================");


	Serial.println("Smoke Density: " + String(smoke));

	if (smoke > 2200) {
		if (millis() - lastDetectedTime > 30 * 1000) {
			Serial.println("SMOKE DETECTED!!!!!");
			anotherSerial.println("s=1"); // Send to gateway board
		}
		lastDetectedTime = millis();
	}
	else {
		Serial.println("SMOKE NOT FOUND");
		anotherSerial.println("s=0"); // Send to gateway board

	}

}
/* For Keypad */
char buf[256];


String input = "";
boolean isEnter;
void startKeyPad(String password) { // กดรหัส ประตูบ้าน 
    char key = keypad.getKey(); 
    if (key != NO_KEY){ 
        if (isEnter || key == 'S') {
            input = "";
            isEnter = 0;
        }
        Serial.println(key);

        if (key != 'N' && key != 'S' && key != 'L' && key != 'R' && key != 'U' && key != 'D') {
            input += key;
            Serial.println(input);
        }
        else if (key == 'N') {
        // lcd.print(input);
            if (input == password ) { //ต้องใช้pin vin
                Serial.println("Door Opened");
                myServo.write(180);//180คืิิอเปิดประตู
                isEnter = 1;
        } 
        else {
            Serial.println("Incorrect Password");
            myServo.write(90);//90คือปิดประตู
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
  	Serial.println("Brightness = " + String(val)); 	

	if (val < 200) { 
		anotherSerial.println("l=1"); // Send to gateway board

	} 
	else {
		anotherSerial.println("l=0"); // Send to gateway board
	}
}

boolean startSensorMotion(){
    int sensorValue = digitalRead(MOTION_SENSOR);

	Serial.println(sensorValue == LOW ? "Motion Detetcted!" : "No Motion");
	Serial.println("==========================");
	anotherSerial.println(sensorValue == LOW ? "m=1" : "m-0");
    return (sensorValue == LOW);
}

int servoPin = 19;  // Pin ที่เชื่อมต่อกับ Servo
void startServo() {
	Serial.println("Door automatically opened.");
	myServo.write(90);  // เปิดประตู
}

unsigned long lastSmokeTime = 0;
unsigned long lastDHTTime = 0;
unsigned long lastPhotoReceptorTime = 0;
unsigned long lastUltraSonicTime = 0;
unsigned long lastKeyPadTime = 0;
unsigned long lastMotionSensorTime = 0;
unsigned long lastSaveDataTime = 0;

void loop() {
    unsigned long currentTime = millis();

    // Smoke Detector 
    if (currentTime - lastSmokeTime >= 5000) {
        lastSmokeTime = currentTime;
        startSmokeDetector();
    }

    // DHT Sensor 
    if (currentTime - lastDHTTime >= 5000) {
        lastDHTTime = currentTime;
        startDHT();
    }

    // PhotoReceptor 
    if (currentTime - lastPhotoReceptorTime >= 5000) {
        lastPhotoReceptorTime = currentTime;
        startPhotoReceptor();
    }

    // Ultrasonic Sensor 
    if (currentTime - lastUltraSonicTime >= 5000) {
        lastUltraSonicTime = currentTime;
        startUltraSonic();
    }

    startKeyPad("12345"); // Door's Password = 12345

    // Motion Sensor
    if (currentTime - lastMotionSensorTime >= 5000) {
        lastMotionSensorTime = currentTime;
        startSensorMotion();
    }

    // Save data to Google Sheet 
    saveDataToSheet();
    
}

