#define BLYNK_TEMPLATE_ID           "TMPL61lOG-kSF"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "foSfONd_gnxdxbSVeqfF1mIl_n5rxAGR"
#define LINE_TOKEN                  "DJTlQcoA6UTw7mkog4TQc43GWsnweROoGCh8VhtoFMF" // Do not DELETE

#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>
#include <ArtronShop_LineNotify.h>

//Pins constant
int TxPin = 22;
int RxPin = 23;
int nightLight = 5;
int sprayLight = 18; //TODO: change pin?

//Variables
char ssid[] = "Dami 14";
char password[] = "BigMi1414";
SoftwareSerial anotherSerial(RxPin, TxPin);
BlynkTimer timer;

bool haveVisitor = false;
bool haveNotified = false;

float humidity = 0;
float temperature = 0;
float heatIndex = 0;
bool allowAutoSprayer = false;

bool isNight = false;

bool allowSaveLight = true;
unsigned long lastMotionTime = 0;

unsigned long lastOnFireTime = 0;
bool onFire = false;

const int freq = 5000;
const byte ledPhoto = 0;
const byte resolution = 8;

void updateBlynk();
void checkHeat();
void highLight();
void autoLight(bool);
void openSpray(bool);
void notifyVisitor(bool);
void fireControlMeasure(bool);
void processCommand(String);

BLYNK_CONNECTED() {
  Serial.println("Blynk connection success");
}

BLYNK_WRITE(V5) {
  allowSaveLight = param.asInt();
}

BLYNK_WRITE(V6) {
  digitalWrite(sprayLight, param.asInt());
  allowAutoSprayer = false;
  Blynk.virtualWrite(V8, allowAutoSprayer);
}

BLYNK_WRITE(V8) {
  allowAutoSprayer = param.asInt();
}

void setup() {
  Serial.begin(9600);
  anotherSerial.begin(19200);

  /*****************WI-FI*****************/
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting To WiFi Network .");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected To The WiFi Network");

  /*****************Line******************/
  LINE.begin(LINE_TOKEN);
  if (LINE.send("Test Nofication")) { 
    Serial.println("Send notify successful");
  } 
  else {
    Serial.printf("Send notify fail. check your token (code: %d)\n", LINE.status_code);
  }

  /*****************Blynk*****************/
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  timer.setInterval(10000L, updateBlynk);


  pinMode(nightLight, OUTPUT);
  pinMode(sprayLight, OUTPUT);

  ledcSetup(ledPhoto, freq, resolution);
  ledcAttachPin(nightLight, ledPhoto);
}

void loop() {
  Blynk.run();
  timer.run();

  //check serial input
  if (anotherSerial.available() > 0) {
    String sensorOutput = anotherSerial.readStringUntil('\n');
    processCommand(sensorOutput);
  }

  //high light timer
  if(allowSaveLight && isNight) {
    if(millis() - lastMotionTime > 10000) ledcWrite(ledPhoto, 150);  //TODO: Medium
  }
}

void processCommand(String command) { //command format : "d=1" or "t=26"
  char sensorId = command[0];
  float value = (command.substring(command.indexOf('=')+1, command.length())).toFloat();
  Serial.print(sensorId);
  Serial.print('-');
  Serial.println(value);

  switch(sensorId) {
    case 'd': //distance
      notifyVisitor(value);
      break;
    case 'h': //humidity
      humidity = value;
      break;
    case 't' : //temperature
      temperature = value;
      break;
    case 'q' : //heat index
      heatIndex = value;
      checkHeat();
      break;
    case 'l' : //light
      autoLight(value);
      break;
    case 'm' : //motion
      if(value) highLight();
      break;
    case 's' : //smoke
      fireControlMeasure(value);
      break;
  }
}

void autoLight(bool isDark) {
  isNight = isDark;
  if(isNight) {
    if(!allowSaveLight) ledcWrite(ledPhoto, 255);
    else ledcWrite(ledPhoto, 150);  //TODO: Medium
  }
  else ledcWrite(ledPhoto, 0);;
}

void highLight() {
  if(allowSaveLight && isNight) {
    ledcWrite(ledPhoto, 255);
    lastMotionTime = millis();
  }
}

void checkHeat() {
  openSpray((heatIndex > 25));
}

void openSpray(bool open) {
  digitalWrite(sprayLight, open);
}

void notifyVisitor(bool detected) {
  if(detected) {
		if (!haveNotified) {
			LINE.send("Visitor Detected");
			haveNotified = true;
		}
    haveVisitor = true;
		Serial.println("life detected");
	}
	else {
		Serial.println("zzz...");
		haveNotified = false;
    haveVisitor = false;
  }
}

void fireControlMeasure(bool isBurning) {
  if(isBurning) {
    if (millis() - lastOnFireTime > 30 * 1000) {
			LINE.send("Smoke Detected");
      lastOnFireTime = millis();
		}
  }
}

void updateBlynk() {
  Serial.println("update blynk");
  // Serial.print("temp : ");
  // Serial.println(temperature);
  // Serial.print("humidity : ");
  // Serial.println(humidity);
  Blynk.virtualWrite(V7, temperature);
  Blynk.virtualWrite(V0, humidity);
  Blynk.virtualWrite(V4, onFire);
}