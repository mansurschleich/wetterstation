#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <hp_BH1750.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <charconversion.h>

// Definition for I2C pins
#define I2C_SDA 1 
#define I2C_SCL 2 
// Aufrufe Klassen von Sensoren
Adafruit_BMP280 bmp;  // I2C
Adafruit_Sensor* bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor* bmp_pressure = bmp.getPressureSensor();
Adafruit_AHTX0 aht;
hp_BH1750 BH1750;
sensors_event_t humidity, temp, temp_event, pressure_event;
float lux;

// WIFI VARIABLES
const char* ssid = "IOT_123";
const char* password = "iotschuni";
WiFiServer server(80);
WiFiClient espClient;
// Variable to store the HTTP request
String header;
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

//MQTT
//in Putty  mosquitto_sub -h 192.168.1.123  -t "#" -v
PubSubClient client(espClient);
const char* mqtt_server = "192.168.1.123";
ValueConverter converter;
// Delay für Publish
unsigned long mqttstartzeit = 0;  //Deklaration Startzeit für Mqtt
#define MQTTINTERVALL 5000        // Konstante für Mqtt Intervall in ms
//Topics
#define tempbmp_out "/NS/TempBMP"
#define humaht_out "/NS/HumAHT"
#define luxbh_out "/NS/LuxBH"
#define presbmp_out "/NS/PresBMP"
#define tempaht_out "/NS/TempAHT"
#define inTopic "/NS/#"


// DEBUG
unsigned long debugstartzeit = 0;  //Deklaration Startzeit für Debug
#define DEBUGINTERVALL 5000        // Konstante für Debug Intervall (Faktor 10)

//MQTT Funcitons
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
 }

// SETUP
void setup() {
  Serial.begin(115200);
  //WIFI
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  //MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //I2C
  Wire.setPins(I2C_SDA, I2C_SCL);
  //BMP280
  while (!Serial) delay(100);  // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp->printSensorDetails();
  // AHT20
  Serial.println("Adafruit AHT10/AHT20 demo!");
  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  //BH1750
  bool avail = BH1750.begin(BH1750_TO_GROUND);
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};
  }
}


// Function for Website
void wifi() {
  WiFiClient client = server.available();  // Listen for incoming clients

  if (client) {  // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");                                             // print a message out in the serial port
    String currentLine = "";                                                   // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {  // if there's bytes to read from the client,
        char c = client.read();  // read a byte, then
        Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<meta http-equiv=\"refresh\" content=\"20\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>METEO</h1>");

            // Display sensor values
            client.println("<p>Temperature BMP280: " + String(temp_event.temperature) + " *C</p>");
            client.println("<p>Pressure BMP280: " + String(pressure_event.pressure) + " hPa</p>");
            client.println("<p>Temperature AHT20: " + String(temp.temperature) + " *C</p>");
            client.println("<p>Humidity AHT20: " + String(humidity.relative_humidity) + " %rH</p>");
            client.println("<p>Lux BH1750: " + String(lux) + " lx</p>");

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

// Function for MQTT
void mqtt(){
  //MQTT
  char payload_float[50]; //payload with data only used for conversion
  
  if (millis() - mqttstartzeit > MQTTINTERVALL) {  //Time delay for publish to minimise workload
    mqttstartzeit = millis();  

    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    // Temparatur BMP
    converter.floatToChar(temp_event.temperature, payload_float); // Convert flaot to Char for MQTT
    client.publish(tempbmp_out,payload_float); // Publish command for mqtt
    // Humidity AHT
    converter.floatToChar(humidity.relative_humidity, payload_float);
    client.publish(humaht_out,payload_float);
    // LUX BH1750
    converter.floatToChar(lux, payload_float);
    client.publish(luxbh_out,payload_float);
    // Pressure BMP
    converter.floatToChar(pressure_event.pressure, payload_float);
    client.publish(presbmp_out,payload_float);
    // Temparatur AHT
    converter.floatToChar(temp.temperature, payload_float);
    client.publish(tempaht_out,payload_float);
  }
}

// Loop Function 
void loop() {
  wifi();
  mqtt();
  //debug();  //uncomment for Serial output Debug Function

  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  aht.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  BH1750.start();
  lux = BH1750.getLux();

}




// Function for Debug (Serial Print)
void debug() {
  //DEBUG
  if (millis() - debugstartzeit > DEBUGINTERVALL) {  //Zeitabgelaufen
    debugstartzeit = millis();                       //Startzeit aktualisiert

    //Action
    Serial.print(F("Temperature BMP280: "));
    Serial.print(temp_event.temperature);
    Serial.println(" °C");
    Serial.print(F("Pressure BMP280: "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");

    Serial.print("Temperature AHT20: ");
    Serial.print(temp.temperature);
    Serial.println(" °C");
    Serial.print("Humidity AHT20: ");
    Serial.print(humidity.relative_humidity);
    Serial.println(" % rH");

    Serial.print("Lux BH1750: ");
    Serial.print(lux);
    Serial.println(" lx");
    Serial.println("...");
  }
}