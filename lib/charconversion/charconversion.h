#include <Arduino.h>

class ValueConverter {
  public:
    // Convert float to char array
    void floatToChar(float value, char* payload) {
      dtostrf(value, 6, 2, payload);
    }

    // Convert int to char array
    void intToChar(int value, char* payload) {
      itoa(value, payload, 10);
    }
};

/*
Use this as follows

ValueConverter converter;
char payload[50];

float floatValue = 123.45;
converter.floatToChar(floatValue, payload);
client.publish("topic", payload);

int intValue = 123;
converter.intToChar(intValue, payload);
client.publish("topic", payload);
*/

/*
Die Größe des Char-Arrays = erwarteten Länge der konvertierten Zahlen  
einschließlich des abschließenden Nullzeichens.
float max 50
int max 12
*/