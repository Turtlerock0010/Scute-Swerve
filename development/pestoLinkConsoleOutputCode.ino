#include <PestoLink-Receive.h>

float myFLoat = 3.14159

void setup() {
  Serial.begin(115200);
  PestoLink.begin("Navigator");
}

void loop() {
  // Creates a set char string
  char outputString[20];

  // Converts the float into a string
  dtostrf(myFloat, 8, 2, outputString);

  // Sends the data to the terminal
  PestoLink.printTerminal(outputString);
}
