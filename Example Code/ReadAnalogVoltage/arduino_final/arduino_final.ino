/*void setup() 
{
  Serial.begin(115200);
  while (!Serial); // optionally wait for serial terminal to open
    Serial.println("MyoWare Example_01_analogRead_TWO_SENSORS");
}

void loop() 
{
  int sensorValue1 = analogRead(A0); // read the input on analog pin A0 (Sensor 1)
  int sensorValue2 = analogRead(A1); // read the input on analog pin A1 (Sensor 2)

  unsigned long currentTime = millis(); // Get the current time in milliseconds
  
  // Print the timestamp
  Serial.print("Timestamp: ");
  Serial.println(currentTime);
  
  // Print the value for Sensor 1 (A0)
  Serial.print("Sensor 1 (A0) Value: ");
  Serial.println(sensorValue1); 
  
  // Print the value for Sensor 2 (A1)
  Serial.print("Sensor 2 (A1) Value: ");
  Serial.println(sensorValue2);
  
  // Add a delay to avoid overloading the serial terminal
  delay(50);
}*/


/**#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address and IP address for your controller
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);  // Set a unique IP address for your device

const int emgPin = A0;  // Pin connected to EMG sensor
char server[] = "script.google.com";  // Google Script URL

EthernetClient client;

void setup() {
  Serial.begin(9600);
  
  // Start the Ethernet connection
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);  // Use static IP if DHCP fails
  }

  delay(1000);  // Wait for the Ethernet to initialize
  Serial.println("Connecting to Google Script...");
}

void loop() {
  int emgValue = analogRead(emgPin);  // Read EMG value

  // Make a connection to the Google Script server
  if (client.connect(server, 80)) {
    client.print("GET /macros/s/1r7gfLJv7RCnDTgFa5BmKKPseXvr83M8eztW-bN0Z3N8/exec?value=");
    client.print(emgValue);
    client.println(" HTTP/1.1");
    client.println("Host: script.google.com");
    client.println("Connection: close");
    client.println();
    client.stop();  // Disconnect after sending data

    Serial.println("Data sent to Google Sheets");
  } else {
    Serial.println("Connection failed");
  }

  delay(10000);  // Wait before sending the next data point
}*/

/**void setup() {
  Serial.begin(115200);
}

void loop() {
  float time = micros() / 1e6;
  int sensorValue1 = analogRead(A0);
  int sensorValue2 = analogRead(A1);
  delay(100);

  Serial.print(time);
  Serial.println(", ");
  Serial.print(sensorValue1);
  Serial.print(", ");
  Serial.println(sensorValue2);

}*/



// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  double sensorValue1 = analogRead(A0);
  double sensorValue2 = analogRead(A1);
  double sensorValue3 = analogRead(A2);
  double sensorValue4 = analogRead(A3);

  //int sensorValue2 = analogRead(A2);
  //int sensorValue2 = analogRead(A3);
  // print out the value you read:
  Serial.print(sensorValue1);
  Serial.print(" ");
  Serial.print(sensorValue2);
  Serial.print(" ");
  Serial.print(sensorValue3);
  Serial.print(" ");
  Serial.println(sensorValue4);
  //Serial.print(sensorValue3);
  //Serial.print(" ");
  //Serial.println(sensorValue4);
  delay(100);        // delay in between reads for stability
}