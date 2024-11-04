
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
  //delay(100);        // delay in between reads for stability
}