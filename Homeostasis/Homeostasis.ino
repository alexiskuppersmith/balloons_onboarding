int heaterControlPin = 23; //TODO: make sure this doesn't collide with anything else

void setup() {
  Serial.begin(9600);
  pinMode(heaterControlPin, OUTPUT);
}

void loop() {
  Serial.println("Temp = 10C...");
  manageHeaters(10);
  delay(5000);
  Serial.println("Temp = 0C...");
  manageHeaters(0);
  delay(5000);
  Serial.println("Temp = -5C...");
  manageHeaters(-5);
  delay(5000);
  Serial.println("Temp = -15C...");
  manageHeaters(-15);
  delay(5000);
}

/* Expects current temperature in Celsius,
 * turns on heaters if temp <= 0C, using proportional control to turn on heaters more
 * if the current temperature is further from 0C (maximum acheived at -10C).
*/
void manageHeaters(double currentTemp) {
  if (currentTemp < 0) {
    double p = min(1, max(0, currentTemp / -10));
    Serial.print("p=");
    Serial.print(p);
    Serial.println();
    analogWrite(heaterControlPin, 255. * p);
  }
}

