#define FLEX_PIN 34  // ADC pin

void setup() {
  Serial.begin(115200);
  pinMode(FLEX_PIN, INPUT);
}

void loop() {
  int flexValue = analogRead(FLEX_PIN);  // Read 0-4095
  
  Serial.print("Flex Sensor: ");
  Serial.println(flexValue);
  
  delay(100);
}