void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    Serial.println(Serial.readString());
  }
}
