const int RELAY_PIN = A5;

void start_fan(){
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);  // used for indication
}

void stop_fan(){
  digitalWrite(RELAY_PIN, LOW); 
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  start_fan();
  delay(5000);
  stop_fan();
  delay(5000);
}