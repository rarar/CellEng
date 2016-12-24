void setup() {                
  pinMode(2, OUTPUT);     
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
}

void loop() {
  digitalWrite(3, HIGH);
  delay(5);          
  digitalWrite(3, LOW); 
  delay(5);          
}
