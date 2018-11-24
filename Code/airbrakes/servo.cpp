int servo0 = 1000; // position 0 is set when we send a high signel in 1000 microseconds(1ms)
int servo180 = 2000; // position 180 is set when we send a high in 2000 microseconds(2ms)
int inc = 20; // increment/decrement signal length with 20 microseconds per step
int pos = servo0; 
int servoPin = 9;
int pulseIntervalMs=20;

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  pinMode(servoPin, OUTPUT);
}

void loop() {
  
  pos += inc;
  if (pos > servo180) {
    Serial.println("REVERSE!");
    pos = servo180;
    inc *= -1;
    delay(500);
  } else if (pos < servo0) {
    Serial.println("FORWARD!");
    pos = servo0;
    inc *= -1;
    delay(500);
  }

  Serial.print("pos = ");
  Serial.println(pos);

  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pos);
  digitalWrite(servoPin, LOW);
  delay(20);
}