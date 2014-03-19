
int dirPin = 11;
int stepperPin = 12;
int laserDPin = 10;
int laserGPin = 13;

// a variable to read incoming serial data into
int incomingByte;

void setup() {
    // initialize serial communication:
    Serial.begin(9600);
    // Init pin
    pinMode(dirPin, OUTPUT);
    pinMode(stepperPin, OUTPUT);
    pinMode(laserDPin, OUTPUT);
    pinMode(laserGPin, OUTPUT);
}

void step(boolean dir, int steps){
  digitalWrite(dirPin,dir);
  delay(50);
  for(int i=0; i<steps; i++){
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(100);
  }
}

void loop() {
  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    Serial.println(incomingByte);

    // if it's a capital H (ASCII 72)
    if (incomingByte == 'H') {
        step(true, 8);
        delay(50);
    }

    // if it's an L (ASCII 76)
    if (incomingByte == 'L') {
        step(false, 8);
        delay(50);
    }

    // if it's an D
    if (incomingByte == 'D') {
        digitalWrite(laserDPin, HIGH);
    }
    // if it's an F
    if (incomingByte == 'C') {
        digitalWrite(laserDPin, LOW);
    }

    // if it's an G
    if (incomingByte == 'G') {
        digitalWrite(laserGPin, HIGH);
    }
    // if it's an B
    if (incomingByte == 'B') {
        digitalWrite(laserGPin, LOW);
    }

  }
}
