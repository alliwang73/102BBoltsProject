
#define IN1 15
#define IN2 32
#define ENA 14

// PWM settings
const int SPEED = 210;  // 75% of 255

void stopMotor() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void forward() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, SPEED);
}

void reverse() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, SPEED);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  stopMotor();

  Serial.println("Motor ready!");
  Serial.println("F = Forward");
  Serial.println("R = Reverse");
  Serial.println("S = Stop");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'f' || command == 'F') {
      forward();
      Serial.println("Forward at 75%");
    }

    else if (command == 'r' || command == 'R') {
      reverse();
      Serial.println("Reverse at 75%");
    }

    else if (command == 's' || command == 'S') {
      stopMotor();
      Serial.println("Stopped");
    }
  }
}
