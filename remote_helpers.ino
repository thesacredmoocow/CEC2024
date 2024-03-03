#define X_PIN A2
#define Y_PIN A3
#define Y2_PIN 5

#define UP_PIN A0
#define DOWN_PIN A1

unsigned long xvalue, yvalue, y2value;

bool up = false, down = false;

void remote_setup() {
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);

  pinMode(UP_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);
  pinMode(Y2_PIN, INPUT);
}


void remote_run() {
  xvalue = pulseIn(X_PIN, HIGH, 21000);
  yvalue = pulseIn(Y_PIN, HIGH, 21000);
  y2value = pulseIn(Y2_PIN, HIGH, 21000);
  if(xvalue == 0) {
    xvalue = 1000;
  }
  if(yvalue == 0) {
    yvalue = 1000;
  }
  if(y2value == 0) {
    y2value = 1000;
  }
  up = digitalRead(UP_PIN);
  down = digitalRead(DOWN_PIN);
  Serial.println(y2value);
  
}
