//======1======//
#define trigPin1 2
#define echoPin1 3
//======2======//
#define trigPin2 4
#define echoPin2 5
//======3======//
#define trigPin3 7
#define echoPin3 8
//======4======//
#define trigPin4 9
#define echoPin4 10


#define l1 11
#define l2 12
#define l3 13
#define r1 A1
#define r2 A2
#define r3 A3
#define f1 A0
#define f2 A4
#define b1 A5
#define b2 6

long duration, distance, RightSensor, BackSensor, FrontSensor, LeftSensor;

void setup()
{
  Serial.begin (9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(l3, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  digitalWrite(l1, LOW);
  digitalWrite(l2, LOW);
  digitalWrite(l3, LOW);
  digitalWrite(r1, LOW);
  digitalWrite(r2, LOW);
  digitalWrite(r3, LOW);
}


void loop() {
  SonarSensor(trigPin1, echoPin1);
  RightSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  LeftSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  FrontSensor = distance;
  SonarSensor(trigPin4, echoPin4);
  BackSensor = distance;

  Serial.print(RightSensor);
  Serial.print(" - ");
  Serial.print(LeftSensor);
  Serial.print(" - ");
  Serial.print(FrontSensor);
  Serial.print(" - ");
  Serial.println(BackSensor);

  led ();

}

void led() {
  if (RightSensor <= 10) {
    digitalWrite(r1, LOW);
    digitalWrite(r2, LOW);
    digitalWrite(r3, LOW);
  }
  else if (RightSensor <= 50) {

    digitalWrite(r1, HIGH);
    digitalWrite(r2, LOW);
    digitalWrite(r3, LOW);
  } else if (RightSensor <= 100) {

    digitalWrite(r1, HIGH);
    digitalWrite(r2, HIGH);
    digitalWrite(r3, LOW);
  } else {

    digitalWrite(r1, HIGH);
    digitalWrite(r2, HIGH);
    digitalWrite(r3, HIGH);
  }
  if (LeftSensor <= 10) {
    digitalWrite(r1, LOW);
    digitalWrite(r2, LOW);
    digitalWrite(r3, LOW);
  }
  else if (LeftSensor <= 50) {
    digitalWrite(l1, HIGH);
    digitalWrite(l2, LOW);
    digitalWrite(l3, LOW);

  } else if (LeftSensor <= 100) {
    digitalWrite(l1, HIGH);
    digitalWrite(l2, HIGH);
    digitalWrite(l3, LOW);
  } else {
    digitalWrite(l1, HIGH);
    digitalWrite(l2, HIGH);
    digitalWrite(l3, HIGH);
  }
  if (FrontSensor <= 20) {
    digitalWrite(f1, LOW);
    digitalWrite(f2, LOW);
  } else if (FrontSensor <= 70) {
    digitalWrite(f1, HIGH);
    digitalWrite(f2, LOW);
  } else {
    digitalWrite(f1, HIGH);
    digitalWrite(f2, HIGH);
  }
  if (BackSensor <= 20) {
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
  }
  else if (BackSensor <= 70)
  {
    digitalWrite(b2, HIGH);
    digitalWrite(b2, LOW);
  }
  else {
    digitalWrite(b2, HIGH);
    digitalWrite(b2, HIGH);
  }
}
void SonarSensor(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.034 / 2);
  if (distance >= 145) {
    distance = 145;
  }

}
