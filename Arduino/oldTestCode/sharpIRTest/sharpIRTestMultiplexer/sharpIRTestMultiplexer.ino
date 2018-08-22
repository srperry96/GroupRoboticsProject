void setup() {
  Serial.begin(9600);
}

int s0 = 3;
int s1 = 4;
int s2 = 5;
float val;

void loop() {
  digitalWrite(s0, 1);
  digitalWrite(s1, 1);
  digitalWrite(s2, 0);

  val = analogRead(0) * 0.0048828125;
  val = 13 * pow(val, -1);

  Serial.print("Select 0: ");
  Serial.println(val);

  delay(500);
/*
  digitalWrite(s0, 1);
  digitalWrite(s1, 1);
  digitalWrite(s2, 0);

  val = analogRead(0) * 0.0048828125;
  val = 13 * pow(val, -1);

  Serial.print("Select 3: ");
  Serial.println(val);

  delay(400);*/
}
