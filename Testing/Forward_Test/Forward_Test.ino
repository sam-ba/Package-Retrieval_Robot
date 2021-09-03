//motor controls
//to control left motor direction and speed
const int IN1 = 6;  
const int IN2 = 7;
const int ENA = 3;
//to control right motor direction and speed
const int IN3 = 4;
const int IN4 = 5;
const int ENB = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");
  
  pinMode(ENA,OUTPUT);  // the pin that controls the A channel
  pinMode(ENB,OUTPUT);  // the pin that controls the A channel
  pinMode(IN1,OUTPUT);  // the pin that controls motor 1 is set to output 
  pinMode(IN2,OUTPUT);  // the pin that controls motor 2 is set to output
  pinMode(IN3,OUTPUT);  // the pin that controls motor 3 is set to output
  pinMode(IN4,OUTPUT);  // the pin that controls motor 4 is set to output
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Start");
  forward();
  delay(10000);
  
//  Serial.println("Turning Left");
//  turn90Left(614);
//  Serial.println("Turning Right");
//  turn90Right(601.8);
}

void forward(){ 
  digitalWrite(IN1,LOW); //set IN1 high level
  digitalWrite(IN2,HIGH);  //set IN2 low level
  digitalWrite(IN3, HIGH);  //set IN3 low level
  digitalWrite(IN4,LOW); //set IN4 high level
  Serial.println("Forward");//send message to serial monitor
  analogWrite(ENA,168); //enable L298n A channel
  analogWrite(ENB,210); //enable L298n B channel
}

void backward(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  Serial.println("Backward");
  analogWrite(ENA,195); //enable L298n A channel
  analogWrite(ENB,200); //enable L298n B channel
}

void testConn(){
  // This function is used to test which motor is on ENA and ENB
  analogWrite(ENA, 0); // disable the enable A channel
  analogWrite(ENB, 200); // enable the enable B channel
  digitalWrite(IN1,LOW); //set IN1 high level
  digitalWrite(IN2,HIGH);  //set IN2 low level
  digitalWrite(IN3, HIGH);  //set IN3 low level
  digitalWrite(IN4,LOW); //set IN4 high level
}

void stopp(){
  analogWrite(ENA,0); //enable L298n A channel
  analogWrite(ENB,0); //enable L298n B channel
}

void turn90Right(int dt){
  analogWrite(ENA, 195);
  analogWrite(ENB, 200);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(dt); 
 }

void turn90Left(int dt){
  analogWrite(ENA, 195);
  analogWrite(ENB, 200);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(dt); 
 }

void turn180(int dt){
  analogWrite(ENA, 195);
  analogWrite(ENB, 200);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(dt);
 }

//void openGrip(int dt){
//  pos = 0.0;
//  myServo.write(pos);
//  delay(dt);
//  return 0 ;
//}
//
//void closeGrip(int dt){
//  pos = 80;
//  myServo.write(pos);
//  delay(dt);
//  return 0;
//}
