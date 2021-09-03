int bluePower = 7;    //to supply power to the bluetooth
double x;   //To store the x coordinate received from python
double y;   //To storese the Y coordinate received from python
double p;
double q;
double a;   //To store the Robot (x) coordinates from python
double b;   //To store the Robot's(y) coordinates from python 
double u;
double v;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(bluePower, OUTPUT);  //prevents the bluetooth from obstructing info transit before code upload
  digitalWrite(bluePower, HIGH);  //gives power to the bluetooth after code upload
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  char input = Serial.read(); //reads the information received on the serial monitor from python
  
  switch (input) {      //uses input for the switch case statement
    case ('x'):     //indicates the start of the X coordinate from python
      x=Serial.parseFloat();  //assigns the first float value to x before the next character.
      break;
    case ('y'):     //indicates the start of the Y coordinates read from python
      y = Serial.parseFloat();  //passes the next float value to y
      break;
    case ('p'):     //indicates the start of the p coordinates read from python
      p = Serial.parseFloat();  //passes the next float value to b
      break;
    case ('q'):     //indicates the start of the q coordinates read from python
      q = Serial.parseFloat();  //passes the next float value to b
      break;
    case ('a'):     //indicates the start of the a coordinates read from python
      a = Serial.parseFloat();  //passes the next float value to a
      break;
    case ('b'):     //indicates the start of the b coordinates read from python
      b = Serial.parseFloat();  //passes the next float value to b
      break;
    case ('u'):
      u = Serial.parseFloat(); 
      break;
    case ('v'):
      v = Serial.parseFloat(); 
      break;
  }

  //To print on the python console instead of the arduino serial port
   Serial.print("Yellow"); Serial.print(" "); Serial.print(x); Serial.print(" "); Serial.print(y);    
   Serial.print(" ");   //for format purposes
   Serial.print("Blue"); Serial.print(" "); Serial.print(p); Serial.print(" "); Serial.print(q);
   Serial.print(" ");
   Serial.print("Robo"); Serial.print(" ");Serial.print(a); Serial.print(" "); Serial.println(b);    //prints the y coordinate read from python back to python



  
}
