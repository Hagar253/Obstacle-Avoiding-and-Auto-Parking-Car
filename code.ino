#include <NewPing.h> // library to use ultrasonic more efficient 
#include <Servo.h> // library for servo motor
#include <Ultrasonic.h>
Servo myservo;// create an object from servo class

// pins of L298n motor driver
#define IN1 9 // backward of right motor
#define IN2 8 // forward of right motor
#define IN3 7 // backward of left motor
#define IN4 6 // forward of left motor

// pins of front ultrasonic
#define trig_front 12
#define echo_front 11

// pins of back ultrasonic
#define trig_back 3
#define echo_back 2

// pins of right ultrasonic
#define trig_right A0
#define echo_right A1

int park_status = 0;// initialize parking status by 0 as start
int distance = 50; // initialize the distance 

#define minimum_limit 15 // Width of the car (cm)
#define minimum_limit1 28 // Length of the car (cm)

// initialize led and button
#define LED  13
#define button  A2

bool ledstate = LOW;
bool buttonstate = LOW;
bool pressed = false;

// initilaize ultrasonic device 
NewPing sonar (trig_front,echo_front);
Ultrasonic  ultrasonic_back(trig_back, echo_back), ultrasonic_side(trig_right, echo_right), ultrasonic_front(trig_front, echo_front);

void setup() {
  myservo.attach(5); // initilaize the pin that servo is attached to
  // initilaize the pins of motors as output 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);

  // initilaize the pins of front ultrasonic
  pinMode(trig_front, OUTPUT); 
  pinMode(echo_front, INPUT);

  //  initilaize the pins of back ultrasonic
  pinMode(trig_back, OUTPUT); 
  pinMode(echo_back, INPUT); 

  // initilaize the pins of right ultrasonic
  pinMode(trig_right, OUTPUT); 
  pinMode(echo_right, INPUT); 

  // initilaize the start angle of servo
  myservo.write(90);

  // set the pins 
  pinMode(button, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
}



void loop() {
  digitalWrite(LED, ledstate);
  buttonstate = digitalRead(button);  //will read HIGH because its pull up circuit
  
  if (buttonstate == LOW) {
    if (pressed == false) {   
      ledstate = !ledstate;             
      pressed = true;         
    }
  } else { //btn == high ya3ny m4 daysa
    pressed = false;

  } 

  if(ledstate == LOW){
    obstacle_avoid();
  }else{
    myservo.write(90);
    Find_Park();
  }
}



// Parking spot search
int Check_Parking_Spot()
{
    long front_Sensor = ultrasonic_front.Ranging(CM);
    long side_Sensor = ultrasonic_side.Ranging(CM);

    // side<=15||park=0 >>> forward||park status=1
    if ((side_Sensor <= minimum_limit) && (park_status == 0))
    {
      // Detected Potential Parking Spot (Initial Condition)
        forward();
        park_status = 1;
    }
    // 28>side>15||park=1 >>> forward||park=2
    if ((side_Sensor > minimum_limit) && (side_Sensor < minimum_limit1) && (park_status == 1))
    {
      // Confirmed Parking Spot Dimensions
        forward();
        park_status = 2;
    }
    //side<=15||park=2 >>> park=3
    if ((side_Sensor <= minimum_limit) && (park_status == 2))
    {
        // Decide to park parallel
        park_status = 3;
    }

    return park_status;
}

void Find_Park()
{
   Check_Parking_Spot();
    if (park_status == 3)
    {
        stopp();
        delay(400);
        park_status = 4;
    }
    if (park_status == 4)
    {
      // Parallel Parking - First Step
        backward();
        delay(200);
        stopp();
        
        right();
        delay(400);
        stopp();
        
        park_status = 5;
    }

    if (park_status == 5)
    {
      // Parallel Parking - Adjust Position
      // turning the wheels to align with the spot.
        long back_Sensor = ultrasonic_back.Ranging(CM);
        backward();
        delay(100);

        if (back_Sensor > 7 && back_Sensor <= 15)
        {
            stopp();
            delay(200);
            park_status = 6;
            
        }
        // return back_Sensor;
    }

    if (park_status == 6)
    {
      // Parallel Parking - Final Alignment
        left();
        delay(200);
        long right_Sensor = ultrasonic_side.Ranging(CM);
        long back_Sensor = ultrasonic_back.Ranging(CM);

        if(right_Sensor <= 8 && back_Sensor <= 9){
          stopp();
          park_status = 7;
        }
        // return right_Sensor;
    }
    if (park_status == 7)
    {
      // Final Check and Adjustments
        long front_Sensor = ultrasonic_front.Ranging(CM);

        if (front_Sensor <= 7)
        {
            stopp();
            park_status = 8; // Parked
        }
        else
        {
            forward();
            delay(100);
        }
        // return front_Sensor;
    }
    if(park_status == 8){
      stopp();
    }
}

// this function is for obstacle avoidance
void obstacle_avoid(){
  int dr = 0;//initilaize right distance as 0
  int dl = 0;// initilaize left distance as 0
  delay(50);
  
  // if the distance is less than 20
  if (distance <= 20){
    stopp();
    delay(300);
    backward();
    delay(400);
    stopp();
    delay(300);
    dr = lookRight();// start calculate right distance
    delay(300);
    dl = lookLeft();// start calculate left distance
    delay(300);
  // if the right distance is greater that left distance
  // then go to the right
     if (dr >= dl){
        right();
      }
     // else go to the left
     else{
        left();
      }
  }
  // if the distance more than 20 then go forward
  else{
    forward();
  }
  // calculate distance 
    distance = readPing();
}




// calculate distance of left side
int lookLeft(){ 
  myservo.write(30);// he will set the angle of servo 50 to start reading
  delay(500);
  int distance = readPing();// start reading 
  delay(100);
  myservo.write(90);// return back to the set position
  return distance;
}

// calculate distance of right side
int lookRight(){
  myservo.write(150); // he will set the angle of servo 170 to start reading
  delay(500);
  int distance = readPing();// start reading
  delay(100);
  myservo.write(90);// return back to the set position
  return distance;
  delay(100);
}

// get the reading of ultrasonic 
int readPing(){
  delay(70);
  int cm = sonar.ping_cm();// send ping and get the distance in cm
  if (cm==0){// if it is close to the ultrasonic
    cm=250;// set as 250 to avoid large numbers that ulrasonic produce
  }
  return cm;
}

// function to stop 
void stopp(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW); 
}

// function to move forward
void forward(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);
}

// function to move backward
void backward(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 
}

// function to turn right
void right(){

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 

  delay(500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 
}

// function to turn left
void left()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW); 

  delay(500);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW); 

}
