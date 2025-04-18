/* 
Position Control on Science Fun Youtube Video
https://www.youtube.com/watch?v=c7JtfogOsTQ&t=307s

PID Controller Based on Ian Carey Youtube video
https://www.youtube.com/watch?v=RZW1PsfgVEI


Ziegler-Nichols Method For tuning PID Explained In This Short Video
https://www.youtube.com/watch?v=dTZnZZ4ZT7I

When Serial Data is in CSV format. Collumn Order Actual ADC, Target ADC, Time
*/
const int Extend_pin = 11;     // connect to motor control board IN2
const int Retract_pin = 10;     // connect to motor control board IN1
const int Position_pin = A0;   // Potentiometer Position
const int Speed_pin = 9;       // PWM pin to set speed. Coneect to motor control baord pin ENA

float Desired_location;          // Distance of desired location
int Desired_POT;               // Potentiometer Value at desired location
float Home_location;             // Distance of home location
int Home_POT;                  // Potentiometer Value at home location
int headingTo;                // Variable assigned the desired or home pot values


int Motor_PWM;                 // Set Motor Speed
int min_PWM;                   // Experimentally determined minimun PWM motr will move at
int deadband = 20;             // Prevents Jittering at accepetably close final position. Smaller Value is more accurate but more jittery
int pot_Max_ADC = 1023;        // Potentiometre ADC value at Full Extension
int pot_Min_ADC = 0;           // Potentiometer ADC Value at Full Retraction


// Timing
unsigned long nextTime;
int timeExtend = 3000;           // milliseconds
int timeRetract = 3000;          // milliseconds

// PID Control
double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;


void setup() {
  pinMode(Extend_pin, OUTPUT);
  pinMode(Retract_pin, OUTPUT);
  pinMode(Speed_pin, OUTPUT);
  // Start with Actuator Motor Off In Both Directions
  digitalWrite(Extend_pin, LOW);    
  digitalWrite(Retract_pin, LOW);

  // Set Desired and Home Locations and Potentiometer values
  Desired_location = 1.10;                                                                          // inches
  Desired_location = constrain(Desired_location, 0, 2);                                             // only accept values from 0 to 2 inches
  Desired_POT = map(Desired_location*1000, 0*1000 , 2*1000, pot_Min_ADC, pot_Max_ADC);              // map distance to POT values. 1000 multiplier converts floats into int for map()
  Home_location = 0.75;                                                                              // inches
  Home_location = constrain(Home_location, 0, 2);                                                   // only accept values from 0 to 2 inches
  Home_POT = map(Home_location*1000, 0*1000 , 2*1000, pot_Min_ADC, pot_Max_ADC);                    // map distance to POT values. 1000 multiplier converts floats into int for map()
  headingTo = Desired_POT; // start with an extension
  
  // Set PWM Speed When not using PID. COmment Out Wen using PID
 //Motor_PWM = 115;
  

  // PID Control
  min_PWM = 100;            // Experimentally determined lowest PWM motor will move at
  kp = 0.8;
  ki = 0;
  kd = 0;
  last_time = 0;

  Serial.begin(9600);
}

void loop() {

  unsigned long currentTime = millis();
  // PID timing for derivatives and integral
  dt = (currentTime - last_time)/1000.00;  // change in time dt in seconds
  if (dt <= 0) dt = 0.00001;                  // Protect against divide-by-zero
  last_time = currentTime;         // saves previus currentTime to be subtracted in previous line


  // timing determines location actuator moves to
  // Value of headingTo swaps between Desired_POT and Home_POT after custom amount of time
  if (currentTime > nextTime) {
    if (headingTo == Desired_POT) {
      headingTo = Home_POT;
      // reset integral and derivative values for new location
      integral = 0;
      previous = 0;
      nextTime = currentTime + timeRetract;
    } else {
      headingTo = Desired_POT;
      // reset integral and derivative values for new location
      integral = 0;
      previous = 0;
      nextTime = currentTime + timeExtend;
    }
  }
  
  // PID function to determine motor speed
  double error = headingTo - analogRead(Position_pin);    // error = input-output
  // Set Motor Speed based on PID. Constrained between min and max of PWM range
  Motor_PWM = constrain(abs(pid(error)), min_PWM, 255);            // abs() of PID to avoid "negative errors" being constrained to min_PWM for when motor travels in reverse.
 
 // motor Function Inputs; (Position, Motor Speed)
 move_To_Position(headingTo, Motor_PWM);    // run motor function
 
 
 // Print PID Control 
 Serial.print("\t");
 Serial.print("Error: ");
 Serial.print(error);
 Serial.print("\tPWM Output: ");
 Serial.print(Motor_PWM);
 Serial.print("\tPressure Reading:  ");
 Serial.print(analogRead(A5));
 Serial.print("\tTime Elapsed:  ");
 Serial.println(millis());
 
 

  

 delay(10); // prevent jiggling 
}


// Function For Moving To Desired Position
void move_To_Position(int desired, int speed){          // plug in desired potentiometer value and speed to run function

int  Actual_POT = analogRead(Position_pin);  // Read the current POT value
  
  if (abs(Actual_POT - desired) <= deadband) { // If within deadband do nothing
    digitalWrite(Extend_pin, LOW);
    digitalWrite(Retract_pin, LOW);
    analogWrite(Speed_pin, 0);
  }

  if (Actual_POT < desired - deadband){   // If too far in, extend at desired speed
    digitalWrite(Extend_pin, HIGH);
    digitalWrite(Retract_pin, LOW);
    analogWrite(Speed_pin, speed);
  }

  if (Actual_POT > desired + deadband){   // If too far out, retract at desired speed
      digitalWrite(Extend_pin, LOW);
      digitalWrite(Retract_pin, HIGH);
      analogWrite(Speed_pin, speed);
    } 
  
  // Print Position
  Serial.print("Actual:");
  Serial.print(Actual_POT);
  Serial.print("\t");
  Serial.print("Target:");
  Serial.print(desired);
  
  /*
  Serial.print(Actual_POT);
  Serial.print(",");
  Serial.print(desired);
  Serial.print(",");
  Serial.println(millis());
  */
}

// PID Function
double pid(double error)
{
  double proportional = error;                    // Kp*error
  integral += error * dt;                         // Ki*sum of errors
  double derivative = (error - previous) / dt;    // Kd*change in error
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}