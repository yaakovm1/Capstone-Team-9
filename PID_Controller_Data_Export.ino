/* 
Position Control on Science Fun Youtube Video
https://www.youtube.com/watch?v=c7JtfogOsTQ&t=307s

PID Controller Based on Ian Carey Youtube video
https://www.youtube.com/watch?v=RZW1PsfgVEI

When Serial Data is in CSV format. Column Order Actual ADC, Target ADC, Time
*/

// Pins
const int Extend_pin = 10;     // connect to motor control board IN2
const int Retract_pin = 11;     // connect to motor control board IN1
const int Position_pin = A0;   // Potentiometer Position
const int Pressure_pin = A6;   // Pressure
const int Speed_pin = 9;       // PWM pin to set speed. Coneect to motor control baord pin ENA
const int Trigger_Pin = 8;      // Sends 5V signal at start of Sketch to signal Labview that the motor has started.

// Location Variables
float Desired_location;          // Distance of desired location
int Desired_POT;               // Potentiometer Value at desired location
float Home_location;             // Distance of home location
int Home_POT;                  // Potentiometer Value at home location
int headingTo;                // Variable assigned the desired or home pot values

// Motor and Potentiometer Specs
int Motor_PWM;                 // Set Motor Speed
int min_PWM;                   // Experimentally determined minimun PWM motr will move at
int pot_Max_ADC = 1023;        // Potentiometre ADC value at Full Extension
int pot_Min_ADC = 0;           // Potentiometer ADC Value at Full Retraction


// Timing
unsigned long nextTime;
int timeExtend = 700/2;           // milliseconds
int timeRetract = 300/2;          // milliseconds

// PID Control
double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;

// Speed Tracking
double location = 0;
double prevLocation = 0;

void setup() {
  pinMode(Extend_pin, OUTPUT);
  pinMode(Retract_pin, OUTPUT);
  pinMode(Speed_pin, OUTPUT);
  pinMode(Trigger_Pin, OUTPUT);

  // Start with Actuator Motor Off In Both Directions
  digitalWrite(Extend_pin, LOW);    
  digitalWrite(Retract_pin, LOW);

  // Set Desired and Home Locations and Potentiometer values
  Desired_location = 0.963;                                                                          // inches
  Desired_location = constrain(Desired_location, 0, 2);                                             // only accept values from 0 to 2 inches
  Desired_POT = map(Desired_location*1000, 0*1000 , 2*1000, pot_Min_ADC, pot_Max_ADC);              // map distance to POT values. 1000 multiplier converts floats into int for map()
  Home_location = 0.25;                                                                              // inches
  Home_location = constrain(Home_location, 0, 2);                                                   // only accept values from 0 to 2 inches
  Home_POT = map(Home_location*1000, 0*1000 , 2*1000, pot_Min_ADC, pot_Max_ADC);                    // map distance to POT values. 1000 multiplier converts floats into int for map()
  headingTo = Desired_POT; // start with an extension

  // PID Control
  min_PWM = 50;            // Experimentally determined lowest PWM motor will move at
  bool airGains = true;
  if (airGains) { // Working Fluid: Air
    kp = 0.7;
    ki = 0.01;
    kd = 0.01;
  } else if (!airGains) { // Working Fluid: Water
    kp = 0.7; // TODO: Tune
    ki = 0.00;
    kd = 0.00;
  }
  last_time = 0;
 
// Trigger Labview
digitalWrite(Trigger_Pin, HIGH);
delay(100);                       // allow time for trigger to be read
  // Start Serial Monitor
  Serial.begin(9600);


// Column Headers for Serial Monitor CSV Output
Serial.print("Actual_POT");
  Serial.print(",");
  Serial.print("headingTo");
  Serial.print(",");
  Serial.print("Motor_PWM");
  Serial.print(",");
  Serial.print("cmSpeed");
  Serial.print(",");
  Serial.println("Run Time");
}

void loop() {

  unsigned long currentTime = millis();
  // PID timing for derivatives and integral
  dt = (currentTime - last_time)/1000.0;  // change in time dt in seconds
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
  int Actual_POT = analogRead(Position_pin);
  double error = headingTo - Actual_POT;    // error = input-output
 
// Set Motor Speed based on PID. Constrained between min and max of PWM range
  double pid_output = pid(error); //signed result
 
 // Motor Function
 move_To_Position(pid_output);    // run motor function

// Speed Tracking
double location = Actual_POT/1023.0*2.0*2.54; //Distance of potentiometer wiper, in cm.
double cmSpeed = (location - prevLocation)/dt;
prevLocation = location;

// Estimating volumetric flow rate
double syringeDiameter = 6.49; //550 cc syringe: 6.49 cm, 60 cc syringe: 2.84 cm
double volFlowRate = cmSpeed*PI/4.0*sq(syringeDiameter);

 
bool debugMode = false; // Set to false for CSV output
  if (debugMode) {
  // Full human-readable debug output
  Serial.print("Actual:");
  Serial.print(Actual_POT);
  Serial.print("\tTarget:");
  Serial.print(headingTo);
  Serial.print("\tError: ");
  Serial.print(error);
  Serial.print("\tPWM Output: ");
  Serial.print(Motor_PWM);
  Serial.print("\tSpeed (cm/s):");
  Serial.println(cmSpeed);
  Serial.print("\tFlow Rate (cc/s):");
  Serial.println(volFlowRate);
  Serial.print("\tPressure Reading: ");
  Serial.print(analogRead(A5));
  Serial.print("\tTime Elapsed: ");
  Serial.println(millis());
} else {
  // CSV format for data logging
  Serial.print(Actual_POT);
  Serial.print(",");
  Serial.print(headingTo);
  Serial.print(",");
  Serial.print(Motor_PWM);
  Serial.print(",");
  Serial.print(cmSpeed);
  Serial.print(",");
  Serial.println(millis());
}

 //delay(10); // prevent jiggling 
}


// Function For Moving To Desired Position
void move_To_Position(double pid_output){          // plug in pid

  Motor_PWM = constrain(abs(pid_output), 0, 255);  
  
  if (Motor_PWM < min_PWM) { // do nothing if PID is below range of motor
    digitalWrite(Extend_pin, LOW);
    digitalWrite(Retract_pin, LOW);
    analogWrite(Speed_pin, 0);
    return;
  }

  if (pid_output > 0){   // headingTo is > than actual
    // extend
    digitalWrite(Extend_pin, HIGH);
    digitalWrite(Retract_pin, LOW);
  }

  else{   
    // If headingTo < actual
    // retract
      digitalWrite(Extend_pin, LOW);
      digitalWrite(Retract_pin, HIGH);
    } 
  
analogWrite(Speed_pin, Motor_PWM);

}

// PID Function
double pid(double error)
{
  double proportional = error;                    // Kp*error
  integral += error * dt;                         // Ki*sum of errors
  double derivative = (error - previous) / dt;    // Kd*change in error
  previous = error;
  // integral and derivative gains are constrained
  double output = (kp * proportional) + constrain((ki * integral),-25, 25) + constrain((kd * derivative), -25, 25);
  return output;
}
