// Based on Science Fun Youtube Video
// https://www.youtube.com/watch?v=c7JtfogOsTQ&t=307s

const int Extend_pin = 10;     // connect to motor control board IN2
const int Retract_pin = 11;     // connect to motor control board IN1
const int Position_pin = A0;   // Potentiometer Position
const int Pressure_pin = A6;   // Pressure
const int Speed_pin = 9;       // PWM pin to set speed. Coneect to motor control baord pin ENA

//float Desired_location;          // Distance of desired location. (Initialized below)
int Desired_POT;               // Potentiometer Value at desired location
//float Home_location;             // Distance of home location. (Initialized below)
int Home_POT;                  // Potentiometer Value at home location
int headingTo;                // Variable assigned the desired or home pot values

int Extend_PWM;                // Set Extend Speed
int Retract_PWM;               // Set Retract Speed
int deadband = 30;             // Prevents Jittering at accepetably close final position. Smaller Value is more accurate but more jittery
int pot_Max_ADC = 1023;        // Potentiometre ADC value at Full Extension
int pot_Min_ADC = 0;           // Potentiometer ADC Value at Full Retraction


// Testing parameters
bool running = false;
float strokeVolume = 30; //mL
float cycleRate = 0.5; // Hz
float Home_location = 0.5;  // in
double syringeDiameter = 6.49; // cm
float Desired_location = (4.0/PI/sq(syringeDiameter)*strokeVolume/2.54) + 0.5;



// Timing
unsigned long nextTime;
int timeExtend = 500/cycleRate;           // milliseconds
int timeRetract = 500/cycleRate;          // milliseconds

// Speed Tracking
unsigned long lastTime = 0;
double location = 0;
double prevLocation = 0;
float dt;

// Additive Target Compensator
// If shooting low on extension, add 1 to desired pot value
// If shooting high on extension, subtract 1 from desired pot value
// If shooting low on retraction, add 1 to home pot value
// If shooting high on retraction, subtract 1 from home pot value

// Currently worsens performance
bool targetCompensation = false;
bool extending = false;
int extendingCorrection = 0;
int retractingCorrection = 0;
int correctionTick = 0;

// Serial Monitor Handling
char incomingChar;



void setup() {
  pinMode(Extend_pin, OUTPUT);
  pinMode(Retract_pin, OUTPUT);
  pinMode(Speed_pin, OUTPUT);
  digitalWrite(Extend_pin, LOW);    // Start with Actuator Motor Off In Both Directions
  digitalWrite(Retract_pin, LOW);


// Set testing parameters


  // Set Desired and Home Locations and Potentiometer values
  //Desired_location = 0.854; %30 mL                                                                      // inches
  Desired_location = constrain(Desired_location, 0, 2);                                             // only accept values from 0 to 2 inches
  Desired_POT = map(Desired_location*1000, 0*1000 , 2*1000, pot_Min_ADC, pot_Max_ADC);                             // map distance to POT values                                                                           // inches
  Home_location = constrain(Home_location, 0, 2);                                                   // only accept values from 0 to 2 inches
  Home_POT = map(Home_location*1000, 0*1000 , 2*1000, pot_Min_ADC, pot_Max_ADC);                                   // map distance to POT values
  headingTo = Desired_POT; // start with an extension
  
  // Set Speeds
  Extend_PWM = 255;
  Retract_PWM = 255;

  Serial.begin(9600);
}

void loop() {

  // While serial data is available, read and parse it.
  if (Serial.available() > 0) {
    ReceiveInstructions();
  }

  // Change actuation parameters based on serial input
  Desired_location = (4.0/PI/sq(syringeDiameter)*strokeVolume/2.54) + 0.5;
  Desired_location = constrain(Desired_location, 0, 2);                                             // only accept values from 0 to 2 inches
  Desired_POT = map(Desired_location*1000, 0*1000 , 2*1000, pot_Min_ADC, pot_Max_ADC);                // map distance to POT values
  
  timeExtend = 500/cycleRate;           // milliseconds
  timeRetract = 500/cycleRate;          // milliseconds
  


  unsigned long currentTime = millis();
  // timing determines location actuator moves to
  // Value of headingTo swaps between Desired_POT and Home_POT after custom amount of time

  // Time increments for speed tracking
  dt = (currentTime - lastTime)/1000.0;  // change in time dt in seconds
  if (dt <= 0) dt = 0.00001;                  // Protect against divide-by-zero
  lastTime = currentTime;         // saves previous currentTime to be subtracted in previous line
 
 // Added target compensation
  if (currentTime > nextTime) {
    
    if (extending) {
      // If target compensation is on, compensate for under/overshoot in dead band while retracting
      if (targetCompensation) {
        // Apply target correction
        headingTo = constrain(Home_POT + retractingCorrection,Home_POT - deadband, Home_POT + deadband);
        // Now that extending is finished, compensate for previous error.
        extendingCorrection = extendingCorrection + correctionTick;
      } else {
        headingTo = Home_POT;
      }
      nextTime = currentTime + timeRetract;
      extending = false;
    } else {
      // If target compensation is on, compensate for under/overshoot in dead band while extending
      if (targetCompensation) {
        // Apply target correction
        headingTo = constrain(Desired_POT + extendingCorrection, Desired_POT - deadband, Desired_POT + deadband);
        // Now that retracting is finished, compensate for previous error.
        retractingCorrection = retractingCorrection + correctionTick;
      } else {
        headingTo = Desired_POT;
      }
      nextTime = currentTime + timeExtend;
      extending = true;
    }
  }

  // Begin by moving to home destination
  while (millis()<1000) {
    int temp = move_To_Position(Home_POT);
  }

if (running) {
  correctionTick = move_To_Position(headingTo);
}

  if (targetCompensation) {
  Serial.print(extendingCorrection);
  Serial.print(","); 
  Serial.print(retractingCorrection);
  }
  
  if (running) {Serial.println();}

  delay(50); // prevent jiggling 
}




// Function For Moving To Desired Position
int move_To_Position(int desired) {          // plug in desired potentiometer value to run function

int correction = 0;

int  Actual_POT = analogRead(Position_pin);  // Read the current POT value
int  Actual_PRE = analogRead(Pressure_pin);  // Read the current POT value
  

  
  if (abs(Actual_POT - desired) <= deadband) { // If within deadband do nothing
    digitalWrite(Extend_pin, LOW);
    digitalWrite(Retract_pin, LOW);
    analogWrite(Speed_pin, 0);
  }

  if (Actual_POT < desired - deadband){   // If too far in, extend at desired speed
    digitalWrite(Extend_pin, HIGH);
    digitalWrite(Retract_pin, LOW);
    analogWrite(Speed_pin, Extend_PWM);
  }

  if (Actual_POT > desired + deadband){   // If too far out, retract at desired speed
      digitalWrite(Extend_pin, LOW);
      digitalWrite(Retract_pin, HIGH);
      analogWrite(Speed_pin, Retract_PWM);
    } 

// If shooting low on extension, add 1 to desired pot value
// If shooting high on extension, subtract 1 from desired pot value
// If shooting low on retraction, add 1 to home pot value
// If shooting high on retraction, subtract 1 from home pot value
int incrementValue = 1;
  if (Actual_POT < desired) {correction = incrementValue;}
  else if (Actual_POT > desired) {correction = -incrementValue;}
  else {correction = 0;}

  // Speed Tracking - Commented out. This should be handled by an external data processor.
double cmDesired = desired/1023.0*2.0*2.54;
double cmLocation = Actual_POT/1023.0*2.0*2.54; //Distance of potentiometer wiper, in cm.
 // double cmSpeed = (location - prevLocation)/dt;
//prevLocation = location;

  // Estimating volumetric flow rate
 // double syringeDiameter = 6.49; //550 cc syringe: 6.49 cm, 60 cc syringe: 2.84 cm
 // double volFlowRate = cmSpeed*PI/4.0*sq(syringeDiameter);

// Pressure conversion to kPa
float kpaPressure = (Actual_PRE/1023.0*5.0 - 0.5)/4.0*206.843;




/* CSV Column order:
- Total run time
- Target position
- Current Position
- Pressure Reading
*/
 if (running) {
  Serial.print(millis());
  Serial.print(","); 
  Serial.print(cmDesired); 
  Serial.print(","); 
  Serial.print(cmLocation);
  Serial.print(","); 
  Serial.print(kpaPressure);
 }
  return correction;
}

// ReceiveInstructions updates the system's parameters based on input from the serial monitor
// 'r': run
// 's': stop
// 'v': Following integer is the new stroke volume, in mL
// 'f': Following float is the new cycle frequency, in Hz
// e.g. r v30 f1
void ReceiveInstructions() {
  int blockIndex = 0;
  int lineIndex = 0;
  bool changeVolume = false;
  bool changeFrequency = false;
  int i;
  char incomingBlock[8] = {'\0','\0','\0','\0','\0','\0','\0','\0'};
  char incomingLine[20];
  int newIndex = 0;
  char temp;
 
  while(Serial.available() > 0) {
    incomingChar = Serial.read();
    delay(10);
    incomingLine[lineIndex] = incomingChar;
    lineIndex++;
    if (incomingChar == ' ') { // if ' ': check update bools, update parameter using block if true and then set to false, clear block, add to line
      if (changeVolume && blockIndex != 0) {
        strokeVolume = atof(incomingBlock);
        changeVolume = false;
      } else if (changeFrequency && blockIndex != 0) {
        cycleRate = atof(incomingBlock);
        changeFrequency = false;
      }
      for (i = 0; i<blockIndex; i++) {
        incomingBlock[i]='\0';
      }
      blockIndex = 0;
    } else if (incomingChar == '\n'){     // else if '\n': clear block, add to line, break.
      if (changeVolume && blockIndex != 0) {
        strokeVolume = atof(incomingBlock);
        changeVolume = false;
      } else if (changeFrequency && blockIndex != 0) {
        cycleRate = atof(incomingBlock);
        changeFrequency = false;
      }
      for (i = 0; i<blockIndex; i++) {
        incomingBlock[i]='0';
      }
      blockIndex = 0;
    } else if (incomingChar == 'r'){     // else if 'r': run, add to line
      running = true;
    } else if (incomingChar == 's'){     // else if 's': stop, add to line
      running = false;
    } else if (incomingChar == 'v'){     // else if 'v': set changeVolume to true, clear block, add to line
      changeVolume = true;
    } else if (incomingChar == 'f'){     // else if 'f': set changeFrequency to true, add to line
      changeFrequency = true;
    } else {        // else add to block
      incomingBlock[blockIndex] = incomingChar;
      blockIndex++;
    }
    //Serial.println(incomingChar);
    //Serial.println(incomingBlock);
  }
  
  /*
  //Serial.print(incomingLine);
  Serial.print("Running: ");
  Serial.println(running);
  Serial.print("Stroke Volume: ");
  Serial.println(strokeVolume);
  Serial.print("Cycle Rate: ");
  Serial.println(cycleRate);
  Serial.println();
  */
  
}



