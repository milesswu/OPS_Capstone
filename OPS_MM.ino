#define LEFT_SPEED 5        //pin D5
#define LEFT_FORWARD A2     //pin A2
#define LEFT_BACKWARD A1    //pin A1

#define RIGHT_SPEED 6       //pin D6
#define RIGHT_FORWARD 8     //pin D7
#define RIGHT_BACKWARD 7    //pin D8

#define LEFT_SENSOR A5      //pin A5
#define RIGHT_SENSOR A4     //pin A4
#define FORWARD_SENSOR A6   //pin A6

const int NUM_SAMPLES = 10;
double ZERO_ERROR;
double prevError;
double pastTime;
double totalError;

double k_p = 0.2;
double k_i = 0;
double k_d = 20;

int leftSpeed, rightSpeed;
int minSpeed, maxSpeed;

bool endReached = false;
bool slowStatus = false;
int slowStart = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT_SPEED, OUTPUT);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);

  pinMode(RIGHT_SPEED, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(FORWARD_SENSOR, INPUT);

  double leftVal, rightVal, forwardVal;
  readSensors(100, leftVal, rightVal, forwardVal);
  ZERO_ERROR = leftVal - rightVal;
  
  delay(1000);
  leftSpeed = 255;
  rightSpeed = 255;
  setMotorSpeed(leftSpeed,rightSpeed);
  
  pastTime = millis();
  prevError = ZERO_ERROR;
  totalError = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (endReached)
  {
    stopMotors();
  }
  else
  {
    goForward();
    
    double left, right, forward;
    readSensors(NUM_SAMPLES, left, right, forward);
    if ((left >= 800 && right >= 800 && forward >= 800) || forward >= 950) // Second conditional is to prevent the car from crashing
      endReached = true;
   
    double currentError = ZERO_ERROR - (left - right);
    int currentTime = millis();
    double d_t = currentTime - pastTime;
    totalError += currentError * d_t;
    double output = k_p * currentError + k_i * totalError + k_d * (currentError - prevError) / d_t;
    
    prevError = currentError;
    pastTime = currentTime;
    
    if (forward >= 200)
    {
      minSpeed = 25;
      maxSpeed = 255;
      if (!slowStatus)
      {
        slowStart = millis();
        slowStatus = true;
      }    
    }
    else if (millis() - slowStart >= 1200)
    {
      minSpeed = 235;
      maxSpeed = 255;
      slowStatus = false;
    }
    
    leftSpeed = constrain(leftSpeed - output, minSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed + output, minSpeed, maxSpeed);

    setMotorSpeed(leftSpeed, rightSpeed);
  }
}

void goForward()
{
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, LOW);
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, LOW);
}

void stopMotors()
{
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, HIGH);
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, HIGH);
}

void setMotorSpeed(int leftspeed, int rightSpeed)
{
  analogWrite(LEFT_SPEED, leftSpeed);
  analogWrite(RIGHT_SPEED, rightSpeed);
}

void readSensors(int numSamples, double& leftReading, double& rightReading, double& forwardReading)
{
  leftReading = rightReading = forwardReading = 0;
  for (int i = 0; i < numSamples ; i++)
  {
    leftReading += analogRead(LEFT_SENSOR);
    rightReading += analogRead(RIGHT_SENSOR);
    forwardReading += analogRead(FORWARD_SENSOR);
  }
  leftReading /= numSamples;
  rightReading /= numSamples;
  forwardReading /= numSamples;
}
