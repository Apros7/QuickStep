struct MotorInfo {
  float acceleration;
  float maxSpeed;
  long currentPosition;
  bool isRunning;
  int positiveDirection;
  long totalSteps;
  long stepsSinceStart;
};

struct MotorParams {
    float acceleration;
    float maxSpeed;
};

class QuickStepper {
  private:
    const int stepPin;
    const int dirPin;

    float acceleration = 10.0; // steps/s^2
    float maxSpeed = 1000.0; // steps/s
    float minSpeed = 1;

    float currentSpeed = 0.0;  // steps/second
    unsigned long stepDelay;   // microseconds
    unsigned long lastStepTime = 0;
    unsigned long constantPhaseStart = 0;
    long totalSteps = 0;
    bool inConstantPhase = false;
    bool deceleration = false;
    unsigned long constantDuration = 0;
    bool isRunning = false;
    long stepsSinceStart;
    long stepsToAccelerate;
    float middleTotalSteps;
    long currentPosition;
    int positiveDirection = 1;

    // Delete
    char motorLabel = '-';
  
  public:
    QuickStepper(int step_pin, int dir_pin): stepPin(step_pin), dirPin(dir_pin) {
      pinMode(stepPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
      digitalWrite(dirPin, HIGH); // initial direction, should be able to change this
    }

    void setConstantDuration(unsigned long duration) {
      constantDuration = duration;
    }

    void moveTo(long dest) {
      // dest is total steps to run for
      isRunning = true;
      currentSpeed = 0.0;
      inConstantPhase = false;
      deceleration = false;
      lastStepTime = micros();
      stepDelay = 1000000;
      totalSteps = dest - currentPosition;
      currentPosition = dest;
      setDirection(totalSteps);
      middleTotalSteps = int(totalSteps / 2);
      stepsSinceStart = 0;
    }

    void stop() {
      isRunning = false;
    }

    bool running() {
      return isRunning;
    }

    void setAcceleration(float newAcceleration) { acceleration = newAcceleration; }
    void setMaxSpeed(float newMaxSpeed) { maxSpeed = newMaxSpeed; }
    void setPosition(float position) { currentPosition = position; }
    float getPercentageDone() {return static_cast<float>(stepsSinceStart) / static_cast<float>(totalSteps); }
    float getAcceleration() {return acceleration; }
    float getMaxSpeed() {return maxSpeed; }

    MotorInfo getInfo() {
      MotorInfo info = {
        acceleration,
        maxSpeed,
        currentPosition,
        isRunning,
        positiveDirection,
        totalSteps,
        stepsSinceStart,
      };
      return info;
    }

    void setLabel(char label) {
      motorLabel = label;
    }

  
  private:

    void setDirection(unsigned int dest) {
      if (dest < 0) {
        positiveDirection = 0;
        digitalWrite(dirPin, LOW);
      } else {
        positiveDirection = 1;
        digitalWrite(dirPin, HIGH);
      }
    }

    void stepMotor() {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1); // This could cause some issues
      digitalWrite(stepPin, LOW);
    }
  
  public:
    bool run() {
      if (!isRunning) return;

      unsigned long currentMicros = micros();

      if (stepsSinceStart >= totalSteps) {
        stop();
        Serial.println(stepsSinceStart);
        return false;
      }

      if (currentMicros - lastStepTime >= stepDelay) {
        Serial.print(motorLabel);
        Serial.println(positiveDirection);
        Serial.println(currentMicros);
        //Serial.println((totalSteps - stepsSinceStart));
        //Serial.println(stepsToAccelerate);
        // Serial.println(((totalSteps - stepsSinceStart) - stepsToAccelerate));
        stepMotor();
        stepsSinceStart++;
        lastStepTime = currentMicros;
        
        // Update speed based on phase
        if (!inConstantPhase && !deceleration) {
  
          // Acceleration phase
          currentSpeed += acceleration * (stepDelay / 1000000.0);
          if (stepsSinceStart > middleTotalSteps) {
            deceleration = true;
          }
          if (currentSpeed >= maxSpeed) {
            inConstantPhase = true;
            stepsToAccelerate = stepsSinceStart * 0.95;
          }
        }
        else if (inConstantPhase) {
          if (((totalSteps - stepsSinceStart) - stepsToAccelerate) < 0) {
            inConstantPhase = false;
            deceleration = true;
          }
        }
        else if (deceleration) {
          currentSpeed -= acceleration * (stepDelay / 1000000.0);
          currentSpeed = max(currentSpeed, minSpeed);
        }
      
        stepDelay = 1000000.0 / currentSpeed;
      };
      return true;
    };
};

class MultiStepper {
  private:
    static const int MAX_MOTORS = 10;
    static const int COORDINATE_DIMENSIONS = 3;
    static const int MAX_CHAINS = 5;
    QuickStepper* motors[MAX_MOTORS];
    int motorCount = 0;

    float maxAcceleration = 1; // steps/s^2
    float maxSpeed = 1;
    long* currentPositions[COORDINATE_DIMENSIONS];
    long* futurePositions[MAX_CHAINS][COORDINATE_DIMENSIONS];
    int futurePositionsIndex = 0;

    // Chaining
    float cornering = 0.2;
    float percentageBeforeTurning = 0.8;


    MotorParams* calculateMotorParams(long* steps, int numMotors) {
      // Find motor with max steps (it will use max speed/accel)
      long maxSteps = 0;
      for (int i = 0; i < numMotors; i++) {
          if (abs(steps[i]) > maxSteps) {
              maxSteps = abs(steps[i]);
          }
      }

      // Calculate time needed for max steps motor
      // Using s = (v²)/(2a) for acceleration and deceleration phases
      // and s = vt for constant speed phase
      float maxTime = sqrt(4 * maxSteps / maxAcceleration);
      float actualMaxSpeed = maxSteps / maxTime;
      
      if (actualMaxSpeed > maxSpeed) {
          // Need to recalculate with speed limit
          actualMaxSpeed = maxSpeed;
          maxTime = (maxSteps / maxSpeed) + (maxSpeed / maxAcceleration);
      }

      // Calculate params for other motors
      MotorParams* params = new MotorParams[numMotors];
      for (int i = 0; i < numMotors; i++) {
          float ratio = abs(steps[i]) / (float)maxSteps;
          params[i].maxSpeed = actualMaxSpeed * ratio;
          params[i].acceleration = maxAcceleration * ratio;
      }

      return params;
    }
  
  public:
    MultiStepper() {
      setMaxSpeed(maxSpeed);
      setAcceleration(maxAcceleration);
    }

    void setCornering(float newCornering) {cornering = newCornering; }
    
    bool addMotor(QuickStepper* motor) {
      if (motorCount >= MAX_MOTORS) {
        return false;  // Cannot add more motors
      }
      motors[motorCount++] = motor;
      return true;
    }
    
    void moveTo(long positions[3]) {

      long steps[motorCount];
      for (int i = 0; i < COORDINATE_DIMENSIONS; i++) {
        currentPositions[i] = &positions[i];
      }
      for (int i = 0; i < motorCount; i++) {
        motors[i]->moveTo(positions[i]);
        steps[i] = motors[i]->getInfo().totalSteps;
      }

      MotorParams* params = calculateMotorParams(steps, motorCount);

      for (int i = 0; i < motorCount; i++) {
          motors[i]->setMaxSpeed(params[i].maxSpeed);
          motors[i]->setAcceleration(params[i].acceleration);
      }

      delete[] params;

    }

    void chainTo(long* positions) {
      // Can only handle cases where a motor is not used in both positions
      // I.e a point of (10, 10, 0) -> (20, 10, 10) will not work
      // as the x motor is used in both of them
      // (10, 10, 0) -> (10, 10, 10) will work

      float percentageBeforeTurning = 1 - cornering;
      for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
        futurePositions[futurePositionsIndex][j] = positions[j];
      }
      futurePositionsIndex++;

    }

    void stop() {
      for (int i = 0; i < MAX_CHAINS; i++) {
        for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
          futurePositions[i][j] = 0; // Set each element to 0
        }
      }
      futurePositionsIndex = 0;
    }

    void startCornering() {
      long* newPositions[COORDINATE_DIMENSIONS];
      for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
        newPositions[j] = futurePositions[0][j];
      }

      for (int i = 1; i < motorCount; i++) {
        for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
          futurePositions[i - 1][j] = futurePositions[i][j];
        }
      }
      long steps[motorCount];
      for (int i = 0; i < motorCount; i++) {
        if (newPositions[i] != currentPositions[i]) {
          motors[i]->moveTo(newPositions[i]);
          steps[i] = motors[i]->getInfo().totalSteps;
        } else {
          steps[i] = 1;
        }
      }

      MotorParams* params = calculateMotorParams(steps, motorCount);

      for (int i = 0; i < motorCount; i++) {
        if (steps[i] != 1) {
          motors[i]->setMaxSpeed(params[i].maxSpeed);
          motors[i]->setAcceleration(params[i].acceleration);
        }
      }

      for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
        currentPositions[j] = newPositions[j];
      }

      delete[] params;

    }
    
    bool run() {
      bool anyRunning = false;
      bool anyBelowThreshold = false;
      for (int i = 0; i < motorCount; i++) {
        if (motors[i]->run()) {
          anyRunning = true;

          if (motors[i]->getPercentageDone() <= percentageBeforeTurning) {
            anyBelowThreshold = true;  // One motor is below threshold
          }
        }
      }
      if (!anyBelowThreshold) { 
        Serial.println("SHOULD CORNER NOW");
        delay(2000);
        startCornering(); 
      }
      if (!anyRunning) { stop(); }
      return anyRunning;
    }
    
    void setMaxSpeed(float newMaxSpeed) {
      maxSpeed = newMaxSpeed;
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setMaxSpeed(newMaxSpeed);
      }
    }

    void setAcceleration(float newAcceleration) { 
      maxAcceleration = newAcceleration;
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setAcceleration(newAcceleration);
      }
    }

    void setPosition(float position) { 
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setPosition(position);
      }
    }
};

QuickStepper stepperX(0, 1);
QuickStepper stepperY(2, 3);
QuickStepper stepperZ(4, 5);

MultiStepper steppers;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  steppers.addMotor(&stepperX);
  steppers.addMotor(&stepperY);
  steppers.addMotor(&stepperZ);
  steppers.setMaxSpeed(100.0);
  steppers.setAcceleration(10.0);

  stepperX.setLabel('X');
  stepperY.setLabel('Y');
  stepperZ.setLabel('Z');

  const float STEPS_PER_CM = 110.65;

  long positions[] = {0, 0, 1 * STEPS_PER_CM};
  long positions2[] = {2 * STEPS_PER_CM, 2 * STEPS_PER_CM, 1 * STEPS_PER_CM};
  long positions3[] = {2 * STEPS_PER_CM, 2 * STEPS_PER_CM, 0 * STEPS_PER_CM};
  steppers.moveTo(positions);
  steppers.chainTo(positions2);
  steppers.chainTo(positions2);

  Serial.println("X");
  Serial.println(stepperX.getInfo().totalSteps);
  Serial.println(stepperX.getAcceleration());
  Serial.println(stepperX.getMaxSpeed());
  Serial.println("Y");
  Serial.println(stepperY.getInfo().totalSteps);
  Serial.println(stepperY.getAcceleration());
  Serial.println(stepperY.getMaxSpeed());
  Serial.println("Z");
  Serial.println(stepperZ.getInfo().totalSteps);
  Serial.println(stepperZ.getAcceleration());
  Serial.println(stepperZ.getMaxSpeed());
  Serial.println("-----------");

  while(steppers.run()) {}

  Serial.println("Positions reached after:");
  Serial.println(millis() / 1000);
}

void loop() {
  //stepperX.moveTo(1000);
  //while (stepperX.run()) {}
  // Serial.println("done");
  // delay(1000);
  // put your main code here, to run repeatedly:

}
