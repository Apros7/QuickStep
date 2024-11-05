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

/* struct RunID {
  bool inConstantPhase = false;
  bool deceleration = false;
  long destination;
} */

class QuickStepper {
  private:
    const int stepPin;
    const int dirPin;

    float acceleration = 10.0; // steps/s^2
    float maxSpeed = 1000.0; // steps/s
    float minSpeed = 10.0; // steps/s

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

    void moveTo(long dest) { // dest = total steps to run
      stepsToAccelerate = 0;
      totalSteps = dest - currentPosition;
      stepsSinceStart = 0;
      setDirection(totalSteps);
      middleTotalSteps = int(totalSteps / 2);
      if (!isRunning) {
        isRunning = true;
        currentSpeed = minSpeed;
        /*inConstantPhase = false;
        deceleration = false;*/
        lastStepTime = micros();
        stepDelay = 1;
      }
      /*Serial.print("Called MoveTo On ");
      Serial.println(motorLabel);
      Serial.println(totalSteps);
      Serial.println(dest);
      Serial.println(currentPosition);*/
    }

    void restartTime() {
      lastStepTime = micros();
    }

    void stop() {
      isRunning = false;
    }

    bool running() {
      return isRunning;
    }

    void setAcceleration(float newAcceleration) { acceleration = newAcceleration; }
    void setMaxSpeed(float newMaxSpeed) { maxSpeed = newMaxSpeed; }
    void setMinSpeed(float newMinSpeed) { minSpeed = newMinSpeed; }
    void setPosition(float position) { currentPosition = position; }
    float getPercentageDone() {return static_cast<float>(stepsSinceStart) / static_cast<float>(totalSteps); }
    float getAcceleration() {return acceleration; }
    float getMaxSpeed() {return maxSpeed; }
    int getIsRunning() {return isRunning; }
    char getMotorLabel() {return motorLabel; }

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
        /*Serial.print(motorLabel);
        Serial.println(positiveDirection);
        Serial.println(stepsSinceStart);*/
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
        if (positiveDirection == 1) {
          currentPosition++;
        } else {
          currentPosition--;
        }
        lastStepTime = currentMicros;
        
        // Update speed based on phase
        if (currentSpeed < maxSpeed && !(stepsSinceStart > middleTotalSteps)) {
          currentSpeed += acceleration * (stepDelay / 1000000.0);
          if (currentSpeed >= maxSpeed) {
            inConstantPhase = true;
            stepsToAccelerate = stepsSinceStart * 0.95;
          }
        }
        else if (
            currentSpeed > maxSpeed 
            || (stepsSinceStart > middleTotalSteps && stepsToAccelerate == 0)
            || ((totalSteps - stepsSinceStart) - stepsToAccelerate) < 0) 
        {
          currentSpeed -= acceleration * (stepDelay / 1000000.0);
          currentSpeed = max(currentSpeed, minSpeed);
        }
      
        stepDelay = 1000000.0 / currentSpeed;
      }
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
    bool isRunning = false;

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

      // Calculate time needed for max steps motor at max speed
      // t = sqrt(2s/a) for acceleration
      // Total time = 2 * acceleration time + constant speed time
      float accelTime = sqrt(2.0 * maxSteps / maxAcceleration);
      float totalTime;
      float actualMaxSpeed;

      if (accelTime * maxAcceleration >= maxSpeed) {
          // We hit speed limit, recalculate with max speed
          actualMaxSpeed = maxSpeed;
          float distanceAtMaxSpeed = maxSteps - (maxSpeed * maxSpeed / maxAcceleration);
          float constantSpeedTime = distanceAtMaxSpeed / maxSpeed;
          totalTime = (2 * maxSpeed / maxAcceleration) + constantSpeedTime;
      } else {
          // We can use triangular profile
          actualMaxSpeed = maxSpeed;
          totalTime = 2 * accelTime;
      }

      // Calculate params for all motors
      MotorParams* params = new MotorParams[numMotors];
      for (int i = 0; i < numMotors; i++) {
          if (steps[i] == 0) {
              params[i].maxSpeed = 0;
              params[i].acceleration = 0;
          } else {
              float ratio = abs(steps[i]) / (float)maxSteps;
              params[i].maxSpeed = actualMaxSpeed * ratio;
              params[i].acceleration = maxAcceleration * ratio;
          }
      }

      return params;
    }
  
  public:
    MultiStepper() {
      setMaxSpeed(maxSpeed);
      setAcceleration(maxAcceleration);
    }

    void setCornering(float newCornering) {
      cornering = newCornering;
      percentageBeforeTurning = 1 - cornering;
    }
    
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
        // Store the actual values, not pointers
        for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
            // Allocate new memory for each position
            futurePositions[futurePositionsIndex][j] = new long(positions[j]);
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
      isRunning = false;
    }

    void printFuturePositions() {
      Serial.println("Future Positions Array:");
      for (int i = 0; i < futurePositionsIndex; i++) {
          Serial.print("Chain ");
          Serial.print(i);
          Serial.print(": ");
          for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
              Serial.print(*futurePositions[i][j]);
              Serial.print(" ");
          }
          Serial.println();
      }
      Serial.println("End of Future Positions");
    }

    void startCornering() {
      if (futurePositionsIndex < 1) {return; }
      
      // Create local array to store the values
      long positions[COORDINATE_DIMENSIONS];
      for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
          positions[j] = *futurePositions[0][j];  // Dereference to get value
          delete futurePositions[0][j];  // Free the memory
      }

      // Shift remaining positions up
      for (int i = 0; i < futurePositionsIndex - 1; i++) {
          for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
              futurePositions[i][j] = futurePositions[i + 1][j];
          }
      }
      futurePositionsIndex--;
      long steps[motorCount];
      for (int i = 0; i < motorCount; i++) {
        /*if (!motors[i]->getIsRunning()) {
          motors[i]->moveTo(newPositions[i]);
          steps[i] = motors[i]->getInfo().totalSteps;
        } else {
          steps[i] = 0;
        }*/
        int stepsLeft = motors[i]->getInfo().totalSteps;// - motors[i]->getInfo().stepsSinceStart;
        motors[i]->moveTo(positions[i]);
        steps[i] = motors[i]->getInfo().totalSteps - stepsLeft;
      }
      //Serial.println("STEPS OVERVIEW:");
      for (int i = 0; i < motorCount; i++) {
        //Serial.println(steps[i]);
      }

      MotorParams* params = calculateMotorParams(steps, motorCount);

      for (int i = 0; i < motorCount; i++) {
        if (steps[i] != 0) {
          motors[i]->setMaxSpeed(params[i].maxSpeed);
          motors[i]->setAcceleration(params[i].acceleration);
        }
      }

      for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
        currentPositions[j] = positions[j];
      }
      delete[] params;

    }
    
    bool run() {
      if (!isRunning) {
        for (int i = 0; i < motorCount; i++) {
          motors[i]->restartTime();
        }
        isRunning = true;
      }
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
      if (!anyBelowThreshold && anyRunning) { 
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

    void setMinSpeed(float newMinSpeed) { 
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setMinSpeed(newMinSpeed);
      }
    }
};

QuickStepper stepperX(0, 1);
QuickStepper stepperY(2, 3);
QuickStepper stepperZ(4, 5);

MultiStepper steppers;

void setup() {
  //const float STEPS_PER_CM = 110.65;
  const float STEPS_PER_CM = 20;
  Serial.begin(9600);
  // put your setup code here, to run once:
  steppers.addMotor(&stepperX);
  steppers.addMotor(&stepperY);
  steppers.addMotor(&stepperZ);
  steppers.setMaxSpeed(2 * STEPS_PER_CM); // 20
  steppers.setAcceleration(2); // 100
  steppers.setMinSpeed(2); // 100
  steppers.setCornering(0.25);

  stepperX.setLabel('X');
  stepperY.setLabel('Y');
  stepperZ.setLabel('Z');

  long positions[] = {0, 0, 1 * STEPS_PER_CM};
  long positions2[] = {2 * STEPS_PER_CM, 2 * STEPS_PER_CM, 3 * STEPS_PER_CM};
  long positions3[] = {2 * STEPS_PER_CM, 2 * STEPS_PER_CM, 2 * STEPS_PER_CM};
  steppers.moveTo(positions);
  steppers.chainTo(positions2);
  steppers.chainTo(positions3);

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
