#include <assert.h>

#define LONG_MAX 2147483647L
#define LONG_MIN -2147483648L
#define PUMP_PIN 6

// Global variables

bool isSucking = false;

struct MotorInfo {
  float acceleration;
  float maxSpeed;
  float minSpeed;
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

struct MotorInitParams {
  // This is used for more complex starting of motors, especially useful when cornering
  long desiredSpeed; // steps/second
  bool fitToSpeedStart; // accel or brake to go to desiredSpeed when this run begins
  bool fitToSpeedEnd; // accel or brake to go to desiredSpeed when this run ends
};

struct MotorData {
  enum Type { LONG, MOTOR_PARAMS} type;
  union {
    long position;
    MotorInitParams initParams;
  } data;
};

class QuickStepper {
  private:
    const int stepPin;
    const int dirPin;

    float acceleration = 10.0; // steps/s^2
    float maxSpeed = 1000.0; // steps/s
    float minSpeed = 10.0; // steps/s
    float brakeAcceleration = 0;

    float currentSpeed = 0.0;  // steps/second
    unsigned long stepDelay;   // microseconds
    unsigned long lastStepTime = 0;
    unsigned long constantPhaseStart = 0;
    long totalSteps = 0;
    unsigned long constantDuration = 0;
    bool isRunning = false;
    long stepsSinceStart;
    long stepsToAccelerate;
    float middleTotalSteps;
    long currentPosition;
    long upperLimit = LONG_MAX;
    long lowerLimit = -LONG_MAX;
    int positiveDirection = 1;
    int prevPositiveDirection = 1;
    bool keepConstantSpeed = false;
    bool allowFastRecovery = true;
    float allowFastRecoveryFactor = 2.5;
    bool catchingUpMode = false;
    long stepDelayBuffer = 0;
    int hasSkippedIndex = 0;
    bool inComplexInitPhase = false;
    MotorData complexInitParams = nullptr;

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

    long calculateAccelerationSteps() {
      // Break down the calculation to prevent overflow
      // Original: s = (v² - v₀²)/(2a)
      
      // First calculate each squared term separately
      long maxSpeedSquared = maxSpeed * maxSpeed;
      long minSpeedSquared = minSpeed * minSpeed;
      
      // Then divide by 2a before subtracting to keep numbers smaller
      long stepsToAccelerate = (maxSpeedSquared/(2.0 * acceleration)) - (minSpeedSquared/(2.0 * acceleration));
      
      // Check for overflow before converting to long
      if (stepsToAccelerate > LONG_MAX) {
          Serial.println("Warning: Acceleration distance overflow!");
          return LONG_MAX;
      }
      /*Serial.print(" --- Calc Accel Steps for ");
      Serial.println(motorLabel);
      Serial.println(stepsToAccelerate);*/
      return long(stepsToAccelerate);
    }

    void proceed() {
      setInComplexInitPhase(false);
    }

    void moveToComplex(MotorData dest) {
      setInComplexInitPhase(true);
      complexInitParams = dest;
    }

    void moveTo(MotorData dest) { // dest = total steps to run
      if (dest.Type == MotorData::MOTOR_PARAMS) { moveToComplex(dest); return; }
      if (inComplexInitPhase) {Serial.println("ERROR: In constant phase but moveTo is called with position"); while(1);}
      long prevTotalSteps = totalSteps;
      totalSteps = dest - currentPosition;
      middleTotalSteps = int(dest - prevTotalSteps / 2);
      stepsSinceStart = 0;
      setDirection(totalSteps);
      if (!isRunning) {
        isRunning = true;
        currentSpeed = minSpeed;
        lastStepTime = micros();
        stepDelay = 1;
      }
      /*Serial.print("Called MoveTo On ");
      Serial.println(motorLabel);
      Serial.println(totalSteps);
      Serial.println(stepsToAccelerate);
      Serial.println(currentPosition);
      Serial.println(currentSpeed);
      Serial.println(middleTotalSteps);*/
    }

    void restart() {
      lastStepTime = micros();
      stepsToAccelerate = calculateAccelerationSteps();
    }

    void stop() {
      isRunning = false;
      brakeAcceleration = 0;
      stepDelayBuffer = 0;
    }

    bool running() {
      return isRunning;
    }

    void setAcceleration(float newAcceleration) { 
      acceleration = newAcceleration; 
      stepsToAccelerate = calculateAccelerationSteps();
    }

    void setLimits(long newLowerLimit, long newUpperLimit) {
      lowerLimit = newLowerLimit;
      upperLimit = newUpperLimit;
    }

    void setMaxSpeed(float newMaxSpeed) { maxSpeed = newMaxSpeed; }
    void setMinSpeed(float newMinSpeed) { minSpeed = newMinSpeed; }
    void setPosition(float position) { currentPosition = position; }
    void setKeepConstantSpeed(bool newKeepConstantSpeed) { keepConstantSpeed = newKeepConstantSpeed; }
    void setAllowFastRecovery(bool newAllowFastRecovery) { allowFastRecovery = newAllowFastRecovery; }
    void setAllowFastRecoveryFactor(float newAllowFastRecoveryFactor) { allowFastRecoveryFactor = newAllowFastRecoveryFactor; }
    void setInComplexInitPhase(bool newInComplexInitPhase) { inComplexInitPhase = newInComplexInitPhase; complexInitParams = nullptr; }
    bool getInComplexInitPhase() { return inComplexInitPhase; }
    bool getKeepConstantSpeed() { return keepConstantSpeed; }
    float getPercentageDone() {return static_cast<float>(stepsSinceStart) / static_cast<float>(totalSteps); }
    float getAcceleration() {return acceleration; }
    float getMaxSpeed() {return maxSpeed; }
    int getIsRunning() {return isRunning; }
    char getMotorLabel() {return motorLabel; }
    float getCurrentSpeed() {return currentSpeed; }
    
    

    MotorInfo getInfo() {
      MotorInfo info = {
        acceleration,
        maxSpeed,
        minSpeed,
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

    void setDirection(long dest) {
      if (dest < 0) {
        positiveDirection = 0;
        digitalWrite(dirPin, LOW);
      } else {
        positiveDirection = 1;
        digitalWrite(dirPin, HIGH);
      }
      if (prevPositiveDirection != positiveDirection) {
        currentSpeed = -currentSpeed;
        middleTotalSteps = 0;
        totalSteps = -totalSteps;
        //stepsSinceStart = 0;
      }
      prevPositiveDirection = positiveDirection;
    }

    void stepMotor() {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1); // This could cause some issues
      digitalWrite(stepPin, LOW);
    }
  
  long calculateStepsDifference(float v1, float v0, float a1) {
    const int MAX_ITERATIONS = 1000;
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      if (v1 <= v0) return i;
      long tempStepDelay = 1000000.0 / abs(v1);
      v1 -= a1 * (tempStepDelay / 1000000.0);
    }
    return MAX_ITERATIONS;
  }
  
  public:
    bool run_complex() {
      // cannot deaccellerate, just have to wait until proceed is called and then will corner to the right place.
      unsigned long currentMicros = micros();
      if (currentMicros - lastStepTime + stepDelayBuffer >= stepDelay) {
        stepDelayBuffer += currentMicros - lastStepTime - stepDelay;
        Serial.print(motorLabel);
        if (currentSpeed < 0) {
          if (positiveDirection == 1) {
            Serial.print(0);
          }
          else {
            Serial.print(1);
          }
        } else {
          Serial.print(positiveDirection);
        }
        if (isSucking) {
          Serial.println(1);
        } else {
          Serial.println(0);
        }
        Serial.println(currentMicros);
        Serial.println(currentSpeed);
        stepMotor();
        if (currentSpeed > 0) {
          currentPosition++;
          assert(currentPosition < upperLimit);
        } else {
          currentPosition--;
          assert(currentPosition > lowerLimit);
        }
        lastStepTime = currentMicros;
        if (currentSpeed > complexInitParams.desiredSpeed) {
          currentSpeed -= acceleration * (stepDelay / 1000000.0);
          currentSpeed = max(currentSpeed, complexInitParams.desiredSpeed);
        }
        else if (currentSpeed < complexInitParams.desiredSpeed) {
          currentSpeed += acceleration * (stepDelay / 1000000.0);
          currentSpeed = min(currentSpeed, complexInitParams.desiredSpeed);
        }
      }
      return true;
    };

    bool run(long stepsBehind) {
      if (!isRunning) return;
      assert(hasSkippedIndex < 5);
      if (stepsBehind > 2) { // buffer to make sure it is enough to accelerate for
        catchingUpMode = true;
        //Serial.print("CATCHING UP MODE ACTIVATE FOR ");
        //Serial.println(motorLabel);
      }

      unsigned long currentMicros = micros();

      if (stepsSinceStart >= abs(totalSteps)) {
        stop();
        /*Serial.print(motorLabel);
        Serial.println(positiveDirection);
        Serial.println(stepsSinceStart);*/
        return false;
      }

      if (currentMicros - lastStepTime + stepDelayBuffer >= stepDelay) {
        hasSkippedIndex += 1;
        stepDelayBuffer += currentMicros - lastStepTime - stepDelay;
        Serial.print(motorLabel);
        if (currentSpeed < 0) {
          if (positiveDirection == 1) {
            Serial.print(0);
          }
          else {
            Serial.print(1);
          }
        } else {
          Serial.print(positiveDirection);
        }
        if (isSucking) {
          Serial.println(1);
        } else {
          Serial.println(0);
        }
        Serial.println(currentMicros);
        Serial.println(currentSpeed);
        //Serial.println((totalSteps - stepsSinceStart));
        //Serial.println(stepsToAccelerate);
        // Serial.println(((totalSteps - stepsSinceStart) - stepsToAccelerate));
        stepMotor();
        if (currentSpeed > 0) {
          currentPosition++;
          stepsSinceStart++;
          assert(currentPosition < upperLimit);
        } else {
          currentPosition--;
          stepsSinceStart--;
          assert(currentPosition > lowerLimit);
        }
        lastStepTime = currentMicros;

        /*if (currentSpeed > -minSpeed && currentSpeed < minSpeed) {
          currentSpeed = minSpeed;
            = abs((totalSteps - stepsSinceStart) / 2);
        }

        else if (currentSpeed < maxSpeed 
            && ((!(stepsSinceStart >= middleTotalSteps)) || middleTotalSteps == 0)
            && ((totalSteps - stepsSinceStart) - stepsToAccelerate) < 0)
        {
          currentSpeed += acceleration * (stepDelay / 1000000.0);
          if (currentSpeed >= maxSpeed) {
            currentSpeed = maxSpeed;
          }
        }
        else if (
            currentSpeed > maxSpeed 
            || (stepsSinceStart > middleTotalSteps)
            || ((totalSteps - stepsSinceStart) - stepsToAccelerate) < 0) 
        {
          currentSpeed -= acceleration * (stepDelay / 1000000.0);
          currentSpeed = max(currentSpeed, minSpeed);
        }
      
        stepDelay = 1000000.0 / abs(currentSpeed);*/

        // Calculate remaining distance
        long remainingSteps = abs(totalSteps - stepsSinceStart);
        
        // Calculate stopping distance at current speed
        float stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * acceleration);

        //if (catchingUpMode) {
        if (false) {
          float stepsRequiredToDecellerate = calculateStepsDifference(currentSpeed, maxSpeed, acceleration);
          /*Serial.print("CATCHING UP MODE ACTIVE FOR ");
          Serial.println(motorLabel);
          Serial.println(currentSpeed);
          Serial.println(stepsRequiredToDecellerate);
          Serial.println(stepsBehind);*/
          if (stepsBehind > stepsRequiredToDecellerate) {
            currentSpeed += acceleration * (stepDelay / 1000000.0);
            currentSpeed = min(currentSpeed, maxSpeed * 1.5);
          } else if (stepsBehind > 1) {
            currentSpeed -= brakeAcceleration * (stepDelay / 1000000.0);
            currentSpeed = max(currentSpeed, minSpeed);
          } else {
            //Serial.print("DISABLING CATCHING UP MODE");
            currentSpeed = maxSpeed;
            catchingUpMode = false;
            //Serial.print("CATCHING UP MODE DEACTIVATE FOR ");
            //Serial.println(motorLabel);
          }
        }
        if (keepConstantSpeed) {
          //Serial.println("keep");
          return true;
        }
        // If we're decelerating, don't allow acceleration changes
        else if (remainingSteps <= stoppingDistance) {
          //Serial.println("break");
          if (brakeAcceleration == 0) {
            brakeAcceleration = acceleration; 
            if (allowFastRecovery && currentSpeed < 0) { brakeAcceleration *= allowFastRecoveryFactor; }
          };
            // We need to decelerate, but in which direction?
          if (currentSpeed > 0) {
            currentSpeed -= brakeAcceleration * (stepDelay / 1000000.0);
            currentSpeed = max(currentSpeed, minSpeed);
          } else {
            currentSpeed += brakeAcceleration * (stepDelay / 1000000.0);
          }
        }
        // Normal acceleration phase
        else if (currentSpeed < maxSpeed 
        && ((!(stepsSinceStart >= middleTotalSteps)) || middleTotalSteps == 0) 
        && remainingSteps > stoppingDistance) 
        {
          //Serial.println("go brr");
          currentSpeed += acceleration * (stepDelay / 1000000.0);
          currentSpeed = min(currentSpeed, maxSpeed);
        }
        else {
          //Serial.println("dafuq");
          /*Serial.print("NOTHING HAPPENED FOR ");
          Serial.println(motorLabel);
          Serial.println(remainingSteps > stoppingDistance);
          Serial.println(stepsSinceStart);
          Serial.println(middleTotalSteps);*/
        }
        if (currentSpeed > -minSpeed && currentSpeed < minSpeed) {
          //Serial.println("switch");
          currentSpeed = minSpeed;
          middleTotalSteps = abs((totalSteps - stepsSinceStart) / 2);
          brakeAcceleration = 0;
        }
        stepDelay = 1000000.0 / abs(currentSpeed);

      } else {
        hasSkippedIndex = 0;
      }
      return true;
    }
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
    PositionData* futurePositions[MAX_CHAINS][COORDINATE_DIMENSIONS];
    int futurePositionsIndex = 0;
    bool isRunning = false;
    bool allowFastRecovery = true;
    int leadMotor = -1; // Motor that has the lead on steps for current running chain
    long* stepsBehind[COORDINATE_DIMENSIONS];
    int chainIndex = 0;

    // Chaining
    int stepsLeftWhenCornering = 10;


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

    int getChainIndex() {
      return chainIndex;
    }

    void setStepsLeftWhenCornering(float newStepsLeftWhenCornering) {
      stepsLeftWhenCornering = newStepsLeftWhenCornering;
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
        if (steps[i] != 0) {
          if (leadMotor == -1) {leadMotor = i; }
          motors[i]->setMaxSpeed(params[i].maxSpeed);
          motors[i]->setAcceleration(params[i].acceleration);
        }
      }

      delete[] params;

    }

    void chainTo(long* positions, MotorInitParams* initParams = nullptr) {
      // Will always overwrite positions if MotorInitParams is given;
      futurePositions[i][j] = new PositionData();
      else if (positions == nullptr) {
          futurePositions[i][j]->type = PositionData::LONG;
          futurePositions[i][j]->data.initParams = initParams;
      } else {
          futurePositions[i][j]->type = PositionData::MOTOR_PARAMS;
          futurePositions[i][j]->data.position = positions;
      }
      futurePositionsIndex++;
    }

    void reset() {
      for (int i = 0; i < MAX_CHAINS; i++) {
        for (int j = 0; j < COORDINATE_DIMENSIONS; j++) {
          if (futurePositions[i][j] != nullptr) {
              delete futurePositions[i][j];  // Free the allocated memory
              futurePositions[i][j] = nullptr;  // Set pointer to null
          }
        }
      }
      futurePositionsIndex = 0;
      isRunning = false;
      leadMotor == -1;
      chainIndex = 0;
      for (int i = 0; i < COORDINATE_DIMENSIONS; i++) {
        stepsBehind[i] = 0;
      }
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
      chainIndex++;
      
      // Create local array to store the values
      MotorData positions[COORDINATE_DIMENSIONS];
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
        int stepsLeft = motors[i]->getInfo().totalSteps - motors[i]->getInfo().stepsSinceStart;
        motors[i]->moveTo(positions[i]);
        //motors[i]->setOffsetFromPreviousRoute(stepsLeft);
        steps[i] = motors[i]->getInfo().totalSteps;// - abs(stepsLeft);
        stepsBehind[i] = stepsLeft;
        if (stepsLeft == 0) {
          leadMotor = i;
        }
      }
      bool anyOtherMotorsPositiveSteps[MAX_MOTORS];
      for (int i = 0; i < motorCount; i++) {
        anyOtherMotorsPositiveSteps[i] = false;
        for (int j = 0; j < motorCount; j++) {
          if (i == j) continue;

          if (steps[j] > 0) {
            anyOtherMotorsPositiveSteps[i] = true;
          }
        }
        if (!anyOtherMotorsPositiveSteps[i]) {
          leadMotor = i;
        }
      }

      MotorParams* params = calculateMotorParams(steps, motorCount);

      for (int i = 0; i < motorCount; i++) {
        if (steps[i] != 0) {
          motors[i]->setMaxSpeed(params[i].maxSpeed);
          motors[i]->setAcceleration(params[i].acceleration);
          /*if (
            motors[i]->getCurrentSpeed() != motors[i]->getInfo().minSpeed
            && anyOtherMotorsPositiveSteps[i]
          ) {

            //motors[i]->setKeepConstantSpeed(true);
          }*/
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
          motors[i]->restart();
        }
        isRunning = true;
      }
      bool anyInComplexPhase = false;
      bool anyRunning = false;
      float lowestSpeed = LONG_MAX;
      for (int i = 0; i < motorCount; i++) {
        if (motors[i]->running()) {
          float currentSpeed = abs(motors[i]->getCurrentSpeed());
          if (currentSpeed < lowestSpeed) {
            lowestSpeed = currentSpeed;
          }
        }
      }

      bool anyBelowThreshold = false;
      for (int i = 0; i < motorCount; i++) {
        long differenceToLeadMotor = max(
          motors[leadMotor]->getInfo().stepsSinceStart + stepsBehind[i] - motors[i]->getInfo().stepsSinceStart
        , 0);
        /*Serial.print("DIFFERENCE TO LEAD MOTOR FOR ");
        Serial.print(motors[i]->getMotorLabel());
        Serial.print(" IS ");
        Serial.println(differenceToLeadMotor);
        Serial.println(leadMotor);
        Serial.println(i);
        Serial.println(motors[leadMotor]->getInfo().stepsSinceStart);
        Serial.println(motors[i]->getInfo().stepsSinceStart);
        Serial.println(*stepsBehind[i]);*/

        if (motors[i]->getInComplexInitPhase()) {
          anyInComplexPhase = true;
          motors[i]->run_complex();
        }

        else if (motors[i]->run(differenceToLeadMotor)) {
          anyRunning = true;
          if (abs(motors[i]->getInfo().totalSteps - motors[i]->getInfo().stepsSinceStart) > stepsLeftWhenCornering ) {
            anyBelowThreshold = true;  // One motor is below threshold
          }
        }
        if (motors[i]->getKeepConstantSpeed()) {
          if (abs(motors[i]->getCurrentSpeed()) <= lowestSpeed) {
            motors[i]->setKeepConstantSpeed(false);
          }
        }
      }
      if (!anyBelowThreshold && anyRunning && !anyInComplexPhase) { 
        startCornering(); 
      }
      if (!anyRunning) { reset(); }
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

    void setMinSpeed(float newMinSpeed) { // Should be 1/10 of maxSpeed
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setMinSpeed(newMinSpeed);
      }
    }

    void setAllowFastRecovery(bool newAllowFastRecovery) {
      // When motor has to change direction, we up the acceleration drastically
      // Only allow if your motors can handle it
      // If False this could mean that your motors could overshoot ~20%
      // This will also allow the motors to go up to 1.5 the max speed to recover
      // positions better and avoid overshooting while cornering
      allowFastRecovery = newAllowFastRecovery;
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setAllowFastRecovery(newAllowFastRecovery);
      }
    }

    void setAllowFastRecoveryFactor(float newAllowFastRecoveryFactor) {
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setAllowFastRecoveryFactor(newAllowFastRecoveryFactor);
      }
    }
};

// Bonus functions for extra capabality (should not be a part of QuickStepper)

bool limitSwitchIsOn(long z_pos, long z_lim) {
  // refactor when in production
  if (z_pos > z_lim) return true;
  return false;
}

void startSucking() {
  digitalWrite(PUMP_PIN, HIGH);
}

void stopSucking() {
  digitalWrite(PUMP_PIN, LOW);
}

QuickStepper stepperX(0, 1);
QuickStepper stepperY(2, 3);
QuickStepper stepperZ(4, 5);

MultiStepper steppers;

void setup() {

  // Maybe test with much easier path to begin with?

  //const float STEPS_PER_CM = 110.65;
  const float STEPS_PER_CM = 50;
  // const int LIMIT_SWITCH_PIN = 0; // maybe as define instead
  
  pinMode(PUMP_PIN, OUTPUT);

  Serial.begin(9600);
  // put your setup code here, to run once:
  steppers.addMotor(&stepperX);
  steppers.addMotor(&stepperY);
  steppers.addMotor(&stepperZ);
  steppers.setMaxSpeed(15); // 20
  steppers.setAcceleration(5); // 100 // no less than 1/3 of maxSpeed
  steppers.setMinSpeed(3); // 100 // should be above 5 no matter what. 
  // If not you risk getting caught waiting too long for a step to take place
  steppers.setStepsLeftWhenCornering(10); // make it about 20% of smallest axis
  // by testing, this seems to give the best results.
  steppers.setAllowFastRecovery(true);
  steppers.setAllowFastRecoveryFactor(3.0);

  stepperX.setLabel('X');
  stepperY.setLabel('Y');
  stepperZ.setLabel('Z');

  // CHAIN 1
  long positions[] = {0, 0, 1 * STEPS_PER_CM};
  long positions2[] = {4 * STEPS_PER_CM, 6 * STEPS_PER_CM, 5 * STEPS_PER_CM};
  long positions3[] = {4 * STEPS_PER_CM, 6 * STEPS_PER_CM, 4 * STEPS_PER_CM};
  // long speeds3[] = {8, 8, 8}; // but z has to go this speed and go down like what the fuck
  MotorInitParams motorSpeeds3[3] = { // x, y, z
    {8.0, true, false},
    {8.0, true, false},
    {8.0, true, false}
  };

  steppers.moveTo(positions);
  steppers.chainTo(positions2);
  steppers.chainTo(nullptr, motorSpeeds3);

  // 0 is fully down. Testing required to fint the correct limits
  long z_pickup_limit = STEPS_PER_CM * 1;

  /*long positions[] = {0, 0, 1 * STEPS_PER_CM};
  long positions2[] = {2 * STEPS_PER_CM, 2 * STEPS_PER_CM, 3 * STEPS_PER_CM};
  long speeds[] = {20, 20, 0};
  long positions3[] = {2 * STEPS_PER_CM, 2 * STEPS_PER_CM, 2 * STEPS_PER_CM};
  steppers.moveTo(positions);
  steppers.chainTo(positions2, speeds);
  steppers.chainTo(positions3);*/

  /*Serial.println("X");
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
  Serial.println("-----------");*/

  bool isSucking = false;

  while(steppers.run()) {
    if (!isSucking && steppers.getChainIndex() == 2) {
      startSucking();
    }
    if (
      steppers.getChainIndex() == 2
      && stepperX.getInConstantPhase() 
      && limitSwitchIsOn(stepperX.getInfo().currentPosition - stepperZ.getInfo().currentPosition, z_pickup_limit)
    ) {// This if is maybe too slow
      stepperZ.stop();
      // wait a bit here maybe to catch on to the item
      stepperX.proceed();
      stepperY.proceed();
      stepperZ.proceed();
      break;
    }
  }
  long curX = stepperX.currentPosition;
  long curY = stepperY.currentPosition;

  long positions4[] = {curX, curY, curX + 1};
  long positions5[] = {10 * STEPS_PER_CM, 1 * STEPS_PER_CM, 9.5 * STEPS_PER_CM};
  long positions6[] = {0 * STEPS_PER_CM, 0 * STEPS_PER_CM, 1 * STEPS_PER_CM};
  steppers.moveTo(positions4);
  steppers.chainTo(positions5);
  steppers.chainTo(positions6);

  bool checkComplete = false;

  while (steppers.run()) {
    if (!checkComplete && steppers.getChainIndex() == 6) {
      stopSucking();
      checkComplete = true;
    }
  }

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
