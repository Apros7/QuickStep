class QuickStepper {
  private:
    const int stepPin;
    const int dirPin;

    float acceleration = 10.0; // steps/s^2
    float maxSpeed = 1000.0; // steps/s

    float currentSpeed = 0.0;  // steps/second
    unsigned long stepDelay;   // microseconds
    unsigned long lastStepTime = 0;
    unsigned long constantPhaseStart = 0;
    long totalSteps = 0;
    bool inConstantPhase = false;
    bool deceleration = false;
    unsigned long constantDuration = 0;
    bool isRunning = false;
    unsigned int stepsSinceStart;
    unsigned int stepsToAccelerate;
    float middleTotalSteps;
    unsigned int currentPosition;
  
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
      Serial.println("DESTINATION");
      Serial.println(currentPosition);
      Serial.println("To move:");
      Serial.println(totalSteps);
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

    void setConfig(float setAcceleration, float setMaxSpeed) {
      acceleration = setAcceleration;
      maxSpeed = setMaxSpeed;
    }
  
  private:

    void setDirection(long dest) {
      if (dest < 0) {
        digitalWrite(dirPin, LOW);
      } else {
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
        return false;
      }

      if (currentMicros - lastStepTime >= stepDelay) {
        // Serial.println(currentMicros);
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
            stepsToAccelerate = stepsSinceStart;
          }
        }
        else if (inConstantPhase) {
          if (totalSteps - stepsToAccelerate < 0) {
            inConstantPhase = false;
            deceleration = true;
          }
        }
        else if (deceleration) {
          currentSpeed -= acceleration * (stepDelay / 1000000.0);
          if (currentSpeed <= 0 || stepsSinceStart >= totalSteps) {
            stop();
            Serial.println(stepsSinceStart);
            return false;
          }
        }
      
        stepDelay = 1000000.0 / currentSpeed;
      };
      return true;
    };
};

class MultiQuickStepper {
  private:
    static const int MAX_MOTORS = 10;
    QuickStepper* motors[MAX_MOTORS];
    int motorCount = 0;
  
  public:
    MultiQuickStepper() {}
    
    bool addMotor(QuickStepper* motor) {
      if (motorCount >= MAX_MOTORS) {
        return false;  // Cannot add more motors
      }
      motors[motorCount++] = motor;
      return true;
    }
    
    void moveTo(long* positions) {
      for (int i = 0; i < motorCount; i++) {
        motors[i]->moveTo(positions[i]);
      }
    }
    
    bool run() {
      bool anyRunning = false;
      for (int i = 0; i < motorCount; i++) {
        if (motors[i]->run()) {
          anyRunning = true;
        }
      }
      return anyRunning;
    }
    
    void setConfig(float acceleration, float maxSpeed) {
      for (int i = 0; i < motorCount; i++) {
        motors[i]->setConfig(acceleration, maxSpeed);
      }
    }
};

QuickStepper stepperX(0, 1);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  stepperX.moveTo(1000);
  while (stepperX.run()) {

  }
  Serial.println("done");
  delay(1000);
  // put your main code here, to run repeatedly:

}
