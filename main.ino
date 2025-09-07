#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>

class MotorPID {
  public:
    // Retrieve Given Motor
    MotorPID(NoU_Motor& givenMotor) : myMotor(givenMotor) {
      // hold
    }
    // PID constants
    float Kp;
    float Ki;
    float Kd;

    void setZieglerNicholsConstants(float Ku, float Tu) {
      // Ziegler-Nichols PID formulas
      Kp = 0.6 * Ku;
      Ki = (1.2 * Ku) / Tu;
      Kd = 0.075 * Ku * Tu;
    }

    void updateMotor() {
    if (millis() - lastPIDTime >= PIDInterval) {
      // Read the current angle from your encoder
      float currentAngle = myMotor.getPosition();

      // ** Step 1: Calculate the Error **
      float error = targetAngle - currentAngle;
      Serial.println(currentAngle);
      Serial.println(targetAngle);

      // ** Step 2: Calculate the PID terms **
      float proportionalTerm = Kp * error;

      // Integral term: sum of all past errors
      integralSum += error;
      float integralTerm = Ki * integralSum;

      // Derivative term: rate of change of the error
      float derivativeTerm = Kd * (error - lastError);

      // ** Step 3: Calculate the total PID output **
      float motorSpeedOutput = proportionalTerm + integralTerm + derivativeTerm;

      // ** Step 4: Constrain and set the motor speed **
      // Constrain the output to a valid range for your motor driver (e.g., -255 to 255)
      motorSpeedOutput = constrain(motorSpeedOutput, -1, 1);
      myMotor.set(-motorSpeedOutput);

      // ** Step 5: Update variables for the next iteration **
      lastError = error;
      lastPIDTime = millis();
    } // Time check closing bracket
  } // updateMotor(); closing bracket

  void setAngle(float givenAngle) {
    targetAngle = givenAngle;
  }

  private:
    // Initalize Given Motor
    NoU_Motor& myMotor;

    // Target Angle Adjustment
    float targetAngle = 0;

    // PID variables  
    float lastError = 0.0;
    float integralSum = 0.0;

    //PID timing
    unsigned long lastPIDTime = 0;
    const unsigned long PIDInterval = 20; // time in ms
};

// Turn Motor Initialization
NoU_Motor FLTurnMotor(3);
NoU_Motor FRTurnMotor(6);
NoU_Motor BLTurnMotor(4);
NoU_Motor BRTurnMotor(5);

// Drive Motor Initalization
NoU_Motor FLDriveMotor(1);
NoU_Motor FRDriveMotor(8);
NoU_Motor BLDriveMotor(2);
NoU_Motor BRDriveMotor(7);

// Turn Motor PID Initalization
MotorPID FLTurnMotorPID(FLTurnMotor);
MotorPID FRTurnMotorPID(FRTurnMotor);
MotorPID BLTurnMotorPID(BLTurnMotor);
MotorPID BRTurnMotorPID(BRTurnMotor);

void setup() {
  Serial.begin(115200);
  PestoLink.begin("Voyager");
  NoU3.begin();

  // Start Encoder Readings
  FLTurnMotor.beginEncoder();
  FRTurnMotor.beginEncoder();
  BLTurnMotor.beginEncoder();
  BRTurnMotor.beginEncoder();

  FLTurnMotorPID.Kp = FRTurnMotorPID.Kp = BLTurnMotorPID.Kp = BRTurnMotorPID.Kp = 0.0019;
  FLTurnMotorPID.Ki = FRTurnMotorPID.Ki = BLTurnMotorPID.Ki = BRTurnMotorPID.Ki = 0.00000;
  FLTurnMotorPID.Kd = FRTurnMotorPID.Kd = BLTurnMotorPID.Kd = BRTurnMotorPID.Kd = 0.0044;

  //sigmaMotor.Kp = 0.0045;
  //sigmaMotor.Ki = 0.00005;
  //sigmaMotor.Kd = 0.01;

  //sigmaMotor.Kp = 0.0011;
  //sigmaMotor.Ki = 0;
  //sigmaMotor.Kd = 0.005;

  FLTurnMotorPID.setAngle(360);
  FRTurnMotorPID.setAngle(360);
  BLTurnMotorPID.setAngle(360);
  BRTurnMotorPID.setAngle(360);

}

// The loop() function runs continuously
void loop() {
  float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);
  // Check if a command is available from the serial port

  //FLTurnMotorPID.updateMotor();
  //FRTurnMotorPID.updateMotor();
  //BLTurnMotorPID.updateMotor();
  //BRTurnMotorPID.updateMotor();

  if (PestoLink.keyHeld(Key::E)) {
    FLDriveMotor.set(1);
    FRDriveMotor.set(1);
    BLDriveMotor.set(1);
    BRDriveMotor.set(1);
  } else {
    FLDriveMotor.set(0);
    FRDriveMotor.set(0);
    BLDriveMotor.set(0);
    BRDriveMotor.set(0);
  }

  if (PestoLink.keyHeld(Key::Q)) {
    FLTurnMotor.set(1);
    FRTurnMotor.set(1);
    BLTurnMotor.set(1);
    BRTurnMotor.set(1);
  } else {
    FLTurnMotor.set(0);
    FRTurnMotor.set(0);
    BLTurnMotor.set(0);
    BRTurnMotor.set(0);
  }
  //char yaw[20];
  //dtostrf(encoderedN20.getPosition(), 8, 2, yaw);
  //PestoLink.printTerminal(yaw);
}
