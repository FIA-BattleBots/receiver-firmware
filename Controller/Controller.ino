#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <ESP32Servo.h>
#include <CodeCell.h>

// Define pins
#define L_DRIVE_PIN 5
#define R_DRIVE_PIN 6
#define R_WPN_PIN 2
#define L_WPN_PIN 3

// Shorthand for the Xbox controller
#define A xboxNotif.btnA
#define B xboxNotif.btnB
#define X xboxNotif.btnX
#define Y xboxNotif.btnY
#define L1 xboxNotif.btnLB
#define R1 xboxNotif.btnRB
#define L2 xboxNotif.trigLT
#define R2 xboxNotif.trigRT
#define axisX xboxNotif.joyLHori
#define axisY xboxNotif.joyLVert
#define axisRX xboxNotif.joyRHori
#define axisRY xboxNotif.joyRVert
#define btnShare xboxNotif.btnShare
#define btnSelect xboxNotif.btnSelect

// Define more buttons here if needed...

// Controller Limits
#define TRIGGER_MIN 0     // DON'T CHANGE
#define TRIGGER_MAX 1023  // DON'T CHANGE
#define STICK_MIN 0       // DON'T CHANGE
#define STICK_MID 32767   // DON'T CHANGE
#define STICK_MAX 65534   // DON'T CHANGE
#define STICK_DEADZONE 4096 
#define STICK_BIAS 0  // Offset the default stick position

// Weapon, throttle and steering limits
#define WPN_MIN -100.0  // Full speed in reverse
#define WPN_OFF 0.0     // Turn off weapon
#define WPN_LOW 25.0    // Low speed for maneuveing
#define WPN_MID 50.0    // Mid speed for max damage
#define WPN_HIGH 75.0   // High speed for charging
#define WPN_MAX 100.0   // Full speed for weapon-on-weapon hits
#define THROTTLE_MIN -100.0 // DON'T CHANGE
#define THROTTLE_OFF 0.0    // DON'T CHANGE
#define THROTTLE_MAX 100.0  // DON'T CHANGE
#define STEER_MIN -100.0    // DON'T CHANGE
#define STEER_OFF 0.0       // DON'T CHANGE
#define STEER_MAX 100.0     // DON'T CHANGE
#define INVERT_LEFT_WHEEL  1  // For inverting left wheel direction
#define INVERT_RIGHT_WHEEL 0  // For inverting right wheel direction
#define INVERT_LEFT_WPN    1  // For inverting left weapon direction
#define INVERT_RIGHT_WPN   0  // For inverting right weapon direction

// PWM Settings
#define PWM_MIN 1000  // DON'T CHANGE
#define PWM_MID 1500  // DON'T CHANGE
#define PWM_MAX 2000  // DON'T CHANGE

// Robot states
#define COMBAT_MODE 0
#define PAIRING_MODE 1
#define FLASHING_MODE 2
#define ASSISTED_COMBAT_MODE 3

// Tuning
#define DRIVE_SPEED_MODIFIER 1.0    // [0, 1]
#define TURN_SPEED_MODIFIER 0.5     // [0, 1]
#define UPDATE_DELAY 10             // Unit: ms
#define K_P 2.0
#define K_I 0.0
#define K_D 5.0
#define ROT_LOCK_THRESHOLD 0.1      // Unit: rad/s
#define CONTROL_AUTHORITY 100.0     // How much the offset can affect the turning

// Global variables :)
int weaponSpeed = WPN_OFF;
int weaponIdleSpeed = WPN_OFF;
int throttle = THROTTLE_OFF;
int steer = STEER_OFF;

bool sharePressHandled = false;
bool selectPressHandled = false;

int mode;
float gyroX = 0.0;    // Unit: rad/s
float gyroY = 0.0;    // Unit: rad/s
float gyroZ = 0.0;    // Unit: rad/s, this is the one we are interested in for turning the robot

float rotX = 0.0;    // Unit: Degrees
float rotY = 0.0;    // Unit: Degrees
float rotZ = 0.0;    // Unit: Degrees, this is the one we are interested in for turning the robot

bool rotLocked = false;
float tgtRotZ = 0.0;
float mix_L; // Mixed steering for left drive to be converted to PWM
float mix_R;
float prevError = 0.0;
float errorIntegral = 0.0;

Servo lDriveESC;
Servo rDriveESC;
Servo lWpnESC;
Servo rWpnESC;

CodeCell myCodeCell;
XboxSeriesXControllerESP32_asukiaaa::Core ctl;



void dumpGamepad() {
    Serial.println("Address: " + ctl.buildDeviceAddressStr());
    Serial.println("battery " + String(ctl.battery) + "%");
    Serial.print(ctl.xboxNotif.toString());
}

void processGamepad() {
    // Select weapon idle speed. B=off, A=low, X=mid, Y=max
    if (ctl.B) {
        Serial.println("(B) Weapon Off");
        weaponIdleSpeed = WPN_OFF;
    } else if (ctl.A) {
        Serial.println("(A) Weapon Low");
        weaponIdleSpeed = WPN_LOW;
    } else if (ctl.X) {
        Serial.println("(X) Weapon Mid");
        weaponIdleSpeed = WPN_MID;
    } else if (ctl.Y) {
        Serial.println("(Y) Weapon High");
        weaponIdleSpeed = WPN_HIGH;
    }

    // Set weapon speed
    if (ctl.R1) {
        Serial.println("(R1) Overriding weapon idle speed");
        weaponSpeed = WPN_MAX; // Full forward weapon
    } else if (ctl.L1) {
        Serial.println("(L1) Overriding weapon idle speed (reverse)");
        weaponSpeed = WPN_MIN; // Full reverse weapon
    } else {
        weaponSpeed = weaponIdleSpeed; // No weapon
    }

    // Set throttle (I don't think deadzone is needed here)
    throttle = map(ctl.R2-ctl.L2, -TRIGGER_MAX, TRIGGER_MAX, THROTTLE_MIN, THROTTLE_MAX);

    // Set steering
    if (abs(ctl.axisX-STICK_MID) > STICK_DEADZONE) {
        int stick_pos = constrain(ctl.axisX + STICK_BIAS, STICK_MIN, STICK_MAX);
        steer = map(stick_pos, STICK_MIN, STICK_MAX, STEER_MIN, STEER_MAX);
        
    } else {
        steer = STEER_OFF;
    }

    // Flash mode toggle
    if (ctl.btnShare) {
        if (!sharePressHandled) {
            if (mode != FLASHING_MODE) {
                setMode(FLASHING_MODE);
            } else {
                setMode(COMBAT_MODE);
            }
        }
        sharePressHandled = true;
    } else {
        sharePressHandled = false;
    }

    // Assistance toggle
    if (ctl.btnSelect) {
        if (!selectPressHandled) {
            if (mode == COMBAT_MODE) {
                setMode(ASSISTED_COMBAT_MODE);
            } else if (mode == ASSISTED_COMBAT_MODE) {
                setMode(COMBAT_MODE);
            }
        }
        selectPressHandled = true;
    } else {
        selectPressHandled = false;
    }


    Serial.println("(INPUT) Throttle: " + String(throttle) + ", Steer: " + String(steer) + ", Weapon: " + String(weaponSpeed));
}

void readSensors() {
    myCodeCell.Motion_Read();
    myCodeCell.Motion_GyroRead(gyroX, gyroY, gyroZ);
    myCodeCell.Motion_RotationRead(rotX, rotY, rotZ);
    Serial.print("(SENSING) ");
    myCodeCell.PrintSensors();
}

float angleDifference(float angle1, float angle2) {
    float diff = fmod(angle2 - angle1 + 180, 360) - 180;
    return (diff < -180) ? diff + 360 : diff; // Ensure the range is [-180, 180]
}

void assistedMixing() {
    // Map throttle and steering to PWM values
    readSensors();

    // Locking logic
    if (abs(steer) > 0) {
        rotLocked = false;
    } else if (abs(gyroZ) < ROT_LOCK_THRESHOLD) {
        rotLocked = true;
        tgtRotZ = rotZ;
        errorIntegral = 0.0;
    }

    // Control logic
    if (rotLocked) {
      // Reglera
      float error = angleDifference(tgtRotZ, rotZ);
      errorIntegral += error;
      float offset = K_P*error + K_I*errorIntegral + K_D*(error-prevError);  // TODO: Expand to PID?

      offset = constrain(offset, -CONTROL_AUTHORITY, CONTROL_AUTHORITY);
      mix_L = throttle - offset;
      mix_R = throttle + offset;
      prevError = error;
    } else {
      // Gå bara på kontroller-input
      elevonMixing();
    }
}

void elevonMixing() {
    // Map throttle and steering to PWM values
    mix_L = DRIVE_SPEED_MODIFIER*throttle - TURN_SPEED_MODIFIER*(steer-STEER_OFF);
    mix_R = DRIVE_SPEED_MODIFIER*throttle + TURN_SPEED_MODIFIER*(steer-STEER_OFF);
}

void applyPWM() {
    int leftPWM;
    int rightPWM;
    int leftWeaponPWM;
    int rightWeaponPWM;

    if (INVERT_LEFT_WHEEL) {
      leftPWM = map(mix_L, THROTTLE_MIN, THROTTLE_MAX, PWM_MAX, PWM_MIN);
    } else {
      leftPWM = map(mix_L, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX);
    }

    if (INVERT_RIGHT_WHEEL) {
      rightPWM = map(mix_R, THROTTLE_MIN, THROTTLE_MAX, PWM_MAX, PWM_MIN);
    } else {
      rightPWM = map(mix_R, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX);
    }


    // left + right wpn inverting
    if (INVERT_LEFT_WPN){
      int leftWeaponPWM = map(weaponSpeed, WPN_MIN, WPN_MAX, PWM_MIN, PWM_MAX);
    } else {
      int leftWeaponPWM = map(weaponSpeed, WPN_MIN, WPN_MAX, PWM_MAX, PWM_MIN);
    }

    if (INVERT_RIGHT_WPN){
      int rightWeaponPWM = map(weaponSpeed, WPN_MIN, WPN_MAX, PWM_MIN, PWM_MAX);
    } else {
      int rightWeaponPWM = map(weaponSpeed, WPN_MIN, WPN_MAX, PWM_MAX, PWM_MIN);
    }

    // Constrain PWM values
    leftPWM = constrain(leftPWM, PWM_MIN, PWM_MAX);
    rightPWM = constrain(rightPWM, PWM_MIN, PWM_MAX);
    leftWeaponPWM = constrain(leftWeaponPWM, PWM_MIN, PWM_MAX);
    rightWeaponPWM = constrain(rightWeaponPWM, PWM_MIN, PWM_MAX);

    // Write PWM values to ESCs
    lDriveESC.writeMicroseconds(leftPWM);
    rDriveESC.writeMicroseconds(rightPWM);
    lWpnESC.writeMicroseconds(leftWeaponPWM);
    rWpnESC.writeMicroseconds(rightWeaponPWM);

    Serial.println(
        "(OUTPUT) leftPWM: " + String(leftPWM) + 
        ", rightPWM: " + String(rightPWM) + 
        ", leftWeaponPWM: " + String(leftWeaponPWM) + 
        ",  rightWeaponPWM: " + String(rightWeaponPWM)
        );
}

void attachMotorPins() {
    // Attach "servos"
    lDriveESC.attach(L_DRIVE_PIN, PWM_MIN, PWM_MAX);
    rDriveESC.attach(R_DRIVE_PIN, PWM_MIN, PWM_MAX);
    lWpnESC.attach(R_WPN_PIN, PWM_MIN, PWM_MAX);
    rWpnESC.attach(L_WPN_PIN, PWM_MIN, PWM_MAX);
}

void detachMotorPins() {
    // Detach "servos"
    lDriveESC.detach();
    rDriveESC.detach();
    lWpnESC.detach();
    rWpnESC.detach();

    // Set pins to input
    pinMode(L_DRIVE_PIN, INPUT);
    pinMode(R_DRIVE_PIN, INPUT);
    pinMode(R_WPN_PIN, INPUT);
    pinMode(L_WPN_PIN, INPUT);
}

void setMode(int newMode) {
    if (mode == newMode) {
      return;
    }

    mode = newMode;
    if (mode == COMBAT_MODE) {
        Serial.println("Entered combat mode");
        myCodeCell.LED(0, 255, 255);    // Cyan
        attachMotorPins();
    } else if (mode == ASSISTED_COMBAT_MODE) {
        Serial.println("Entered assisted combat mode");
        myCodeCell.LED(255, 0, 255);    // Magenta
        attachMotorPins();
    } else if (mode == PAIRING_MODE) {
        Serial.println("Entered pairing mode");
        myCodeCell.LED(255, 0, 0);      // Red
        weaponIdleSpeed = WPN_OFF; // Change weapon state to off upon restart.

        // Stop all motors
        lDriveESC.writeMicroseconds(PWM_MID);
        rDriveESC.writeMicroseconds(PWM_MID);
        lWpnESC.writeMicroseconds(PWM_MID);
        rWpnESC.writeMicroseconds(PWM_MID);

    } else if (mode == FLASHING_MODE) {
        Serial.println("Entered flashing mode");
        myCodeCell.LED(0, 255, 0);      // Green
        detachMotorPins();
    } else {
        Serial.println("Unknown mode! Defaulting to combat mode...");
        setMode(COMBAT_MODE);   // Default to combat mode
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");
    ctl.begin();

    myCodeCell.Init(MOTION_GYRO + MOTION_ROTATION + LIGHT);
    myCodeCell.LED_SetBrightness(10);
    myCodeCell.LED(255, 255, 255);

    setMode(PAIRING_MODE);
}

void loop() {
    ctl.onLoop();
    if (ctl.isConnected()) {
        if (ctl.isWaitingForFirstNotification()) {
            Serial.println("waiting for first notification");
        } else {
            if (mode == PAIRING_MODE) {
                setMode(COMBAT_MODE);
            }

            // Check whether in flash or control mode
            dumpGamepad();
            processGamepad();

            if (mode == COMBAT_MODE) {
                elevonMixing();
                applyPWM();
            } else if (mode == ASSISTED_COMBAT_MODE) {
                assistedMixing();
                applyPWM();
            }
        }
    } else {
        Serial.println("not connected");
        setMode(PAIRING_MODE);

        // Restart ESP to re-pair
        if (ctl.getCountFailedConnection() > 2) {
            ESP.restart();
        }
    }
    delay(UPDATE_DELAY);
}

// TODOS:
// - Run loop with myCodeCell.Run(UPDATE_FREQUENCY) instead
//   - Fix LED resetting to blue breathing when using myCodeCell.Run()
// - Break applyPWM() into elevonMixing()/PIDControl() and applyPWM()
// - Add gyro stabilization
