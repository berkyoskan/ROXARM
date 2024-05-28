/*
  This has been coded by Maciej Zychowski.
  To find out more about the project check the github page: https://github.com/berkyoskan/ROXARM
  This code is in the public domain.
*/
#include <Arduino.h>
#include <uMyo_BLE.h>
#include <ESP32Servo.h>

TaskHandle_t Task0;
TaskHandle_t Task1;
Servo servoMotor[5];


// User configurable settings.

const int numDataPoints = 10;              // The number of data points that will be stored from the sensor. Impacts the weighted average calculation.
const int minTriggerValue = 150;           // The minimum value that the weighted average must exceed for the hand to begin gripping.
const int maxTriggerValue = 1000;          // The maximum value that, if the weighted average exceeds, will result in full gripping.
const double inputVoltage = 5.0;           // The fixed input voltage supplied to the servos and current measuring circuitry.
const double minTargetCurrent = 0.1;       // The minimum current in amps that will always be allowed to each servo when closing.
const double maxTargetCurrent = 0.7;       // The maximum current in amps that the user requests under maximum muscle activity.
const double shuntResistorOhm = 0.5;       // The resistance value in ohms of each shunt resistor, measuring the current of each servo.

bool requestGrip = false;
double normalizedMuscleLevel = 0;
double targetCurrentDraw = 0;

double dataPoints[numDataPoints];
double weights[numDataPoints];

// The following arrays are set up as:
// {pinky, ring, middle, index, thumb}
const int servoPin[5] = {23, 22, 21, 19, 18};
const int sensePin[5] = {36, 39, 34, 35, 32};
const int forcePin[4] = {27, 14, 12, 13};
const int hapticPin[4] = {33, 25, 4, 2};


double measuredResistorVDrop[5] = {0};
double actualResistorVDrop[5] = {0};
double actualCurrentDraw[5] = {0};
double measuredForce[4] = {0};
int hapticSpeed[4] = {0};
int servoPosition[5] = {180, 180, 180, 0, 180};

void loop0(void * parameter) {
  for (;;) {
    // Retrieving a new data point from the sensor.
    double newData = uMyo.getMuscleLevel(0);

    // Moving each entry of the dataPoint array one spot to the right.
    for (int i = numDataPoints - 1; i > 0; i--) {
      dataPoints[i] = dataPoints[i - 1];
    }

    // Placing the new data point in the leftmost entry of the dataPoint array.
    dataPoints[0] = newData;

    // Declaring and initializing variables to calculate the weighted average.
    double weightedSum = 0;
    double weightSum = 0;

    // Multipling each entry of the dataPoint array with the corresponding entry of the weights array, which gets added up as
    // the weightedSum. Also, the entries of the weights array are added up to give the weightSum.
    for (int i = 0; i < numDataPoints; i++) {
      weightedSum += dataPoints[i] * weights[i];
      weightSum += weights[i];
    }

    // Calculating the weighted average.
    double weightedAverage = weightedSum / weightSum;

    // Printing the weighted average to the serial monitor.
    Serial.println();
    Serial.print("Weighted Average: ");
    Serial.print(weightedAverage);

    // Scenario when user is requesting maximum grip.
    if (weightedAverage > minTriggerValue && weightedAverage >= maxTriggerValue) {
      requestGrip = true;
      normalizedMuscleLevel = 1;
    }
    // Scenario when user is requesting variable amount of grip.
    else if (weightedAverage > minTriggerValue && weightedAverage < maxTriggerValue) {
      requestGrip = true;
      normalizedMuscleLevel = (weightedAverage - minTriggerValue) / (maxTriggerValue - minTriggerValue);
    }
    // Scenario when user is not requesting grip.
    else {
      requestGrip = false;
      normalizedMuscleLevel = 0;
    }

    for (int i = 0; i < 4; i++) {
      // Measuring the force applied onto each fingertip via tactile sensors.
      measuredForce[i] = analogRead(forcePin[i]);
      hapticSpeed[i] = map(measuredForce[i], 0, 2000, 100, 255);

      // Scenario when the measured force in a fingertip exceeds the preset trigger force.
      if (measuredForce[i] > 200){
        analogWrite(hapticPin[i], hapticSpeed[i]);
      }
      // Scenario when the measured force in a fingertip does not exceed the preset trigger force.
      else {
        analogWrite(hapticPin[i], 0);
      }
    }
  }
}

void loop1(void * parameter) {
  for (;;) {
    for (int i = 0; i < 5; i++) {
      // Measuring the voltage drop across resistors and converting it to current draw.
      measuredResistorVDrop[i] = analogRead(sensePin[i]);
      actualResistorVDrop[i] = (measuredResistorVDrop[i] / 4095) * inputVoltage;
      actualCurrentDraw[i] = actualResistorVDrop[i] / shuntResistorOhm;

      // Setting a target current draw based on muscle activity level of the user, plus some offset.
      targetCurrentDraw = ((maxTargetCurrent - minTargetCurrent) * normalizedMuscleLevel) + minTargetCurrent;

      // Adjusting each servo position, based on whether the user is requesting the hand to close.
      if (requestGrip == true) {
        if (i == 0 || i == 1 || i == 2) {   // Pinky, ring, middle fingers.
          if (actualCurrentDraw[i] < (targetCurrentDraw * 0.9)) {
            servoPosition[i] -= 2;
          }
          else if (actualCurrentDraw[i] > (targetCurrentDraw * 1.1)) {
            servoPosition[i] += 2;
          }
        }
        else if (i == 3) {    // Index finger — inverted movement.
          if (actualCurrentDraw[i] < (targetCurrentDraw * 0.9)) {
            servoPosition[i] += 2;
          }
          else if (actualCurrentDraw[i] > (targetCurrentDraw * 1.1)) {
            servoPosition[i] -= 2;
          }
        }
        else if (i == 4) {    // Thumb — faster movement.
          if (actualCurrentDraw[i] < (targetCurrentDraw * 0.9)) {
            servoPosition[i] -= 3;
          }
          else if (actualCurrentDraw[i] > (targetCurrentDraw * 1.1)) {
            servoPosition[i] += 3;
          }
        }
      }
      // If the user is not requesting the hand to close, each finger will incrementally open.
      else if (requestGrip == false) {
        if (i == 0 || i == 1 || i == 2) {
          servoPosition[i] += 2;
        }
        else if (i == 3) {
          servoPosition[i] -= 2;
        }
        else if (i == 4) {
          servoPosition[i] += 3;
        }
      }

      // Contraining the servo position values within a range of 0 to 180 degrees.
      servoPosition[i] = constrain(servoPosition[i], 0, 180);
      // Writing each servo position value to the respective servo.
      servoMotor[i].write(servoPosition[i]);
    }
  }
}

void setup() {
  // Initializing serial connection with computer.
  Serial.begin(115200);
  Serial.println("Setup started.");

  // Intializing uMyo sensor to collect data.
  uMyo.begin();

  // Setting up an array to hold the weights that will later be used to calculated the weighted average.
  for (int i = 0; i < numDataPoints; i++) {
    weights[i] = (double)(numDataPoints - i) / (double)numDataPoints;
  }

  // Attaching servos to appropriate pins on microcontroller.
  for (int i = 0; i < 5; i++) {
    servoMotor[i].attach(servoPin[i]);
    servoMotor[i].write(servoPosition[i]);
  }

  // Designating pins that will measure current as inputs.
  for (int i = 0; i < 5; i++) {
    pinMode(sensePin[i], INPUT);
  }

  // Designating pins that will measure force as inputs.
  for (int i = 0; i < 4; i++) {
    pinMode(forcePin[i], INPUT);
  }

  // Designating pins that will output to haptic motors.
  for (int i = 0; i < 4; i++) {
    pinMode(hapticPin[i], OUTPUT);
  }

  // Pinning the first task to core zero.
  xTaskCreatePinnedToCore(
      loop0,    /* Function to implement the task */
      "Task0",  /* Name of the task */
      10000,    /* Stack size in words */
      NULL,     /* Task input parameter */
      0,        /* Priority of the task */
      &Task0,   /* Task handle. */
      0);       /* Core where the task should run */
  
  // Pinning the second task to core one.
  xTaskCreatePinnedToCore(
      loop1,    /* Function to implement the task */
      "Task1",  /* Name of the task */
      10000,    /* Stack size in words */
      NULL,     /* Task input parameter */
      0,        /* Priority of the task */
      &Task1,   /* Task handle. */
      1);       /* Core where the task should run */

  Serial.println("Setup finished.");
}

void loop() {
  delay(1);
}
