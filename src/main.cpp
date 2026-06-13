#include "robot_controller.hpp"
#include <Arduino.h>

RobotController robot(4, 16, 17, 19, 5, 18, 26, 25);

void setup() {
    Serial.begin(9600);
    robot.begin();
    Serial.println("Robot Controller Initialized");
}

void loop() {
    robot.update();
    delay(100);
}
