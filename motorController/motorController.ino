#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

const int enable_motorA = PA1;
const int motorA1 = PA2;
const int motorA2 = PA3;
const int motorB1 = PB6;
const int motorB2 = PB7;
const int enable_motorB = PB8;

float linear_velocity = 0.0;
float angular_velocity = 0.0;

const int max_pwm = 255;
const float max_linear_speed = 1.0;
const float max_angular_speed = 1.0;

void cmd_cb( const geometry_msgs::Twist& msg){
  linear_velocity = msg.linear.x;
  angular_velocity = msg.angular.z;
}


ros::Subscriber<std_msgs::UInt16> sub("cmd_vel", cmd_cb);


void calcMotorSpeed() {
    int speedA = (linear_velocity - angular_velocity) * (max_pwm / max_linear_speed);
    int speedB = (linear_velocity + angular_velocity) * (max_pwm / max_linear_speed);

    // Clamp the speed to the max PWM range
    speedA = constrain(speedA, -max_pwm, max_pwm);
    speedB = constrain(speedB, -max_pwm, max_pwm);

    setMotorSpeeds(speedA, speedB);
}

void setMotorSpeeds(int speedA, int speedB) {
    // Motor A (Left motor)
    analogWrite(enable_motorA, abs(speedA));
    if (speedA > 0) {
        digitalWrite(motorA1, HIGH);
        digitalWrite(motorA2, LOW);
    } else if (speedA < 0) {
        digitalWrite(motorA1, LOW);
        digitalWrite(motorA2, HIGH);
    } else {
        digitalWrite(motorA1, LOW);
        digitalWrite(motorA2, LOW);
    }

    // Motor B (Right motor)
    analogWrite(enable_motorB, abs(speedB));
    if (speedB > 0) {
        digitalWrite(motorB1, HIGH);
        digitalWrite(motorB2, LOW);
    } else if (speedB < 0) {
        digitalWrite(motorB1, LOW);
        digitalWrite(motorB2, HIGH);
    } else {
        digitalWrite(motorB1, LOW);
        digitalWrite(motorB2, LOW);
    }
}

void setup(){
  pinMode(enable_motorA, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(enable_motorB, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  calcMotorSpeeds();
  delay(10);
}
