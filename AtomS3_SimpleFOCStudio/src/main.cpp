#include <Arduino.h>
/**
 * Comprehensive BLDC motor control example using magnetic sensor
 *
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values
 *
 * See more info in docs.simplefoc.com/commander_interface
 */
#include <SimpleFOC.h>
#include <Wire.h>

// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7, 5.6, 260);  // BDUAV 2206-260KV 14 poles 5.6Ω 260KV 
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  Wire.begin(2, 1);
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 5;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // control loop type and torque mode 
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;
  motor.motion_downsample = 0.0;

  // velocity loop PID
  motor.PID_velocity.P = 0.02;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000.0;
  motor.PID_velocity.limit = 50.0;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.01;
  // angle loop PID
  motor.P_angle.P = 20.0;
  motor.P_angle.I = 0.5;
  motor.P_angle.D = 0.1;
  motor.P_angle.output_ramp = 1000.0;
  motor.P_angle.limit = 50.0;
  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.01;
  // Limits 
  motor.velocity_limit = 50.0;
  motor.voltage_limit = 5.0;
  motor.current_limit = 2.0;




  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0;

  // define the motor id
  command.add('A', onMotor, "motor");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor.move();

  motor.monitor();

  // user communication
  command.run();
}