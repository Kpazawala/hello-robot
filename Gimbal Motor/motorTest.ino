/**
 * Using Ardunio Zero USB Port
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */
#include <SimpleFOC.h>

#define PHASE_RESISTANCE 4.03
#define STALL_CURRENT 1.4
#define PP 14
#define MOTOR_VLIMIT 40

float voltage_limit = PHASE_RESISTANCE * STALL_CURRENT * 1;

double max_angle = 0;
double min_angle = 0;
float target = 0;
float initial = 0;

int mode = 0;
bool ramp_enable = 0;

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(8, 14, 0x3FFF);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(PP);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 4, 5, 2);

// angle set point variable

// instantiate the commander
Commander command = Commander(SerialUSB);
void doTarget(char* cmd) {
  char _cmd = cmd[0];
  
  if(_cmd == 'S'){
    ramp_enable = 1;
    initial = target;
    command.scalar(&target, &cmd[1]);
  } else {
    command.scalar(&target, cmd);
    ramp_enable = 0;
  }

}
void doMode(char* cmd) {
  //   torque            = 0x00,     //!< Torque control
  // velocity          = 0x01,     //!< Velocity motion control
  // angle             = 0x02,     //!< Position/angle motion control

  SerialUSB.println(cmd);
  mode = atof(cmd);
  if (mode == 1) { //velocity_control
    cmd = "C1";
  } else if (mode == 2) { // angle_control
    target = 0;
    cmd = "C2";
  } else if (mode == 3) { // torque_control
    cmd = "C0";
  } else if (mode == 4) { // PID_tuning
    cmd = "C1";
  } else {
    SerialUSB.println("Mode Does Not Exist");
  }
  SerialUSB.println(cmd);
  command.motion(&motor, cmd);
}
void doPID(char* cmd){
  command.pid(&motor.PID_velocity, cmd); 
}

void sd_setup(){
  // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  SerialUSB.println("Sensor ready");
  _delay(100);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
}

void motor_check(){
  int count = 0; bool flag = 0; int v = 10;
  while(1) {
    motor.loopFOC();
    motor.sensor_offset = min_angle;
    if(!flag){
      motor.move(-v);
      if(round(motor.shaftVelocity()) == 0 && count > 500){
        min_angle = motor.shaftAngle();
        motor.sensor_offset = min_angle;
        flag = 1;
        count = 0;
      }
    } else {
      motor.move(v); 
      if(round(motor.shaftVelocity()) == 0 && count > 500){
        max_angle = motor.shaftAngle();
        break;
      }
    }
    count++;
  }
  motor.controller = MotionControlType::angle;
  while(1){
    motor.loopFOC();
    motor.move(0.05);
    if(fabs(motor.shaftAngle() - 0.05) < .005){
      break;
    }
  }
  motor.velocity_limit = MOTOR_VLIMIT;
   SerialUSB.print(F("min_angle: "));
  SerialUSB.print(F("min_angle: "));
  SerialUSB.print(min_angle);
  SerialUSB.print(F(", max_angle: "));
  SerialUSB.println(max_angle);
  delay(1000);

}
void motor_setup(){
  motor.voltage_limit = voltage_limit;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.LPF_velocity.Tf = 0.01f;
  motor.controller = MotionControlType::velocity;
  mode = 1;
  motor.voltage_sensor_align = 4;

  motor.PID_velocity.P = 0.6;
  motor.PID_velocity.I = 0.3;
  motor.PID_velocity.D = 0.0;

}

void setup() {
  while(!SerialUSB.available())
  SerialUSB.println('0');
  SerialUSB.begin(115200);
  sd_setup();
  motor_setup();
  
  motor.useMonitoring(SerialUSB);
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  motor_check();
  
  motor.controller = MotionControlType::velocity;
  mode = 1;
  // add target command T
  command.add('T', doTarget, "target");
  command.add('M', doMode, "target mode");
  command.add('P', doPID, "target PID");
  SerialUSB.println(F("Motor ready."));
  SerialUSB.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  if(mode == 1 && ramp_enable && fabs(target - initial) > 0){
    float ramp_speed = .05f;
    initial = initial + ((initial < target) ? ramp_speed : -ramp_speed);
    motor.move(initial);
  } else {
    if(mode == 1 || mode == 3){     
      motor.move(target);
    } else if (mode == 2) {
      float t = target * (3.14 / 180);
      if(!max_angle){
        if (t >= 0 && t <= max_angle) {
              motor.move(t);  // Move to t if it's within the valid range
          } else if (t > max_angle) {
              motor.move(max_angle);  // Move to max_angle if t exceeds it
          } else {
              motor.move(0.0);  // Move to 0 if t is below the minimum bound
          }
      } else {
        motor.move(t);
      }
    } else if (mode == 4) {
      motor.move(target);
    } else {
      motor.move(0);
    }
  }
  motor.monitor();
  command.run();
}
