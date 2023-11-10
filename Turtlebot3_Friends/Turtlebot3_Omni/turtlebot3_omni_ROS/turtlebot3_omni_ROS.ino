/*******************************************************************************
code ROS modifier pour intergre une node ros 
de commande de vélocité pour ce déplacer
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "turtlebot3_omni.h"

/******************************************************************************
 * Intégration des commandes ROS
 ******************************************************************************/
#include <ros.h> 
#include <geometry_msgs/Twist.h>

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for RC100 remote conroller
*******************************************************************************/
RC100 remote_controller;
double const_cmd_vel    = 0.2;

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

double linear_x                = 0.0;
double linear_y                = 0.0;
double angular_z               = 0.0;
double goal_linear_x_velocity  = 0.0;
double goal_linear_y_velocity  = 0.0;
double goal_angular_velocity   = 0.0;

/*******************************************************************************
* procédure de traitement de la vitesse
*******************************************************************************/
void twistcallback(const geometry_msgs::Twist& data)
{
  linear_x=data.linear.x;
  linear_y=data.linear.y;
  angular_z=data.angular.z;
      if (linear_x > MAX_LINEAR_VELOCITY)
    {
      linear_x = MAX_LINEAR_VELOCITY;
    }

    if (angular_z > MAX_ANGULAR_VELOCITY)
    {
      angular_z = MAX_ANGULAR_VELOCITY;
    }
     goal_linear_x_velocity  = linear_x;
  goal_linear_y_velocity  = linear_y;
  goal_angular_velocity   = angular_z;
}

/*******************************************************************************
* Declaration de ros et de la souscription au topic de déplacement
*******************************************************************************/
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistcallback);

void setup()
{
  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for RC100 remote control and cmd_vel
  remote_controller.begin(1);  //57600bps for RC100

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  // Start Dynamixel Control Interrupt
  startDynamixelControlInterrupt();

  // intialisation de ROS
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
 // receiveRemoteControl();
  nh.spinOnce();
  delay(1);
  /*  
   // utiliser pour voir si les moteurs tournent
    linear_x  = 0.00;
    linear_y  = 0.01;
    angular_z = 0.0;
    
    goal_linear_x_velocity  = linear_x;
    goal_linear_y_velocity  = linear_y;
    goal_angular_velocity   = angular_z;
*/
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlOmni);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Receive RC100 remote controller data
*******************************************************************************/
void receiveRemoteControl(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if (received_data & RC100_BTN_U)
    {
      linear_x  += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }
    else if (received_data & RC100_BTN_D)
    {
      linear_x  -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }

    if (received_data & RC100_BTN_L)
    {
      linear_y -= VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
    }
    else if (received_data & RC100_BTN_R)
    {
      linear_y += VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
    }

    if (received_data & RC100_BTN_1)
    {

    }
    else if (received_data & RC100_BTN_2)
    {
      angular_z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if (received_data & RC100_BTN_3)
    {

    }
    else if (received_data & RC100_BTN_4)
    {
      angular_z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }

    if (received_data & RC100_BTN_6)
    {
      linear_x  = const_cmd_vel;
      linear_y  = 0.0;
      angular_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      linear_x  = 0.0;
      linear_y  = 0.0;
      angular_z = 0.0;
    }

    if (linear_x > MAX_LINEAR_VELOCITY)
    {
      linear_x = MAX_LINEAR_VELOCITY;
    }

    if (angular_z > MAX_ANGULAR_VELOCITY)
    {
      angular_z = MAX_ANGULAR_VELOCITY;
    }
     
    linear_x  = 0.1;
    linear_y  = 0.0;
    angular_z = 0.0;
    
    goal_linear_x_velocity  = linear_x;
    goal_linear_y_velocity  = linear_y;
    goal_angular_velocity   = angular_z;
  }
}

/*******************************************************************************
* Control mecanum speed
*******************************************************************************/
void controlOmni()
{
  bool dxl_comm_result = false;

  int64_t wheel_value[OMNIWHEEL_NUM] = {0, 0, 0};
  double wheel_angular_velocity[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};

  wheel_angular_velocity[0] = (goal_linear_x_velocity * 0) + (goal_linear_y_velocity * (1 / WHEEL_RADIUS)) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[1] = (goal_linear_x_velocity * (sqrt(3) / (2 * WHEEL_RADIUS))) + (goal_linear_y_velocity * (-1 / (2 * WHEEL_RADIUS))) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[2] = (goal_linear_x_velocity * (sqrt(3) / (-2 * WHEEL_RADIUS))) + (goal_linear_y_velocity * (-1 / (2 * WHEEL_RADIUS))) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));

  for (int id = 0; id < OMNIWHEEL_NUM; id++)
  {
    wheel_value[id] = wheel_angular_velocity[id] * 9.54 /  RPM_CONSTANT_VALUE;

    if (wheel_value[id] > LIMIT_X_MAX_VALUE)       wheel_value[id] =  LIMIT_X_MAX_VALUE;
    else if (wheel_value[id] < -LIMIT_X_MAX_VALUE) wheel_value[id] = -LIMIT_X_MAX_VALUE;
  }

#ifdef DEBUG
  Serial.print("Vx : ");  Serial.print(goal_linear_x_velocity);
  Serial.print(" Vy : "); Serial.print(goal_linear_y_velocity);
  Serial.print(" W : "); Serial.println(goal_angular_velocity);
#endif

  dxl_comm_result = motor_driver.controlMotor((int64_t)wheel_value[0], (int64_t)wheel_value[1], (int64_t)wheel_value[2]);
  if (dxl_comm_result == false)
    return;
}