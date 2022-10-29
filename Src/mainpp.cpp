/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>


ros::NodeHandle nh;
ugv_t ugv;
char sendBuffer [BUFFER_SIZE] = {0};


/********************************* ROS MESSAGES ***************************************/
std_msgs::Int32   msg_ugv_speed;
std_msgs::Float32 msg_ugv_heading;
std_msgs::Bool    msg_drive_alarm;
std_msgs::Bool    msg_servo_alarm;

std_msgs::Bool    msg_drive_pid_en;
std_msgs::Bool    msg_servo_pid_en;
std_msgs::Int32   msg_drive_setpoint;
std_msgs::Int32   msg_servo_setpoint;


/********************************* ROS PUBLISHERS *************************************/
ros::Publisher pub_ugv_speed   ("ugv_speed",   &msg_ugv_speed);
ros::Publisher pub_ugv_heading ("ugv_heading", &msg_ugv_heading);
ros::Publisher pub_drive_alarm ("drive_alarm", &msg_drive_alarm);
ros::Publisher pub_servo_alarm ("servo_alarm", &msg_servo_alarm);


/********************************* ROS CALLBACKS **************************************/
void drive_pid_en_cb   (const std_msgs::Bool &msg);
void drive_setpoint_cb (const std_msgs::Int32 &msg);
void servo_pid_en_cb   (const std_msgs::Bool &msg);
void servo_setpoint_cb (const std_msgs::Int32 &msg);

/******************************** ROS SUBSCRIBERS *************************************/
ros::Subscriber<std_msgs::Bool>  sub_drive_pid_en   ("drive_pid_en",   &drive_pid_en_cb);
ros::Subscriber<std_msgs::Int32> sub_drive_setpoint ("drive_setpoint", &drive_setpoint_cb);
ros::Subscriber<std_msgs::Bool>  sub_servo_pid_en   ("servo_pid_en",   &servo_pid_en_cb);
ros::Subscriber<std_msgs::Int32> sub_servo_setpoint ("servo_setpoint", &servo_setpoint_cb);

void setup(void)
{
	nh.initNode();
    
	// PUBLISHER SETUP
    nh.advertise(pub_ugv_speed);
    nh.advertise(pub_ugv_heading);
    nh.advertise(pub_drive_alarm);
    nh.advertise(pub_servo_alarm);

    // SUBSCRIBER SETUP
    nh.subscribe(sub_drive_pid_en);
    nh.subscribe(sub_drive_setpoint);
    nh.subscribe(sub_servo_pid_en);
    nh.subscribe(sub_servo_setpoint);

    ugv.speed = 0;
    HAL_TIM_Base_Start_IT(&htim14);
}

void loop(void)
{
	nh.spinOnce();
}

/**
 * @brief Timer callback function; triggers on timer rollover
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	ugv.heading++;
	ugv.speed++;

	msg_ugv_speed.data = ugv.speed;
	msg_ugv_heading.data = ugv.heading;

	pub_ugv_speed.publish(&msg_ugv_speed);
	pub_ugv_heading.publish(&msg_ugv_heading);

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
}

void drive_pid_en_cb (const std_msgs::Bool &msg)
{
    ugv.drive_pid_en = msg.data;

#ifdef DEBUG_ENABLE
    memset(sendBuffer, 0x00, BUFFER_SIZE);
    sprintf(sendBuffer, "\r\nNew Drive Motor PID Mode: %d\r\n", ugv.drive_pid_en);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) sendBuffer, sizeof(sendBuffer));
#endif
}

void drive_setpoint_cb (const std_msgs::Int32 &msg)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    ugv.drive_setpoint = msg.data;

#ifdef DEBUG_ENABLE
    memset(sendBuffer, 0x00, BUFFER_SIZE);
    sprintf(sendBuffer, "\r\nNew Drive Motor Setpoint: %d\r\n", ugv.drive_setpoint);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) sendBuffer, sizeof(sendBuffer)); 
#endif
}

void servo_pid_en_cb (const std_msgs::Bool &msg)
{
	ugv.servo_pid_en = msg.data;
#ifdef DEBUG_ENABLE
    memset(sendBuffer, 0x00, BUFFER_SIZE);
    sprintf(sendBuffer, "\r\nNew Servo Motor PID Mode: %d\r\n", ugv.servo_pid_en);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) sendBuffer, sizeof(sendBuffer));
#endif
}

void servo_setpoint_cb (const std_msgs::Int32 &msg)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    ugv.servo_setpoint = msg.data;

#ifdef DEBUG_ENABLE
    memset(sendBuffer, 0x00, BUFFER_SIZE);
    sprintf(sendBuffer, "\r\nNew Servo Motor Setpoint: %d\r\n", ugv.servo_setpoint);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) sendBuffer, sizeof(sendBuffer));
#endif
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->reset_rbuf();
}

