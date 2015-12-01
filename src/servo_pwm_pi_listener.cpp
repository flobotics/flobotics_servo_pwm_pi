#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>


int file;
char filename[11]; // = "/dev/i2c-1";
__u8 addr = 0x40; //  0b01000000;        // The I2C address of the PWM-Hat

enum pwm_regs {
  // Registers/etc.
  __MODE1              = 0x00,
  __MODE2              = 0x01,
  __SUBADR1            = 0x02,
  __SUBADR2            = 0x03,
  __SUBADR3            = 0x04,
  __PRESCALE           = 0xFE,
  __LED0_ON_L          = 0x06,
  __LED0_ON_H          = 0x07,
  __LED0_OFF_L         = 0x08,
  __LED0_OFF_H         = 0x09,
  __ALL_LED_ON_L       = 0xFA,
  __ALL_LED_ON_H       = 0xFB,
  __ALL_LED_OFF_L      = 0xFC,
  __ALL_LED_OFF_H      = 0xFD,
  __RESTART            = 0x80,
  __SLEEP              = 0x10,
  __ALLCALL            = 0x01,
  __INVRT              = 0x10,
  __OUTDRV             = 0x04
};

void setAllPWM(int on, int off){
	int res;
	res = i2c_smbus_write_byte_data(file, __ALL_LED_ON_L, on & 0xFF);
	if(res < 0)
		perror("resultaterror");

	res = i2c_smbus_write_byte_data(file, __ALL_LED_ON_H, on  >> 8);
	if(res < 0)
		perror("err2");

	res = i2c_smbus_write_byte_data(file, __ALL_LED_OFF_L, off & 0xFF);
	if(res < 0)
		perror("err3");

	res = i2c_smbus_write_byte_data(file, __ALL_LED_OFF_H, off >> 8);
	if(res < 0)
		perror("err4");
};

void setPWM(int channel, int value){
	int on = 0; // change it if you need to start PWM cycle with phase delay
	int off = value;
	int res;

	res = i2c_smbus_write_byte_data(file, __LED0_ON_L+4*channel, on & 0xFF);
	if(res < 0)
		perror("ierr1");

	res = i2c_smbus_write_byte_data(file, __LED0_ON_H+4*channel, on >> 8);
	if(res < 0)
		perror("ierr2");

	res = i2c_smbus_write_byte_data(file, __LED0_OFF_L+4*channel, off & 0xFF);
	if(res < 0)
		perror("ierr3");

	res = i2c_smbus_write_byte_data(file, __LED0_OFF_H+4*channel, off >> 8);
	if(res < 0)
		perror("ierr4");

	
};

void setPWMFreq(int freq){
	int prescale;
	char oldmode, newmode;
	int res;

	float prescaleval = 25000000.0; // 25MHz
	prescaleval /= 4096.0;
	prescaleval /= (float)freq;
	prescaleval -= 1.0;

	ROS_INFO("Setting PWM frequency to %d Hz", freq);
	ROS_INFO("Estimated pre-scale: %f", prescaleval);

	prescale = floor(prescaleval + 0.5);
	ROS_INFO("Final pre-scale: %d", prescale);

	nanosleep((const struct timespec[]){{1, 000000L}}, NULL); 


	oldmode = i2c_smbus_read_byte_data(file, __MODE1);
	newmode = (oldmode & 0x7F) | 0x10; // sleep
	
	res = i2c_smbus_write_byte_data(file, __MODE1, newmode); // go to sleep
	if(res < 0)
		perror("serr0");

	res = i2c_smbus_write_byte_data(file, __PRESCALE, (int)(floor(prescale)));
	if(res < 0)
		perror("serr1");

	res = i2c_smbus_write_byte_data(file, __MODE1, oldmode);
	if(res < 0)
		perror("serr2");

	nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec,

	res = i2c_smbus_write_byte_data(file, __MODE1, oldmode | 0x80);
	if(res < 0)
		perror("serr3");
	


};

void initPWMHat(){
	int res;
	char mode1res;

	if ((file = open(filename, O_RDWR)) < 0) {
    		/* ERROR HANDLING: you can check errno to see what went wrong */
    		perror("Failed to open the i2c bus");
    		exit(1);
	}

	if (ioctl(file,I2C_SLAVE,addr) < 0) {
	    	printf("Failed to acquire bus access and/or talk to slave.\n");
    		/* ERROR HANDLING; you can check errno to see what went wrong */
    		exit(1);
	}	

	setAllPWM(0,0);

	res = i2c_smbus_write_byte_data(file, __MODE2, __OUTDRV);
	if(res < 0)
		perror("erro1");

	res = i2c_smbus_write_byte_data(file, __MODE1, __ALLCALL);
	if(res < 0)
		perror("erro2");

	nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

	mode1res = i2c_smbus_read_byte_data(file, __MODE1);
	mode1res = mode1res & ~__SLEEP; //                 # wake up (reset sleep)

	res = i2c_smbus_write_byte_data(file, __MODE1, mode1res);
	if(res < 0)
		perror("erro3");

	nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci



}


void chatterCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	short int arr[16];
	int i=0;

	for(std::vector<short int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		arr[i] = *it;
		//i++;
		ROS_INFO("val:[%d]", arr[i]);
		
		if(arr[i] < 4096)
			setPWM(i,arr[i]);
		i++;
	}

  //ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char **argv)
{
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", 1);

	initPWMHat();

	setPWMFreq(60);  //60 Hz

	ros::init(argc, argv, "servo_pwm_pi_listener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("servo_pwm_pi_sub", 1000, chatterCallback);

	ros::spin();

	close(file);

  return 0;
}
