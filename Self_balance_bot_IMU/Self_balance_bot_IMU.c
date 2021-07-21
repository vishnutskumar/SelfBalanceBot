/*
 * Self_balance_bot_IMU.c
 *
 * Created: 31-Mar-15 7:08:48 PM
 *  Author: VishnuTS
 */
#define F_CPU 14745600UL 
#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.1415926535897
#define dt 0.00054
#define P_GAIN 0.8
#define I_GAIN 0.005
#define D_GAIN 0.01
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm

#include "mpu6050.h"

#define UART_BAUD_RATE 57600
#include "uart.h"

int main(void) {

	#if MPU6050_GETATTITUDE == 0
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    double axg = 0;
    double ayg = 0;
    double azg = 0;
    double gxds = 0;
    double gyds = 0;
    double gzds = 0;
	double accXangle = 0;
	double  gyroXangle = 0;
	double Xangle = 0 ;
	double error = 0;
	double I_error = 0;
	double D_error = 0;
	double previous_error = 0 ;
	double outputspeed = 0;
	#endif
	
	#if MPU6050_GETATTITUDE == 0
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
	mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
	accXangle = (atan2(ayg,azg)+PI)*RAD_TO_DEG;
	gyroXangle = accXangle;
	#endif

    //init uart
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

	//init interrupt
	sei();

	//init mpu6050
	mpu6050_init();
	_delay_ms(50);
	for(;;) {
		#if MPU6050_GETATTITUDE == 0
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		#endif
		
        accXangle = (atan2(ayg,azg)+PI)*RAD_TO_DEG;
		gyroXangle = accXangle + gxds*dt;
		Xangle = 0.98*gyroXangle + 0.02*accXangle;
		
		error = 180 - Xangle;
		I_error += (error)*dt;
		D_error = (error - previous_error)/dt;
		
		outputspeed = (P_GAIN * error) + (I_GAIN * I_error) + (D_GAIN * D_error);
		previous_error = error;
		
		#if MPU6050_GETATTITUDE == 0
		char itmp[10];
		/*dtostrf(ax, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(ay, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(az, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(gx, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(gy, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(gz, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	    dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');*/
		dtostrf(accXangle, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyroXangle, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(Xangle, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(error, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(I_error, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(D_error, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(outputspeed, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		uart_puts("\r\n");
		#endif 

	}

}
