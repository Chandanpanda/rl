#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0

static int addr = 0x28;

// Initialise Accelerometer Function
void accel_init(void){


    // Check to see if connection is correct
    sleep_ms(1000); // Add a short delay to help BNO005 boot up
    uint8_t reg = 0x00;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, chipID, 1, false);

    if(chipID[0] != 0xA0){
        while(1){
            printf("Chip ID Not Correct - Check Connection!");
            sleep_ms(5000);
        }
    }

    // Use internal oscillator
    uint8_t data[2];
    data[0] = 0x3F;
    data[1] = 0x40;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Reset all interrupt status bits
    data[0] = 0x3F;
    data[1] = 0x01;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Configure Power Mode
    data[0] = 0x3E;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);

    // Defaul Axis Configuration
    data[0] = 0x41;
    data[1] = 0x24;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Default Axis Signs
    data[0] = 0x42;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Set units to m/s^2
    data[0] = 0x3B;
    data[1] = 0b0001000;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(30);

    // Set operation to acceleration only
    data[0] = 0x3D;
    data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(100);
}


int main(void){
    stdio_init_all(); // Initialise STD I/O for printing over serial

    // Configure the I2C Communication
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Call accelerometer initialisation function
    accel_init();

    uint8_t accel[6]; // Store data from the 6 acceleration registers
    int16_t accelX, accelY, accelZ; // Combined 3 axis data
    float f_accelX, f_accelY, f_accelZ; // Float type of acceleration data
    uint8_t val_a = 0x08; // Start register address

    uint8_t gyro[6]; // Store data from the 6 acceleration registers
    int16_t gyroX, gyroY, gyroZ; // Combined 3 axis data
    float f_gyroX, f_gyroY, f_gyroZ; // Float type of acceleration data
    uint8_t val_g = 0x14; // Start register address

    uint8_t mag[6]; // Store data from the 6 acceleration registers
    int16_t magX, magY, magZ; // Combined 3 axis data
    float f_magX, f_magY, f_magZ; // Float type of acceleration data
    uint8_t val_m = 0x0E; // Start register address

    uint8_t quat[8]; // Store data from the 6 acceleration registers
    int16_t rawW, rawX, rawY, rawZ; // Combined 3 axis data
    uint8_t val_q = 0x20; // Start register address
    float f_W, f_X, f_Y, f_Z; // Float type of acceleration data
	
    // Infinite Loop
    while(1){
        i2c_write_blocking(I2C_PORT, addr, &val_a, 1, true);
        i2c_read_blocking(I2C_PORT, addr, accel, 6, false);

        accelX = ((accel[1]<<8) | accel[0]);
        accelY = ((accel[3]<<8) | accel[2]);
        accelZ = ((accel[5]<<8) | accel[4]);

        f_accelX = accelX / 100.00;
        f_accelY = accelY / 100.00;
        f_accelZ = accelZ / 100.00;

        i2c_write_blocking(I2C_PORT, addr, &val_g, 1, true);
        i2c_read_blocking(I2C_PORT, addr, gyro, 6, false);

        gyroX = ((gyro[1]<<8) | gyro[0]);
        gyroY = ((gyro[3]<<8) | gyro[2]);
        gyroZ = ((gyro[5]<<8) | gyro[4]);

        f_gyroX = gyroX / 100.00;
        f_gyroY = gyroY / 100.00;
        f_gyroZ = gyroZ / 100.00;

        i2c_write_blocking(I2C_PORT, addr, &val_m, 1, true);
        i2c_read_blocking(I2C_PORT, addr, mag, 6, false);

        magX = ((mag[1]<<8) | mag[0]);
        magY = ((mag[3]<<8) | mag[2]);
        magZ = ((mag[5]<<8) | mag[4]);

        f_magX = magX / 100.00;
        f_magY = magY / 100.00;
        f_magZ = magZ / 100.00;

        i2c_write_blocking(I2C_PORT, addr, &val_q, 1, true);
        i2c_read_blocking(I2C_PORT, addr, quat, 8, false);

        rawW = ((quat[1]<<8) | quat[0]);
        rawX = ((quat[3]<<8) | quat[2]);
        rawY = ((quat[5]<<8) | quat[4]);
        rawZ = ((quat[6]<<8) | quat[7]);		

		f_W = float(rawW)/16384.0f;	
		f_X = float(rawX)/16384.0f;
		f_Y = float(rawY)/16384.0f;
		f_Z = float(rawZ)/16384.0f;
		
        // Print to serial monitor
        printf("Ax:%6.2f Ay:%6.2f Az:%6.2f Gx:%6.2f Gy:%6.2f Gz:%6.2f Mx:%6.2f My:%6.2f Mz:%6.2f W:%4.2f X:%4.2f Y:%4.2f Z:%4.2f\n", f_accelX, f_accelY, f_accelZ, f_gyroX, f_gyroY, f_gyroZ,f_magX, f_magY, f_magZ, f_W, f_X, f_Y, f_Z);
        sleep_ms(300);
    }
}