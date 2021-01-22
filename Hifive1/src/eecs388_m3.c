#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "eecs388_lib.h"
#include "metal/i2c.h"

struct metal_i2c *i2c;
uint8_t bufWrite[5];
uint8_t bufRead[1];
volatile int g_angle;

//The entire setup sequence
void set_up_I2C()
{
    uint8_t oldMode;
    uint8_t newMode;
    _Bool success;

    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = MODE1_RESTART;
    printf("%d\n",bufWrite[0]);
    
    i2c = metal_i2c_get_device(0);

    if(i2c == NULL){
        printf("Connection Unsuccessful\n");
    }
    else{
        printf("Connection Successful\n");
    }
    
    //Setup Sequence
    metal_i2c_init(i2c,I2C_BAUDRATE,METAL_I2C_MASTER);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//reset
    delay(100);
    printf("resetting PCA9685 control 1\n");

    //Initial Read of control 1
    bufWrite[0] = PCA9685_MODE1;//Address
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Read success: %d and control value is: %d\n", success, bufWrite[0]);
    
    //Configuring Control 1
    oldMode = bufRead[0];
    newMode = (oldMode & ~MODE1_RESTART) | MODE1_SLEEP;
    printf("sleep setting is %d\n", newMode);
    bufWrite[0] = PCA9685_MODE1;//address
    bufWrite[1] = newMode;//writing to register
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sleep
    bufWrite[0] = PCA9685_PRESCALE;//Setting PWM prescale
    bufWrite[1] = 0x79;
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sets prescale
    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = 0x01 | MODE1_AI | MODE1_RESTART;
    printf("on setting is %d\n", bufWrite[1]);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//awake
    delay(100);
    printf("Setting the control register\n");
    bufWrite[0] = PCA9685_MODE1;
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Set register is %d\n",bufRead[0]);
} 

void breakup(int bigNum, uint8_t* low, uint8_t* high)
{
    int l = bigNum;
    int h = bigNum;
    *low = l & (0xFF);
    *high = (h>>8);

}

void steering(int angle)
{
    uint8_t low;
    uint8_t high;    
    breakup(getServoCycle(angle),&low,&high);
    bufWrite[0] = PCA9685_LED0_ON_L + 4;
    bufWrite[1] = 0;
    bufWrite[2] = 0;
    bufWrite[3] = low;
    bufWrite[4] = high;
    _Bool truth =  metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,5,bufWrite,METAL_I2C_STOP_ENABLE);
}

void stopMotor()
{
    uint8_t low;
    uint8_t high;    
    breakup(280,&low,&high);
    bufWrite[0] = PCA9685_LED0_ON_L;
    bufWrite[1] = 0;
    bufWrite[2] = 0;
    bufWrite[3] = low;
    bufWrite[4] = high;
    _Bool truth =  metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,5,bufWrite,METAL_I2C_STOP_ENABLE);
}

void driveForward(uint8_t speedFlag)
{
    uint8_t low;
    uint8_t high;  
    int speed;
    if (speedFlag == 1)
    {
        speed = 303;
    } else if (speedFlag == 2)
    {
        speed = 305;
    }
    else if (speedFlag == 3)
    {
        speed = 307;
    }
    breakup(speed,&low,&high);
    bufWrite[0] = PCA9685_LED0_ON_L;
    bufWrite[1] = 0;
    bufWrite[2] = 0;
    bufWrite[3] = low;
    bufWrite[4] = high;
    _Bool truth =  metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,5,bufWrite,METAL_I2C_STOP_ENABLE);

}

void driveReverse()
{
    uint8_t low;
    uint8_t high;  
    int speed = 280;
    while (speed >= 267)
    {
        breakup(speed,&low,&high);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0;
        bufWrite[2] = 0;
        bufWrite[3] = low;
        bufWrite[4] = high;
        _Bool truth =  metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,5,bufWrite,METAL_I2C_STOP_ENABLE);
        speed--;
    }
    while (speed <= 280)
    {
        breakup(speed,&low,&high);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0;
        bufWrite[2] = 0;
        bufWrite[3] = low;
        bufWrite[4] = high;
        _Bool truth =  metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,5,bufWrite,METAL_I2C_STOP_ENABLE);
        speed++;
    }
    while (speed >= 267)
    {
        breakup(speed,&low,&high);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0;
        bufWrite[2] = 0;
        bufWrite[3] = low;
        bufWrite[4] = high;
        _Bool truth =  metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,5,bufWrite,METAL_I2C_STOP_ENABLE);
        speed--;
    }
}

void raspberrypi_int_handler(int devid)
{
    char* array = malloc (20 * sizeof(char));
    int leng = ser_readline(devid, 20, array);
    printf("%s\n",array);
    char* angle = "angle";
    int count = 5;

    int ret = strncmp(array, angle, count);
    if (ret==0)
    {
        sscanf(array+count,"%d",&g_angle);
        printf("%d\n",g_angle);
    }
    free(array);
}

int collision_detection()
{
    ser_setup(1);

    ser_printline(1,"Setup completed.\n");

    int dist; // read distance value. 
    int temp = 0;

    while (1) {
        /* 
            Task 1: 
            - read a data frame from the TFmini sensor
            - print the read distance data. e.g., "dist: 45 cm"
              (you can use either printf or sprintf & ser_printline function)
        */
        if ('Y' == ser_read(1) && 'Y' == ser_read(1)) {
            dist = (ser_read(1)|(ser_read(1)<<8));
        }
        if (dist != temp)
        {
            printf("dist: %d cm\n", dist);
        }
        ser_read(1);ser_read(1);ser_read(1);ser_read(1);ser_read(1);

        /* 
            Task 2: 
            - turn on the red light if the distance is less than 50cm. 
            - otherwise turn on the green light 
        */
        if (dist <= 75) {
            return 1;
        } else {
            return 0;
        }

        temp = dist;
    }
}


int main()
{
    // Initialize I2C
    set_up_I2C();
    delay(2000);

    // Calibrate Motor
    printf("Calibrate Motor.\n");
    stopMotor();
    delay(2000);

    // initialize UART channels
    ser_setup(0); // uart0 (receive from raspberry pi)
    
    printf("Setup completed.\n");
    printf("Begin the main loop.\n");
    
    // Initialize global angle
    g_angle = 0;

    // Drive loop
    while (1) {
        if (ser_isready(0)&2)
        {
            raspberrypi_int_handler(0);
            steering(g_angle);
            printf("%d\n",ser_isready(0));
            if (collision_detection()==1)
            {
                stopMotor();
            }
        }

    }
    return 0;
}
