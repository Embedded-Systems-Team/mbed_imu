#include "mbed.h"
#include "ahrs.h"
#include "icm20948.h"
#include <cmath>
#include <cstdio>
#include <stdint.h>
#include <time.h>


using namespace std::chrono;
time_t t1;
typedef unsigned char byte;
float selft[6];
static BufferedSerial pc(USBTX, USBRX);

char msg[255];
char data_to_send[50];

PwmOut speaker(p21), p(p22), r(p23);
BusOut myleds(LED1, LED2, LED3, LED4);


void setup()
{
     //Set up I2C
    
    pc.set_baud(9600);
    pc.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
  // Reset ICM20948
  begin();
 
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAGS);
  thread_sleep_for(100);
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  thread_sleep_for(100);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(ICM20948_ADDRESS, WHO_AM_I_ICM20948);
  
  if (c == 0xEA) // WHO_AM_I should always be 0x71
  {
    // sprintf(msg,"ICM20948 is online...\n");
    // pc.write(msg, strlen(msg));
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
    // Start by performing self test and reporting values
    ICM20948SelfTest(selft);
    calibrateICM20948(gyroBias, accelBias);

    initICM20948();
   
 
    getAres();
    getGres();
    // getMres();
    // magCalICM20948(magBias, magScale);
    // sprintf(msg,"[%f, %f, %f]\n", magBias[0], magBias[1],magBias[2] );
    // pc.write(msg, strlen(msg));

    // sprintf(msg,"[%f, %f, %f]\n", magScale[0], magScale[1],magScale[2] );
    // pc.write(msg, strlen(msg));


    
    // magBias[0] = 24565.998047; // pre-calculated values
    // magBias[1] = 24565.998047;
    // magBias[2] = 24565.998047;
    // magScale[0] = 1.0;
    // magScale[1] = 1.0;
    // magScale[2] = 1.0;

    thread_sleep_for(2000); // Add delay to see results before pc spew of data
  } // if (c == 0x71)
  else
  {
    sprintf(msg,"Could not connect to ICM20948: 0x%x",c);
    pc.write(msg, strlen(msg));
    sprintf(msg," Communication failed, abort!\n");
    pc.write(msg, strlen(msg));
    exit(0);
  }
}

void blink_rled()
{
    r = 1.0;
    wait_us(1e6); 
    r = 0.0;
}

void blink_pled()
{
    p = 1.0;
    wait_us(1e6); 
    p = 0.0;
}

int main(void)
{   
    setup();

    
    double phi_acc, theta_acc, phi_dt, theta_dt, phi_gyro, theta_gyro, phi_hat, theta_hat, phi_deg, theta_deg,psi_deg, psi_y_acc,psi_x_acc, psi_gyro, psi_hat,psi_acc; 
    double phi = 0.0, theta = 0.0, psi =0.0, dt = 0.0; 
    float alpha = 0.8, alpha2 = 0.8, scaled_r = 0.0,scaled_p = 0.0;
    double start_time = time(&t1);
    int alert_count = 0;

    while(true)
    {
     
        dt = time(&t1) - start_time; 
        
        readAccelData(accelCount);
        ax = (float)accelCount[0] * aRes - accelBias[0]; 
        ay = (float)accelCount[1] * aRes - accelBias[1]; 
        az = (float)accelCount[2] * aRes - accelBias[2];
        
        sprintf(msg,"[%f, %f, %f]\n", ax, ay, az);
        
        // pc.write(msg, strlen(msg)); //mg


        readGyroData(gyroCount);  
        wx = (float)gyroCount[0] * gRes - gyroBias[0];
        wy = (float)gyroCount[1] * gRes - gyroBias[1];
        wz = (float)gyroCount[2] * gRes - gyroBias[2];
        sprintf(msg,"[%f, %f, %f]\n",wx, wy, wz);
        // pc.write(msg, strlen(msg)); // deg/s
        
        // readMagData(magCount);  // Read the x/y/z adc values
        // mx = (float)magCount[0] * mRes - magBias[0];
        // my = (float)magCount[1] * mRes - magBias[1];
        // mz = (float)magCount[2] * mRes - magBias[2];

        // sprintf(msg,"[%f, %f, %f]\n",mx, my, mz);
        // pc.write(msg, strlen(msg)); // deg/s
        
        

        phi_acc = atan2(ay, sqrt(pow(ax, 2.0) + pow(az, 2.0))); 
        theta_acc = atan2(-ax, sqrt(pow(ay, 2.0)+ pow(az,2.0)));

        phi_dt = wx + sin(phi) * tan(theta) * wy + cos(phi) * tan(theta) * wz; 
        theta_dt = cos(phi) * wy - sin(phi) * wz; 

        phi_gyro = phi + phi_dt * dt; 
        theta_gyro = theta + theta_dt * dt; 

        phi_hat = alpha* phi_acc + (1- alpha) * phi_gyro; 
        theta_hat = alpha * theta_acc + (1- alpha) * theta_gyro;

    
        phi = phi_hat; 
        theta = theta_hat;

        // psi_y_acc = mz * sin(theta_acc) - my * cos (theta_acc); 
        // psi_x_acc = mx * cos(phi_acc) + sin(phi_acc)*(my*sin(theta_acc)+ mz*cos(theta_acc));
        // psi_acc = atan2(-psi_y_acc, psi_x_acc);

        // psi_gyro = psi + wz * dt; 

        // psi_hat = (1-alpha2) * psi_gyro + (alpha2) * psi_acc;
        // psi = psi_hat; 



    
        phi_deg = phi * 57.324;
        theta_deg = theta * (57.324);
        // psi_deg = psi * (57.324);
    



        if (theta_deg > 60.0 || theta_deg < -60.0 || phi_deg > 60.0 || phi_deg < -60.0)
        {
            speaker = 0.5; 
            myleds = alert_count % 16;
            alert_count ++; 

        }
        else
        {
            speaker = 0.0;
            alert_count = 0;
            myleds = alert_count;
        }

        // sprintf(msg, "Theta (pitch): %f\n", theta_deg); 
        // pc.write(msg, strlen(msg)); 
        
        // sprintf(msg, "Phi(roll): %f\n", phi_deg); 
        // pc.write(msg, strlen(msg));
        // sprintf(msg, "Psi (yaw): %f\n", psi_deg); 
        // pc.write(msg, strlen(msg));

        // r = 1.0; 
        // p = 1.0;

        scaled_r = abs(phi/90.0);
        scaled_p = abs(theta/90.0);
        if (scaled_r > 0.6)
        {
            blink_rled();
        }
        else
        {
            r = scaled_r;
            wait_us(1e6);
        }

        if (scaled_p > 0.6)
        {
            blink_pled();
        }
        else 
        {
            p = scaled_p;
            wait_us(1e6);
        }
        

        

        snprintf(data_to_send, sizeof(data_to_send), "%.2f,%.2f\n", phi_deg, theta_deg);
        pc.write(data_to_send, strlen(data_to_send));
        
        updateTime();
        wait_us(1e6/10);
      
        
        }
  return 0;
}








    
  