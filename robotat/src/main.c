
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include <math.h>

#include "bldc.h"

#include "i2c.h"
#include "mpu9250.h"
#include "magAK8963.h"
#include "driver/adc.h"
#include "mpu9250_calibrate.h"
#include "Madgwick.h"
#include "robotat_linalg.h"
#include "robotat_control.h"
//#include "robotat.h"


//----------------------------------------------BLDC---------------------------------------------------------------------

#define GPIO_MOTOR1    26
#define GPIO_MOTOR2    14
#define GPIO_MOTOR3    12
#define GPIO_MOTOR4    2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_MOTOR1) | (1ULL<<GPIO_MOTOR2) | (1ULL<<GPIO_MOTOR3) | (1ULL<<GPIO_MOTOR4))

#define LEDC_CH0_MOTOR1        LEDC_CHANNEL_0
#define LEDC_CH1_MOTOR2        LEDC_CHANNEL_1
#define LEDC_CH2_MOTOR3        LEDC_CHANNEL_2
#define LEDC_CH3_MOTOR4        LEDC_CHANNEL_3

#define LEDC_TIMER_MOTOR1      LEDC_TIMER_0
#define LEDC_TIMER_MOTOR2      LEDC_TIMER_1
#define LEDC_TIMER_MOTOR3      LEDC_TIMER_2
#define LEDC_TIMER_MOTOR4      LEDC_TIMER_3
//--------------------------------------CONSTANTES CONTROLADOR-----------------------------------------------------------

#define SCALE (0.0006f)
#define KP_roll (50000*SCALE)
#define KD_roll (0*SCALE)

#define KP_pitch (40000*SCALE)
#define KD_pitch (0*SCALE)

#define KP_yaw (90000*SCALE)
#define KD_yaw (275000*SCALE)

#define des_roll (0)
#define des_p (0)
#define des_pitch (0)
#define des_q (0)
#define des_yaw (0)  //Programar para que sea el valor actual, así no se debe colocar en un punto específico
#define des_r (0)


//----------------------------------------------MPU-9250------------------------------------------------------------------

#define SAMPLE_FREQ_Hz (100)
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_FREQ_Hz) // Sample Rate in milliseconds

#define DEG2RAD(deg) (deg * M_PI / 180.0f)

uint64_t i = 0;

calibration_t cali = {
    .mag_offset = {.x = 105.931641, .y = 10.652344, .z = 49.617188},
    .mag_scale = {.x = 0.938692, .y = 0.954877, .z = 1.126847},
    .accel_offset = {.x = 0.032609, .y = -0.060470, .z = -0.785884},
    .accel_scale_lo = {.x = 1.003865, .y = 0.973296, .z = 0.899577},
    .accel_scale_hi = {.x = -0.992302, .y = -1.023748, .z = -2.254087},

    .gyro_bias_offset = {.x = -2.407657, .y = 0.631795, .z = 0.232099}};

bldc_esc_t motor1;
bldc_esc_t motor2;
bldc_esc_t motor3;
bldc_esc_t motor4;


//uint8_t device_id;
//uint8_t AK8963_id;
vector_t vg;
vector_t va;
vector_t vm;

bool is_first_run = true;
bool init_controller = false;
bool init_suave = false;
int incremento = 3000;
int vel_max_init = 5000;
int vel_max = 7000;
int test = 10000; 
uint16_t cont = 0;

//----------------------------------------TRANSFORMACIÓN LECTURAS DE ÁNGULOS-------------------------------------


static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */


static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

//----------------------------------Variables de controlador---------------------------------------------

//Declaración variables para matrices
matf32_t pqr_transf, rpy_dot, u2, w_transf, Vel_motor_des, w_u, K, X, Xss, e, pqr;
pid_info_t pidp;
pid_info_t pidq;
pid_info_t pidr;

//Declaración de parámetros del drone
float m = 0.7; //kg
float g = 9.8;   

//Variables para lectura de datos
float roll, pitch, yaw = 0;
float rolld, pitchd, yawd = 0;
float roll_prev, pitch_prev, yaw_prev = 0;
float dt = 0.01;    //Este valor hay que medirlo o establecerlo
float p, q, r = 0;

//Datos para matrices
float pqr_transf_data[3 * 3];
float rpy_dot_data[3 * 1];
float u2_data[3 * 1];
/*float w_transf_data[] = {1,  0, -1,  1,
                  1,  1,  0, -1,
                  1,  0,  1,  1,
                  1, -1,  0, -1};*/
float w_transf_data[] = {1,  0, 0,  1,
                        1,  0,  0, -1,
                        1,  0,  0,  1,
                        1, 0,  0, -1};

float Vel_motor_des_data[4 * 1];
float w_u_data[4 * 1];
float K_data[] = {KP_roll,     0,       0,     KD_roll,       0,           0,
                  0,       KP_pitch,    0,        0,       KD_pitch,       0,
                  0,           0,     KP_yaw,     0,            0,       KD_yaw};
float X_data[6*1];
float Xss_data[] = {des_roll, des_pitch, des_yaw, des_p, des_q, des_r};
float e_data[6*1];
float pqr_data[3*1];
float e_nonorm = 0;
float e_r_prev = 0;


//Variables para PID (Hover)
float u1 = 0;


//Variables para  velocidades de motor
float wF = 3500;
float wh = 0;
float w_motor1, w_motor2, w_motor3, w_motor4 = 0;    //Para empezar fuera de la zona muerta del BLDC 

//------------------------------- Prototipos de funciones-------------------------------------------------

// Creación de estructura para semáforo para protección de datos
SemaphoreHandle_t sem_binario;
void task_LecturaAngulos(void *param);
void task_Controlador(void *param);

//---------------------------------------------------------------------------------------------------------
void app_main(void)
{
  
   //-------------------------------------------MPU-9250--------------------------------------------------- 
    
    i2c_mpu9250_init(&cali);
    MadgwickAHRSinit(SAMPLE_FREQ_Hz, 3*0.8);
      
  //-------------------------------------------BLDC-------------------------------------------------------  
    
    
    bldc_esc_init(&motor1, 1000, 12.0, 2.0, 1.0);
    bldc_esc_init(&motor2, 1000, 12.0, 2.0, 1.0);
    bldc_esc_init(&motor3, 1000, 12.0, 2.0, 1.0);
    bldc_esc_init(&motor4, 1000, 12.0, 2.0, 1.0);
    
    config_gpio(GPIO_OUTPUT_PIN_SEL);

    PWM_Init(GPIO_MOTOR1, LEDC_CH0_MOTOR1, LEDC_TIMER_MOTOR1);
    PWM_Init(GPIO_MOTOR2, LEDC_CH1_MOTOR2, LEDC_TIMER_MOTOR2);
    PWM_Init(GPIO_MOTOR3, LEDC_CH2_MOTOR3, LEDC_TIMER_MOTOR3);
    PWM_Init(GPIO_MOTOR4, LEDC_CH3_MOTOR4, LEDC_TIMER_MOTOR4);
    
    inicializacion_ESC_drone();

    //---------------------------------------------Controlador---------------------------------------------------------
    
    //wh = sqrtf(m*g/4);   //Velocidad de cada motor para hover F = mg/4

    //Inicialización de matrices
  
    matf32_init(&pqr_transf, 3, 3, pqr_transf_data);
    matf32_init(&rpy_dot, 3, 1, rpy_dot_data);
    matf32_init(&u2, 3, 1, u2_data);
    matf32_init(&w_transf, 4, 4, w_transf_data);
    matf32_init(&Vel_motor_des, 4, 1, Vel_motor_des_data);
    matf32_init(&w_u, 4, 1, w_u_data);
    matf32_init(&K, 3, 6, K_data);
    matf32_init(&X, 6, 1, X_data);
    matf32_init(&Xss, 6, 1, Xss_data);
    matf32_init(&pqr, 3, 1, pqr_data);
    matf32_init(&e, 6, 1, e_data);

    //Conexión al sistema OptiTrack a través de MQTT
    //robotat_connect();

    //Creación del semáforo
    sem_binario = xSemaphoreCreateMutex();

    // Creación de los tasks
    //Task "productor" de datos
    xTaskCreate(task_LecturaAngulos, "LecturaAngulos", 1024*2, NULL, 3, NULL);
    //xSemaphoreTake(sem_binario, portMAX_DELAY);
    //Task "consumidor" de datos
    xTaskCreate(task_Controlador, "Controlador", 1024*2, NULL, 3, NULL);

}

void task_LecturaAngulos(void *param){

    
    while(1){

        //-----------------------------------------OBTENER ANGULOS-----------------------------------------------------
        
        //calibrate_gyro();
        //calibrate_mag();

            
        // Get the Accelerometer, Gyroscope and Magnetometer values.
        ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

        // Transform these values to the orientation of our device.
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);

        // Apply the AHRS algorithm
        MadgwickAHRSupdate(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                            va.x, va.y, va.z,
                            vm.x, vm.y, vm.z);
                

        xSemaphoreTake(sem_binario, portMAX_DELAY);

        MadgwickGetEulerAnglesDegrees(&yaw, &pitch, &roll);
        roll = -roll+90;
        //roll = -roll+85.3;
        yaw = -yaw+180;

        //printf("Roll: %f \t Pitch: %f \t Yaw: %f\n", roll, pitch, yaw);
        yaw = yaw*M_PI/180.0;
        roll = roll*M_PI/180.0;
        pitch = pitch*M_PI/180.0;

        //Release el semáforo
        xSemaphoreGive(sem_binario);

    }

    //Borra el task automáticamente (necesario para multi-tasking)
    vTaskDelete(NULL);

}

void task_Controlador(void *param){

    //Variables locales para lectura de datos de task2
    float roll2, pitch2, yaw2 = 0;

    //Declaración de variables para bloquear la función un dt fijo
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; //Modificar al valor de ms deseado

    //Inicialización de variable de última vez despertado el task
    xLastWakeTime = xTaskGetTickCount();
    

  
    while(1){

        //Función que permite bloquear el task el tiempo determinado
        vTaskDelayUntil(&xLastWakeTime, xFrequency );
        
        // Revisión del semáforo libre para lectura de ángulos
        xSemaphoreTake(sem_binario, portMAX_DELAY);

        roll2 = roll;
        pitch2 = pitch;
        yaw2 = yaw;

        //Release el semáforo
        xSemaphoreGive(sem_binario);

        if(init_controller == false){
            cont = cont+1;
        }
        
        if (cont == 1100){
            init_suave = true;
            cont = 0;
        }

        if(init_suave == true){
            for(int j=0; j<1001; j++){
            /*PWM_output(&motor1, LEDC_CH0_MOTOR1, incremento+500);
            PWM_output(&motor2, LEDC_CH1_MOTOR2, incremento);
            PWM_output(&motor3, LEDC_CH2_MOTOR3, incremento);
            PWM_output(&motor4, LEDC_CH3_MOTOR4, incremento);*/
            PWM_output(&motor1, LEDC_CH0_MOTOR1, 0);
            PWM_output(&motor2, LEDC_CH1_MOTOR2, 0);
            PWM_output(&motor3, LEDC_CH2_MOTOR3, 0);
            PWM_output(&motor4, LEDC_CH3_MOTOR4, 0);

            incremento = incremento+1;
            //printf("Incremento: %d\n",  incremento);
            }
            if(incremento > 4000){
            init_controller = true;
            init_suave = false;
            }
        }  
        
        //-------------------------------------IMPLEMENTACIÓN CONTROLADOR----------------------------------------------

        if (init_controller == true){
            // Cálculo velocidad de roll, pitch, yaw
            rolld = -(roll_prev - roll2)/dt;
            roll_prev = roll2;
            pitchd = -(pitch_prev - pitch2)/dt;
            pitch_prev = pitch2;
            yawd = -(yaw_prev - yaw2)/dt;
            yaw_prev = yaw2;

            rpy_dot.p_data[0] = rolld;
            rpy_dot.p_data[1] = pitchd;
            rpy_dot.p_data[2] = yawd;

            // mat(i,j) = mat.p_data[i*mat.num_cols + j] 
            pqr_transf.p_data[0*3 + 0] = cosf(pitch2);
            pqr_transf.p_data[0*3 + 1] = 0;
            pqr_transf.p_data[0*3 + 2] = -cosf(roll2)*sinf(pitch2);
            pqr_transf.p_data[1*3 + 0] = 0;
            pqr_transf.p_data[1*3 + 1] = 1;
            pqr_transf.p_data[1*3 + 2] = sinf(roll2);
            pqr_transf.p_data[2*3 + 0] = sinf(pitch2);
            pqr_transf.p_data[2*3 + 1] = 0;
            pqr_transf.p_data[2*3 + 2] = cosf(roll2)*cosf(pitch2);

            //Cálculo de p, q, r
            matf32_mul(&pqr_transf, &rpy_dot, &pqr);



            //Actualización de vector X
            X.p_data[0] = roll2;
            X.p_data[1] = pitch2;
            X.p_data[2] = yaw2;
            X.p_data[3] = pqr.p_data[0];
            X.p_data[4] = pqr.p_data[1];
            X.p_data[5] = pqr.p_data[2];

            //Controlador ----------------------------------------
            matf32_sub(&Xss, &X,&e);

            //Aquí se arregla el problema del wrap around
            e_nonorm = e.p_data[2];
            e.p_data[2] = atan2f(sinf(e_nonorm),cosf(e_nonorm));

            e.p_data[5] = ((e.p_data[2]-e_r_prev)/dt)*-1;
            e_r_prev = e.p_data[2];
            matf32_mul(&K, &e, &u2);

            // cálculo de u1 con aceleración en z
            u1 = wh + wF;

            //Conversión a valores del motor
            w_u.p_data[0] = u1;
            w_u.p_data[1] = u2.p_data[0];
            w_u.p_data[2] = u2.p_data[1];
            w_u.p_data[3] = u2.p_data[2];

            matf32_mul(&w_transf, &w_u, &Vel_motor_des);   //Se obtienen las 4 velocidades deseadas para motores

            //matf32_print(&Vel_motor_des);
            //vTaskDelay(500 / portTICK_PERIOD_MS);
            
            w_motor1 = saturation(Vel_motor_des.p_data[0], 0, vel_max);
            w_motor2 = saturation(Vel_motor_des.p_data[1], 0, vel_max);
            w_motor3 = saturation(Vel_motor_des.p_data[2], 0, vel_max);
            w_motor4 = saturation(Vel_motor_des.p_data[3], 0, vel_max);


            if(is_first_run ==  false){
                PWM_output(&motor1, LEDC_CH0_MOTOR1, w_motor1);
                PWM_output(&motor2, LEDC_CH1_MOTOR2, w_motor2);
                PWM_output(&motor3, LEDC_CH2_MOTOR3, w_motor3);
                PWM_output(&motor4, LEDC_CH3_MOTOR4, w_motor4);
                //printf("w_motor1: %f \n w_motor2: %f \n w_motor3: %f\n w_motor4: %f\n \n", w_motor1, w_motor2, w_motor3, w_motor4);
            }

            if(is_first_run == true){
                PWM_output(&motor1, LEDC_CH0_MOTOR1, vel_max_init);
                PWM_output(&motor2, LEDC_CH1_MOTOR2, vel_max_init+600);
                PWM_output(&motor3, LEDC_CH2_MOTOR3, vel_max_init);
                PWM_output(&motor4, LEDC_CH3_MOTOR4, vel_max_init);
                printf("M1: %d\n", vel_max_init);
                is_first_run = false;
            }
            
        }
          
    }

    //Borra el task automáticamente (necesario para multi-tasking)
    vTaskDelete(NULL);
}


//------------------------------ Funciones-----------------------------------------------------------------
/*
void pause(void){

  static uint64_t start = 0;
  uint64_t end = xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;

  if (start == 0)
  {
    start = xTaskGetTickCount() / configTICK_RATE_HZ;
  }

  int32_t elapsed = end - start;
  if (elapsed < SAMPLE_INTERVAL_MS)
  {
    vTaskDelay((SAMPLE_INTERVAL_MS - elapsed) / portTICK_RATE_MS);
  }
  start = xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;
}
*/

