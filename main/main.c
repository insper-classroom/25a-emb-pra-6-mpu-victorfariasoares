#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

typedef struct {
    int axis;   // 0=X, 1=Y, 2=CLICK
    int val;    // valor do movimento (já filtrado e mapeado para -255 ... 255)
} adc_t;

QueueHandle_t xQueueADC;


static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);


    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);


    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    while(1) {
        // leitura da MPU, sem fusao de dados
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = { 
            .axis.x = gyro[0]/131.0f, 
            .axis.y = gyro[1]/131.0f, 
            .axis.z = gyro[2]/131.0f };

        FusionVector accelerometer = { 
             .axis.x = acceleration[0]/16384.0f,
             .axis.y = acceleration[1]/16384.0f, 
             .axis.z = acceleration[2]/16384.0f };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        // 0=X, 1=Y, 2=CLICK
        adc_t valor_X;
        valor_X.axis = 0;
        valor_X.val = e.angle.roll;

        adc_t valor_Y;
        valor_Y.axis = 1;
        valor_Y.val = e.angle.pitch;

        adc_t click_X;
        click_X.axis = 2;
        click_X.val = (accelerometer.axis.x)*100; //talvez multiplicar por alguma coisa pra aumentar o valor e ficar mais facil de avaliar dps

        // enviar tudo para as filas
        xQueueSend(xQueueADC, &valor_X, portMAX_DELAY);
        xQueueSend(xQueueADC, &valor_Y, portMAX_DELAY);

        // pra não ficar apertando sempre colocar um valor minimo de aceleração
        if (fabsf(click_X.val) > 90){
            xQueueSend(xQueueADC, &click_X, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p){
    adc_t adc_data;
    while (true){
        if (xQueueReceive(xQueueADC, &adc_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            // if (adc_data.axis == 0){ // se for x
            //     printf("Dado recebido X: %d\n", adc_data.val); 
            // }
            // else if (adc_data.axis == 1){
            //     printf("Dado recebido Y: %d\n", adc_data.val);
            // }
            //Envio para uart, tem que ser todos char???
            // uint8_t sync = 0xFF;
            uint8_t eixo = adc_data.axis;
            uint16_t valor = adc_data.val;
            uint8_t msb = (uint8_t)((valor >> 8) & 0xFF);
            uint8_t lsb = (uint8_t)(valor & 0xFF);
            uint8_t eop = 0xFF;  // end of package
            // putchar(sync);
            putchar_raw(eixo);
            putchar_raw(lsb);
            putchar_raw(msb);
            putchar_raw(eop);
        } 
        // else {
        //     // Caso ocorra timeout (nenhum dado recebido em 100 ms)
        //     // printf("Timeout ao aguardar dado na fila.\n");
        // }
    }
}

int main() {
    stdio_init_all();
    xQueueADC = xQueueCreate(32, sizeof(adc_t));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart task", 4095, NULL, 1, NULL);
    

    vTaskStartScheduler();

    while (true)
        ;
}
