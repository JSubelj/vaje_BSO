/* Simple example for I2C / BMP180 / Timer & Event Handling
 *
 * This sample code is in the public domain.
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "i2c/i2c.h"

#include "queue.h"


#define PCF_ADDRESS 0x38

#define I2C_BUS	0
#define SCL_PIN 14
#define SDA_PIN 12

#define BUTTON1	0x20
#define BUTTON2	0X10
#define BUTTON3	0X80
#define BUTTON4	0X40

#define LED1	0xfe
#define LED2	0xfd
#define LED3	0xfb
#define LED4	0xf7

void delay_ms(int m_sec){
	vTaskDelay(m_sec / portTICK_PERIOD_MS);
}

void task1(void *pvParameters){
	uint8_t data;

	while(1){
		data = LED3;
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
		delay_ms(1000);
		data = 0xff;
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
		delay_ms(1000);

		data = LED4;
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
		delay_ms(1000);
		data = 0xff;
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
		delay_ms(1000);
		
	}
}

void task_read_buttons(void *pvParameters){
	uint8_t data;
	uint8_t tmp;
	while(1){
		i2c_slave_read(I2C_BUS,PCF_ADDRESS,NULL,&data,1);
		tmp = 0xff;
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &tmp, 1);
		if((data & BUTTON1) == 0){
			tmp = LED1;
		}else if((data & BUTTON2) == 0){
			tmp = LED2;
		}else if((data & BUTTON3) == 0){
			tmp = LED3;
		}else if((data & BUTTON4) == 0){
			tmp = LED4;
		}
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &tmp, 1);

		delay_ms(10);


	}

}

// Setup HW
void user_setup(void)
{
    // Set UART Parameter
    uart_set_baud(0, 115200);

    // Give the UART some time to settle
    sdk_os_delay_us(500);

	// Init I2C
	i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
	gpio_enable(SCL_PIN, GPIO_OUTPUT);
}

extern "C" void user_init(void); // one way
void user_init(void)
{
	// Setup HW
    user_setup();

    //xTaskCreate(task1, "task1", 512, NULL, 2, NULL);
    xTaskCreate(task_read_buttons, "task_read_buttons", 512, NULL, 2, NULL);

}
