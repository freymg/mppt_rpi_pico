#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define PWM_SW1 16
#define PWM_SW2 18

#define I2C_PORT i2c0

static uint8_t addr = 0x40;
float previousVoltage = 0.0;
float previousCurrent = 0.0;
float current_val, voltage_val; // Float type of meassured data

bool repeating_timer_callback(struct repeating_timer *t)
{
    printf("%f \n", voltage_val);
    return true;
}

void ina219_init()
{
    uint8_t conf_val[3];
    conf_val[0] = 0x00;
    conf_val[1] = 0x01;
    conf_val[2] = 0x9F;
    uint8_t cal_val[3];
    cal_val[0] = 0x05;
    cal_val[1] = 0x85;
    cal_val[2] = 0x54;

    // escribe los registros de configuración y calibracion del modulo ina219
    sleep_ms(30);
    i2c_write_blocking(I2C_PORT, addr, conf_val, 3, false);
    sleep_ms(30);
    i2c_write_blocking(I2C_PORT, addr, cal_val, 3, false);
    sleep_ms(30);
}

bool mppt_smd(float voltage, float current)
{
    float diffvpv;
    float diffcpv;
    float swfcn;
	float deltaVoltage = 0.0;
	float deltaCurrent = 0.0;
    bool D;
	
	deltaVoltage = voltage - previousVoltage;
	deltaCurrent = current - previousCurrent;
	swfcn = (deltaVoltage/deltaCurrent)+(voltage/current);
	previousVoltage = voltage;
	previousCurrent = current;
	
	if(swfcn < 0)
	{
		D = true;
	}
	
	if(swfcn > 0)
	{
		D = false;
	}
	
	return D;
}

int main()
{
    uint8_t bus_voltage = 0x02;
    uint8_t current_dir = 0x04;

    bool level;

    uint8_t current_i2c[2], voltage_i2c[2];
    voltage_val = 0;
    stdio_init_all(); // Iniciliza STD I/O para comunicacion a traves de puerto serie

    // Configura la comunicación I2C
    i2c_init(I2C_PORT, 400000);
    gpio_init(12);
    gpio_init(13);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);

    gpio_init(PWM_SW1);
    gpio_init(PWM_SW2);
    gpio_set_dir (PWM_SW1, true);
    gpio_set_dir (PWM_SW2, true);

    // inicializa modulo ina219
    ina219_init();

    //define una interrupcion por timer a ejecutarse continuamente cada 10 segundos
    struct repeating_timer timer;
    add_repeating_timer_ms(10000, repeating_timer_callback, NULL, &timer);

    // ciclo principal
    while(true)
    {
        voltage_val = 0;
        i2c_write_blocking(I2C_PORT, addr, &current_dir, 1, false);
        i2c_read_blocking(I2C_PORT, addr, current_i2c, 2, false);
        current_val = ((current_i2c[0] << 8) | current_i2c[1])*12;

        i2c_write_blocking(I2C_PORT, addr, &bus_voltage, 1, false);
        i2c_read_blocking(I2C_PORT, addr, voltage_i2c, 2, false);
        voltage_val = ((voltage_i2c[0] << 5) | (voltage_i2c[1] >> 3))*4;

        level = mppt_smd(voltage_val, current_val);
        gpio_put(PWM_SW1, level);
        gpio_put(PWM_SW2, !level);
    }
}