#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#define PWM_SW1 16
#define PWM_SW2 18

#define I2C_PORT i2c0

static uint8_t addr = 0x40;
float previousVoltage = 0.0;
float previousCurrent = 0.0;

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

void setup_pwm(uint pin, uint slice, uint channel)
{
    // configurar gpio
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    // configurar modulo PWM
    pwm_set_clkdiv(slice, 2.0);
    pwm_set_wrap(slice, 624);
    pwm_set_chan_level(slice, channel, 312);
}

float mppt_incond(float voltage, float current)
{
	float deltaCurrent;
	float deltaVoltage;
    float D;
    float Dstep = 0.001;
    float Dmax = 0.9;
    float Dmin = 0.1;

	deltaVoltage = voltage - previousVoltage;
	deltaCurrent = current - previousCurrent;

	if (deltaVoltage == 0)
	{
		if (deltaCurrent != 0)
		{
			if (deltaCurrent > 0)
			{
				D = D - Dstep;
			}
			else
			{
				D = D + Dstep;
			}
		}
		else
		{
			D = D;
		}
	}
	else if ((deltaCurrent / deltaVoltage) != (-current / voltage))
	{
		if (((deltaCurrent / deltaVoltage) > (-current / voltage)))
		{
			D = D - Dstep;
		}
		else
		{
			D = D + Dstep;
		}
	}
	else
	{
		D = D;
	}

	if (D > Dmax)
	{
		D = Dmax;
	}
	else if (D < Dmin)
	{
		D = Dmin;
	}

	previousVoltage = voltage;
	previousCurrent = current;

	return (624*D);
}

int main()
{
    uint slice_a =  pwm_gpio_to_slice_num(PWM_SW1);
    uint slice_b =  pwm_gpio_to_slice_num(PWM_SW2);
    uint channel_a = pwm_gpio_to_channel(PWM_SW1);
    uint channel_b = pwm_gpio_to_channel(PWM_SW2);

    uint8_t bus_voltage = 0x02;
    uint8_t current_dir = 0x04;
    uint8_t power_dir = 0x03;

    uint16_t level;

    uint8_t current_i2c[2], voltage_i2c[2], power_i2c[2];
    float current_val, voltage_val, power_val; // Float type of meassured data
    stdio_init_all(); // Iniciliza STD I/O para comunicacion a traves de puerto serie

    // Configura la comunicación I2C
    i2c_init(I2C_PORT, 400000);
    gpio_init(12);
    gpio_init(13);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);

    // inicializa modulo ina219
    ina219_init();

    //i nicializa modulos pwm y desfasa el segundo 180 grados
    setup_pwm(PWM_SW1, slice_a, channel_a);
    setup_pwm(PWM_SW2, slice_b, channel_b);
    pwm_set_mask_enabled(0x00000003);
    for(int j = 0; j <= 311; j++)
    {
        pwm_retard_count(slice_b);
    }

    printf("V_cell duty \n");

    // ciclo principal
    while(true)
    {
        voltage_val = 0;
        i2c_write_blocking(I2C_PORT, addr, &current_dir, 1, false);
        i2c_read_blocking(I2C_PORT, addr, current_i2c, 2, false);
        current_val = ((current_i2c[0] << 8) | current_i2c[1])*12;

        i2c_write_blocking(I2C_PORT, addr, &bus_voltage, 1, false);
        i2c_read_blocking(I2C_PORT, addr, voltage_i2c, 2, false);
        voltage_val = (((voltage_i2c[0] << 5) | (voltage_i2c[1] >> 3))*4)/1000;

        level = (unsigned int) mppt_incond(voltage_val, current_val);

        pwm_set_chan_level(slice_a, channel_a, level);
        pwm_set_chan_level(slice_b, channel_b, level);
        printf("%f %d \n", voltage_val, level);

        busy_wait_ms(10000);
    }
}