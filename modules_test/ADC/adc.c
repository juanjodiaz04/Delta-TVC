#include "adc.h"

void adc_util_init(adc_t* adc, uint8_t gpio_pin, uint8_t adc_input, float vref)
{
    adc->adc_input = adc_input;
    adc->gpio_pin = gpio_pin;
    adc->vref = vref;

    adc_init();
    adc_gpio_init(gpio_pin);
    adc_select_input(adc_input);
}

uint16_t adc_util_read_raw(adc_t* adc)
{
    adc_select_input(adc->adc_input);
    return adc_read();
}

float adc_util_read_voltage(adc_t* adc)
{
    uint16_t raw = adc_util_read_raw(adc);
    const float conversion_factor = adc->vref / (1 << 12);
    return raw * conversion_factor;
}

float adc_util_read_current_acs712(adc_t* adc, float v_offset, float sensitivity)
{
    float voltage = adc_util_read_voltage(adc);
    float current = (voltage - v_offset) / sensitivity;
    return current;
}