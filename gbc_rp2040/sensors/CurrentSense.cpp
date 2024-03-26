#include "CurrentSense.h"

CurrentSense::CurrentSense(uint32_t aSensePin, uint32_t bSensePin, int indexA, int indexB) {
    adc_init();
    adc_gpio_init(aSensePin);
    adc_gpio_init(bSensePin);
    index_a = indexA;
    index_b = indexB;
}

void CurrentSense::setup() {
    adc_select_input(index_a);
    adc_a_offset = adc_read();
    adc_select_input(index_b);
    adc_b_offset = adc_read();
}

void CurrentSense::update() {
    adc_select_input(index_a);
    uint16_t adc_a = adc_read();
    adc_a = (adc_a > adc_a_offset) ? adc_a -  adc_a_offset : 0;

    adc_select_input(index_b);
    uint16_t adc_b = adc_read();
    adc_b = (adc_b > adc_b_offset) ? adc_b -  adc_b_offset : 0;

    ia = adc_to_current(adc_a, adc_a_max);
    ib = adc_to_current(adc_b, adc_b_max);
}

float CurrentSense::adc_to_current(uint16_t adc_val, uint16_t maxVal) {
    return float(adc_val/float(maxVal));
}