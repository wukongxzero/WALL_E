#ifndef ADC_PROPERLLOR_SPI
#define ADC_PROPERLLOR_SPI
void adc_init(int csPin, int sclPin, int doPin, int diPin);
void test_adc_heartbeat();
int adc_in(int channel);
#endif
