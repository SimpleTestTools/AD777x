#include <SPI.h>

// Make the C-style headers work
extern "C"
{
    #include "ad7779.h"
    #include "error.h"
}

ad7779_dev ad777x;

void setup()
{
    delay(5000);
    Serial.begin(115200);

    // Enable ~CS pin
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);

    SPI.begin();

    ad7779_init_param init_param = { 0 };

    // C++ doesn't support dot-style struct initialization, apparently?
    init_param.spi_init.chip_select = 10;
    init_param.gpio_reset.number = 9;
    init_param.ctrl_mode = AD7779_SPI_CTRL;
	init_param.spi_crc_en = AD7779_ENABLE;
    for (size_t i = 0; i < 8; i++)
    {
        init_param.state[i] = AD7779_ENABLE;
    }
    for (size_t i = 0; i < 8; i++)
    {
        init_param.gain[i] = AD7779_GAIN_1;
    }
    init_param.dec_rate_int = 0x0000;
    init_param.dec_rate_dec = 0x0000;
    init_param.ref_type = AD7779_INT_REF;
    init_param.pwr_mode = AD7779_HIGH_RES;
    init_param.dclk_div = AD7779_DCLK_DIV_1;
    // init_param.sync_offset = { 0 };
    // init_param.offset_corr = { 0 };
    // init_param.gain_corr = { 0 };
    init_param.ref_buf_op_mode[0] = AD7779_REF_BUF_ENABLED;
    init_param.ref_buf_op_mode[1] = AD7779_REF_BUF_ENABLED;
    init_param.sinc5_state = AD7779_DISABLE;

    int32_t ret = ad7779_init(&ad777x, init_param);
    if (SUCCESS != ret)
    {
        Serial.println(ret);
        delay(50);
        Serial.println();

        uint8_t reg = 0;
		int32_t r = ad7779_spi_int_reg_read(&ad777x,
					       AD7779_REG_GEN_ERR_REG_1_EN,
					       &reg);
        Serial.print(r);
        Serial.println();
        Serial.print(reg, HEX);
        Serial.println();

        Serial.println("PANIC: Failed to initialize ad777x!");
        while (1) { }
    }
}

void loop()
{
    uint16_t sar_code = 0;
    int32_t ret = ad7779_do_single_sar_conv(&ad777x, AD7779_AVDD1A_AVSSX_ATT, &sar_code);
    Serial.print(ret);
    Serial.print(" ");
    Serial.print(sar_code);
    Serial.println();

    delay(2000);
}

/* delay.h */
void mdelay(uint32_t msecs)
{
    Serial.print("delay ");
    Serial.println(msecs);
    delay(msecs);
}


/* gpio.h */
struct gpio_desc gpio_9 = {
    .number = 9,
};

int32_t gpio_get(struct gpio_desc **desc,
		 const struct gpio_init_param *param)
{
    int32_t success = SUCCESS;

    switch (param->number)
    {
    case 0:
        // If we don't actually have a GPIO configured, just return success
        *desc = NULL;
        break;
    case 9:
        *desc = &gpio_9;
        break;
    default:
        *desc = NULL;
        success = FAILURE;
        break;
    }

    return success;
}

int32_t gpio_set_value(struct gpio_desc *desc,
		       uint8_t value)
{
    if (NULL == desc)
    {
        return SUCCESS;
    }

    Serial.print("gpio_set_value ");
    Serial.print(micros());
    Serial.print(" ");
    Serial.print(desc->number);
    Serial.print(" ");
    Serial.print(value);
    Serial.println();

    digitalWrite(desc->number, value);

    return SUCCESS;
}

int32_t gpio_direction_output(struct gpio_desc *desc,
			      uint8_t value)
{
    if (NULL == desc)
    {
        return SUCCESS;
    }

    pinMode(desc->number, OUTPUT);

    return gpio_set_value(desc, value);
}

/* spi.h */
struct spi_desc spi_dev;

int32_t spi_init(struct spi_desc **desc,
		 const struct spi_init_param *param)
{
    spi_dev.chip_select = param->chip_select;
    *desc = &spi_dev;

    return SUCCESS;
}

int32_t spi_write_and_read(struct spi_desc *desc,
			   uint8_t *data,
			   uint16_t bytes_number)
{
    digitalWrite(desc->chip_select, LOW);
    Serial.print("TX: [ ");
    for (int i = 0; i < bytes_number; i++)
    {
        Serial.print("0x");
        Serial.print(data[i], HEX);
        Serial.print(", ");
    }
    Serial.println();
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(data, bytes_number);
    SPI.endTransaction();
    Serial.print("RX: [ ");
    for (int i = 0; i < bytes_number; i++)
    {
        Serial.print("0x");
        Serial.print(data[i], HEX);
        Serial.print(", ");
    }
    Serial.println("]");
    digitalWrite(desc->chip_select, HIGH);

    return SUCCESS;
}
