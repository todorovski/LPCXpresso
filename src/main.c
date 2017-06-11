#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "light.h"
#include "oled.h"
#include "temp.h"
#include "acc.h"
#include "math.h"
#include "rotary.h"
#include "led7seg.h"

#include "pca9532.h"
#include "acc.h"
#include "rgb.h"

#include "joystick.h"
#include "eeprom.h"

#define UART_DEV LPC_UART3

static void init_uart(void)
{
	PINSEL_CFG_Type PinCfg;
	UART_CFG_Type uartCfg;

	/* Initialize UART3 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	UART_Init(UART_DEV, &uartCfg);

	UART_TxCmd(UART_DEV, ENABLE);

}


enum STATE
{
	TEMPERATURE_STATE = 0,
	LIGHT_STATE = 1,
	POTENCIOMETER_STATE = 2,
	ROTATORY_STATE = 3,
	EE_READ = 4,
	EE_WRITE = 5,
	EEPROM_OPER = 6
};

enum STATE current_state, previos_state;

static uint32_t msTicks = 0;
static uint8_t buf[10];

static uint16_t screen_draw_pos = 0;

//temperature variables
//static uint8_t temperatures[300]={20};
static uint16_t temperature_pos = 1;
static uint64_t temperature_sum = 23;
static float_t avg_temperature = 23;

// light variables
static uint8_t light_values[90] = {0};
static uint16_t light_pos = 0;
static uint64_t light_sum = 0;
static float_t avg_light = 0;
static uint8_t written_lights=0;

//rotatory variables
static uint64_t rotatory_sum=0;
static float_t avg_rotatory = 0;
static uint16_t rotatory_count=0;
static uint8_t rotator_value;

//potenc_variables
static uint64_t potenc_sum=0;
static float_t avg_potenc = 0;
static uint16_t potenc_count=0;
static uint8_t potenc_value;

//eeprom variables
#define E_WRITE_LEN 96
BOOL_8 eeprom_oper = FALSE;

uint16_t offset = 240;
uint16_t b[E_WRITE_LEN];
int16_t len = 0;

//joystick variables
uint8_t joy = 0;
uint8_t joy_val = 0;

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    // the buffer must not be null and at least have a length of 2 to handle one
    // digit and null-terminator
    if (pBuf == NULL || len < 2)
    {
        return;
    }

    // a valid base cannot be less than 2 or larger than 36
    // a base value of 2 means binary representation. A value of 1 would mean only zeros
    // a base larger than 36 can only be used if a larger alphabet were used.
    if (base < 2 || base > 36)
    {
        return;
    }

    // negative value
    if (value < 0)
    {
        tmpValue = -tmpValue;
        value    = -value;
        pBuf[pos++] = '-';
    }

    // calculate the required length of the buffer
    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);


    if (pos > len)
    {
        // the len parameter is invalid.
        return;
    }

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = pAscii[value % base];
        value /= base;
    } while(value > 0);

    return;

}

static void write_to_eeprom()
{
	oled_clearScreen(OLED_COLOR_WHITE);
	written_lights = light_pos;

	len = eeprom_write(light_values, offset, written_lights);

	UART_SendString(UART_DEV, (uint8_t*)"EEPROM example\r\n");

	if (len != light_pos) {
		UART_SendString(UART_DEV, (uint8_t*)"EEPROM: Failed to write data\r\n");
		return 1;
	}

	UART_SendString(UART_DEV, (uint8_t*)"EEPROM: Data written\r\n");

	intToString(written_lights, buf, 10, 10);
	oled_putString(0,20, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	oled_putString(0,50, "OK WRITTING", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	Timer0_Wait(2000);
	eeprom_oper = FALSE;
}

static void read_from_eeprom()
{
	memset(b, 0, written_lights);

	UART_SendString(UART_DEV, (uint8_t*)"EEPROM: Reading\r\n");
	len = eeprom_read(b, offset, written_lights);

	int i;
	for (i = 0; i < written_lights; i++)
	{
		intToString(light_values[i], buf, 10, 10);
		oled_putString(0,i*8, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	}

	Timer0_Wait(8000);

    if (len != written_lights) {
    	UART_SendString(UART_DEV, (uint8_t*)"EEPROM: Failed to read all data\r\n");
        oled_putString(0,40, "FAIL", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
        return 1;
    }

    UART_SendString(UART_DEV, (uint8_t*)"EEPROM: Verifing\r\n");
    for (i = 0; i < written_lights; i++)
    {
    	if(b[i] != light_values[i])
    	{
        	UART_SendString(UART_DEV, (uint8_t*)"EEPROM: Invalid data\r\n");
        	oled_putString(0,40, "DIFFERENT", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            return 1;
        }
    }

    UART_SendString(UART_DEV, (uint8_t*)"EEPROM: OK\r\n");
    oled_putString(0,50, "OK R", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    eeprom_oper = FALSE;

    Timer0_Wait(8000);
}

static void checkJoystick()
{
	joy = joystick_read();

	if ((joy & JOYSTICK_CENTER) != 0)
	{
		joy_val++;
		intToString(joy_val, buf, 10, 10);
		oled_putString(50, joy_val*8, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	}

	if ((joy & JOYSTICK_LEFT) != 0)
	{
		eeprom_oper = TRUE;
		current_state = EE_WRITE;
		write_to_eeprom();
	}

	if ((joy & JOYSTICK_RIGHT) != 0)
	{
		eeprom_oper = TRUE;
		current_state = EE_READ;
		read_from_eeprom();
	}
}

static uint8_t height = 64;
static uint8_t gap = 10;

static BOOL_8 changed_screen = TRUE;

void SysTick_Handler(void) {
    msTicks++;
}

static uint32_t getTicks(void)
{
    return msTicks;
}

static void potenciometar(uint32_t value)
{
	value/=128;

	potenc_count++;
	potenc_sum += value;
	avg_potenc = potenc_sum / potenc_count;

    intToString(value, buf, 10, 10);
    oled_fillRect(60,0, 68, 20, OLED_COLOR_WHITE);
	oled_putString(60,0, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

static void show_potenc_graph()
{
	oled_fillRect(screen_draw_pos,height - gap, screen_draw_pos+1, height - gap - avg_potenc, OLED_COLOR_BLACK);
	screen_draw_pos++;
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_adc(void)
{
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init ADC pin connect
	 * AD0.0 on P0.23
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 23;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC :
	 * 	Frequency at 0.2Mhz
	 *  ADC channel 0, no Interrupt
	 */
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,ADC_CHANNEL_0,DISABLE);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_0,ENABLE);

}

static uint8_t ch7seg = '0';
static void change7Seg(uint8_t rotaryDir)
{

    if (rotaryDir != ROTARY_WAIT) {

        if (rotaryDir == ROTARY_RIGHT) {
            ch7seg++;
        }
        else {
            ch7seg--;
        }

        if (ch7seg > '9')
            ch7seg = '0';
        else if (ch7seg < '0')
            ch7seg = '9';

        rotator_value = ch7seg-'0';

        rotatory_count++;
        rotatory_sum += rotator_value;
        avg_rotatory = rotatory_sum / rotatory_count;
    }
}

static void show_rotatory_graph()
{
	oled_fillRect(screen_draw_pos,height - gap, screen_draw_pos+1, height - gap - avg_rotatory, OLED_COLOR_BLACK);
	led7seg_setChar(rotator_value+'0', FALSE);
	screen_draw_pos++;
}

// scale light
static int32_t scale_light(int32_t light)
{
	int result;

	if(light <= 100)
	{
		result = round(light / 20);
	}
	else
	{
		result = round(light / 780);
		result +=5;
		/*
		result = light / 780;
		int intpart = (int)result;
		ostatok = result - intpart;

		result = floor(result);
		result += 5;
		if(ostatok <= 0.1282)
			result-=1;
		*/
	}
	return result;
}

static void measure_light()
{
	uint16_t current_light = 0;
	uint16_t current_light_scaled = 0;
	light_pos++;

	current_light = light_read();
	// show current light
	//	intToString(current_light, buf, 10, 10);
	//	oled_putString((1+9*6),40, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	current_light_scaled = scale_light(current_light);
	// show scaled current lignt
	//	intToString(current_light_scaled, buf, 10, 10);
	//	oled_fillRect((1+9*6),50, 80, 8, OLED_COLOR_WHITE);
	//	oled_putString((1+9*6),50, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	light_sum += current_light_scaled;
	avg_light = (light_sum / light_pos);

	led7seg_setChar(current_light_scaled+'0', FALSE);
	light_values[light_pos-1] = avg_light; //avg_light
}

static void show_light_graph()
{
	oled_fillRect(screen_draw_pos,height - gap, screen_draw_pos+1, height - gap - (avg_light * 2), OLED_COLOR_BLACK);
	screen_draw_pos++;
}

static void measure_temperature()
{
    uint32_t current_temp = 0;
    temperature_pos++;

    current_temp = temp_read();
    current_temp /= 10;
    // show current temp
    if(current_state == TEMPERATURE_STATE)
	{
    	intToString(current_temp, buf, 10, 10);
    	oled_putString(50,0, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	}

    temperature_sum += current_temp;
    avg_temperature = (temperature_sum / temperature_pos);
}

static void show_temperature_graph()
{
	oled_fillRect(screen_draw_pos,height - gap, screen_draw_pos+1, height - gap - avg_temperature, OLED_COLOR_BLACK);
	screen_draw_pos++;
}

uint16_t time;
static void setTime()
{
	uint32_t trim = 0;
	uint8_t taster = ((GPIO_ReadValue(0) >> 4) & 0x01);
	oled_putString(0,0, "TIME?", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	while(1)
	{
		ADC_StartCmd(LPC_ADC,ADC_START_NOW);
		//Wait conversion complete
		while (!(ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_0,ADC_DATA_DONE)));
		trim = ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_0);
		trim/=128;
		trim*=10;
		intToString(trim, buf, 10, 10);
		oled_fillRect(0,20, 68, 20, OLED_COLOR_WHITE);
		oled_putString(0,20, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

		taster = ((GPIO_ReadValue(0) >> 4) & 0x01);
		if(taster==0)
			break;
	}
	time = trim;
}

int main (void)
{
    int32_t xoff = 0;
    int32_t yoff = 0;
    int32_t zoff = 0;

    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;

    int32_t t = 0;
    
    uint32_t trim = 0;

    init_i2c();
    init_ssp();
    init_adc();

    oled_init();
    light_init();
    // acc_init();
    rotary_init();
	led7seg_init();

	pca9532_init();
	rgb_init();

	init_uart();
	eeprom_init();
    joystick_init();

    temp_init (&getTicks);


	if (SysTick_Config(SystemCoreClock / 1000)) {
		    while (1);  // Capture error
	}

    uint8_t taster = ((GPIO_ReadValue(0) >> 4) & 0x01);
    uint8_t taster_clicks = 0;

    current_state = TEMPERATURE_STATE;
    previos_state = current_state;

    light_enable();
    light_setRange(LIGHT_RANGE_4000);

    oled_clearScreen(OLED_COLOR_WHITE);

    setTime();

    oled_clearScreen(OLED_COLOR_WHITE);
    intToString(time, buf, 10, 10);
    oled_putString(0,0, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

    taster=0;
    while(1)
        {
                taster = ((GPIO_ReadValue(0) >> 4) & 0x01);
                checkJoystick();

                if(taster==0)
                {
                	taster_clicks++;
                	current_state = taster_clicks % 4;
                }

                if(previos_state != current_state)
                {
                	changed_screen = TRUE;
                	previos_state = current_state;
                	oled_clearScreen(OLED_COLOR_WHITE);
                	screen_draw_pos=0;
                }

                measure_temperature();
                measure_light();

                if(current_state == TEMPERATURE_STATE)
    			{
    				show_temperature_graph();

    				if(changed_screen)
    				{
    					oled_fillRect(20,0, 70, 20, OLED_COLOR_WHITE);
    					oled_putString(20,0, "TEMP: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    					changed_screen = FALSE;
    				}
    			}
    			else if(current_state == LIGHT_STATE)
    			{
    				measure_light();
    				show_light_graph();
    				if(changed_screen)
    				{
    					oled_fillRect(30,0, 70, 20, OLED_COLOR_WHITE);
    					oled_putString(30,0, "LIGHT", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    					changed_screen = FALSE;
    				}
    			}
    			else if(current_state == EE_WRITE)
				{
    				oled_putString(0,0, "EEPROM_W", OLED_COLOR_BLACK, OLED_COLOR_WHITE);EEPROM_
				}
    			else if(current_state == EE_READ)
    			{
    				oled_putString(0,0, "EEPROM_R", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    			}
    			else if(current_state == ROTATORY_STATE)
    			{
    	        	change7Seg(rotary_read());
    				show_rotatory_graph();
    				if(changed_screen)
    				{
    					oled_fillRect(30,0, 70, 20, OLED_COLOR_WHITE);
    					oled_putString(30,0, "ROTATORY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    					changed_screen = FALSE;
    				}
    			}
    			else if(current_state == POTENCIOMETER_STATE)
    			{
    				ADC_StartCmd(LPC_ADC,ADC_START_NOW);
    				//Wait conversion complete
    				while (!(ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_0,ADC_DATA_DONE)));
    				trim = ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_0);

    				potenciometar(trim);
    				show_potenc_graph();

    				if(changed_screen)
    				{
    					oled_fillRect(10,0, 70, 20, OLED_COLOR_WHITE);
    					oled_putString(0,0, "POTENC:", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    					changed_screen = FALSE;
    				}
    			}


                /* delay */
                Timer0_Wait(time);

                if(screen_draw_pos==96)
                {
                	screen_draw_pos=0;
                	oled_clearScreen(OLED_COLOR_WHITE);
                	changed_screen = TRUE;
                }
            }
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
