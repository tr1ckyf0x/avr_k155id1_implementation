/* Name: main.c
 * Author: <Vladislav Lisyanskiy>
 */

#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>

// PWM Configuration
#define PWM_PORT PORTB
#define PWM_DDR DDRB
#define PWM_PIN PORTB2
#define PWM_DDR_BIT DDB2

#define PWM_CONTROL_REGISTER_A TCCR0A
#define PWM_CONTROL_REGISTER_B TCCR0B
#define PWM_COMPARE_REGISTER OCR0A

#define PWM_DUTY_CYCLE 0

// Decoder Configuratuon
#define DECODER_OUTPUTS_COUT 10

#define DECODER_X0_DDR DDRD
#define DECODER_X1_DDR DDRD
#define DECODER_X2_DDR DDRA
#define DECODER_X3_DDR DDRA
#define DECODER_X0_DDR_BIT DDD0
#define DECODER_X1_DDR_BIT DDD1
#define DECODER_X2_DDR_BIT DDA1
#define DECODER_X3_DDR_BIT DDA0
#define DECODER_X0_PIN PIND
#define DECODER_X1_PIN PIND
#define DECODER_X2_PIN PINA
#define DECODER_X3_PIN PINA
#define DECODER_X0_PIN_BIT PIND0
#define DECODER_X1_PIN_BIT PIND1
#define DECODER_X2_PIN_BIT PINA1
#define DECODER_X3_PIN_BIT PINA0

#define DECODER_Y0_DDR DDRD
#define DECODER_Y1_DDR DDRD
#define DECODER_Y2_DDR DDRD
#define DECODER_Y3_DDR DDRD
#define DECODER_Y4_DDR DDRD
#define DECODER_Y5_DDR DDRB
#define DECODER_Y6_DDR DDRB
#define DECODER_Y7_DDR DDRB
#define DECODER_Y8_DDR DDRB
#define DECODER_Y9_DDR DDRB
#define DECODER_Y0_DDR_BIT DDD2
#define DECODER_Y1_DDR_BIT DDD3
#define DECODER_Y2_DDR_BIT DDD4
#define DECODER_Y3_DDR_BIT DDD5
#define DECODER_Y4_DDR_BIT DDD6
#define DECODER_Y5_DDR_BIT DDB0
#define DECODER_Y6_DDR_BIT DDB1
#define DECODER_Y7_DDR_BIT DDB5
#define DECODER_Y8_DDR_BIT DDB6
#define DECODER_Y9_DDR_BIT DDB7
#define DECODER_Y0_PORT PORTD
#define DECODER_Y1_PORT PORTD
#define DECODER_Y2_PORT PORTD
#define DECODER_Y3_PORT PORTD
#define DECODER_Y4_PORT PORTD
#define DECODER_Y5_PORT PORTB
#define DECODER_Y6_PORT PORTB
#define DECODER_Y7_PORT PORTB
#define DECODER_Y8_PORT PORTB
#define DECODER_Y9_PORT PORTB
#define DECODER_Y0_PORT_BIT PORTD2
#define DECODER_Y1_PORT_BIT PORTD3
#define DECODER_Y2_PORT_BIT PORTD4
#define DECODER_Y3_PORT_BIT PORTD5
#define DECODER_Y4_PORT_BIT PORTD6
#define DECODER_Y5_PORT_BIT PORTB0
#define DECODER_Y6_PORT_BIT PORTB1
#define DECODER_Y7_PORT_BIT PORTB5
#define DECODER_Y8_PORT_BIT PORTB6
#define DECODER_Y9_PORT_BIT PORTB7

#define CLEAR_BIT(port, bit) (port &= ~(1 << bit))
#define SET_BIT(port, bit) (port |= 1 << bit)

static unsigned char decoder_input_value;

static volatile unsigned char *decoder_y_ports[] = {
                                            &DECODER_Y0_PORT,
                                            &DECODER_Y1_PORT,
                                            &DECODER_Y2_PORT,
                                            &DECODER_Y3_PORT,
                                            &DECODER_Y4_PORT,
                                            &DECODER_Y5_PORT,
                                            &DECODER_Y6_PORT,
                                            &DECODER_Y7_PORT,
                                            &DECODER_Y8_PORT,
                                            &DECODER_Y9_PORT
                                        };

static unsigned char decoder_y_port_bits[] = {
                                                DECODER_Y0_PORT_BIT,
                                                DECODER_Y1_PORT_BIT,
                                                DECODER_Y2_PORT_BIT,
                                                DECODER_Y3_PORT_BIT,
                                                DECODER_Y4_PORT_BIT,
                                                DECODER_Y5_PORT_BIT,
                                                DECODER_Y6_PORT_BIT,
                                                DECODER_Y7_PORT_BIT,
                                                DECODER_Y8_PORT_BIT,
                                                DECODER_Y9_PORT_BIT
                                            };

static inline void setup_pwm() {
    // Setup PWM Pin
    SET_BIT(PWM_DDR, PWM_DDR_BIT);

    // SETUP PWM Timer
    // Set PWM Mode
    PWM_CONTROL_REGISTER_A |= 1 << WGM01 | 1 << COM0A0;

    // PWM Freq. calculated by
    // F = Fclk_I/O / (N * 256)
    // where N is a prescaler factor (1, 8, 32, 64, 128, 256, or 1024)

    // Set PWM Freq.
    SET_BIT(PWM_CONTROL_REGISTER_B, CS00);

    // Set PWM duty cycle
    PWM_COMPARE_REGISTER = PWM_DUTY_CYCLE;
}

static inline void setup_decoder_pins() {
    // Input pins are already configured as Hi-Z by default
    // Setup Output pins
    SET_BIT(DECODER_Y0_DDR, DECODER_Y0_DDR_BIT);
    SET_BIT(DECODER_Y1_DDR, DECODER_Y1_DDR_BIT);
    SET_BIT(DECODER_Y2_DDR, DECODER_Y2_DDR_BIT);
    SET_BIT(DECODER_Y3_DDR, DECODER_Y3_DDR_BIT);
    SET_BIT(DECODER_Y4_DDR, DECODER_Y4_DDR_BIT);
    SET_BIT(DECODER_Y5_DDR, DECODER_Y5_DDR_BIT);
    SET_BIT(DECODER_Y6_DDR, DECODER_Y6_DDR_BIT);
    SET_BIT(DECODER_Y7_DDR, DECODER_Y7_DDR_BIT);
    SET_BIT(DECODER_Y8_DDR, DECODER_Y8_DDR_BIT);
    SET_BIT(DECODER_Y9_DDR, DECODER_Y9_DDR_BIT);
}

static inline void read_decoder_input_value() {
    decoder_input_value = 0;
    decoder_input_value |= ((DECODER_X0_PIN & (1 << DECODER_X0_PIN_BIT)) >> DECODER_X0_PIN_BIT) << 0;
    decoder_input_value |= ((DECODER_X1_PIN & (1 << DECODER_X1_PIN_BIT)) >> DECODER_X1_PIN_BIT) << 1;
    decoder_input_value |= ((DECODER_X2_PIN & (1 << DECODER_X2_PIN_BIT)) >> DECODER_X2_PIN_BIT) << 2;
    decoder_input_value |= ((DECODER_X3_PIN & (1 << DECODER_X3_PIN_BIT)) >> DECODER_X3_PIN_BIT) << 3;
}

static inline void set_decoder_output_pins() {
    for (unsigned short i = 0; i < DECODER_OUTPUTS_COUT; ++i) {
        CLEAR_BIT(*decoder_y_ports[i], decoder_y_port_bits[i]);
    }
    SET_BIT(*decoder_y_ports[decoder_input_value], decoder_y_port_bits[decoder_input_value]);
}

int main(void) {

    setup_pwm();
    setup_decoder_pins();

	for(;;) {
        read_decoder_input_value();
        set_decoder_output_pins();
	}

    return 0;   /* never reached */
}
