// motor pole pair count
//  * 100 for 0.9deg stepper motors
//  * 50  for 1.8deg stepper motors
#define MOTOR_POLE_PAIRS 50
#define ROTATIONS_PER_MM (48.0f/20.0f/1.0f)
#define ENCODER_MAGNET_PITCH 3.0f

// Single Axis Board
#define PIN_BUILTIN_LED 16
#define PIN_USER_BUTTON PIN_GPIO_2

#define PIN_GPIO_0 0
#define PIN_GPIO_1 1
#define PIN_GPIO_2 2

#define PIN_M_PWM_A_POS  28
#define PIN_M_PWM_A_NEG  29
#define PIN_M_PWM_B_POS  27
#define PIN_M_PWM_B_NEG  26

#define PIN_MOTOR_EN      7
#define PIN_MOTOR_PWMA    6
#define PIN_MOTOR_PWMB    8

// note single axis board uses spi1
#define ENCODER_SPI spi1
#define PIN_ENCODER_CS 13
#define PIN_ENCODER_SCK 10
#define PIN_ENCODER_MISO 12
#define PIN_ENCODER_MOSI 11