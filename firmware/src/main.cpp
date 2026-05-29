// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "main.h"

#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"

#include <NeoPixelConnect.h>
#include <Wire.h>
#include <algorithm>

#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"
#include "utilities/frequency_counter.h"
#include "version.h"
#include "hw_config.h"

//*** GLOBALS ***************************************************************************

MT6835Encoder encoder(ENCODER_SPI, PIN_ENCODER_CS);


TB6612MotorDriver motor_driver(
  PIN_MOTOR_EN, PIN_M_PWM_A_POS, PIN_M_PWM_A_NEG, PIN_MOTOR_PWMA,
  PIN_MOTOR_EN, PIN_M_PWM_B_POS, PIN_M_PWM_B_NEG, PIN_MOTOR_PWMB
);

NeoPixelConnect strip(PIN_BUILTIN_LED, 1);

//*** CLASS *****************************************************************************

class EmbossingController {
  public:
    EmbossingController(TB6612MotorDriver* motor_driver, MT6835Encoder* encoder) {
      EmbossingController::encoder = encoder;
      EmbossingController::motor_driver = motor_driver;

      encoder_angle_to_distance = -2*ENCODER_MAGNET_PITCH/Constants::TWO_PI_F;
      pos_to_field_angle = Constants::TWO_PI_F*MOTOR_POLE_PAIRS*ROTATIONS_PER_MM;
      rapid_velocity = 1.3f;
      embossing_velocity = 0.2f;

      target_encoder_value = 0.0f;
      measurement_pos = 0.0f;
      motor_position = 0.0f;
      collision_motor_pos = 0.0f;
    }

    void set_target_encoder_value(float target_value) {
      EmbossingController::target_encoder_value = target_value;
    }

    void set_measurement_pos(float pos) {
     EmbossingController::measurement_pos = pos;
    }

    bool init(float target_length_offset=0.0f) {
      bool init_ok = find_collision(3.0f); // find collision
      collision_motor_pos = get_motor_pos();

      if(init_ok == false)
        return false;

      // back off from collision point
      move_to_ex(collision_motor_pos+1.0f, 1.5f, 1.0f);

      // store position for measurement
      set_measurement_pos(get_motor_pos());

      // use length measurement as target length
      float target_pos = read_encoder_pos();
      set_target_encoder_value(target_pos+target_length_offset);

      return true;
    }

    void run_embossing(float start_length_offset=0.0f) {
      float embossing_start_pos = collision_motor_pos+start_length_offset;
      float p = collision_motor_pos+start_length_offset;

      while(true) {
        float error = measure_length_error(false);
        LOG_RAW(">lenght_error: %f", error);
        if(error<0.0f)
          break;

        p -= error*0.9f;
        move_to_ex(embossing_start_pos, rapid_velocity, 1.0f);
        move_to_ex(p, 0.3, 0.6f);
       // move_to_ex(collision_motor_pos, 0.3, 0.6f);
      }

      move_to_ex(measurement_pos, rapid_velocity, 1.0f);
    }

    float measure_length_error(bool move_to_prev_pos) {
      float prev_motor_pos = get_motor_pos();
      move_to_ex(measurement_pos, rapid_velocity, 1.0f);
      sleep_ms(50);
      float enc_pos = read_encoder_pos();
      sleep_ms(50);
      
      // return to previous pos
      if(move_to_prev_pos)
        move_to_ex(prev_motor_pos, rapid_velocity, 1.0f);

      return enc_pos-target_encoder_value;
    }

    void move_to_ex(float pos, float velocity, float current) {
      motor_driver->set_amplitude_smooth(current, 0.1);
      move_to(pos, velocity);
      motor_driver->set_amplitude_smooth(0.3, 0.1);
    }

    void move_to(float pos, float velocity) {
      float delta = pos-motor_position;
      motor_driver->rotate_field(delta*pos_to_field_angle, velocity*pos_to_field_angle, [this](){
        // update encoder regularily
        encoder->read_abs_angle();
      });
      motor_position += delta;
    }

    float get_motor_pos() {
      return motor_position;
    }

    float read_encoder_pos() {
      return encoder->read_abs_angle()*encoder_angle_to_distance;
    }

    bool find_collision(float search_range, float velocity=0.2f) {
      float step = -0.05;
      float detection_delta = 0.05;
      float start_pos = get_motor_pos();
      float start_encoder_value = read_encoder_pos();
      
      for(int i=0; i<(int)abs(search_range/step); i++) {
        move_to(start_pos + step*i, velocity);
        if(start_encoder_value-read_encoder_pos() > detection_delta) {
          LOG_INFO("Collision detected");
          return true;
        }
      }

      LOG_INFO("No collision within searchrange found");
      return false;
    }

  private:
    MT6835Encoder* encoder;
    TB6612MotorDriver* motor_driver;

    float target_encoder_value;
    float measurement_pos;
    float embossing_velocity;
    float rapid_velocity;

    float collision_motor_pos;

    float motor_position;
    float encoder_angle_to_distance;
    float pos_to_field_angle;
};

EmbossingController embossing_controller(&motor_driver, &encoder);

//*** FUNCTIONS *************************************************************************

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
  strip.neoPixelSetValue(0, r, g, b, false);
  sleep_us(2000);
  strip.neoPixelShow();
}

void led_blink(uint8_t r, uint8_t g, uint8_t b, int count, int period_time_ms) {
  for(int i=0; i<count; i++) {
    set_led_color(r, g, b);
    sleep_ms(period_time_ms/2);
    set_led_color(0, 0, 0);
    sleep_ms(period_time_ms/2);
  }
}

void setup() {
  sleep_ms(100);  // Allow time for serial monitor to connect
  led_blink(0, 0, 10, 3, 100);

  Logger::instance().begin(115200, false);
  //while(!Serial);
  sleep_ms(100);  // Allow time for serial monitor to connect
  
  LOG_INFO("\n\nBall Joint Embossing ToolFirmware: %s", FIRMWARE_VERSION);

  MT6835Encoder::setup_spi(ENCODER_SPI, PIN_ENCODER_SCK, PIN_ENCODER_MOSI, PIN_ENCODER_MISO, 1000000);
  encoder.init(0x5, 0x4);

  motor_driver.begin();
  motor_driver.set_amplitude_smooth(0.3, 0.5);
  motor_driver.enable();

  pinMode(PIN_USER_BUTTON, INPUT_PULLUP);

  set_led_color(10, 4, 0);
  bool ok = embossing_controller.init(-0.00f);
  if(ok == false) {
    set_led_color(10, 0, 0);
    while(true);
  }

  set_led_color(0, 2, 0);
  LOG_INFO("Initialization finished");
  LOG_INFO(" ");

    // embossing_controller.move_to(1.0f, 0.5f);
}

float t = 0.0f;
void loop() {
  // wait for button
  while(digitalRead(PIN_USER_BUTTON) == true) {
    set_led_color(0, 10, 0);
    float error = embossing_controller.measure_length_error(false);
    LOG_RAW(">lenght_error: %f", error);
    sleep_ms(500);
  }

  set_led_color(10, 0, 10);
  embossing_controller.run_embossing();

  //float pos = embossing_controller.read_encoder_pos();
  //LOG_RAW(">encoder_poss: %f", pos);

//  sleep_ms(1000);

//  float abs_angle = encoder.read_abs_angle();
//  LOG_RAW(">angle: %f", abs_angle);

  //t = t*0.9f + abs_angle*0.1f;
  //motor_driver.set_field_angle(t*MOTOR_POLE_PAIRS);
//  t += 0.1f;
//  sleep_ms(10);  // Allow time for serial monitor to connect
}