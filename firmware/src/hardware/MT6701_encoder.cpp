#include "MT6701_encoder.h"

MT6701Encoder::MT6701Encoder(TwoWire& wire, uint8_t i2c_addr)
    : wire(wire), address(i2c_addr) {
}

void MT6701Encoder::init() {
  wire.begin();
  last_raw_angle = 0;
  abs_raw_angle = 0;
}

int32_t MT6701Encoder::read_abs_angle_raw() {
    wire.beginTransmission(address);
    wire.write(0x03); // ANGLE_H register
    wire.endTransmission(false);

    wire.requestFrom(address, (uint8_t)2);
    if (wire.available() < 2) return -1;

    uint8_t angle_h = wire.read();
    uint8_t angle_l = wire.read();

    int32_t raw_angle = (angle_h << 6) | (angle_l >> 2);
    return update_abs_raw_angle(raw_angle);
}

float MT6701Encoder::read_abs_angle() {
    int32_t raw = read_abs_angle_raw();
    return float(raw) * RAW_TO_RAD;
}

void MT6701Encoder::set_hysteresis(uint8_t hyst) {
    if (hyst > 7) return;

    uint8_t hyst2 = (hyst >> 2) & 0x01;
    uint8_t hyst10 = hyst & 0x03;

    // --- Register 0x32 ---
    wire.beginTransmission(address);
    wire.write(0x32);
    wire.endTransmission(false);
    wire.requestFrom(address, (uint8_t)1);
    uint8_t reg32 = wire.read();
    reg32 = (reg32 & 0x7F) | (hyst2 << 7);

    wire.beginTransmission(address);
    wire.write(0x32);
    wire.write(reg32);
    wire.endTransmission();

    // --- Register 0x34 ---
    wire.beginTransmission(address);
    wire.write(0x34);
    wire.endTransmission(false);
    wire.requestFrom(address, (uint8_t)1);
    uint8_t reg34 = wire.read();
    reg34 = (reg34 & 0x3F) | (hyst10 << 6);

    wire.beginTransmission(address);
    wire.write(0x34);
    wire.write(reg34);
    wire.endTransmission();
}

MT6701Encoder::AbsRawAngleType MT6701Encoder::update_abs_raw_angle(int32_t raw_angle) {
    if (raw_angle >= 0) {
        int32_t half_max = CPR >> 1;
        int d = raw_angle - last_raw_angle;
    
        if (d > half_max) d -= CPR;
        else if (d < -half_max) d += CPR;

        abs_raw_angle += d;
        last_raw_angle = raw_angle;
    }

    return abs_raw_angle;
}
