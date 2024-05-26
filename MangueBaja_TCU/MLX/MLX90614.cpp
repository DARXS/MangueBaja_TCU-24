#include "MLX90614.h"

MLX90614::MLX90614(PinName sda, PinName scl)
{
    i2c_ = new I2C(sda, scl);
    i2c_->frequency(400000);
}

MLX90614::MLX90614(I2C *i2c) : i2c_(i2c) {
    }

MLX90614::~MLX90614() {
    delete i2c_;
    }

float MLX90614::get_temp(uint8_t reg) {
    char cmd[3] = { 0 };
    // read the temperature data (kelvin)
    cmd[0] = ram_access | reg;
    i2c_->write(default_addr,cmd,1,true); i2c_->read(default_addr,cmd,3);
    // convert to meaningful units, still in kelvin - just normalized
    return 0.02 * static_cast<float>((cmd[1]<<8)|cmd[0]);
}

float MLX90614::read_temp(int select) {
    uint8_t reg_addrs[] = { T_ambient, T_obj1 };
    float tt = 0.0;
    if (select == 0){
        tt = get_temp(reg_addrs[0])-273.15;
        }
    if (select == 1){
        tt = get_temp(reg_addrs[1])-273.15;
        }
    return tt;
    }