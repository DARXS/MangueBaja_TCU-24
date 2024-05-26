/*
    Copyright (c) 2016 Jens Strümper based on the work of Jonathan Jones. 
 
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
 
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
 
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef MLX90614_H
#define MLX90614_H

#ifndef MBED_H
#include "mbed.h"
#endif

namespace js {
    
/** Due to changes in mbed's I2C stack older implementation of the MLX9064 library do currently not work. 
 * This recent implementation wraps the example code of Jonathan Jones into a library.  
 *
 * The float read_temp(int select) memeber function will take one argument:
 *  <ul>
 *  <li> "0" to return ambient temperature </li>
 *  <li> "1" to retunr object temperature </li> 
 *  </ul>
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "MLX90614.h"
 *
 * I2C i2c(p16, p17); //I2C_SDA, I2C_SCL 
 * MLX90614 mlx90614(&i2c);
 *
 * float temp;
 * int select; // 0 = object temp, 1 = ambient temp
 *
 * int main(){
    while(true){
      temp=mlx90614.read_temp(1);
      printf("%4.2f Celcius\r\n", temp);
      wait(1);
    }
}
 * @endcode
 */

class MLX90614 {

public:

    MLX90614(PinName sda, PinName scl);

    MLX90614(I2C *i2c);

   ~MLX90614();

    float read_temp(int select);

protected:

    float get_temp(uint8_t reg);

private:

    I2C *i2c_;

    static const int default_addr = 0x00;
    static const int T_ambient = 0x06;
    static const int T_obj1 = 0x07;
    static const int ram_access = 0x00;

};

} //namespace js

using namespace js;

#endif