/* Implemented in 2023 by João Gabriel, electronics coordinator of Mangue Baja 2023/24 */
#ifndef FIR_H
#define FIR_H

#include <stdio.h>
#include <stdlib.h>

/* You can change the size of the vetor if you needed */
#define SIZE 2

/* Can use only in one variable. Using the second variable the value of the static float has change */
class FIR {
    public:
    /*========================================================================================================
        * @param a_coef, @param b_coef is the coefficients to the filter;
        *   The higher the value of the coefficient, the slower its update rate, but the stronger its filter.
    ========================================================================================================*/
        FIR(float a_coef, float b_coef);
        ~FIR();
    /*========================================================================================================
        * Filter based on convolution between previous inputs.
        * @param x is the variable used for the filtering;
        * @param type_filtering if needeed apply your filter two times type_filtering=true.
    ==========================================================================================================*/
        float filt(float x, bool type_filtering = false);
    private:
    /*=========================
        * Update the vetor
    ==========================*/
        void move_vec(float *vetorAddr, int size, float value); 
    /*=========================
        * Second filter
    ==========================*/
        float filtfilt(float A, float B, float x2);
        bool flag;
    protected:
        float _a;
        float _b; 
};

FIR::FIR(float a_coef, float b_coef)
{
    if(a_coef!=0 && b_coef!=0)
    {
        _a = a_coef;
        _b = b_coef;
    }

    else
    {
        _a = 0.6;
        _b = 0.6;
    }
};

FIR::~FIR() {/**/};

float FIR::filt(float x, bool type_filtering)
{
    this->flag = type_filtering | 0x00;

    static float y_pass[SIZE] = {0,0}, x_pass[SIZE] = {0,0};

    float y = (_a+_b)*y_pass[0] - _a*_b*y_pass[1] + (1-_a-_b + _a*_b)*x_pass[1];
    
    move_vec(y_pass, SIZE, y);
    move_vec(x_pass, SIZE, x);

    return (flag) ? filtfilt(_a, _b, y) : y;
};

void FIR::move_vec(float *vetorAddr, int size, float value)
{
    for (int k = size-1; k > 0; k--)
    {
        *(vetorAddr+k) = *(vetorAddr+k-1);
    }

    *vetorAddr = value;
};

float FIR::filtfilt(float A, float B, float x2)
{
    static float y2_pass[SIZE] = {0,0}, x2_pass[SIZE] = {0,0};

    float y2 = (A+B)*y2_pass[0] - A*B*y2_pass[1] + (1-A-B + A*B)*x2_pass[1];
    
    move_vec(y2_pass, SIZE, y2);
    move_vec(x2_pass, SIZE, x2);

    return y2;
};


#endif
