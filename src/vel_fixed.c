/* library needed to include the int16_t and int32_t types.
it is needed also in the floating point controllers for the 
definition of outputs and inputs. */
#include <inttypes.h>

#define K 21408  // 196/75*2^13
#define N 13 // fractiona bits
#define KB 10704 // K*b - dos not need to calculate every time
#define KHTI 2367 // k*h/Ti - new Constant

int16_t I_16; 
int16_t vel_fixed(int16_t r, int16_t y){

    /**********************************/
    // Implement your controller here //
    /**********************************/
    /*
    Use only int16_t variables for the fixed-point 
    implementation.
    */
    
    
    int16_t u = (int16_t) (((int32_t)KB*r)>>(N)) - 
            (((int32_t)K*y)>>(N)) + I_16; // calculates u 
    
    // Saturation if-else statement for u 
    if (u>511){ //upper limit 
        u = 511; // saturation
        }
    else if (u<-512){ //low limit
        u = -512; //saturates u 
        }
    
    I_16 = I_16 + (int16_t)((int32_t) (KHTI * (r-y)>>(N))); //calcs I

    return (int16_t) u; // write output to D/A and end function
}