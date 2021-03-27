/* library needed to include the int16_t and int32_t types.
it is needed also in the floating point controllers for the 
definition of outputs and inputs. */
#include <inttypes.h>

#define K 196/75  // examle of how parameters are defined in C
#define Ti 147/325 //Ti-value from 4.1.1
#define beta 0.5 // beta-varable 
#define h 0.05 //sample

float I;

int16_t vel_float(int16_t r, int16_t y){

    /**********************************/
    // Implement your controller here //
    /**********************************/
    /*
    Use only int16_t variables for the fixed-point 
    implementation.
    */
    float u = K * beta *r - K*y + I; // calculates u 
    I = I + K*h/Ti*(r-y); // Calculates next I 
    
    // Saturation if-else statement for u 
    if (u>511){ //upper limit 
        u = 511; // saturation
        }
    else if (u<-512){ //low limit
        u = -512; //saturates u 
        }

    return (int16_t) u; // write output to D/A and end function
}


