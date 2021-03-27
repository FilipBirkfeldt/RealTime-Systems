/* library needed to include the int16_t and int32_t types.
it is needed also in the floating point controllers for the 
definition of outputs and inputs. */
#include <inttypes.h>

#define l1 2.0361 // observer 2
#define l2 1.2440 // obsever pole 2
#define lv 3.2096 // extended
#define kr 1.7831 // Kr = Kc 
#define k1 3.2898 // feedback 1
#define k2 1.7831 // feedback 1
#define phi11 0.994 // Phi-matrix
#define phi12 0 // Phi-matrix
#define phi21 0.2493 // Phi-matrix
#define phi22 1.00 // Phi-matrix
#define gamma1 0.1122 // Phi-matrix
#define gamma2 0.0140 // Phi-matrix

float x1, x2, v; // declares variables
int16_t pos_float(int16_t r, int16_t y){

    /**********************************/
    // Implement your controller here //
    /**********************************/
    /*
    Use only int16_t variables for the fixed-point 
    implementation.
    */
    
    float u = ((float)kr * r - k1 * x1 - k2 * x2 - v); // calculates control-signal
    float e = ((float)y - x2); //calculates error
    
    // Saturation if-else statement for u 
    if (u>511){ //upper limit 
        u = 511; // saturation
        }
    else if (u<-512){ //low limit
        u = -512; //saturates u 
        }
   
    x1=((float)phi11*x1+phi12*x2 + gamma1*(u+v)+l1*e); //calculates first x
    x2=((float)phi21*x1+phi22*x2 + gamma2*(u+v)+l2*e); //calculates first x
    v=v+lv*e; // v given old v and observer and error 
    
    
    return u; // write output to D/A and end function
}