/* library needed to include the int16_t and int32_t types.
it is needed also in the floating point controllers for the 
definition of outputs and inputs. */
#include <inttypes.h>

#define l1 16680 //observer pole
#define l2 10191 //observer pole
#define lv 26293 // extended
#define kr 14607 // Kr = Kc 
#define k1 26950 // feedback
#define k2 14607 // feedback
#define phi11 8143 // Phi-Matrix
#define phi12 0 // Phi-Matrix
#define phi21 2042 // Phi-Matrix
#define phi22 8192 // Phi-Matrix
#define gamma1 919 // Gamma-matrix
#define gamma2 115 // Gamma-matrix
#define N 13 //N - for bit-shifting

int16_t xx1, xx2, vv, xx11; // declares variables 
int16_t pos_fixed(int16_t r, int16_t y){

    /**********************************/
    // Implement your controller here //
    /**********************************/
    /*
    Use only int16_t variables for the fixed-point 
    implementation.
    */
    int16_t u = (int16_t) ((int32_t)(((int32_t)kr*r-k1*xx1-k2*xx2)>>N) -vv); //calcs control-signal, saves as 16-bit
    
    int16_t e=(int16_t)((int32_t)y-xx2); //calcs error, saving as 16-bits
    
     // Saturation if-else statement for u 
    if (u>511){ //upper limit 
        u = 511; // saturation
        }
    else if (u<-512){ //low limit
        u = -512; //saturates u 
        }   
    
    xx11=(int16_t) (((int32_t)phi11*xx1+phi12*xx2+gamma1*(u+vv)+l1*e)>>N); //calc x1, down here to reduce latenct
    xx2 = (int16_t) (((int32_t)phi21*xx1+phi22*xx2+gamma2*(u+vv)+l2*e)>>N); // calcs x2 
    vv = (int16_t) ((int32_t) vv + (lv*e>>N));
    
    xx1 = (int16_t) xx11;
    
    return u; // write output to D/A and end function
}



















