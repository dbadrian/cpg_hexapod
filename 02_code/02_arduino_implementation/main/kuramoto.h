#ifndef KURAMOTO_H
#define KURAMOTO_H

#include <math.h>     //standard c library


enum gait_type{
  STOP  = 0,
  FWD   = 1,
  BWD   = 2,
  CWR   = 3,
  CCWR  = 4
};


/*------------------------------------------------------------------------------
* Subroutine Declarations
*-----------------------------------------------------------------------------*/
void updateOscillators();
void change_speed(double speed);
void switch_gait(enum gait_type gt);

/*------------------------------------------------------------------------------
* Variables for Euler Integration
*-----------------------------------------------------------------------------*/
double T=0.01; // Time Step for integration (Euler)
double Tms=T*1000; // Time Step in milliseconds


/*------------------------------------------------------------------------------
* Central Pattern Generator Variables
*-----------------------------------------------------------------------------*/
//Coupling Weights
static const double W_max = 3.5;
static const double W_default = 2.0;
double W[3] = {W_default, W_default, W_default};

double w[3][3] = {  
  {0, 0, 0} ,   /*  initializers for row indexed by 0 */
  {10, 0, 0} ,   /*  initializers for row indexed by 1 */
  {10, 0, 0}   /*  initializers for row indexed by 2 */
};

// Phase Bias
double phi[3][3] = {  
  {0, 0, 0} ,   /*  initializers for row indexed by 0 */
  {PI/2, 0, 0} ,   /*  initializers for row indexed by 1 */
  {-PI/2, 0, 0}   /*  initializers for row indexed by 2 */
};


// "Constant" variables (e.g., max amplitudes etc.)
int N = 3; // Number of Oscillators Defined
int ar = 4;
//int ax = 500;
int R[3] = {20, 40, 40};
double X[3] = {0.0, 0.0, 0.0};

// State Variables
double Phi[3] = {0.0, 0.0, 0.0};
double r[3]   = {0.0, 0.0, 0.0};
double r_d[3] = {0.0, 0.0, 0.0};
double x[3]   = {0.0, 0.0, 0.0};
double x_d[3] = {0.0, 0.0, 0.0};

// State Variable Buffers
double n_Phi[3] = {0.0, 0.0, 0.0};
double n_r[3]   = {0.0, 0.0, 0.0};
double n_r_d[3] = {0.0, 0.0, 0.0};
double n_x[3]   = {0.0, 0.0, 0.0};
double n_x_d[3] = {0.0, 0.0, 0.0};

// Output (in degree)
double output[3] = {0.0, 0.0, 0.0};

/*------------------------------------------------------------------------------
* Subroutines
*-----------------------------------------------------------------------------*/
void updateOscillators()
{
   //////////////////// Determine New State Variables ///////////////
  // PHI //
  for(int i=0; i<N; i++)
  {
    double Phi_d = W[i];
    for(int j=0; j<N; j++)
    {
      Phi_d += w[i][j] * r[j] * sin(Phi[j] - Phi[i] - phi[i][j]);
    }
    n_Phi[i] = Phi[i] + T * Phi_d;
  }

  // R //
  for(int i=0; i<N; i++)
  {
    double r_dd = ar * ((ar/4)*(R[i]-r[i])-r_d[i]);
    n_r_d[i]= r_d[i] + T * r_dd;
    n_r[i] = r[i] + T * r_d[i];
  }

  // X //
//  for(int i=0; i<N; i++)
//  {
//    double x_dd = ax * ((ax/4)*(X[i]-x[i])-x_d[i]);
//    n_x_d[i] = x_d[i] + T * x_dd;
//    n_x[i] = x[i] + T * x_d[i];
//  }

   //////////////////// Determine New Angles ///////////////
  for(int i=0; i<N; i++)
  {
    Phi[i] = n_Phi[i];
    r_d[i] = n_r_d[i];
    r[i] = n_r[i];
//    x_d[i] = n_x_d[i];
//    x[i] = n_x[i];

    output[i] = r[i] * sin(Phi[i]);
  }
}


void change_speed(double speed)
{
  for(int i=0; i<N; i++)
  {
    W[i] = speed;
  }
}

void switch_gait(enum gait_type gt)
{
  // zero out values, just to make sure
  memset(phi, 0, sizeof(phi[0][0]) * N * N);

  if (gt == gait_type::FWD)
  {
    phi[1][0] = PI/2;
    phi[2][0] = PI/2;
  } 
  else if (gt == gait_type::BWD)
  {
    phi[1][0] = -PI/2;
    phi[2][0] = -PI/2;
  } 
  else if (gt == gait_type::CWR)
  {
    phi[1][0] = PI/2;
    phi[2][0] = -PI/2;
  } 
  else if (gt == gait_type::CCWR)
  {
    phi[1][0] = -PI/2;
    phi[2][0] = PI/2;
  } 
  else // STOP
  {
    // reset to foward
    phi[1][0] = PI/2;
    phi[2][0] = PI/2;
    change_speed(0.0);
  }
}


#endif
