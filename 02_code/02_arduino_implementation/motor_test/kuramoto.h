#ifndef KURAMOTO_H
#define KURAMOTO_H

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
void change_speed(float speed);
void switch_gait(enum gait_type);

/*------------------------------------------------------------------------------
* Variables for Euler Integration
*-----------------------------------------------------------------------------*/
float T=0.01; // Time Step for integration (Euler)
float Tms=T*1000; // Time Step in milliseconds


/*------------------------------------------------------------------------------
* Central Pattern Generator Variables
*-----------------------------------------------------------------------------*/
//Coupling Weights
static const float W_max = 3.5;
static const float W_default = 2.0;
float W[3] = {W_default, W_default, W_default};

float w[3][3] = {  
  {0, 0, 0} ,   /*  initializers for row indexed by 0 */
  {0.5, 0, 0} ,   /*  initializers for row indexed by 1 */
  {0.5, 0, 0}   /*  initializers for row indexed by 2 */
};

// Phase Bias
float phi[3][3] = {  
  {0, 0, 0} ,   /*  initializers for row indexed by 0 */
  {PI/2, 0, 0} ,   /*  initializers for row indexed by 1 */
  {-PI/2, 0, 0}   /*  initializers for row indexed by 2 */
};

// "Constant" variables (e.g., max amplitudes etc.)
int N = 3; // Number of Oscillators Defined
int ar = 4;
int ax = 4;
int R[3] = {12, 40, 40};
float X[3] = {0.0, 0.0, 0.0};

// State Variables
float Phi[3] = {0.0, 0.0, 0.0};
float r[3]   = {0.0, 0.0, 0.0};
float r_d[3] = {0.0, 0.0, 0.0};
float x[3]   = {0.0, 0.0, 0.0};
float x_d[3] = {0.0, 0.0, 0.0};

// State Variable Buffers
float n_Phi[3] = {0.0, 0.0, 0.0};
float n_r[3]   = {0.0, 0.0, 0.0};
float n_r_d[3] = {0.0, 0.0, 0.0};
float n_x[3]   = {0.0, 0.0, 0.0};
float n_x_d[3] = {0.0, 0.0, 0.0};

// Output (in degree)
float output[3] = {0.0, 0.0, 0.0};

/*------------------------------------------------------------------------------
* Subroutines
*-----------------------------------------------------------------------------*/
void updateOscillators()
{
   //////////////////// Determine New State Variables ///////////////
  // PHI //
  for(int i=0; i<N; i++)
  {
    float Phi_d = W[i];
    for(int j=0; j<N; j++)
    {
      Phi_d += w[i][j] * r[j] * sin(Phi[j] - Phi[i] - phi[i][j]);
    }
    n_Phi[i] = Phi[i] + T * Phi_d;
  }

  // R //
  for(int i=0; i<N; i++)
  {
    float r_dd = ar * ((ar/4)*(R[i]-r[i])-r_d[i]);
    n_r_d[i]= r_d[i] + T * r_dd;
    n_r[i] = r[i] + T * r_d[i];
  }

  // X //
  for(int i=0; i<N; i++)
  {
    float x_dd = ax * ((ax/4)*(X[i]-x[i])-x_d[i]);
    n_x_d[i] = x_d[i] + T * x_dd;
    n_x[i] = x[i] + T * x_d[i];
  }

   //////////////////// Determine New Angles ///////////////
  for(int i=0; i<N; i++)
  {
    Phi[i] = n_Phi[i];
    r_d[i] = n_r_d[i];
    r[i] = n_r[i];
    x_d[i] = n_x_d[i];
    x[i] = n_x[i];

    output[i] = x[i] + r[i] * sin(Phi[i]);
  }
}


void update_W(float speed)
{
  for(int i=0; i<N; i++)
  {
    W[i] = speed;
  }
}

void switch_gait(enum gait_type)
{
  // implement a switch of gait type
}


#endif
