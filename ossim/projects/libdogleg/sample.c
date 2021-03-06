#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "dogleg.h"

// This is a trivial sample application to demonstrate libdogleg in action.
// Let's say that I have a simple non-linear model
//
// a*b * x**2 + b*c * y**2 + c * x*y + d * x + e * y * f = measurements
//
// here I'm trying to estimate the vector (a,b,c,d,e,f) to most closely fit the
// data vector measurements. This problem is clearly non-sparse and libdogleg is
// overkill to solve it, but it serves decently well as a demo
//
// First I generate some noise-corrupted data, and then use libdogleg to solve
// the problem.

// My state vector (a,b,c,d,e,f) has 6 elements
#define Nstate 6

// I simulate my measurements using these as the TRUE values for the model
#define REFERENCE_A 1.0
#define REFERENCE_B 2.0
#define REFERENCE_C 3.0
#define REFERENCE_D 4.0
#define REFERENCE_E 5.0
#define REFERENCE_F 6.0

// I simulate by sampling the x-y space in a grid. This grid is defined here
#define SIMULATION_GRID_WIDTH   10
#define SIMULATION_GRID_MIN    -10
#define SIMULATION_GRID_DELTA   2.0
#define Nmeasurements (SIMULATION_GRID_WIDTH*SIMULATION_GRID_WIDTH)


static double allx                [Nmeasurements];
static double ally                [Nmeasurements];
static double allm_simulated_noisy[Nmeasurements];

static void simulate(void)
{
  for(int i=0; i<Nmeasurements; i++)
  {
    double x = allx[i];
    double y = ally[i];

    allm_simulated_noisy[i] =
      REFERENCE_A*REFERENCE_B * x*x +
      REFERENCE_B*REFERENCE_C * y*y +
      REFERENCE_C * x*y +
      REFERENCE_D * x +
      REFERENCE_E * y +
      REFERENCE_F +
      ((double)random() / (double)RAND_MAX - 0.5) * 1.0; // +- 0.5 units of uniformly-random noise
  }
}

static void generateSimulationGrid(void)
{
  int i = 0;

  for(int ix=0; ix<SIMULATION_GRID_WIDTH; ix++)
  {
    double x = SIMULATION_GRID_MIN + ix*SIMULATION_GRID_DELTA;

    for(int iy=0; iy<SIMULATION_GRID_WIDTH; iy++)
    {
      double y = SIMULATION_GRID_MIN + iy*SIMULATION_GRID_DELTA;
      allx[i] = x;
      ally[i] = y;
      i++;
    }
  }
}

static void optimizerCallback(const double*   p,
                              double*         x,
                              cholmod_sparse* Jt,
                              void*           cookie __attribute__ ((unused)) )
{

  // These are convenient so that I only apply the casts once
  int*    Jrowptr = (int*)Jt->p;
  int*    Jcolidx = (int*)Jt->i;
  double* Jval    = (double*)Jt->x;

  int iJacobian = 0;
#define STORE_JACOBIAN(col, g)                  \
        do                                      \
        {                                       \
          Jcolidx[ iJacobian ] = col;           \
          Jval   [ iJacobian ] = g;             \
          iJacobian++;                          \
        } while(0)


  double norm2_x = 0.0;

  for(int i=0; i<Nmeasurements; i++)
  {
    x[i] =
      p[0] * p[1] * allx[i]*allx[i] +
      p[1] * p[2] * ally[i]*ally[i] +
      p[2] *        allx[i]*ally[i] +
      p[3] *        allx[i] +
      p[4] *        ally[i] +
      p[5]
      - allm_simulated_noisy[i];

    norm2_x += x[i]*x[i];

    // In this sample problem, every measurement depends on every element of the
    // state vector, so I loop through all the state vectors here. In practice
    // libdogleg is meant to be applied to sparse problems, where this internal
    // loop would be MUCH shorter than Nstate long
    Jrowptr[i] = iJacobian;
    STORE_JACOBIAN( 0, p[1]*allx[i]*allx[i] );
    STORE_JACOBIAN( 1, p[0]*allx[i]*allx[i] + p[2] * ally[i]*ally[i] );
    STORE_JACOBIAN( 2, p[1] * ally[i]*ally[i] + allx[i]*ally[i] );
    STORE_JACOBIAN( 3, allx[i] );
    STORE_JACOBIAN( 4, ally[i] );
    STORE_JACOBIAN( 5, 1.0  );
  }
  Jrowptr[Nmeasurements] = iJacobian;


  fprintf(stderr, "Callback finished. 2-norm is %f\n", norm2_x);
}

int main(void)
{
  srandom( time(NULL) );

  generateSimulationGrid();
  simulate();

  dogleg_setDebug(1); // request debugging output from the solver


  double p[Nstate];

  // I start solving with all my state variables set to some random noise
  for(int i=0; i<Nstate; i++)
    p[i] = ((double)random() / (double)RAND_MAX - 0.5) * 1.0; // +- 0.5 units of uniformly-random noise

  fprintf(stderr, "starting state:\n");
  for(int i=0; i<Nstate; i++)
    fprintf(stderr, "  p[%d] = %f\n", i, p[i]);

  // This demo problem is dense, so every measurement depends on every state
  // variable. Thus ever element of the jacobian is non-zero
  int Jnnz = Nmeasurements * Nstate;

  // first, let's test our gradients. This is just a verification step to make
  // sure the optimizerCallback() is written correctly. Normally, you would do
  // this as a check when developing your program, but would turn this off in
  // the final application. This will generate LOTS of output. You need to make
  // sure that the reported and observed gradients match (the relative error is
  // low)
  fprintf(stderr, "have %d variables\n", Nstate);
  for(int i=0; i<Nstate; i++)
  {
    fprintf(stderr, "checking gradients for variable %d\n", i);
    dogleg_testGradient(i, p, Nstate, Nmeasurements, Jnnz, &optimizerCallback, NULL);
  }


  fprintf(stderr, "SOLVING:\n");

  double optimum = dogleg_optimize(p, Nstate, Nmeasurements, Jnnz, &optimizerCallback, NULL, NULL);

  fprintf(stderr, "Done. Optimum = %f\n", optimum);

  fprintf(stderr, "optimal state:\n");
  for(int i=0; i<Nstate; i++)
    fprintf(stderr, "  p[%d] = %f\n", i, p[i]);

  return 0;
}
