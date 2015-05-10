/***********************
 * ParticleSystem class
 ***********************/

/**
 * The particle system class simply "manages" a collection of particles.
 * Its primary responsibility is to run the simulation, evolving particles
 * over time according to the applied forces using Euler's method.
 * This header file contains the functions that you are required to implement.
 * (i.e. the rest of the code relies on this interface)
 * In addition, there are a few suggested state variables included.
 * You should add to this class (and probably create new classes to model
 * particles and forces) to build your system.
 */

#ifndef __PARTICLE_SYSTEM_H__
#define __PARTICLE_SYSTEM_H__

#define IX(i,j,k) ((i)*(N+2)*(N+2)+(j)*(N+2)+(k))
#define FOR_EACH_GRID(N,i,j,k)	\
		for(i = 1; i <= N; i++)	\
for(j = 1; j <= N; j++) 	\
for(k = 1; k <= N; k++)
#include "vec.h"

#include <vector>
#include <map>
using namespace std;


class Particle {

		public:

				Particle(): position(0,0,0),velocity(0,0,0),force(0,0,0),mass(1) {};

				Vec3f position;
				Vec3f velocity;
				Vec3f force;
				float mass;
};

class FluidSystem{
		public:
				int N;
				double dt, diff, visc;
				double * u, * v, * u0, * v0;
				double * w, * w0;
				double * dens, * dens0;
				int iteration_count = 15;
				FluidSystem(int size){
						int _size=(size+2)*(size+2)*(size+2);
						int i,j,k;
						N = size;
						//default
						dt = 0.1f;
						diff = 0.1f;
						visc = 0.1f;
						u = (double *) malloc ( _size*sizeof(double) );
						v = (double *) malloc ( _size*sizeof(double) );
						w = (double *) malloc ( _size*sizeof(double) );
						u0 = (double *) malloc ( _size*sizeof(double) );
						v0 = (double *) malloc ( _size*sizeof(double) );
						w0 = (double *) malloc ( _size*sizeof(double) );
						dens = (double *) malloc ( _size*sizeof(double) );	
						dens0 = (double *) malloc ( _size*sizeof(double) );
						for ( i=0 ; i <_size ; i++ ) {
								w[i] = w0[i] = u[i] = v[i] = u0[i] = v0[i] = dens[i] = dens0[i] = 0.0f;
						}
						//default moving up
						v[IX(N/2,1,N/2)] = 5.0f;
						dens[IX(N/2,1,N/2)] = 50.0f;
						/*
						   for ( i=0 ; i <_size ; i++ ) {
						   if(u[i] != 0)
						 	  cout << i<<","<<u[i] << endl;
						   }
						 */
				}
				void draw_fluid ( void );
				void update_fluid(double dt);
				void add_to_array(double* x, int index, double quantity);
				void gs_solver(int N, double* x, double *x0, double rate, double div);
				void project(int N, double *u, double *v, double *w, double *g, double *g0);
				void diffuse(int N, double *d, double *d0, double rate);
				void advect(int N, double *d, double *d0, double *u, double *v, double *w, double dt);
				void dens_step(int N, double *d, double *d0, double *u, double *v, double *w, double dif, double dt);
				void vel_step(int N, double *u, double *v, double *w, double *u0, double *v0, double *w0, double vis, double dt);
};

class ParticleSystem {

		public:

				/** Constructor **/
				ParticleSystem();

				/** Destructor **/
				virtual ~ParticleSystem();
				FluidSystem ss = FluidSystem(60);

				/** Simulation fxns **/
				// This fxn should render all particles in the system,
				// at current time t.
				virtual void drawParticles(float t);

				// This fxn should save the configuration of all particles
				// at current time t.
				virtual void bakeParticles(float t);

				// This function should compute forces acting on all particles
				// and update their state (pos and vel) appropriately.
				virtual void computeForcesAndUpdateParticles(float t);

				// This function should reset the system to its initial state.
				// When you need to reset your simulation, PLEASE USE THIS FXN.
				// It sets some state variables that the UI requires to properly
				// update the display.  Ditto for the following two functions.
				virtual void resetSimulation(float t);

				// This function should start the simulation
				virtual void startSimulation(float t);

				// This function should stop the simulation
				virtual void stopSimulation(float t);

				// This function should clear out your data structure
				// of baked particles (without leaking memory).
				virtual void clearBaked();

				void ponyTail_computeForcesAndUpdateParticles(float t);
				void cloth_computeForcesAndUpdateParticles(float t);
				void pipe_computeForcesAndUpdateParticles(float t);


				// These accessor fxns are implemented for you
				float getBakeStartTime() { return bake_start_time; }
				float getBakeEndTime() { return bake_end_time; }
				float getBakeFps() { return bake_fps; }
				bool isSimulate() { return simulate; }
				bool isDirty() { return dirty; }
				void setDirty(bool d) { dirty = d; }

				static Vec4f particleOrigin;
				static Vec4f particleOrigin_pipe;
				static Vec4f particleOrigin_pony;
				static Vec4f particleOrigin_cloth;
				static bool pipe;
				static bool pony;
				static bool cloth;
				static bool bounceOff;
				static Vec4f cloth_start;
				static Vec4f cloth_end;

				float time;
				float time2;

		protected:


				/** Some baking-related state **/
				float bake_fps;						// frame rate at which simulation was baked
				float bake_start_time;				// time at which baking started 
				// These 2 variables are used by the UI for
				// updating the grey indicator 
				float bake_end_time;				// time at which baking ended

				/** General state variables **/
				bool simulate;						// flag for simulation mode
				bool dirty;							// flag for updating ui (don't worry about this)



				/*** particles***/

				std::vector<Particle*> particles;
				std::vector<Particle*> particles_pipe;
				std::vector<Particle*> ponyTail_particles;

				Particle** cloth_particles;

				//    std::vector<std::vector<Particle*> > bake_particles;


				std::map<float, std::vector<Vec3f> > bake_particles;
				std::map<float, std::vector<Vec3f> > bake_particlesPipe;
				std::map<float, std::vector<Vec3f> > bake_particlesPony;
				std::map<float, std::vector<Vec3f> > bake_particlesCloth;


				static Vec3f gravity;
				static float airResistance;
				static float particleRadius;
				static int particleNum;
				static int particleReal;
				static int particleNum_ponyTail;
				static int particleNum_cloth_row;
				static int particleNum_cloth_col;
				static float spring_K;
				static float spring_cloth_K;
				static float deltaX;
};


#endif	// __PARTICLE_SYSTEM_H__
