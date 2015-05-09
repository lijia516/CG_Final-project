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

#define IX(i,j,k) (i*(N)*(N)+ j*(N) +k)
#define FOR_EACH_GRID(N,i,j,k)	\
		for(i = 0; i < N; i++)	\
for(j = 0; j < N; j++) 	\
for(k = 0; k < N; k++)
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
				float dt, diff, visc;
				float force, source;
				int dvel;
				float * u, * v, * u0, * v0;
				float * w, * w0;
				float * dens, * dens0;
				FluidSystem(int size){
						int _size=(size)*(size)*(size);
				int i,j,k;
						dt = 0.1f;
						diff = 0.0f;
						visc = 0.0f;
						force = 5.0f;
						source = 20.0f;
						N = size;
						u			= (float *) malloc ( _size*sizeof(float) );
						v			= (float *) malloc ( _size*sizeof(float) );
						w			= (float *) malloc ( _size*sizeof(float) );
						u0		= (float *) malloc ( _size*sizeof(float) );
						v0		= (float *) malloc ( _size*sizeof(float) );
						w0		= (float *) malloc ( _size*sizeof(float) );
						dens		= (float *) malloc ( _size*sizeof(float) );	
						dens0	= (float *) malloc ( _size*sizeof(float) );
						for ( i=0 ; i <_size ; i++ ) {
								w[i] = w0[i] = u[i] = v[i] = u0[i] = v0[i] = dens[i] = dens0[i] = 0.0f;
						}
						u[IX(1,1,1)] = 2.0f;
						v[IX(1,1,1)] = 2.0f;
						w[IX(1,1,1)] = 2.0f;
						dens[IX(1,1,1)] = 20.0f;
				}
				void draw_fluid ( void );
				void update_fluid(float dt);
				void add_to_array(float* x, int index, float quantity);
				void gs_solver(int N, float* x, float *x0, float rate, float div);
				void project(int N, float *u, float *v, float *w, float *g, float *g0);
				void diffuse(int N, float *d, float *d0, float rate);
				void advect(int N, float *d, float *d0, float *u, float *v, float *w, float dt);
				void dens_step(int N, float *d, float *d0, float *u, float *v, float *w, float dif, float dt);
				void vel_step(int N, float *u, float *v, float *w, float *u0, float *v0, float *w0, float vis, float dt);
};

class ParticleSystem {

		public:

				/** Constructor **/
				ParticleSystem();

				/** Destructor **/
				virtual ~ParticleSystem();
				FluidSystem ss = FluidSystem(4);


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
