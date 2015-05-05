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

#include "vec.h"

#include <vector>
#include <map>


class Particle {
    
public:
    
    Particle(): position(0,0,0),velocity(0,0,0),force(0,0,0),mass(1) {};
    
    Vec3f position;
    Vec3f velocity;
    Vec3f force;
    float mass;
};



class ParticleSystem {

public:

	/** Constructor **/
	ParticleSystem();

	/** Destructor **/
	virtual ~ParticleSystem();

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
