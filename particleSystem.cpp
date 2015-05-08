#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerdraw.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <vector>
#include <map>

#include <FL/gl.h>

using namespace std;

static float prevT;
float ParticleSystem::particleRadius = 0.15;
Vec3f ParticleSystem::gravity = Vec3f(0, -9.8f, 0);
float ParticleSystem::airResistance = 0.5;
int ParticleSystem::particleNum = 500;
int ParticleSystem::particleReal = 0;
float ParticleSystem::spring_K = 500;
int ParticleSystem::particleNum_ponyTail = 8;
int ParticleSystem::particleNum_cloth_row = 15;
int ParticleSystem::particleNum_cloth_col = 5;
float ParticleSystem::deltaX = 0;
bool ParticleSystem::pipe = false;
bool ParticleSystem::pony = false;
bool ParticleSystem::cloth = false;
bool ParticleSystem::bounceOff = false;

Vec4f ParticleSystem::particleOrigin_pony = Vec4f(0,0,0,1);
Vec4f ParticleSystem::particleOrigin_cloth = Vec4f(0,0,0,1);

Vec4f ParticleSystem::pelvisPosition = Vec4f(0,0,0,1);

Vec4f ParticleSystem::cloth_start = Vec4f(0,0,0,1);
Vec4f ParticleSystem::cloth_end = Vec4f(0,0,0,1);

Vec4f ParticleSystem::particleOrigin_pipe = Vec4f(0,0,0,1);


Vec3f ori_position = Vec3f(0,0,0);

float ParticleSystem::spring_cloth_K = 100;

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	// TODO
    time = 0;
    time2 = 0;
}


/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO


}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
	// TODO
    
    time  = 0;
    time2 = 0;
    // pony tail
    
    if (pony) {
    
        particleReal = 0;
        ori_position = Vec3f(particleOrigin);
        
        ponyTail_particles.resize(particleNum_ponyTail, NULL);
        Particle* p = new Particle();
        p->position = Vec3f(particleOrigin_pony);
        p->velocity = Vec3f(0,0,0);
        p->force = Vec3f(0,0,0);
        p->mass = 1.0;
        ponyTail_particles[0] = p;
        
        
        for (int i = 1; i < particleNum_ponyTail; i++) {
            
            Particle* p = new Particle();
            p->position = Vec3f(particleOrigin_pony[0] + i * 0.5, particleOrigin_pony[1] + i * 0.1, particleOrigin_pony[2] + i * 0.1);
            p->velocity = Vec3f(0.15,0.1,0);
            p->force = Vec3f(gravity);
            p->mass = 1.0;
            ponyTail_particles[i] = p;
        }
    
    }
    
    // cloth
    
    if (cloth) {
    
      //  std::cout <<"start simulate " << t << std::endl;
        cloth_particles = new Particle*[particleNum_cloth_row];
        for (int i = 0; i < particleNum_cloth_row; i++) {
            cloth_particles[i] = new Particle[particleNum_cloth_col];
        }

        
        deltaX = (cloth_start[0] - cloth_end[0]) * 1.0 / 4.0;
        float deltaY = (cloth_start[1] - cloth_end[1]) * 1.0 / 4.0;
        float deltaZ = (cloth_start[2] - cloth_end[2]) * 1.0 / 4.0;
        
        
        for (int j = 0; j < particleNum_cloth_col; j++) {
            cloth_particles[0][j].position = Vec3f( cloth_start[0] - deltaX * j, cloth_start[1] - deltaY * j + 0.3, cloth_start[2] - deltaZ * j);
            cloth_particles[0][j].velocity = Vec3f(0,0,0);
            cloth_particles[0][j].force = Vec3f(gravity);
            cloth_particles[0][j].mass = 1.0;
        }
        
        
        for (int i = 1; i < particleNum_cloth_row; i++) {
            for (int j = 0; j < particleNum_cloth_col; j++) {
                
                cloth_particles[i][j].position = Vec3f(cloth_particles[0][0].position[0] - deltaX * j, cloth_particles[0][0].position[1] - 0.2* i, cloth_particles[0][0].position[2] + 0.15 * i);
                
                cloth_particles[i][j].force = Vec3f(gravity);
                cloth_particles[i][j].velocity = Vec3f(0,0,0);
                cloth_particles[i][j].mass = 1.0;
                
            }
            
        }
        
      //  std::cout <<"position: " << cloth_particles[0][0].position[0] <<"," << cloth_particles[0][0].position[1] << "," << cloth_particles[0][0].position[2] << std::endl;
      //  std::cout <<"position: " << cloth_particles[9][4].position[0] <<"," << cloth_particles[9][4].position[1] << "," << cloth_particles[9][4].position[2] << std::endl;
    }
    
	// These values are used by the UI ...
	// negative bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
	// TODO
    
    particleReal = 0;
    particles.clear();
    if (pony) ponyTail_particles.clear();
    
    if (cloth) {
        for (int i = 0; i < particleNum_cloth_row; i++) delete[] cloth_particles[i];
        delete[] cloth_particles;
    }
    
	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
	// TODO

    particleReal = 0;
    particles.clear();
    if (pony) ponyTail_particles.clear();
    
    if (cloth) {
        for (int i = 0; i < particleNum_cloth_row; i++) delete[] cloth_particles[i];
        delete[] cloth_particles;
    }
        
	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	// TODO
    
   //  std::cout <<"time: " << t << std::endl;
    
    if (isSimulate()) {
        
        
        if (t - time > 0.3) {
        
            time = t;
            
            
          //  if (bounceOff) {
                
              //   particleNum = 5;
          //  }
            
            
            
            if  (particles.size() < particleNum) {
                
                Particle* p = new Particle();
                p->position = Vec3f(particleOrigin);
                p->velocity = Vec3f(-1, -2, 0);
                p->force = Vec3f(gravity);
                particles.push_back(p);
                particleReal++;
            }
        }
        
        float deltaT = t - prevT;

        typedef vector<Particle*>::const_iterator iter;
        
        for(iter i = particles.begin(); i != particles.end(); ++i) {
            
            // F = 1/2 * C * S * V^2
            Vec3f airResistance_cur = Vec3f(0,0,0);
           
            airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
            airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
            airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];
            
            if ((*i)->velocity[0] > 0) {
                
                (*i)->force[0] -= airResistance_cur[0];
            } else {
                
                 (*i)->force[0] += airResistance_cur[0];
            }
            
            if ((*i)->velocity[1] > 0) {
                
                (*i)->force[1] -= airResistance_cur[1];
            } else {
                
                (*i)->force[1] += airResistance_cur[1];
            }
            
            if ((*i)->velocity[2] > 0) {
                
                (*i)->force[2] -= airResistance_cur[2];
            } else {
                
                (*i)->force[2] += airResistance_cur[2];
            }
            
            
            (*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.1;
            
            (*i)->position[0] += (*i)->velocity[0] * deltaT;
            (*i)->position[1] += (*i)->velocity[1] * deltaT;
            (*i)->position[2] += (*i)->velocity[2] * deltaT;
            
            
            if(bounceOff) {
                
                if ((*i)->position[1] < 0) {
                    
                    float len = ((*i)->velocity).length();
                    
                    Vec3f N = Vec3f(0,1,0);
                    Vec3f V = -1 * Vec3f((*i)->velocity);
                    V.normalize();
                    
                    
                    double NVv = N * V;
                    Vec3f R = N * 2 * NVv - V;
                    
                    R.normalize();
                    
                    
                    (*i)->velocity[0] = len * R[0];
                    (*i)->velocity[1] = len * R[1];
                    (*i)->velocity[2] = len * R[2];
  
                    
                }
                
            }
            

        }
        
        if (pipe) {
            
            pipe_computeForcesAndUpdateParticles(t);
        }
        
        if (pony) {
            ponyTail_computeForcesAndUpdateParticles(t);
        }
        
        if (cloth) {
            cloth_computeForcesAndUpdateParticles(t);
        }
        
        bakeParticles(t);
        
    }
    
	// Debugging info
	if( t - prevT > .04 )
		printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
	prevT = t;
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	// TODO
    
    
    // state update
    
     if (isSimulate()) {
         
         typedef vector<Particle*>::const_iterator iter;
         
         
         if (!bounceOff) {
         
             
             for (iter i = particles_pipe.begin(); i != particles_pipe.end(); i++) {
                     
                     if ((*i) -> position[1] < 0 || (*i) -> position[0] > 5 || (*i) -> position[1] > 10) {
                         
                         particles_pipe.erase(i);
                         
                     }
             }
             
             
             
         for (iter i = particles.begin(); i != particles.end(); i++) {
 
             if ((*i)->position[1] < 0) {
                 
                 if ((*i) -> position[1] < 0) {
                     
                     particles.erase(i);
                     particleReal--;
                 }
             }
         }
             
         }
         
         // particles
         
         
         for(iter i = particles.begin(); i != particles.end(); ++i) {
    
             
             
             setDiffuseColor( rand() % 100 * 1.0/ 100 , rand() % 100 * 1.0/ 100, rand() % 100 * 1.0/ 100 );
             setAmbientColor( 1, 1, 1 );
             
             glPushMatrix();
             glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
             drawSphere(particleRadius);
             glPopMatrix();
         }
         
         if (pipe) {
             
             
             setDiffuseColor(176.0/256,196.0/256,222.0/256);
             setAmbientColor(0,0,0);
             
             for(iter i = particles_pipe.begin(); i != particles_pipe.end(); ++i) {
                 
                 glPushMatrix();
                 glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
                 drawSphere(0.1);
                 glPopMatrix();
             }
         }
         
         
         //ponyTail
         
         if (pony) {
         
             
             setDiffuseColor( 255.0 / 255 , 255.0 / 255, 240.0 / 255 );
             setAmbientColor (85.0/ 256,107.0 / 256,47.0 / 256);
             
             for(iter i = ponyTail_particles.begin(); i != ponyTail_particles.end(); ++i) {
                 
                 
                 glPushMatrix();
                    glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
                    drawSphere(particleRadius);
                 glPopMatrix();
                 
                 
                 if (i != ponyTail_particles.begin()) {
                 
                     glLineWidth(2.5);
                     glColor3f(0.0, 0.0, 1.0);
                     glBegin(GL_LINES);
                        glVertex3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
                        glVertex3f((*(i-1))->position[0], (*(i-1))->position[1], (*(i-1))->position[2]);
                     glEnd();
                 }
                 
             }
         }
         
         // cloth
         
         
         if (cloth) {
         
             for (int i = 0; i < particleNum_cloth_row - 1; i++)
             {
                 for (int j = 0; j < particleNum_cloth_col - 1; j++)
                 {
                     Vec3f p;
                     
                     p = cloth_particles[i][j].position;
                     
                     setDiffuseColor( 143.0 / 255 , 188.0 / 255, 143.0 / 255 );
                     setAmbientColor (0,0,0);
                     
                     glPushMatrix();
                     glTranslated(p[0], p[1], p[2]);
                     drawSphere(0.05);
                     glPopMatrix();
                     
                         glBegin(GL_QUADS);
                            glVertex3f(p[0], p[1], p[2]);
                            p = cloth_particles[i][j + 1].position;
                            glVertex3f(p[0], p[1], p[2]);
                            p = cloth_particles[i + 1][j + 1].position;
                            glVertex3f(p[0], p[1], p[2]);
                            p = cloth_particles[i + 1][j ].position;
                            glVertex3f(p[0], p[1], p[2]);
                         glEnd();
                     
                 }
             }
             
         }
         
     } else {
         
         std::vector<Vec3f> current_particles_position;
         std::map<float, std::vector<Vec3f> >::iterator result = bake_particles.find(t);
         if (result != bake_particles.end())
             current_particles_position = (*result).second;
         
         typedef vector<Vec3f>::const_iterator iter;
         
         for(iter i = current_particles_position.begin(); i != current_particles_position.end(); ++i) {
             
             setDiffuseColor( rand() % 100 * 1.0/ 100 , rand() % 100 * 1.0/ 100, rand() % 100 * 1.0/ 100 );
             setAmbientColor( 1, 1, 1 );
             glPushMatrix();
                glTranslated((*i)[0], (*i)[1], (*i)[2]);
                drawSphere(particleRadius);
             glPopMatrix();
             
         }
         
         
         
         
         if (pipe) {
             
             std::vector<Vec3f> current_particlesPipe_position;
             std::map<float, std::vector<Vec3f> >::iterator result = bake_particlesPipe.find(t);
             if (result != bake_particlesPipe.end())
                 current_particlesPipe_position = (*result).second;
             
             typedef vector<Vec3f>::const_iterator iter;
             
             for(iter i = current_particlesPipe_position.begin(); i != current_particlesPipe_position.end(); ++i) {
                 
                 setDiffuseColor(176.0/256,196.0/256,222.0/256);
                 setAmbientColor(0,0,0);
                 glPushMatrix();
                 glTranslated((*i)[0], (*i)[1], (*i)[2]);
                 drawSphere(0.1);
                 glPopMatrix();
                 
             }
             
         }
         
         
         
         if (pony) {
         
             std::vector<Vec3f> current_particlesPony_position;
             std::map<float, std::vector<Vec3f> >::iterator result2 = bake_particlesPony.find(t);
             if (result2 != bake_particlesPony.end())
                 current_particlesPony_position = (*result2).second;
             
             typedef vector<Vec3f>::const_iterator iter;
             
             
             for(iter i = current_particlesPony_position.begin(); i != current_particlesPony_position.end(); ++i) {
                 
                 setDiffuseColor( 255.0 / 255 , 255.0 / 255, 240.0 / 255 );
                 setAmbientColor (85.0/ 256,107.0 / 256,47.0 / 256);
                 
                 glPushMatrix();
                 glTranslated((*i)[0], (*i)[1], (*i)[2]);
                 drawSphere(particleRadius);
                 glPopMatrix();
                 
                 
                 if (i != current_particlesPony_position.begin()) {
                     
                     glLineWidth(2.5);
                     glColor3f(0.0, 0.0, 1.0);
                     glBegin(GL_LINES);
                     glVertex3f((*i)[0], (*i)[1], (*i)[2]);
                     glVertex3f((*(i-1))[0], (*(i-1))[1], (*(i-1))[2]);
                     glEnd();
                 }
             }
         }
         
         
         if (cloth) {
             
         
             int index = -1;
             
             std::vector<Vec3f> current_particlesCloth_position;
             std::map<float, std::vector<Vec3f> >::iterator result3 = bake_particlesCloth.find(t);
             if (result3 != bake_particlesCloth.end())
                 current_particlesCloth_position = (*result3).second;
             
             typedef vector<Vec3f>::const_iterator iter;
             
             for(iter i = current_particlesCloth_position.begin(); i != current_particlesCloth_position.end(); ++i) {
                 
                 
                 
                 index++;
                 
                  //std::cout <<"index: " << index << std::endl;
                 
                 setDiffuseColor( 143.0 / 255 , 188.0 / 255, 143.0 / 255 );
                 setAmbientColor (0,0,0);
                 
                 glPushMatrix();
                 glTranslated((*i)[0], (*i)[1], (*i)[2]);
                 drawSphere(0.05);
                 glPopMatrix();
                 
                 
                 int row = index / particleNum_cloth_col;
                 int col = index % particleNum_cloth_col;
                 
                 
                 if (row != particleNum_cloth_row - 1 && col != particleNum_cloth_col - 1) {
                     
                     Vec3f p;
                     
                     glBegin(GL_QUADS);
                     glVertex3f(current_particlesCloth_position[row * particleNum_cloth_col + col][0], current_particlesCloth_position[row * particleNum_cloth_col + col][1], current_particlesCloth_position[row * particleNum_cloth_col + col][2]);
                     
                     glVertex3f(current_particlesCloth_position[row * particleNum_cloth_col + col + 1][0], current_particlesCloth_position[row * particleNum_cloth_col + col + 1][1], current_particlesCloth_position[row * particleNum_cloth_col + col + 1][2]);
                     
                     glVertex3f(current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col + 1][0], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col + 1][1], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col + 1][2]);
                     
                     
                     glVertex3f(current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col][0], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col][1], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col][2]);
                     glEnd();
                     
                     
                 }
                 
             }
             
         }
         
     }

}



/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
	// TODO
    
    std::vector<Vec3f> current_particles_position;
    typedef vector<Particle*>::const_iterator iter;
    
    // state update
    
    for(iter i = particles.begin(); i != particles.end(); ++i) {
        
        Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
        
        current_particles_position.push_back(position);
        
    }
    
    bake_particles.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particles_position));
    
    
    if (pipe) {
        
        
        std::vector<Vec3f> current_particlesPipe_position;
        typedef vector<Particle*>::const_iterator iter;
        
        // state update
        
        for(iter i = particles_pipe.begin(); i != particles_pipe.end(); ++i) {
            
            Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
            
            current_particlesPipe_position.push_back(position);
            
        }
        
        bake_particlesPipe.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particlesPipe_position));
        
        
        
    }
    
    
    if (pony) {
    
        std::vector<Vec3f> current_particlesPony_position;
        
        for(iter i = ponyTail_particles.begin(); i != ponyTail_particles.end(); ++i) {
            
            Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
            
            current_particlesPony_position.push_back(position);
            
        }
        
        bake_particlesPony.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particlesPony_position));
    
    }
    
    if (pony) {
    
        std::vector<Vec3f> current_particlesCloth_position;
        
        for(int i = 0; i < particleNum_cloth_row; i++) {
            for (int j = 0; j < particleNum_cloth_col; j++) {
                
                Vec3f position = Vec3f(cloth_particles[i][j].position[0], cloth_particles[i][j].position[1], cloth_particles[i][j].position[2]);
                current_particlesCloth_position.push_back(position);
            }
        }
        
        bake_particlesCloth.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particlesCloth_position));
    
    }
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
    
    bake_particles.clear();
    if (pony) bake_particlesPony.clear();
    if (cloth) bake_particlesCloth.clear();

}


void ParticleSystem::ponyTail_computeForcesAndUpdateParticles(float t)
{
    // TODO
    
    if (isSimulate()) {
        
        float dx = 0;
        float dy = 0;
        float dz = 0;
        
        float deltaT = t - prevT;
        
        typedef vector<Particle*>::const_iterator iter;
        
        iter i = ponyTail_particles.begin();
    
        
        if ((*i)->position[0] != particleOrigin_pony[0] ||  (*i)->position[1] != particleOrigin_pony[1] || (*i)->position[2] != particleOrigin_pony[2] ) {
            
            (*i)->position = Vec3f(particleOrigin_pony);
            dx = particleOrigin_pony[0] - (*i)->position[0];
            dy = particleOrigin_pony[1] - (*i)->position[1];
            dz = particleOrigin_pony[2] - (*i)->position[2];
            
        }
        
        
        int index = 0;
        
        for(iter i = ponyTail_particles.begin() + 1; i != ponyTail_particles.end(); ++i) {
            
            index++;
            
            // F = 1/2 * C * S * V^2
            Vec3f airResistance_cur = Vec3f(0,0,0);
            airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
            airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
            airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];
            
            if ((*i)->velocity[0] > 0) {
                
                (*i)->force[0] -= airResistance_cur[0];
            } else {
                
                (*i)->force[0] += airResistance_cur[0];
            }
            
            if ((*i)->velocity[1] > 0) {
                
                (*i)->force[1] -= airResistance_cur[1];
            } else {
                
                (*i)->force[1] += airResistance_cur[1];
            }
            
            if ((*i)->velocity[2] > 0) {
                
                (*i)->force[2] -= airResistance_cur[2];
            } else {
                
                (*i)->force[2] += airResistance_cur[2];
            }
            
         

            
            if (index < particleNum_ponyTail - 1) {
                
                
                Vec3f left = Vec3f((*i)->position[0] - (*(i - 1))->position[0], (*i)->position[1] - (*(i - 1))->position[1], (*i)->position[2] - (*(i - 1))->position[2]);
                
                float distance_left = left.length();
                left.normalize();
                Vec3f force_left = spring_K * (1 - distance_left) * left;
                Vec3f right = Vec3f((*i)->position[0] - (*(i+1))->position[0], (*i)->position[1] - (*(i+1))->position[1], (*i)->position[2] - (*(i+1))->position[2]);
                float distance_right = right.length();
                right.normalize();
                Vec3f force_right = spring_K * (1 - distance_right) * right;
                
                (*i)->force[0] += force_left[0];
                (*i)->force[0] += force_right[0];
                
                (*i)->force[1] += force_left[1];
                (*i)->force[1] += force_right[1];
                
                (*i)->force[2] += force_left[2];
                (*i)->force[2] += force_right[2];
                
            } else {
                
                
                Vec3f left = Vec3f((*i)->position[0] - (*(i - 1))->position[0], (*i)->position[1] - (*(i - 1))->position[1], (*i)->position[2] - (*(i - 1))->position[2]);
                float distance_left = left.length();
                left.normalize();
                Vec3f force_left = spring_K * (1 - distance_left) * left;
                
                (*i)->force[0]  +=  force_left[0];
                (*i)->force[1]  +=  force_left[1];
                (*i)->force[2]  +=  force_left[2];
            }

            
            if ((*i)->force.length() > 10) {
                (*i)->force.normalize();
                (*i)->force *= 10;
            }
            
            (*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.05;
            (*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.05;
            (*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.05;
            
            (*i)->position[0] += (*i)->velocity[0] * deltaT;
            (*i)->position[1] += (*i)->velocity[1] * deltaT;
            (*i)->position[2] += (*i)->velocity[2] * deltaT;
            
            
            (*i)->position[0] += dx;
            (*i)->position[1] += dy;
            (*i)->position[2] += dz;
        }
    }
}



void ParticleSystem::cloth_computeForcesAndUpdateParticles(float t)
{
    // TODO
    
    if (isSimulate()) {
    
        float deltaT = t - prevT;
        
        for (int i = 1; i < particleNum_cloth_row; i++) {
            for (int j = 0; j < particleNum_cloth_col; j++) {

                // F = 1/2 * C * S * V^2
                Vec3f airResistance_cur = Vec3f(0,0,0);
                airResistance_cur[0] = 0.5 * airResistance * 0.054 * cloth_particles[i][j].velocity[0] * cloth_particles[i][j].velocity[0];
                airResistance_cur[1] = 0.5 * airResistance * 0.054 * cloth_particles[i][j].velocity[1] * cloth_particles[i][j].velocity[1];
                airResistance_cur[2] = 0.5 * airResistance * 0.054* cloth_particles[i][j].velocity[2] * cloth_particles[i][j].velocity[2];
                
                if (cloth_particles[i][j].velocity[0] > 0) {
                    
                    cloth_particles[i][j].velocity[0] -= airResistance_cur[0];
                } else {
                    
                    cloth_particles[i][j].velocity[0] += airResistance_cur[0];
                }
                
                if (cloth_particles[i][j].velocity[1] > 0) {
                    
                    cloth_particles[i][j].force[1] -= airResistance_cur[1];
                } else {
                    
                    cloth_particles[i][j].force[1] += airResistance_cur[1];
                }
                
                if (cloth_particles[i][j].velocity[2] > 0) {
                    
                    cloth_particles[i][j].force[2] -= airResistance_cur[2];
                } else {
                    
                    cloth_particles[i][j].force[2] += airResistance_cur[2];
                }
                
                Vec3f force_left = Vec3f(0,0,0);
                Vec3f force_right = Vec3f(0,0,0);
                Vec3f force_up = Vec3f(0,0,0);
                Vec3f force_down = Vec3f(0,0,0);
                
                
                if (j != 0) {
                    
                    Vec3f left = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i][j - 1].position[0], cloth_particles[i][j].position[1] - cloth_particles[i][j - 1].position[1], cloth_particles[i][j].position[2] - cloth_particles[i][j - 1].position[2]);
                    
                    float distance_left = left.length();
                    left.normalize();
                    force_left = spring_cloth_K * (deltaX - distance_left) * left;
                    
                }

                
                if (j != particleNum_cloth_col - 1) {
                
                    Vec3f right = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i][j + 1].position[0], cloth_particles[i][j].position[1] - cloth_particles[i][j+1].position[1], cloth_particles[i][j].position[2] - cloth_particles[i][j + 1].position[2]);
                    float distance_right = right.length();
                    right.normalize();
                    force_right = spring_cloth_K * (deltaX - distance_right) * right;
                }
                
                if (i != 0) {
                    
                    Vec3f up = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i - 1 ][j].position[0], cloth_particles[i][j].position[1] - cloth_particles[i - 1][j].position[1], cloth_particles[i][j].position[2] - cloth_particles[i - 1][j].position[2]);
                    
                    float distance_up = up.length();
                    up.normalize();
                    force_up = spring_cloth_K * (0.15 - distance_up) * up;
                    
                }
     
                
                if (i != particleNum_cloth_row - 1) {
                    
                    Vec3f down = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i + 1 ][j].position[0], cloth_particles[i][j].position[1] - cloth_particles[i + 1][j].position[1], cloth_particles[i][j].position[2] - cloth_particles[i + 1][j].position[2]);
                    
                    float distance_down = down.length();
                    down.normalize();
                    force_down = spring_cloth_K * (0.15 - distance_down) * down;
                }
  
                    cloth_particles[i][j].force[0] += force_left[0];
                    cloth_particles[i][j].force[0] += force_right[0];
                    cloth_particles[i][j].force[0] += force_up[0];
                    cloth_particles[i][j].force[0] += force_down[0];
                
                    cloth_particles[i][j].force[1] += force_left[1];
                    cloth_particles[i][j].force[1] += force_right[1];
                    cloth_particles[i][j].force[1] += force_up[1];
                    cloth_particles[i][j].force[1] += force_down[1];
                    cloth_particles[i][j].force[1] += 0.001;
                
                    cloth_particles[i][j].force[2] += force_left[2];
                    cloth_particles[i][j].force[2] += force_right[2];
                    cloth_particles[i][j].force[2] += force_up[2];
                    cloth_particles[i][j].force[2] += force_down[2];
                    cloth_particles[i][j].force[2] += 0.8;
                
                
                if (cloth_particles[i][j].force.length() > 10) {
                    cloth_particles[i][j].force.normalize();
                    cloth_particles[i][j].force *= 10;
                }
                
                
                cloth_particles[i][j].velocity[0] += cloth_particles[i][j].force[0] * 1.0 / cloth_particles[i][j].mass * deltaT * 0.01;
                cloth_particles[i][j].velocity[1] += cloth_particles[i][j].force[1] * 1.0 / cloth_particles[i][j].mass * deltaT * 0.01;
                cloth_particles[i][j].velocity[2] += cloth_particles[i][j].force[2] * 1.0 / cloth_particles[i][j].mass * deltaT * 0.01;
                
                    cloth_particles[i][j].position[0] += cloth_particles[i][j].velocity[0] * deltaT;
                    cloth_particles[i][j].position[1] += cloth_particles[i][j].velocity[1] * deltaT;
                    cloth_particles[i][j].position[2] += cloth_particles[i][j].velocity[2] * deltaT;
                
            }
        
        }
    }
}


void ParticleSystem::pipe_computeForcesAndUpdateParticles(float t){
    
    if (isSimulate()) {
        
        
        if (t - time2 > 0.05) {
            
            time2 = t;
 
            if  (particles_pipe.size() < particleNum) {
                
                Particle* p = new Particle();
                p->position = Vec3f(particleOrigin_pipe[0]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[1]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[2]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1);
                p->velocity = Vec3f(4, 6, 0);
                p->force = Vec3f(gravity);
                particles_pipe.push_back(p);
                
                
                p = new Particle();
                p->position = Vec3f(particleOrigin_pipe[0]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[1]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[2]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1);
                p->velocity = Vec3f(4, 6, 0);
                p->force = Vec3f(gravity);
                particles_pipe.push_back(p);
                
                
                p = new Particle();
                p->position = Vec3f(particleOrigin_pipe[0]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[1]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[2]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1);
                p->velocity = Vec3f(4, 6, 0);
                p->force = Vec3f(gravity);
                particles_pipe.push_back(p);
                
            }
        }
        
        float deltaT = t - prevT;
        
        typedef vector<Particle*>::const_iterator iter;
        
        for(iter i = particles_pipe.begin(); i != particles_pipe.end(); ++i) {
            
            // F = 1/2 * C * S * V^2
            Vec3f airResistance_cur = Vec3f(0,0,0);
            
            airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
            airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
            airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];
            
            if ((*i)->velocity[0] > 0) {
                
                (*i)->force[0] -= airResistance_cur[0];
            } else {
                
                (*i)->force[0] += airResistance_cur[0];
            }
            
            if ((*i)->velocity[1] > 0) {
                
                (*i)->force[1] -= airResistance_cur[1];
            } else {
                
                (*i)->force[1] += airResistance_cur[1];
            }
            
            if ((*i)->velocity[2] > 0) {
                
                (*i)->force[2] -= airResistance_cur[2];
            } else {
                
                (*i)->force[2] += airResistance_cur[2];
            }
            
            
            (*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.1;
            
            (*i)->position[0] += (*i)->velocity[0] * deltaT;
            (*i)->position[1] += (*i)->velocity[1] * deltaT;
            (*i)->position[2] += (*i)->velocity[2] * deltaT;
        }
    }
}
