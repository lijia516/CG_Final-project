// The sample robotarm model.  You should build a file
// very similar to this for when you make your model.
#pragma warning (disable : 4305)
#pragma warning (disable : 4244)
#pragma warning(disable : 4786)
#pragma warning (disable : 4312)

#include "humanInfor.h"
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "particleSystem.h"
#include "ik.h"

#include <math.h>
#include "mat.h"
#include <FL/gl.h>
#include <cstdlib>


using namespace std;

#define M_DEFAULT 2.0f
#define M_OFFSET 3.0f
#define P_OFFSET 0.3f
#define MAX_VEL 200
#define MIN_STEP 0.1

Vec4f ParticleSystem::particleOrigin = Vec4f(0,0,0,1);

// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.
enum RobotArmControls
{
    
    PELVIS_R=0, LTHIGH_RX, LTHIGH_RY,LTHIGH_RZ, RTHIGH_RX, RTHIGH_RY, RTHIGH_RZ, LSHIN_RX, RSHIN_RX, LFOOT_RX, LFOOT_RY, LFOOT_RZ, RFOOT_RX, RFOOT_RY, RFOOT_RZ, SPINE_RX, SPINE_RY, SPINE_RZ, HEAD_RX, HEAD_RY, HEAD_RZ, LSCAPULA_RX, LSCAPULA_RY, LSCAPULA_RZ, RSCAPULA_RX, RSCAPULA_RY, RSCAPULA_RZ, RFOREARM_RX, RFOREARM_RY, RFOREARM_RZ, LFOREARM_RX, LFOREARM_RY, LFOREARM_RZ, LHAND_RX, LHAND_RY, LHAND_RZ, RHAND_RX, RHAND_RY, RHAND_RZ, THIGH_EXLEN,
        LBICEP_EXLEN, RBICEP_EXLEN, TWIST,
    
    PARTICLE_COUNT, NUMCONTROLS,
    LTHIGH_H, RTHIGH_H, LSHIN_H, RSHIN_H,
    
    //BASE_ROTATION=0, LOWER_TILT, UPPER_TILT, CLAW_ROTATION,
      //  BASE_LENGTH, LOWER_LENGTH, UPPER_LENGTH, PARTICLE_COUNT, NUMCONTROLS,
};

void ground(float h);
void thigh(float h);
void pelvis(float h);
void shin(float h);
void foot(float h);
void abdomen(float h);
void spine(float h);
void head(float h);
void scapula(float h, char d);
void bicep(float h);
void forearm(float h);
void hand(float h);
void chest(float h);
void clavicle(float h);


void y_box(float h);
Mat4f glGetMatrix(GLenum pname);
Vec3f getWorldPoint(Mat4f matCamXforms);

// To make a RobotArm, we inherit off of ModelerView
class Human : public ModelerView
{
public:
    Human(int x, int y, int w, int h, char *label)
        : ModelerView(x,y,w,h,label) {}
    virtual void draw();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createHuman(int x, int y, int w, int h, char *label)
{ 
    return new Human(x,y,w,h,label);
}

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))


// Utility function.  Use glGetMatrix(GL_MODELVIEW_MATRIX) to retrieve
//  the current ModelView matrix.
Mat4f glGetMatrix(GLenum pname)
{
    GLfloat m[16];
    glGetFloatv(pname, m);
    Mat4f matCam(m[0],  m[1],  m[2],  m[3],
                            m[4],  m[5],  m[6],  m[7],
                            m[8],  m[9],  m[10], m[11],
                            m[12], m[13], m[14], m[15] );

    // because the matrix GL returns is column major...
    return matCam.transpose();
}





// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out RobotArm
void Human::draw()
{
	/* pick up the slider values */

    
    float pelvis_r = VAL( PELVIS_R );
    float lthigh_rx = VAL( LTHIGH_RX );
    float lthigh_ry = VAL( LTHIGH_RY );
    float lthigh_rz = VAL( LTHIGH_RZ );
    float rthigh_rx = VAL( RTHIGH_RX );
    float rthigh_ry = VAL( RTHIGH_RY );
    float rthigh_rz = VAL( RTHIGH_RZ );
    
    float lshin_rx = VAL( LSHIN_RX );
    float rshin_rx = VAL( RSHIN_RX );
    
    
    float lfoot_rx = VAL( LFOOT_RX );
    float lfoot_ry = VAL( LFOOT_RY );
    float lfoot_rz = VAL( LFOOT_RZ );
    
    
    float rfoot_rx = VAL( RFOOT_RX );
    float rfoot_ry = VAL( RFOOT_RY );
    float rfoot_rz = VAL( RFOOT_RZ );
    
    float spine_rx = VAL( SPINE_RX );
    float spine_ry = VAL( SPINE_RY );
    float spine_rz = VAL( SPINE_RZ );
    
    float head_rx = VAL( HEAD_RX );
    float head_ry = VAL( HEAD_RY );
    float head_rz = VAL( HEAD_RZ );
    
    float lscapula_rx = VAL( LSCAPULA_RX );
    float lscapula_ry = VAL( LSCAPULA_RY );
    float lscapula_rz = VAL( LSCAPULA_RZ );
    
    float rscapula_rx = VAL( RSCAPULA_RX );
    float rscapula_ry = VAL( RSCAPULA_RY );
    float rscapula_rz = VAL( RSCAPULA_RZ );
    
    
    float lforearm_rx = VAL( LFOREARM_RX );
    float lforearm_ry = VAL( LFOREARM_RY );
    float lforearm_rz = VAL( LFOREARM_RZ );
    
    float rforearm_rx = VAL( RFOREARM_RX );
    float rforearm_ry = VAL( RFOREARM_RY );
    float rforearm_rz = VAL( RFOREARM_RZ );
    
    float lhand_rx = VAL( LHAND_RX );
    float lhand_ry = VAL( LHAND_RY );
    float lhand_rz = VAL( LHAND_RZ );
    
    float rhand_rx = VAL( RHAND_RX );
    float rhand_ry = VAL( RHAND_RY );
    float rhand_rz = VAL( RHAND_RZ );
    
    
    float thigh_exlen = VAL( THIGH_EXLEN );
    float lbicep_len = VAL(LBICEP_EXLEN);
    float rbicep_len = VAL(RBICEP_EXLEN);
    
    float twist = VAL(TWIST);
    

    // This call takes care of a lot of the nasty projection 
    // matrix stuff
    ModelerView::draw();

    // Save the camera transform that was applied by
    // ModelerView::draw() above.
    // While we're at it, save an inverted copy of this matrix.  We'll
    // need it later.
    Mat4f matCam = glGetMatrix( GL_MODELVIEW_MATRIX );
    Mat4f matCamInverse = matCam.inverse();



	static GLfloat lmodel_ambient[] = {0.4,0.4,0.4,1.0};
    
    
    glPushMatrix();
    glTranslated(0, 0, 0);
    drawSphere(0.5);
    glPopMatrix();
    
    
    glPushMatrix();
    glTranslated(-1, 5, -1);
    drawSphere(0.1);
    glPopMatrix();
    
    
    // arm start
    glPushMatrix();
    glTranslated(0.33282,3.65,-0.49923);
    drawSphere(0.1);
    glPopMatrix();

    
    // hand start
    glPushMatrix();
    glTranslated(0.33282, 1.97, -0.49923);
    drawSphere(0.1);
    glPopMatrix();
    

	// define the model

	ground(-0.2);
    
    glRotatef( twist, 1, 1, 1 );

    glTranslatef( 0.0, 2.45, 0.0 );
    
    glPushMatrix();
    
        glTranslatef( 0.0, thigh_exlen, 0.0 );
        glRotatef(pelvis_r, 0.0, 1.0, 0.0 );
        pelvis(1);
    
        Mat4f particleXformPel = matCamInverse * glGetMatrix(GL_MODELVIEW_MATRIX);
        ParticleSystem::pelvisPosition = particleXformPel * Vec4f(0,0,0,1);
    
    
    
            // left leg and foot
            glPushMatrix();
                glTranslatef( 0.3, 0.0, 0.0 );
                glRotatef(lthigh_rx, 1.0, 0.0, 0.0 );
                glRotatef(lthigh_ry, 0.0, 1.0, 0.0 );
                glRotatef(lthigh_rz, 0.0, 0.0, 1.0 );
                thigh(thigh_exlen);
                glTranslatef( 0.0, -thigh_exlen, 0.0 );
                glPushMatrix();
                    glTranslatef( 0.0, -1, 0.0 );
                    glRotatef(lshin_rx, 1.0, 0.0, 0.0 );
                    shin(1);
                    glPushMatrix();
                        glTranslatef( 0.0, -1.3, 0.0 );
                        glRotatef(lfoot_rx, 1.0, 0.0, 0.0 );
                        glRotatef(lfoot_ry, 0.0, 1.0, 0.0 );
                        glRotatef(lfoot_rz, 0.0, 0.0, 1.0 );
                        foot(2);
                    glPopMatrix();
    
                glPopMatrix();
            glPopMatrix();
    
            // right leg and foot
            glPushMatrix();
                glTranslatef( -0.3, 0.0, 0.0 );
                glRotatef(rthigh_rx, 1.0, 0.0, 0.0 );
                glRotatef(rthigh_ry, 0.0, 1.0, 0.0 );
                glRotatef(rthigh_rz, 0.0, 0.0, 1.0 );
                thigh(thigh_exlen);
                glTranslatef( 0.0, -thigh_exlen, 0.0 );
                glPushMatrix();
                    glTranslatef( 0.0, -1, 0.0 );
                    glRotatef(rshin_rx, 1.0, 0.0, 0.0 );
                    shin(1);
                    glPushMatrix();
                        glTranslatef( 0.0, -1.3, 0.0 );
                        glRotatef(rfoot_rx, 1.0, 0.0, 0.0 );
                        glRotatef(rfoot_ry, 0.0, 1.0, 0.0 );
                        glRotatef(rfoot_rz, 0.0, 0.0, 1.0 );
                        foot(2);
                    glPopMatrix();
                glPopMatrix();
            glPopMatrix();
    
    
            // trunk
            glPushMatrix();
                glRotatef(spine_rx, 1.0, 0.0, 0.0 );
                glRotatef(spine_ry, 0.0, 1.0, 0.0 );
                glRotatef(spine_rz, 0.0, 0.0, 1.0 );
                abdomen(1);
                    glPushMatrix();
                        glTranslatef( 0.0, 1, 0.0 );
                        spine(1);
    
                        glPushMatrix();
                            glTranslatef( 0.0, 0.6, 0.0 );
                            glRotatef(head_rx, 1.0, 0.0, 0.0 );
                            glRotatef(head_ry, 0.0, 1.0, 0.0 );
                            glRotatef(head_rz, 0.0, 0.0, 1.0 );
                            head(1);
    
                        glPopMatrix();
    
                        glPushMatrix();
                            glTranslatef( 0.0, -0.1, 0.0 );
                            clavicle(1);
    
                        glPopMatrix();
    
                        glPushMatrix();
                            glTranslatef( 0.0, -0.2, 0.0 );
                            chest(1);
    
                        glPopMatrix();
    
                        // left arm
                        glPushMatrix();
                            glTranslatef( 0.6, 0.2, 0.0 );
                            scapula(1, 'l');
    
    
    
                                glPushMatrix();
                                glRotatef(lscapula_rx, 1.0, 0.0, 0.0 );
                                glRotatef(lscapula_ry, 0.0, 1.0, 0.0 );
                                glRotatef(lscapula_rz, 0.0, 0.0, 1.0 );
                                bicep(lbicep_len);
                                Mat4f particleXform3 = matCamInverse * glGetMatrix(GL_MODELVIEW_MATRIX);
                                ParticleSystem::cloth_start = particleXform3 * Vec4f(0,0,0,1);
    
                                glTranslatef( 0.0, -lbicep_len, 0.0 );
                                    glPushMatrix();
                                        glTranslatef( 0.0, -0.8, 0.0 );
                                        glTranslatef( 0.0, -lbicep_len, 0.0 );
                                        glRotatef(lforearm_rx, 1.0, 0.0, 0.0 );
                                        glRotatef(lforearm_ry, 0.0, 1.0, 0.0 );
                                        glRotatef(lforearm_rz, 0.0, 0.0, 1.0 );
                                        forearm(1);
                                        glPushMatrix();
                                            glTranslatef( 0.0, -0.7 , 0.0 );
                                            glRotatef(lhand_rx, 1.0, 0.0, 0.0 );
                                            glRotatef(lhand_ry, 0.0, 1.0, 0.0 );
                                            glRotatef(lhand_rz, 0.0, 0.0, 1.0 );
                                            hand(1);
    
                                            Mat4f particleXform2 = matCamInverse * glGetMatrix(GL_MODELVIEW_MATRIX);
                                            ParticleSystem::particleOrigin_pony = particleXform2 * Vec4f(0,0,0,1);
    
                                        glPopMatrix();
                                    glPopMatrix();
                                glPopMatrix();
                        glPopMatrix();
    
    
                        // right arm
                        glPushMatrix();
                            glTranslatef( -0.6, 0.2, 0.0 );
                            scapula(1, 'r');
                            Mat4f particleXform4 = matCamInverse * glGetMatrix(GL_MODELVIEW_MATRIX);
                            ParticleSystem::cloth_end = particleXform4 * Vec4f(0,0,0,1);
    
                            glPushMatrix();
                                glRotatef(rscapula_rx, 1.0, 0.0, 0.0 );
                                glRotatef(rscapula_ry, 0.0, 1.0, 0.0 );
                                glRotatef(rscapula_rz, 0.0, 0.0, 1.0 );
                                bicep(rbicep_len);
                                glTranslatef( 0.0, -rbicep_len, 0.0 );
                                glPushMatrix();
                                    glTranslatef( 0.0, -0.8, 0.0 );
                                    glTranslatef( 0.0, -rbicep_len, 0.0 );
                                    glRotatef(rforearm_rx, 1.0, 0.0, 0.0 );
                                    glRotatef(rforearm_ry, 0.0, 1.0, 0.0 );
                                    glRotatef(rforearm_rz, 0.0, 0.0, 1.0 );
                                    forearm(1);
                                    glPushMatrix();
                                        glTranslatef( 0.0, -0.7, 0.0 );
    
                                        glRotatef(rhand_rx, 1.0, 0.0, 0.0 );
                                        glRotatef(rhand_ry, 0.0, 1.0, 0.0 );
                                        glRotatef(rhand_rz, 0.0, 0.0, 1.0 );
    
                                        hand(1);
    
                                        Mat4f particleXform = matCamInverse * glGetMatrix(GL_MODELVIEW_MATRIX);
                                        ParticleSystem::particleOrigin = particleXform * Vec4f(0,0,0,1);
    
                                    glPopMatrix();
                                glPopMatrix();
                            glPopMatrix();
                        glPopMatrix();
    
                    glPopMatrix();
    
            glPopMatrix();
    
    
    
    glPopMatrix();
    
    

    
    
  //  std::cout<<"ori:" << ParticleSystem::particleOrigin[0] << "," << ParticleSystem::particleOrigin[1] << "," << ParticleSystem::particleOrigin[2] << std::endl;

	//*** DON'T FORGET TO PUT THIS IN YOUR OWN CODE **/
	endDraw();
}

void ground(float h) 
{
	glDisable(GL_LIGHTING);
	glColor3f(0.65,0.45,0.2);
	glPushMatrix();
	glScalef(30,0,30);
	y_box(h);
	glPopMatrix();
	glEnable(GL_LIGHTING);
}



void pelvis(float pelvis_r) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glColor3f(0,0,1);
    
    glPushMatrix();
        glRotatef(90,0,1,0);
        glScalef(trunk_fat*1.33 * 2,pelvis_len/2 * 3,trunk_width/2*1.5 * 3);
        drawSphere(1);
    glPopMatrix();
    
}


void thigh(float h) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
     glPushMatrix();
        glScalef(leg_fat*1.75 * 3, thigh_len/2 * 3 + h / 2, leg_fat*1.75 * 3);
        glTranslatef(0, -1, 0);
        drawSphere(1);
        y_box(h);
    glPopMatrix();
    
}

void shin(float h) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
        glScalef(leg_fat * 3, shin_len/2 * 3.5, leg_fat * 3);
        glTranslatef(0, -1, 0);
        drawSphere(1);
    glPopMatrix();
    
}

void foot(float h) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
        glRotatef(90,0,1,0);
        glTranslatef(-heel_len - 0.05, -ankle_height, -0.125);
        glScalef(foot_len/2 * 3,.02 * 3, foot_len/4 * 3);
        drawBox(2,2,2);
    glPopMatrix();
}



void chest(float chest_r) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glColor3f(0,0,1);
    
    glPushMatrix();
    glRotatef(90,0,1,0);
    glScalef(trunk_fat*1.33 * 3,pelvis_len/2 * 3,trunk_width/2*1.5 * 3);
    drawSphere(1);
    glPopMatrix();
    
}


void clavicle(float h) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
    //glRotatef(90,1,0,0);
    glTranslatef(0.0, pelvis_len/2, 0.0);
    glScalef(trunk_fat * 4.5, abdomen_len/2 * 2, trunk_width/2 * 3);
    glTranslatef(0, 1, 0);
    drawSphere(1);
    glPopMatrix();
}

void abdomen(float h) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
        //glRotatef(90,1,0,0);
        glTranslatef(0.0, pelvis_len/2, 0.0);
        glScalef(trunk_fat * 3, abdomen_len/2 * 2, trunk_width/2 * 3);
        glTranslatef(0, 1, 0);
        drawSphere(1);
    glPopMatrix();
}


void spine(float h) {
    
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
    // glRotatef(90,0,1,0);
        glScalef(trunk_fat*.66 * 3, spine_len/2 * 3, trunk_fat*.66 * 3);
        glTranslatef(-0.3, 0, -0.8);
        drawBox(0.8,2.8,0.8);
    glPopMatrix();
}


void head(float h) {
    
  //  setDiffuseColor (255.0 / 256,160.0 / 256,122.0 / 256);
  //  setAmbientColor (184.0 / 256,134.0 / 256,11.0 / 256);
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
    // glRotatef(90,0,1,0);
        glScalef(head_width/2 * 3, head_len/2 * 3, head_width/2 * 3);
        glTranslatef(0., 1.+neck_len/head_len*.5, 0.);
        drawSphere(1);
    glPopMatrix();
}



void scapula(float h, char d) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
    
    if (d == 'l') {
        glRotatef(45,0,0,1);
    } else {
        glRotatef(-45,0,0,1);
    }
    
        glScalef(trunk_fat * 2, scapula_len/2 * 2, trunk_fat * 2);
        glTranslatef(0, 1, 0);
        drawSphere(1);
    glPopMatrix();
    
}



void bicep(float h) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
    // glRotatef(90,0,1,0);
    
        glScalef(arm_fat*1.75 * 3, bicep_len/2 * 3 + h, arm_fat*1.75 * 3);
        glTranslatef(0, -1, 0);
        drawSphere(1);
    glPopMatrix();
    
}


void forearm(float h) {
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
    // glRotatef(90,0,1,0);
    
        glScalef(arm_fat * 3, forearm_len/2 * 3, arm_fat * 3);
        glTranslatef(0, -1, 0);
        drawSphere(1);
    glPopMatrix();
    
}


void hand(float h) {
    
    
    setDiffuseColor(176.0/256,196.0/256,222.0/256);
    setAmbientColor(0,0,0);
    
    glPushMatrix();
   // glRotatef(90,0,1,0);
    
        glScalef(arm_fat * 2, hand_len/2 * 2, arm_fat/4 * 2);
        glTranslatef(0, -1, 0);
        drawSphere(1);
    glPopMatrix();
    
}


void y_box(float h) {
    
    glBegin( GL_QUADS );
    
    glNormal3d( 1.0 ,0.0, 0.0);			// +x side
    glVertex3d( 0.25,0.0, 0.25);
    glVertex3d( 0.25,0.0,-0.25);
    glVertex3d( 0.25,  h,-0.25);
    glVertex3d( 0.25,  h, 0.25);
    
    glNormal3d( 0.0 ,0.0, -1.0);		// -z side
    glVertex3d( 0.25,0.0,-0.25);
    glVertex3d(-0.25,0.0,-0.25);
    glVertex3d(-0.25,  h,-0.25);
    glVertex3d( 0.25,  h,-0.25);
    
    glNormal3d(-1.0, 0.0, 0.0);			// -x side
    glVertex3d(-0.25,0.0,-0.25);
    glVertex3d(-0.25,0.0, 0.25);
    glVertex3d(-0.25,  h, 0.25);
    glVertex3d(-0.25,  h,-0.25);
    
    glNormal3d( 0.0, 0.0, 1.0);			// +z side
    glVertex3d(-0.25,0.0, 0.25);
    glVertex3d( 0.25,0.0, 0.25);
    glVertex3d( 0.25,  h, 0.25);
    glVertex3d(-0.25,  h, 0.25);
    
    glNormal3d( 0.0, 1.0, 0.0);			// top (+y)
    glVertex3d( 0.25,  h, 0.25);
    glVertex3d( 0.25,  h,-0.25);
    glVertex3d(-0.25,  h,-0.25);
    glVertex3d(-0.25,  h, 0.25);
    
    glNormal3d( 0.0,-1.0, 0.0);			// bottom (-y)
    glVertex3d( 0.25,0.0, 0.25);
    glVertex3d(-0.25,0.0, 0.25);
    glVertex3d(-0.25,0.0,-0.25);
    glVertex3d( 0.25,0.0,-0.25);
    
    glEnd();
}


int main()
{
    
    ModelerControl controls[NUMCONTROLS ];

    
	controls[PELVIS_R ] = ModelerControl("plevis rotation", -360.0, 360.0, 0.1f, 0.0f );
    controls[LTHIGH_RX ] = ModelerControl("lthigh x rotation", -360.0, 360.0, 0.1f, 0.0f );
    controls[LTHIGH_RY ] = ModelerControl("lthigh y rotation", -360.0, 360.0, 0.1f, 0.0f );
    controls[LTHIGH_RZ ] = ModelerControl("lthigh z rotation", -360.0, 360.0, 0.1f, 0.0f );
    
    controls[RTHIGH_RX ] = ModelerControl("rthigh x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RTHIGH_RY ] = ModelerControl("rthigh y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RTHIGH_RZ ] = ModelerControl("rthigh z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    
    controls[LSHIN_RX ] = ModelerControl("lshin z rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RSHIN_RX ] = ModelerControl("rshin z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    controls[LFOOT_RX ] = ModelerControl("lfoot x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LFOOT_RY ] = ModelerControl("lfoot y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LFOOT_RZ ] = ModelerControl("lfoot z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    
    
    controls[RFOOT_RX ] = ModelerControl("rfoot x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RFOOT_RY ] = ModelerControl("rfoot y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RFOOT_RZ ] = ModelerControl("rfoot z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    controls[SPINE_RX ] = ModelerControl("spine x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[SPINE_RY ] = ModelerControl("spine y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[SPINE_RZ ] = ModelerControl("spine z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    controls[HEAD_RX ] = ModelerControl("head x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[HEAD_RY ] = ModelerControl("head y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[HEAD_RZ ] = ModelerControl("head z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    
    controls[LSCAPULA_RX ] = ModelerControl("lscapula x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LSCAPULA_RY ] = ModelerControl("lscapula y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LSCAPULA_RZ ] = ModelerControl("lscapula z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    controls[RSCAPULA_RX ] = ModelerControl("rscapula x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RSCAPULA_RY ] = ModelerControl("rscapula y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RSCAPULA_RZ ] = ModelerControl("rscapula z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    
    controls[LFOREARM_RX ] = ModelerControl("lforearm x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LFOREARM_RY ] = ModelerControl("lforearm y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LFOREARM_RZ ] = ModelerControl("lforearm z rotation", -360, 360, 0.0, 0.1 );
    
    controls[RFOREARM_RX ] = ModelerControl("rforearm x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RFOREARM_RY ] = ModelerControl("rforearm y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RFOREARM_RZ ] = ModelerControl("rforearm z rotation", -360, 360, 0.0, 1.0 );
    
    
    controls[LHAND_RX ] = ModelerControl("lhand x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LHAND_RY ] = ModelerControl("lhand  y rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[LHAND_RZ ] = ModelerControl("lhand  z rotation", -360.0, 360.0, 0.1, 0.0 );
    
    controls[RHAND_RX ] = ModelerControl("rhand x rotation", -360.0, 360.0, 0.1, 0.0 );
    controls[RHAND_RY ] = ModelerControl("rhand  y rotation", -180.0, 360.0, 0.1, 0.0 );
    controls[RHAND_RZ ] = ModelerControl("rhand  z rotation", -180.0, 360.0, 0.1, 0.0 );
    
    controls[THIGH_EXLEN ] = ModelerControl("thigh extending length", 0, 1, 0.1, 0.0 );
    controls[RBICEP_EXLEN ] = ModelerControl("rbicep extending length", 0, 1, 0.1, 0.0 );
    controls[LBICEP_EXLEN ] = ModelerControl("lbicep extending length", 0, 1, 0.1, 0.0 );
    
    
    controls[PARTICLE_COUNT] = ModelerControl("particle count (pc)", 0.0, 5.0, 0.1, 5.0 );
    
    controls[TWIST] = ModelerControl("human twist", -360, 360, 0.1, 0 );
    

	// You should create a ParticleSystem object ps here and then
	// call ModelerApplication::Instance()->SetParticleSystem(ps)
	// to hook it up to the animator interface.

  //  ParticleSystem *ps = new ParticleSystem();
  //  ModelerApplication::Instance()->SetParticleSystem(ps);
    
    
    ModelerApplication::Instance()->Init(&createHuman, controls, NUMCONTROLS);
    return ModelerApplication::Instance()->Run();
}
