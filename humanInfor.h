#ifndef _HUMANINFOR_H
#define _HUMANINFOR_H
#include <cmath>

#define total_mass  75.
#define head_mass  .0792 // total_mass,
#define pelvis_mass  .16  // total_mass,		  // pelvis+abdomen  26.39%
#define abdomen_mass  .26 // total_mass - pelvis_mass,
#define spine_mass  .10  // total_mass,
#define scapula_mass  .05  // total_mass,
#define bicep_mass  .0275  // total_mass,
#define forearm_mass  .016  // total_mass,
#define hand_mass  .0612  // total_mass,
#define thigh_mass  .105  // total_mass,
#define shin_mass  .046  // total_mass,
#define foot_mass  .014  // total_mass,

#define leg_fat  .04
#define foot_len  .18
#define heel_len  .02
#define ankle_height  .105
#define shin_len  .405
#define thigh_len  .425

#define trunk_fat  .08
#define trunk_width  .22
#define pelvis_len  .14
#define abdomen_len  .30
#define hip_shoulder_len  .52
#define full_spine_len  hip_shoulder_len - pelvis_len / 2 - abdomen_len
#define spine_len  full_spine_len + .07
#define neck_len  .04
#define head_len  .19
#define head_width  .14

#define arm_fat  .03
#define shoulder_width  .41
#define scapula_len   sqrt(pow(full_spine_len,2) + pow(shoulder_width,2) * 1.0 / 4)
#define shrug_angle  -atan2(shoulder_width/2, full_spine_len)
#define bicep_len  .31
#define forearm_len  .25
#define hand_len  .18

#define RGB_SKIN       (ulong) 0xffd28e80


// Frames per seconds
// this is the redraw rate
static const int FPS = 50;

// Colors
#define COLOR_RED		1.0f, 0.0f, 0.0f
#define COLOR_GREEN		0.0f, 1.0f, 0.0f
#define COLOR_BLUE		0.0f, 0.0f, 1.0f
#define COLOR_GRAY		0.8f, 0.8f, 0.8f

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))
#define SETVAL(x, d) (ModelerApplication::Instance()->SetControlValue(x, d))
#define MAX(x) (ModelerApplication::Instance()->GetMaxValue(x))
#define MIN(x) (ModelerApplication::Instance()->GetMinValue(x))

#endif
