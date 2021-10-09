
#ifndef __ACC_H__
#define __ACC_H__

#include "apo.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
# include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#ifdef _MSC_VER
# define strcasecmp(a, b) _stricmp(a,b)
#endif

#define NOTSET	-99999
#define kmh2ms        ( 0.277777777777777778)
#define M_MAX(a,b)      ((a) > (b) ? (a) :  (b))
#define MaxObj 100

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct {
    char   IsActive;		/* acc on/off */
    double DesrSpd;		/* desired speed */
    double DesrDist;		/* desired distance  */
    double DesrTGap;		/* desired time gap */
    double DesrAx;		/* target longitudinal acceleration */
    double dc_kd;		/* distance control: gain for distance */
    double dc_kv;		/* distance control: gain for velocity */
    double sc_kv;		/* speed control:    gain for velocity */
    double axmin, axmax;	/* min./max. acceleration */
    double dsmin;		/* min. distance */
    double Time2Collision;	/* time until collision = rel_dist / rel_vel */
    double BrakeThreshold;	/* acc deactivates if the driver brake threshold
                     is passed over */
    double p_gain;		/* pedal controller */
    double i_gain;		/* pedal controller */
} tACC_ECU;

typedef enum {
    SCState_Unknown = 0,
    SCState_Init = 1,

    SCState_Idle = 2,	/* idle */

    SCState_Start = 3,	/* start test run parametrization     */
    SCState_StartWait = 4,	/* wait in main thread while start    */
    SCState_StartWaitAnim = 5,	/* wait until animation is ready      */
    SCState_StartSim = 6,	/* prepare test run start conditions  */
    SCState_StartLastCycle = 7,	/* last cycle before simulate	      */

    SCState_Simulate = 8,	/* simulate			      */

    SCState_End = 9,	/* start test run end		      */
    SCState_EndIdleGet = 10,	/* get everything idle		      */
    SCState_EndIdleSet = 11,	/* set everything to idle	      */
    SCState_EndClean = 12,	/* free everything not needed	      */
    SCState_EndWait = 13,	/* wait in main thread while end      */
    SCState_EndLastCycle = 14,	/* last cycle before idle	      */

    SCState_ShutDown = 15,	/* end program			      */
    SCState_Pause = 16,	/* simulation pause for vehicle change */
    SCState_NumStates = 17
} tSCState;

/*Used as array index, must have same order as UAQNames*/
typedef enum {
    SCState = 0, //SC.State: Simulation state (integer)
    DMBrake,
    DriverTgtSpeed, //Driver.ReCon.Speed: Target speed for the selected driving mode
    VhclVelocity,
    VhclPoIAx_1, // Vhcl.POI.ax_1: Acceleration vector for PoI ("Point of Interest") in vehicle frame (Fr1), m/s2
    VCGas,
    VCBrake,
    RadarNObj, //Sensor.Radar.RA00.nObj: number of detected objects (integer)
	UAQCount // Item count in the enum
}UAQs;

void InitACC();
void RunACC(double* radarObjs);
void ShutDownACC();

#ifdef __cplusplus
}
#endif

#endif /* !defined(__ACC_H__) */