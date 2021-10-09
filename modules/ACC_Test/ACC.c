#include "ApoClnt.h"
#include "ApoClnt.h"
#include "GuiCmd.h"
#include "DVA.h"
#include <math.h>
#include <stdbool.h>
#include "ACC.h"

static double c_i;
static bool initialized;
tACC_ECU ACC_ECU;
//todo: the radar sensor has to be named as RA00 for now.
static char* UAQNames[] = { "SC.State", "DM.Brake", "Driver.ReCon.Speed", "Vhcl.v", "Vhcl.PoI.ax_1", "VC.Gas", "VC.Brake", "Sensor.Radar.RA00.nObj"};
static double UAQValues[UAQCount];
// static double RadarObjValues[MaxObj];

static void
error_and_exit (const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    exit(EXIT_FAILURE);
}


/** Basic APO client infrastructure *******************************************/

/* Connection parameters. */

enum { CmdChannel  = 2             };		/* CarMaker command channel. */
enum { ChannelMask = 1<<CmdChannel };

static const char *AppClass = "CarMaker";
static const char *AppHost  = "10.144.160.78";
static const char *AppUser  = "*";


/* Connection handle to the application (CarMaker in this case). */
static tApoSid Srv;


/* Time to wait between calls to ApoClnt_PollAndSleep().
   The duration must be fine-tuned to the actual subscription parameters used.
   If too short you may hog the CPU, if too long you may loose quantity data. */
static int SleepDelta_ms = 20;


static int
connection_ok (void)
{
    return (ApocGetStatus(Srv, NULL) & ApoConnUp) != 0;
}


/*
 * ApoClnt_PollAndSleep()
 *
 * This function has to be provided by the client application.
 * It will be called from within the ApoClnt code whenever the
 * application needs to wait for something without blocking the
 * rest of the client.
 * 
 * You may adapt the function's code to your needs, provided that
 * the basic APO communication structure is kept as is.
 *
 * Don't assume this function being called with a fixed timing.
 */
void
ApoClnt_PollAndSleep (void)
{
    if (ApocWaitIO(SleepDelta_ms) == 1) {
	ApocPoll();	/* _Always_ poll, even if connection is down. */

	if (connection_ok()) {
	    unsigned long vecid;
	    int msglen, msgch;
	    char msgbuf[APO_ADMMAX];

	    while ((vecid = ApocGetData(Srv)) > 0) {
		if (ApoClnt_UpdateQuantVars(vecid))
		    return;

		/* Put your own quantity data handling code here. */
		/* ... */
	    }
	    if (ApocDictChanged(Srv))
		ApoClnt_Resubscribe(Srv);

	    while (ApocGetAppMsg(Srv, &msgch, msgbuf, &msglen) > 0) {
		if (GuiCmd_HandleMsgs(msgch, msgbuf, msglen))
		    continue;
		if (DVA_HandleMsgs(msgch, msgbuf, msglen))
		    continue;

		/* Put your own message handling code here. */
		/* ... */
	    }
	}
    }

    /* Put your code handling other regular tasks here. */
    /* ... */
}


static void
pause (int milliseconds)
{
    double tstart = ApoGetTime_ms();
    do {
	ApoClnt_PollAndSleep();
    } while (ApoGetTime_ms()-tstart < milliseconds);
}


static void
setup_client (void)
{
    if (ApocInit() != ApoErrOk)
	error_and_exit("Fail to initialize APO library\n");

    Srv = ApoClnt_Connect(AppClass, AppHost, AppUser, ChannelMask);
    if (Srv == NULL)
	error_and_exit("Could not connect\n");

    if (!GuiCmd_IsReady(Srv))
	error_and_exit("GUI not ready\n");
}


static void
teardown_client (void)
{
    ApocCloseServer(Srv);
}


static void
ECU_ReadAndWrite()
{
    printf("Load and start testrun 'Hockenheim'\n");

    GuiCmd_Eval(Srv, "LoadTestRun Examples/VehicleDynamics/Handling/Racetrack_Hockenheim 1");
    GuiCmd_Eval(Srv, "StartSim");
    GuiCmd_Eval(Srv, "WaitForStatus running");
    pause(2000);

    static char *names[] = { "DM.Gas", "VC.Gas", "VC.Brake", NULL };

	if (DVA_Read(Srv, names, 1000) == DVA_EOK) {
	    const double *p = DVA_GetResult();
	    printf("\t%s %g\t%s %g\t%s %g\n", names[0], p[0], names[1], p[1], names[2], p[2]);
	} else {
	    printf("DVA read error\n");
	}

    // The calculation code of the ECU here..., e.g. VC.Gas

	pause(1000);

    printf("DVA_WriteAbs(\"VC.Gas\" to 0.0 for 10s)\n");
    printf("DVA_WriteAbs(\"VC.Brake\" to 1.0 for 10s)\n");
    pause(2000);
    DVA_WriteAbs(Srv, 10000, "VC.Gas", 0.0);
    pause(2000);
    DVA_WriteAbs(Srv, 10000, "VC.Brake", 1.0);
    pause(2000);

    printf("Read again to check the updated value\n");
    if (DVA_Read(Srv, names, 1000) == DVA_EOK) {
	    const double *p = DVA_GetResult();
	    printf("\t%s %g\t%s %g\t%s %g\n", names[0], p[0], names[1], p[1], names[2], p[2]);
	} else {
	    printf("DVA read error\n");
	}

    pause(10000);
    printf("Read again to check the updated value\n");
       if (DVA_Read(Srv, names, 1000) == DVA_EOK) {
   	    const double *p = DVA_GetResult();
   	    printf("\t%s %g\t%s %g\t%s %g\n", names[0], p[0], names[1], p[1], names[2], p[2]);
   	} else {
   	    printf("DVA read error\n");
   	}
    GuiCmd_Eval(Srv, "StopSim");
    GuiCmd_Eval(Srv, "WaitForStatus idle 10000");
}

static void
ACC_ReadUAQ(char* names[], double* storeArray, int count)
{
    pause(1);
    if (DVA_Read(Srv, names, 1000) == DVA_EOK)
    {
    	const double* p = DVA_GetResult();
    	if(p != NULL)
    	{
    		for(int i=0; i<count; i++)
			{
				storeArray[i] = p[i];
			}
    	}
    }
    else
    {
        printf("DVA read error\n");
    }
    pause(1);  
}

//todo: Parameters should be passed in from Apollo conf file.
static void
AccelCtrl_Init()
{
    double vInit;

    c_i = 0.0;

    ACC_ECU.p_gain = 0.001;
    ACC_ECU.i_gain = 1.0;

    /* Active / switched on ? */
    ACC_ECU.IsActive = 1;

    /* Limit of driver brake to deactivate ACC */
    ACC_ECU.BrakeThreshold = 0.2;

    /* initial time gap / speed */
    ACC_ECU.DesrTGap = 1.8;
    vInit = UAQValues[DriverTgtSpeed] > 10.0 ? UAQValues[DriverTgtSpeed] * kmh2ms : 100 * kmh2ms;
    ACC_ECU.DesrSpd = vInit;

    /* controller parameters */
    ACC_ECU.dc_kd = 36.0;
    ACC_ECU.dc_kv = 2.0;
    ACC_ECU.sc_kv = 13.0;

    /* min/max values */
    ACC_ECU.axmin = -2.5;
    ACC_ECU.axmax = 1.0;
    ACC_ECU.dsmin = 20.0;
}

/* todo: need to combine the RCS and DistY to find the relavant target */
static bool
FindRelativeTarget(double* relvTgtDvp, double* relvTgtDsp, double* radarObjValues)
{
	if(UAQValues[RadarNObj] > 0)
	{
		int objNum = UAQValues[RadarNObj];

		// Find the min DistY object as potential relevant target
		int relTgtIndex = 0;
		double minDistY = fabs(radarObjValues[1]);
		for(int i = 0; i < objNum; i++)
		{
			if(fabs(radarObjValues[4*i + 1]) < minDistY)
			{
				minDistY = fabs(radarObjValues[4*i + 1]);
				relTgtIndex = i;
			}
		}

		if(minDistY < 1.0 ) // hard code to size of half car width
		{
			*relvTgtDsp = radarObjValues[4*relTgtIndex];
			*relvTgtDvp = radarObjValues[4*relTgtIndex +2];
			return TRUE;
		}
	}
	return FALSE;
}

static void
DesrAccelFunc_ACC(double dt, double* radarObjValues)
{
    double ax, ax_sc, delta_ds;
    double relvTgtDvp=0, relvTgtDsp=0;

    bool relvTgtDtct = FindRelativeTarget(&relvTgtDvp, &relvTgtDsp, radarObjValues);

    /* Driver Brake Limit */
    if (UAQValues[DMBrake] > ACC_ECU.BrakeThreshold)
        ACC_ECU.IsActive = 0;

    /* Time until collision    */
    if (relvTgtDvp < 0) {
        ACC_ECU.Time2Collision = relvTgtDsp / -relvTgtDvp;
    }
    else {
        ACC_ECU.Time2Collision = 0;
    }

    if (!ACC_ECU.IsActive) {
        /* ACC off -> set desired speed to current car speed */
        ACC_ECU.DesrSpd = UAQValues[VhclVelocity];
        ACC_ECU.DesrAx = NOTSET;
        return;
    }

    /* ACC active */
    if (relvTgtDtct) {
        /* if target detected set desired distance,
           DesrDistance[m] = Target.v[m/s] / Desired Time Gap(Init= 1.8[s])
           or if target stand still DSMIN: 20[m] distance */
        ACC_ECU.DesrDist = M_MAX(((UAQValues[VhclVelocity] +
            relvTgtDvp) * ACC_ECU.DesrTGap), ACC_ECU.dsmin);

        /* Distance Control Algorithm: result = desired ax */
        delta_ds = relvTgtDsp - ACC_ECU.DesrDist; /* d_ist-d_soll */
        ax = (delta_ds) / (ACC_ECU.dc_kd) + relvTgtDvp / ACC_ECU.dc_kv;
        /* ax_sc = desired ax from Speed Control */
        ax_sc = (ACC_ECU.DesrSpd - UAQValues[VhclVelocity]) / ACC_ECU.sc_kv;

        /* Limitation */
        if (ax > ax_sc) ax = ax_sc;
        if (ax > ACC_ECU.axmax) ax = ACC_ECU.axmax;
        if (ax < ACC_ECU.axmin) ax = ACC_ECU.axmin;
    }
    else {
        /* Speed Control Algorithm: result = desired ax */
        /*      s->relvTarget.ds = -1; */
      //  ax = (ACC_ECU.DesrSpd - UAQValues[VhclVelocity]) / ACC_ECU.sc_kv;
        /* Limitation */
      //  if (ax > ACC_ECU.axmax) ax = ACC_ECU.axmax;
      //  if (ax < -0.35) ax = -0.35;
    	ACC_ECU.DesrSpd = UAQValues[VhclVelocity];
    	ACC_ECU.DesrAx = NOTSET;
    	return;
    }

    ACC_ECU.DesrAx = ax;
    return;
}

static void
ECU_ACC(double* radarObjValues)
{
    double dt = 1;
    ACC_ReadUAQ(UAQNames, UAQValues, UAQCount);
    if (UAQValues[SCState] != SCState_Simulate)
    {
        return;
    }
    if(!initialized)
    {
        AccelCtrl_Init();
        initialized = TRUE;
    }

    double c, delta_ax, c_p;

    /* Calculate target longitudinal acceleration ax */
    DesrAccelFunc_ACC(dt, radarObjValues);

    /* Controller for converting desired ax to gas or brake */
    if (ACC_ECU.DesrAx == NOTSET) {
        /* no control required */
        c_i = UAQValues[VCGas];
        return;
    }

    delta_ax = ACC_ECU.DesrAx - UAQValues[VhclPoIAx_1];
    c_p = ACC_ECU.p_gain * delta_ax;
    c_i += ACC_ECU.i_gain * delta_ax * dt; 
    c = c_p + c_i;	/* PI-Controller */

    /* Limitation */
    if (c > 1) c = 1;
    if (c < -1) c = -1;
    c_i = c - c_p;

    /* Gas or Brake */
    if (c >= 0) {
        DVA_WriteAbs(Srv, 10000, "VC.Gas", c);
        DVA_WriteAbs(Srv, 10000, "VC.Brake", 0.0);
        pause(1);
    }
    else {
        DVA_WriteAbs(Srv, 10000, "VC.Gas", 0.0);
        DVA_WriteAbs(Srv, 10000, "VC.Brake", -c);
        pause(1);
    }       
}




/******************************************************************************/
//ACC ECU Controller functions that are called from Apollo.

//todo: the setup_client and teardown_client are only needed temporarly as we have not implmemented the powertrain emulator yet. 
void
InitACC ()
{
    setup_client();
}

void
RunACC (double * radarObjs)
{
    ECU_ACC(radarObjs);
}

void
ShutDownACC ()
{
    teardown_client();
}



