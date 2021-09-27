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
static char* UAQNames[] = { "SC.State", "DM.Brake", "Driver.ReCon.Speed", "Vhcl.v", "Vhcl.PoI.ax_1", "VC.Gas", "VC.Brake", "Sensor.Radar.RA00.nObj", "Sensor.Object.RadarL.relvTgt.NearPnt.ds_p", "Sensor.Object.RadarL.relvTgt.dtct" };
static char* radarUAQNames[100];
static char buffer[4*15][100];
static double UAQValues[UAQCount];
static double RadarObjValues[MaxObj];

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


static void
AccelCtrl_Init()
{
    double vInit;

    c_i = 0.0;

    //todo: should be configurable parameters
    ACC_ECU.p_gain = 0.001;
    ACC_ECU.i_gain = 1.0;


    /* Active / switched on ? */
    ACC_ECU.IsActive = 1;

    /* Limit of driver brake to deactivate ACC */
    ACC_ECU.BrakeThreshold = 0.2;

    /* initial time gap / speed */
    ACC_ECU.DesrTGap = 1.8;
    //todo: DM.v.Trgt?
    vInit = UAQValues[DMVelocityTrgt] > 10.0 ? UAQValues[DMVelocityTrgt] * kmh2ms : 100 * kmh2ms;
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

//todo: the radar sensor name must be RA00 right now.
static void
BuildRadarObjectUAQName()
{
	int objNum = UAQValues[RadarNObj];

	for(int index = 0; index < objNum; index++)
	{

		sprintf(buffer[4*index], "%s""%d""%s", "Sensor.Radar.RA00.Obj", index, ".DistX");
		radarUAQNames[4*index] = buffer[4*index]; //longitudinal distance

		sprintf(buffer[4*index +1], "%s""%d""%s", "Sensor.Radar.RA00.Obj", index, ".DistY");
		radarUAQNames[4*index + 1] = buffer[4*index + 1]; // lateral distance

		sprintf(buffer[4*index + 2], "%s""%d""%s", "Sensor.Radar.RA00.Obj", index, ".VrelX");
		radarUAQNames[4*index + 2] = buffer[4*index + 2]; // relative longitudinal velocity

		sprintf(buffer[4*index + 3], "%s""%d""%s", "Sensor.Radar.RA00.Obj", index, ".RCS");
		radarUAQNames[4*index + 3] = buffer[4*index + 3];
	}
}

/* relvTgtDvp: Relative radial speed of the relevant target
 * relvTgtDsp: Distance to the relevant target */
static bool
FindRelativeTarget(double* relvTgtDvp, double* relvTgtDsp)
{
	if(UAQValues[RadarNObj] > 0)
	{
		int objNum = UAQValues[RadarNObj];

		BuildRadarObjectUAQName();

		ACC_ReadUAQ(radarUAQNames, RadarObjValues, objNum*4);

		// Find the min DistY object as potential relevant target
		int relTgtIndex = 0;
		double minDistY = fabs(RadarObjValues[1]);
		//int maxRCSIndex = 0;
		//double maxRCS = RadarObjValues[3];
		for(int i = 0; i < objNum; i++)
		{
			if(fabs(RadarObjValues[4*i + 1]) < minDistY)
			{
				minDistY = fabs(RadarObjValues[4*i + 1]);
				relTgtIndex = i;
			}
		/*	if(RadarObjValues[4*i + 3] > maxRCS)
			{
				maxRCS = RadarObjValues[4*i + 1];
				maxRCSIndex = i;
			} */
		}

		if(minDistY < 1.0 ) // hard code to size of half car width
		{
			*relvTgtDsp = RadarObjValues[4*relTgtIndex];
			*relvTgtDvp = RadarObjValues[4*relTgtIndex +2];
			return TRUE;
		}

		/*if(RadarObjValues[4*maxRCSIndex + 1] < 1.0)
		{
			*relvTgtDsp = RadarObjValues[4*maxRCSIndex];
			*relvTgtDvp = RadarObjValues[4*maxRCSIndex +2];
			return TRUE;
		}*/


	}
	return FALSE;
}

static void
DesrAccelFunc_ACC(double dt)
{
    double ax, ax_sc, delta_ds;
    double relvTgtDvp=0, relvTgtDsp=0;

    bool relvTgtDtct = FindRelativeTarget(&relvTgtDvp, &relvTgtDsp);

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
           DesrDistance[m] = Target.v[m/s] * 3.6 / Desired Time Gap(Init= 1.8[s])
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
ECU_ACC()
{
  //  ACC_ReadUAQ(UAQNames, UAQValues, UAQCount);
  //  AccelCtrl_Init();
    while (1)
    {
        double dt = 1;
        ACC_ReadUAQ(UAQNames, UAQValues, UAQCount);
        if (UAQValues[SCState] != SCState_Simulate)
        {
            continue;
        }
        if(!initialized)
        {
        	AccelCtrl_Init();
        	initialized = TRUE;
        }

        double c, delta_ax, c_p;

        /* Calculate target longitudinal acceleration ax */
        DesrAccelFunc_ACC(dt);

         /* Controller for converting desired ax to gas or brake */
         if (ACC_ECU.DesrAx == NOTSET) {
             /* no control required */
             c_i = UAQValues[VCGas];
             continue;
         }

         delta_ax = ACC_ECU.DesrAx - UAQValues[VhclPoIAx_1];
         c_p = ACC_ECU.p_gain * delta_ax;
         c_i += ACC_ECU.i_gain * delta_ax * dt; // todo: what is dt?
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
}




/******************************************************************************/

void
RunACC ()
{

    setup_client();


  /*  if(strcasecmp(example, "ECUReadAndWrite") == 0)
    {
        ECU_ReadAndWrite();
    } */
   // else if (strcasecmp(example, "ECU_ACC") == 0) {
        ECU_ACC();
   // }
  /*  else
    {
	    printf("Invalid example specified.\n");
	    printf("Run with '-help' to see a list of available examples.\n");
    }*/

    teardown_client();

    return EXIT_SUCCESS;
}

