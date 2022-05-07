/*
 * XPlaneAsVisualizerPlugin.c
 * 
 * Use UDP packet to continuosly position the airplane with
 * the dynamic engine of X-Plane turned off.
 *
 */

#define XPLM200


#include "platformDef.h"
#include <windows.h>
#include <GL/gl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMCamera.h"
#include "XPLMScenery.h"

#include "XPlane_GIMcom.h"

/*
 * Global Variables.  
 */

double pi=3.141592653589793;

XPLMWindowID	gWindow = NULL;
XPLMWindowID	gWindow_2 = NULL;
int				gClicked = 0, isCamView = 0;
long			count=0;
float			count2=0;
char 			freq_str[20];
#if 0
double			recvLat=0,recvLon=0,recvAlt=0,posX=0,posY=0,posZ=0;
float			recvPitch=0,recvRoll=0,recvYaw=0,recvGear=0,recvFlap=0;
double			recvObjLat=0, recvObjLon=0, recvObjAlt=0, posObjX=0, posObjY=0, posObjZ=0;
float			recvObjYaw=0, recvObjPitch=0, recvObjRoll=0;
float			camAzEl[2] = { 0,0 }, Xpc_abc[3];
double			recvCamLat = 0, recvCamLon = 0, recvCamAlt = 0, posCamX = 0, posCamY = 0, posCamZ = 0;
float			recvCamYaw = 0, recvCamPitch = 0, recvCamRoll = 0, recvCamAZ = 0, recvCamEL = 0;
#endif
double			ned0Lla[3], trgtLla[3];
int				methodFlag=0;

//=======Definitions For WinSocket======
char			UdpRecBuffer[1024];
int				dataReady1=0,dataReady2=0;
//=============================================


//==========+====RS232=========================
XPlane_GIMcom  gimbal_Comm;
//=============================================



// Data refs, Send to Controller. //////////////////
XPLMDataRef		gPlaneSpeed; //(kias)
XPLMDataRef		gPlanePrad;  //(rad/s)
XPLMDataRef		gPlaneQrad;  //(rad/s)
XPLMDataRef		gPlaneRrad;  //(rad/s)
XPLMDataRef		gPlaneTheta;//pitch   (deg)
XPLMDataRef		gPlanePhi;  //roll    (deg)
XPLMDataRef		gPlanePsi;  //heading (deg)
XPLMDataRef		gPlaneAlpha; //(deg)
XPLMDataRef		gPlaneBeta;  //(deg)
XPLMDataRef		gPlaneVpath; //(deg)
XPLMDataRef		gPlaneHpath; //(deg)
XPLMDataRef		gPlaneAlt; //(m)
XPLMDataRef		gPlaneVx; //(m/s)
XPLMDataRef		gPlaneVy; //(m/s)
XPLMDataRef		gPlaneVz; //(m/s)
XPLMDataRef		gPlaneLat;
XPLMDataRef		gPlaneLon;
XPLMDataRef		gPlaneIndV;

// Data refs, From Controller commands. //////////////////
XPLMDataRef		gPlaneAil;	//(ratio)
XPLMDataRef		gPlaneEle;	//(ratio)
XPLMDataRef		gPlaneRud;	//(ratio)
XPLMDataRef		gPlaneFlap;	//(ratio)
XPLMDataRef		gPlaneGear;	//(ratio)(boolean/int)
XPLMDataRef		gPlaneBrake; //(ratio)
XPLMDataRef		gPlaneThr; //(ratio)
XPLMDataRef		gPlaneRpmSet; //(rad/s)

// Data refs, Others.
XPLMDataRef gPlaneElv1;
XPLMDataRef gPlaneElv2;
XPLMDataRef gPlaneAlv1;
XPLMDataRef gPlaneRdv1;
XPLMDataRef gPlaneCElv1;
XPLMDataRef gPlaneMW1El1;
XPLMDataRef gPlaneMW2El2;
XPLMDataRef gPlaneOvrJoy;
XPLMDataRef gPlaneYokeP;
XPLMDataRef gPlaneYokeR;
XPLMDataRef gPlaneYokeH;
XPLMDataRef gPlaneAcfAilUp;
XPLMDataRef gPlaneToyPCntr;

XPLMDataRef gPlanePropdeg;

XPLMDataRef gPlaneOvrrdSim;
XPLMDataRef gPlaneOvrrdGear;
XPLMDataRef gPlaneOvrrdJoy;

XPLMDataRef gPlaneTireWOS;

XPLMDataRef		gPlaneX = NULL;
XPLMDataRef		gPlaneY = NULL;
XPLMDataRef		gPlaneZ = NULL;

XPLMDataTypeID tp1;

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*float	MyFlightLoopCallback(
                                   float                inElapsedSinceLastCall,    
                                   float                inElapsedTimeSinceLastFlightLoop,    
                                   int                  inCounter,    
                                   void *               inRefcon);    */

void MyDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon);    
           
float  MyProcessLoopCallback(
                                   float                inElapsedSinceLastCall,    
                                   float                inElapsedTimeSinceLastFlightLoop,    
                                   int                  inCounter,    
                                   void *               inRefcon);   


void MyHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus);    

int MyHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon);  

int	ObjDrawCallback(	    XPLMDrawingPhase     inPhase,    
                            int                  inIsBefore,    
                            void *               inRefcon);

int MyOrbitPlaneFunc(
                                   XPLMCameraPosition_t * outCameraPosition,   
                                   int                  inIsLosingControl,    
                                   void *               inRefcon);

int	AircraftDrawCallback(	XPLMDrawingPhase     inPhase,    
                            int                  inIsBefore,    
                            void *               inRefcon);


PLUGIN_API int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{
	unsigned long tru=1;


	strcpy(outName, "XPlaneAsSimEnvPlugin");
	strcpy(outSig, "Warren Kuo.XPlaneAsSimEnvPlugin");
	strcpy(outDesc, "Continuously sending out Euler angles of the airplane by COM PORT.");
	

	//===================================RS232======================================================
	gimbal_Comm.Ini();
	//==============================================================================================

	// Data refs //////////////////

	gPlaneTheta		= XPLMFindDataRef("sim/flightmodel/position/theta");
	gPlanePhi		= XPLMFindDataRef("sim/flightmodel/position/phi");
	gPlanePsi		= XPLMFindDataRef("sim/flightmodel/position/psi");

	gPlaneAlt		= XPLMFindDataRef("sim/flightmodel/position/elevation");
	gPlaneLat		= XPLMFindDataRef("sim/flightmodel/position/latitude");
	gPlaneLon		= XPLMFindDataRef("sim/flightmodel/position/longitude");
	
	// Data refs //////////////////
	gPlaneAil		= XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio");
	gPlaneEle		= XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio");
	gPlaneRud		= XPLMFindDataRef("sim/cockpit2/controls/yoke_heading_ratio");
	gPlaneFlap		= XPLMFindDataRef("sim/cockpit2/controls/flap_ratio");
//	gPlaneGear		= XPLMFindDataRef("sim/cockpit2/controls/gear_handle_down");
//	gPlaneGear		= XPLMFindDataRef("sim/multiplayer/position/plane1_gear_deploy");
	gPlaneGear		= XPLMFindDataRef("sim/cockpit/switches/gear_handle_status");

	gPlaneOvrrdSim	= XPLMFindDataRef("sim/operation/override/override_planepath");
	gPlaneOvrrdGear	= XPLMFindDataRef("sim/operation/override/override_gearbrake");
	gPlaneOvrrdJoy	= XPLMFindDataRef("sim/operation/override/override_joystick");

	gPlaneIndV	= XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");

	gPlaneX		= XPLMFindDataRef("sim/flightmodel/position/local_x");
	gPlaneY		= XPLMFindDataRef("sim/flightmodel/position/local_y");
	gPlaneZ		= XPLMFindDataRef("sim/flightmodel/position/local_z");

	gPlaneTireWOS = XPLMFindDataRef("sim/flightmodel/parts/tire_vrt_frc_veh");

	/* Register our callback for once a second.  Positive intervals
	 * are in seconds, negative are the negative of sim frames.  Zero
	 * registers but does not schedule a callback for time. */
	gWindow = XPLMCreateWindow(
					760, 265, 1010, 15,			/* Area of the window. */ //left,top,right,bottom
					1,							/* Start visible. */
					MyDrawWindowCallback,		/* Callbacks */
					MyHandleKeyCallback,
					MyHandleMouseClickCallback,
					NULL);						/* Refcon - not used. */

	XPLMRegisterFlightLoopCallback(		
			MyProcessLoopCallback,	/* Callback */
			1,					/* Interval */
			NULL);					/* refcon not used. */

	/* Next register teh drawing callback.  We want to be drawn 
	 * after X-Plane draws its 3-d objects. */
	XPLMRegisterDrawCallback(
					ObjDrawCallback,	
					xplm_Phase_Objects, 	/* Draw when sim is doing objects */
					0,						/* After objects */
					NULL);					/* No refcon needed */

	//========================================//

	return 1;
}

/////////////////////////////////////////////////////
PLUGIN_API void	XPluginStop(void)
{

	/* Unregister the callback */
	XPLMUnregisterFlightLoopCallback(MyProcessLoopCallback, NULL);
	
	/* Unregitser the callback on quit. */
	XPLMUnregisterDrawCallback(
					ObjDrawCallback,
					xplm_Phase_LastCockpit, 
					0,
					NULL);	

	XPLMDestroyWindow(gWindow);

	gimbal_Comm.Close(); 

}
//////////////////////////////////////////////////////
PLUGIN_API void XPluginDisable(void)
{
	int i=0;
	XPLMSetDatavi(gPlaneOvrrdSim,&i,0,1);
	XPLMSetDatai(gPlaneOvrrdJoy,0);
}
/////////////////////////////////////////////////////
PLUGIN_API int XPluginEnable(void)
{
	//int i=1;
	//InitialState();
	return 1;
}
//////////////////////////////////////////////////////////
PLUGIN_API void XPluginReceiveMessage(
					XPLMPluginID	inFromWho,
					long			inMessage,
					void *			inParam)
{
}

/*
 * MyDrawingWindowCallback
 * 
 * This callback does the work of drawing our window once per sim cycle each time
 * it is needed.  It dynamically changes the text depending on the saved mouse
 * status.  Note that we don't have to tell X-Plane to redraw us when our text
 * changes; we are redrawn by the sim continuously.
 * 
 */
void MyDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon)
{
	int		left, top, right, bottom;
	char	str[80],str2[80];
	float	color[] = { 1.0, 1.0, 1.0 }; 	/* RGB White */
	int		selreturn=0;
	int		posScnXA=0, posScnYA=0, posScnXB=0, posScnYB=0;
	int		recvGear_i;
	/* First we get the location of the window passed in to us. */
	XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
	
	/* We now use an XPLMGraphics routine to draw a translucent dark
	 * rectangle that is our window's shape. */
	//XPLMDrawTranslucentDarkBox(left+200, top-15, right+5, bottom);
	XPLMDrawTranslucentDarkBox(left, top, right, bottom);

	/* Finally we draw the text into the window, also using XPLMGraphics
	 * routines.  The NULL indicates no word wrapping. */
	XPLMDrawString(color, left + 10, top - 10, "<The Plugin Uses XPlane As A Sim Env.>", NULL, xplmFont_Basic);
	XPLMDrawString(color, left + 10, top - 25, "Plugin's States & Informations :", NULL, xplmFont_Basic);


	strcpy(str,"Process Counter :");
	strcat(str,ltoa(count,str2,10));
	XPLMDrawString(color, left + 10, top - 40, str, NULL, xplmFont_Basic);

}                                               
          

float	MyProcessLoopCallback(
                                   float                inElapsedSinceLastCall,    
                                   float                inElapsedTimeSinceLastFlightLoop,    
                                   int                  inCounter,    
                                   void *               inRefcon)
{
	long	freq_int,freq_point;
	char	str1[20],str2[20];

	freq_int =count/5;
	freq_point =count%5*1000/5;
	if (count==50)
		count=0;
	else count++;
	
	ltoa(freq_int,str1,10);
	ltoa(freq_point,str2,10);
	strcat(str1,".");
	strcat(str1,str2);
	strcpy(freq_str,str1);

	gimbal_Comm.roll  = XPLMGetDataf(gPlanePhi) / 57.2957795;
	gimbal_Comm.pitch = XPLMGetDataf(gPlaneTheta) / 57.2957795;
	gimbal_Comm.yaw   = XPLMGetDataf(gPlanePsi) / 57.2957795;
	
	gimbal_Comm.send_to_gimbal();

	return 0.02;//Control the frequency
}    


void MyHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus)
{
}                                   


int MyHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon)
{
	if ((inMouse == xplm_MouseDown) || (inMouse == xplm_MouseUp))
		gClicked = 1 - gClicked;

	return 1;
}   


int	ObjDrawCallback(	    XPLMDrawingPhase     inPhase,
                            int                  inIsBefore,
                            void *               inRefcon)
{
	
	return 1;
}


int 	MyOrbitPlaneFunc(
                                   XPLMCameraPosition_t * outCameraPosition,   
                                   int                  inIsLosingControl,    
                                   void *               inRefcon)
{
	
	return 1;
}                                   

int	AircraftDrawCallback(	XPLMDrawingPhase     inPhase,
                            int                  inIsBefore,
                            void *               inRefcon)
{
	
	return 1;
}




