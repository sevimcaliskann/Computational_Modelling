/******************************************************************************/
/*                                                                            */
/* MODULE  : ROBOT_Snooker.cpp                                                */
/*                                                                            */
/* PURPOSE : Robot virtual reality demonstration hitting balls into a target. */
/*                                                                            */
/* DATE    : 04/Nov/2000                                                      */
/*                                                                            */
/* CHANGES                                                                    */
/*                                                                            */
/* V1.0  JNI 04/Nov/2000 - Initial development of module.                     */
/*                                                                            */
/* V2.0  JNI 20/Jul/2006 - Renamed during the big shake-up of 2006.           */
/*                                                                            */
/******************************************************************************/

#include <motor.h>

/******************************************************************************/

#define BALLS 5

/******************************************************************************/

// balls of which robot is the first
matrix pos(3,BALLS),vel(3,BALLS),acc(3,BALLS);
matrix force(3,BALLS);
matrix rpos(3,1);    // robot position
matrix rrforce(3,1); // robot force
matrix vmax(3,1),vmin(3,1);
double PercentX=0.7,PercentY=1.0;
double OffsetX=0.0,OffsetY=0.0;
double ValX[2]={ 1.0,0.0 },ValY[2]={ 1.0,0.0 };

float mag;
matrix unit;
float dist;

float RadiusCursor=1.5;
float RadiusBall=1.5;
float RadiusTarget=3.0;

//float SpringConstant=-1.0;
float SpringConstant=-30.0;
matrix vradius(1,BALLS);
matrix vdamp(1,BALLS);
double kdamp=0.0;
matrix mass(1,BALLS);
matrix grav(1,BALLS);
float dt=0.001;  
int i,j;
matrix f;
matrix targ(3,1);

BOOL GameStarted=FALSE;
BOOL Robot2D=FALSE;
float Zpos=0;

/******************************************************************************/

void reset_ball( int j )
{
static matrix ppos(3,1);

    // Set position of ball...
    ppos(1,1) = vmin(1,1)+(vmax(1,1)-vmin(1,1))*(float)(j-1)/(BALLS-1);
    ppos(2,1) = vmin(2,1)+5.0;
    ppos(3,1) = Zpos;

    // Make sure robot cursor isn't occupying the same position...
    if( norm(ppos-rpos) <= (1.1 * vradius(1,j)) )
    {
        return;
    }

    // Reset values for the ball...
    pos(1,j) = ppos(1,1);
    pos(2,j) = ppos(2,1);
    pos(3,j) = ppos(3,1);

    vel(1,j) = 0;
    vel(2,j) = 0;
    vel(3,j) = 0;

    acc(1,j) = 0;
    acc(2,j) = 0;
    acc(3,j) = 0;

    force(1,j) = 0;
    force(2,j) = 0;
    force(3,j) = 0;
}

/******************************************************************************/

void reset_all( void )
{
    for( j=1; (j <= BALLS); j++ ) reset_ball(j);
}

/******************************************************************************/

void ProgramExit( void )
{
    // Stop graphics system.
    GRAPHICS_Stop();

    ROBOT_Stop();
    ROBOT_Close();

    ExitProgram();
}

/******************************************************************************/

void GlutKeyboard( unsigned char key, int x, int y )
{
    switch( key )
    {
       case ESC : 
       case 'q' :
       case 'Q' :
          ProgramExit();

       default :
          printf("Reseting...\n");
          reset_all();
          break;
    }
}

/******************************************************************************/

void OPERATOR_forces( matrix &rrpos, matrix &rforce )
{
static long int k=0;

    k++;

    rpos = rrpos;

    force.zeros();
    rforce.zeros();

    // Fix Z axis to focal plane if 2D system...
    if( Robot2D )
    {
        rpos(3,1) = Zpos;
    }

    if( !GameStarted )
    {
        if( rpos(2,1) <= vmin(2,1) )
        {
            GameStarted = TRUE;
        }

        return;
    }

    //ball interactions
    for(i=1;i<=BALLS;i++)
    {
      dist=norm(pos[i]-rpos);
      if(dist<=vradius(1,i))
	{
	  unit=(pos[i]-rpos)/dist;
	  mag=-SpringConstant*(vradius(1,i)-dist);

	  force(1,i)+=mag*unit(1,1);
	  force(2,i)+=mag*unit(2,1);

	  rforce(1,1)-=mag*unit(1,1);
	  rforce(2,1)-=mag*unit(2,1);

          if( Robot2D )
          {
              force(3,i) = 0;
              rforce(3,1) = 0;
          }
          else
          {
              force(3,i) +=mag*unit(3,1);
              rforce(3,1) -=mag*unit(3,1);
          }
	}

        // Apply gravity to Z axis if 3D and X axis if 2D...
        //force(Robot2D ? 1 : 3,i)+=(norm(vel[i])>0.1)*grav(1,i)*mass(1,i);

        force(1,i)+=(norm(vel[i])>0.1)*grav(1,i)*mass(1,i);
    }

    // Make 5th ball stay still
    force(1,5)=0;
    force(2,5)=0;
    force(3,5)=0;
  
    vdamp = vel * kdamp;;
    acc = (force-vdamp) / (mass % mass % mass);

    vel+=acc*dt;
    pos+=vel*dt;

    // Make global for debugging...
    rrforce = rforce;
}

/******************************************************************************/

void Calc( void )
{
int j;
static int hit=0; 

    if( !GameStarted )
    {
        return;
    }

//    if( !hit )
    {
      for(j=1;j<=BALLS;j++)
	{
	    if(norm(pos[j]-targ)<RadiusBall+(RadiusTarget/2))
	    {
             BEEPER_Beep(800,50);
//             reset_ball(j);
             reset_all();
	     hit=1;
	     break;
	   }
	}
    }
  /*
  if(hit)
    {
      hit=0;
      for(j=1;j<=BALLS;j++)
	{
	    if(norm(pos[j]-targ)<RadiusBall+(RadiusTarget/2))
	    {
	      hit=1;
	      break;
	    }
	}
      
    }
*/
    // Reset ball if it goes outside workspace...
    for( j=1; j<=BALLS; j++ )
    {
        if( sum(pos[j]>(vmax+5.0)) || sum(pos[j]<(vmin-5.0)) ) reset_ball(j);
    }
}

/******************************************************************************/

void Draw( void )
{    
int  j;

    // Clear "stereo" graphics buffers...
    GRAPHICS_ClearStereo();

    // Processing loop for each eye...
    GRAPHICS_EyeLoop(eye)
    {
        GRAPHICS_ViewCalib(eye);

        // Clear "mono" graphics buffers...
        GRAPHICS_ClearMono();

        // Cursor ball...
        GRAPHICS_Sphere(&rpos,RadiusCursor,GREEN);
      
        // Playing balls...
        if( GameStarted )
        {
            set_color(RED);

            for(j=1;j<=BALLS;j++)
            {
                GRAPHICS_Sphere(&pos[j],RadiusBall,RED);
            }
        }

        // Target sphere...
        GRAPHICS_Sphere(&targ,RadiusTarget,YELLOW);
    }      

/*
if( TIMER_EveryHz(2.0) )
{
   printf("Centre %4.1lf %4.1lf %4.1lf  Robot %4.1lf %4.1lf %4.1lf Force %4.1lf %4.1lf %4.1lf\n",
          GRAPHICS_CalibCentrePOMX(1,1),
          GRAPHICS_CalibCentrePOMX(2,1),
          GRAPHICS_CalibCentrePOMX(3,1),
          rpos(1,1),
          rpos(2,1),
          rpos(3,1),
          rrforce(1,1),
          rrforce(2,1),
          rrforce(3,1));
} */

    swapbuffers();
}

/******************************************************************************/

void GlutDisplay( void )
{
    Calc();
    Draw();
}

/******************************************************************************/

void GlutIdle( void ) 
{ 
    glutPostRedisplay(); 
}

/******************************************************************************/

STRING RobotName="";
STRING DisplayModeText="";
int    DisplayMode=GRAPHICS_DISPLAY_STEREO;

/******************************************************************************/

void Usage( void )
{
    printf("\n");
    printf("ROBOT_Snooker /R:robot /D:display /K:spring(N/cm) /X:x%% /Y:y%%\n");
    printf("\n");
    printf("   Display=%s\n",STR_TextList(GRAPHICS_DisplayText));
}

/******************************************************************************/

BOOL Parameters( int argc, char *argv[] )
{
BOOL    ok;
int     i;
char   *data;

    for( ok=TRUE,i=1; ((i < argc) && ok); i++ )
    {
        switch( CMDARG_code(argv[i],&data) )
        {
            case 'R' :
               ok = CMDARG_data(RobotName,data,STRLEN);
               break;

            case 'D' :
               if( !CMDARG_data(DisplayModeText,data,STRLEN) )
               {
                   ok = FALSE;
               }
               else
               if( (DisplayMode=STR_TextCode(GRAPHICS_DisplayText,DisplayModeText)) == STR_NOTFOUND )
               {
                   ok = FALSE;
               }
               break;

            case 'X' :
               if( (ok=CMDARG_data(ValX,data,2)) )
               {
                   PercentX = ValX[0];
                   OffsetX = ValX[1];
               }
               break;

            case 'Y' :
               if( (ok=CMDARG_data(ValY,data,2)) )
               {
                   PercentY = ValY[0];
                   OffsetY = ValY[1];
               }
               break;

            case 'Z' :
               ok = CMDARG_data(Zpos,data);
               break;

            case 'K' :
               ok = CMDARG_data(SpringConstant,data);
               break;

            case 'T' :
               ok = CMDARG_data(RadiusTarget,data);
               break;

            case 'B' :
               ok = CMDARG_data(RadiusBall,data);
               break;

            case 'C' :
               ok = CMDARG_data(RadiusCursor,data);
               break;

            case 'W' :
               ExitProgramWait = TRUE;
               break;

            case '?' :
               return(FALSE);

            default :
               ok = FALSE;
               break;
        }

        if( !ok )
        {
            printf("Invalid argument: %s\n",argv[i]);
        }
    }

    if( STR_null(RobotName,STRLEN) )
    {
        printf("Must specify robot name.\n");
        ok = FALSE;
    }

    return(ok);
}

/******************************************************************************/

void    main( int argc, char *argv[] )
{
BOOL Exit;

    if( !Parameters(argc,argv) )
    {
        Usage();
        ExitProgram();
    }

    if( ROBOT_Open(RobotName) == ROBOT_INVALID )
    {
        printf("Cannot open robot: %s.\n",RobotName);
        ExitProgram();
    }

    printf("Robot %s opened.\n",RobotName);
  
    if( !GRAPHICS_Start(DisplayMode) )
    {
        printf("Cannot start GRAPHICS system.\n");
        ExitProgram();
    }
    
    GRAPHICS_OpenGL(GRAPHICS_FLAG_LIGHTING,LIGHTBLUE);

    Robot2D = ROBOT_2D();
    Zpos = GRAPHICS_CalibCentrePOMX(3,1); // GRAPHICS_FocalPlane;

    // Work-space dimensions...
    vmin(1,1) = PercentX * GRAPHICS_CalibRange[GRAPHICS_X][GRAPHICS_MIN];
    vmin(2,1) = OffsetY + (PercentY * GRAPHICS_CalibRange[GRAPHICS_Y][GRAPHICS_MIN]);
    vmin(3,1) = GRAPHICS_CalibRange[GRAPHICS_Z][GRAPHICS_MIN];

    vmax(1,1) = PercentX * GRAPHICS_CalibRange[GRAPHICS_X][GRAPHICS_MAX];
    vmax(2,1) = OffsetY + (PercentY * GRAPHICS_CalibRange[GRAPHICS_Y][GRAPHICS_MAX]);
    vmax(3,1) = GRAPHICS_CalibRange[GRAPHICS_Z][GRAPHICS_MAX];

    // Damping constant.
    kdamp=0.008;
 
    // Mass.
    mass(1,1)=0.1;
    mass(1,2)=1.0;
    mass(1,3)=0.1;
    mass(1,4)=0.1;
    mass(1,5)=0.1;
    mass = mass / 2.5;

    // "Gravity"
    grav(1,1)=0;
    grav(1,2)=0;
    grav(1,3)=-5.0; //-9.8;
    grav(1,4)= 5.0; //9.8;
    grav(1,5)=0;
 
    // Effective "collision detect" radius...
    vradius(1,1) = RadiusCursor+RadiusBall;
    vradius(1,2) = RadiusCursor+RadiusBall;
    vradius(1,3) = RadiusCursor+RadiusBall;
    vradius(1,4) = RadiusCursor+RadiusBall;
    vradius(1,5) = RadiusCursor+RadiusBall;
 
    // object 1 normal
    // object 2 heavy
    // object 3 down gravity
    // object 4 up gravity
    // object 5 fixed will not move

    // Target position...
    targ(1,1) = (vmax(1,1)+vmin(1,1))/2.0;
    targ(2,1) = vmax(2,1);
    targ(3,1) = Zpos;

    reset_all();

    if( !ROBOT_Start(OPERATOR_forces) )
    {
        printf("Cannot start robot.\n");
        ROBOT_Close();
        ExitProgram();
    }
  
    glutDisplayFunc(GlutDisplay);
    glutIdleFunc(GlutIdle);
    glutKeyboardFunc(GlutKeyboard);
    glutMainLoop(); 
}

/******************************************************************************/

