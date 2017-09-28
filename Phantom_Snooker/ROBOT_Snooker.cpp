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

#define OS_WIN32
#include <motor.h>

#include <WROBOT\WRobot.h>
#include <WROBOT\WRobot.cpp>
#include <SOIL.h>
#include <GLFW\glfw3.h>
#include "stb_image.h"
/******************************************************************************/

#define BALLS 5

/******************************************************************************/

// balls of which robot is the first
matrix pos(3,BALLS),vel(3,BALLS),acc(3,BALLS);
matrix force(3,BALLS);
matrix rpos(3,1);    // robot position
matrix rrforce(3,1); // robot force
matrix vmax(3,1),vmin(3,1);
double PercentX=1,PercentY=1.3;
double OffsetX=0,OffsetY=8,OffsetZ=5;
double ValX[2]={ 1.0,0.0 },ValY[2]={ 1.0,0.0 };

float mag;
matrix unit;
float dist;

float RadiusCursor=0.5;
float RadiusBall=0.5;
float RadiusTarget=1.5;

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

BOOL display_postext;
matrix postext_pos(3,1);
float postext_size = 0.3;
char postext[128];


int robot_id;
WROBOT_TYPE robot_type;

matrix textpos(3,1);
char text[128];
int hitcnt = 0;


GLfloat x_min = -0.25f, x_max = 0.25f;
GLfloat y_min = -0.25f, y_max = 0.25f;
GLfloat z_min = -0.25f, z_max = 0.25f;
GLfloat xrot = 0.5f, yrot = 0.5f, zrot = 0.5f;
GLfloat xtrans = 0.0f, ytrans = 0.0f, ztrans = -5.0f;
GLfloat eyeOffset = 0.0f;
GLfloat nearPlane = 0.5f, farPlane = 20.0f;

GLuint texture1, texture2;//"LNGCarrier.bmp"
/******************************************************************************/

	static int LoadGLTexture(const char* path, GLuint &texture) {
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		texture = SOIL_load_OGL_texture(path, SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, 
			(SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT));
		if (texture == 0){
			printf( "SOIL loading error: '%s'\n", SOIL_last_result() );
			system("pause");
			return false;
		}
		 
		
		return true;
	}

bool isImageLoaded = false;

void key_callback(unsigned char key, int x, int y) {
	switch (key) {
	case 27: {
		exit(0);
		break;
	}
	case 'w': {
		xrot += 0.3f;
		yrot += 0.2f;
		zrot += 1.0f;
		break;
	}
	case 's': {
		xrot -= 0.3f;
		yrot -= 0.2f;
		zrot -= 1.0f;
		break;
	}
	case '1': {
		ytrans += 0.1f;
		break;
	}
	case '3': {
		ytrans -= 0.1f;
		break;
	}
	case 'q': {
		eyeOffset += 0.01f;
		break;
	}
	case 'a': {
		eyeOffset -= 0.01f;
		break;
	}


	}
	glutPostRedisplay();
}

void special_key_callback(int key, int x, int y){
	float ratio = farPlane/nearPlane;
	switch (key) {
	case GLUT_KEY_UP: {
		ratio += 1.0f;
		break;
	}
	case GLUT_KEY_DOWN: {
		ratio -= 1.0f;
		break;
	}

	}
	if(ratio < 1.0f)
		ratio = 1.0f;
	farPlane = ratio*nearPlane;
	glutPostRedisplay();
}


void mouse_button_callback(int button, int state, int x, int y) {
	if (button==GLUT_LEFT_BUTTON && state== GLUT_DOWN) {
		xrot += 0.3f;
		yrot += 0.2f;
		zrot += 0.4f;
	}
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
		xrot -= 0.3f;
		yrot -= 0.2f;
		zrot -= 0.4f;
	}
	glutPostRedisplay();
}


void reset_ball( int j )
{
static matrix ppos(3,1);

    // Set position of ball...
    ppos(1,1) = vmin(1,1)+(vmax(1,1)-vmin(1,1))*(float)(j-1)/(BALLS-1);
    ppos(2,1) = vmin(2,1);//+5.0;
    ppos(3,1) = Zpos + rand(1,1)(1,1)*4.0-2;

    // Make sure robot cursor isn't occupying the same position...
    while( norm(ppos-rpos) <= (1.1 * vradius(1,j)) )
    {
		//increase z-pos of ball
		ppos(3,1)++;
		if(ppos(3,1) > vmax(3,1))
		{
			ppos(3,1) = vmin(3,1);
		}
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

    WROBOT_Stop(robot_id, robot_type);
    WROBOT_Close(robot_id, robot_type);

    ExitProgram();
}

/******************************************************************************/

void GlutKeyboard( unsigned char key, int x, int y )
{
    switch( key )
    {
		case 'p':
		case 'P':
			display_postext = !display_postext;
		break;

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

void OPERATOR_forces( matrix &rrpos, matrix &robot_vel, matrix &rforce )
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
  
    vdamp = vel * kdamp;
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
		 hitcnt+=1;
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
        if( sum(pos[j]>(vmax+5.0)) || sum(pos[j]<(vmin-5.0)) )
		{
			printf("Resetting ball - cur. pos.: %f, %f, %f\n", pos[j](1,1),pos[j](2,1),pos[j](3,1));
			reset_ball(j);
		}
    }
}

/******************************************************************************/

void GraphicsDisplayText( char *string, float size, matrix &pos )
{
static matrix p;
void *font=GLUT_STROKE_MONO_ROMAN;
int i,w;
float s=size*0.015;

    p = pos;
    w = strlen(string);

    p(1,1) -= (w / 2) * (s * 100.0);
  
    glPushMatrix();

    glLineWidth(2.0);
    translate(p);
    glScalef(s,s,1);

    GRAPHICS_ColorSet(BLACK);

    for( i=0; (i < w); i++ )
        glutStrokeCharacter(font,string[i]);

    glPopMatrix();
}


/******************************************************************************/

float * CreateDefaultStereoParameters(
    float viewportWidthInches,
    float viewportHeightInches,
    float worldScaleInInches,
    float stereoExaggeration,
	bool rightChannel, 
	float farZ, float nearZ )
{
    // The default stereo parameters produced by this method are based on two assumptions:
    // 1. The viewer's eyes are 24 inches from the display, and
    // 2. The viewer's eyes are separated by 1.25 inches (interocular distance.)
    const float DEFAULT_VIEWER_DISTANCE_IN_INCHES = 24.0f;
    const float DEFAULT_INTEROCULAR_DISTANCE_IN_INCHES = 1.25f;

    float viewportWidth = viewportWidthInches / worldScaleInInches;
    float viewportHeight = viewportHeightInches / worldScaleInInches;
    float viewerDistance = DEFAULT_VIEWER_DISTANCE_IN_INCHES / worldScaleInInches;
    float interocularDistance = DEFAULT_INTEROCULAR_DISTANCE_IN_INCHES / worldScaleInInches * stereoExaggeration;

    float yScale = 2.f * viewerDistance / viewportHeight;
    float xScale = 2.f * viewerDistance / viewportWidth;

    float mFactor = - interocularDistance / viewportWidth;

    if (!rightChannel)
    {
        mFactor = -mFactor;
    }

    float m22 = farZ / (nearZ - farZ);

    // Construct a stereo perspective projection matrix based on assumptions
    // about the viewer and specified stereo parameters. Note that compared
    // to a mono perspective projection matrix, there are two differences:
    //  - a non-zero x:z component (m20)
    //  - a non-zero x:w component (m30)
    // The values of these two factors affect both the x-offset between the
    // left and right eyes, as well as the depth at which they converge. The
    // math used to arrive at these values will often need to change depending
    // on the content being presented in order to ensure a comfortable viewing
    // experience. For example, the factors for rendering massive exterior
    // landscapes will be different than those used for rendering building
    // interiors. Because of this, developers are encouraged to experiment
    // with different techniques for generating these values.
	float XMATRIX[16] = {
        xScale, 0, 0, 0,
        0, yScale, 0, 0,
        mFactor, 0, m22, -1,
        viewerDistance * mFactor, 0, nearZ * m22, 0
	};
    return XMATRIX;
}


void DrawForTheEye(int eye, const GLuint &texture, const int &eyeOffs){
	//GLfloat *m = CreateDefaultStereoParameters(20, 15, 60, 45, true, 20.0, 1.5 );
	//glPushMatrix();
	GLfloat *m = NULL;
	if(eye==1){
		GRAPHICS_ViewCalib(eye);
		m = CreateDefaultStereoParameters(600, 450, 100, 1, false, farPlane, nearPlane );
	}
	else{
		GRAPHICS_ViewCalib(eye, 0.0);
		m = CreateDefaultStereoParameters(600, 450, 100, 1, true, farPlane, nearPlane );
	}

	glLoadMatrixf(m);
	glPushMatrix();
	glBindTexture(GL_TEXTURE_2D, texture);		
			
			//glLoadIdentity();
			//glTranslatef(xtrans, ytrans, ztrans);
	

			//glBindTexture(GL_TEXTURE_2D, texture[0]);
	glBegin(GL_QUADS);
    // Front Face
	glTexCoord2f(0.0f, 0.0f); glVertex3f(x_min, y_min, z_max);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f); glVertex3f(x_max, y_min, z_max);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f); glVertex3f(x_max, y_max, z_max);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f); glVertex3f(x_min, y_max, z_max);  // Top Left Of The Texture and Quad
															  // Back Face
	glTexCoord2f(1.0f, 0.0f); glVertex3f(x_min, y_min, z_min);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f); glVertex3f(x_min, y_max, z_min);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f); glVertex3f(x_max, y_max, z_min);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f); glVertex3f(x_max, y_min, z_min);  // Bottom Left Of The Texture and Quad
															   // Top Face
	glTexCoord2f(0.0f, 1.0f); glVertex3f(x_min, y_max, z_min);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f); glVertex3f(x_min, y_max, z_max);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f); glVertex3f(x_max, y_max, z_max);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f); glVertex3f(x_max, y_max, z_min);  // Top Right Of The Texture and Quad
																	  // Bottom Face
	glTexCoord2f(1.0f, 1.0f); glVertex3f(x_min, y_min, z_min);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f); glVertex3f(x_max, y_min, z_min);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f); glVertex3f(x_max, y_min, z_max);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f); glVertex3f(x_min, y_min, z_max);  // Bottom Right Of The Texture and Quad
																	   // Right face
	glTexCoord2f(1.0f, 0.0f); glVertex3f(x_max, y_min, z_min);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f); glVertex3f(x_max, y_max, z_min);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f); glVertex3f(x_max, y_max, z_max);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f); glVertex3f(x_max, y_min, z_max);  // Bottom Left Of The Texture and Quad
																	  // Left Face
	glTexCoord2f(0.0f, 0.0f); glVertex3f(x_min, y_min, z_min);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f); glVertex3f(x_min, y_min, z_max);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f); glVertex3f(x_min, y_max, z_max);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f); glVertex3f(x_min, y_max, z_min);  // Top Left Of The Texture and Quad
	glEnd();
			
	glPopMatrix();
	//glPopMatrix();

	glFlush();
}

void Draw( void )
{    
int  j;

    // Clear "stereo" graphics buffers...
    GRAPHICS_ClearStereo();
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Processing loop for each eye...
    GRAPHICS_EyeLoop(eye)
    {

        /*GRAPHICS_ViewCalib(eye);

        // Clear "mono" graphics buffers...
        GRAPHICS_ClearMono();
		GRAPHICS_Sphere(&rpos,RadiusCursor, YELLOW);*/
		
        //GameStarted = true;
        // Playing balls...
        if( GameStarted )
        {
			/*set_color(RED);
			
			for(j=1;j<=BALLS;j++)
            {
				if(eye==0)
				{
                    GRAPHICS_Sphere(&pos[j],RadiusBall,PURPLE);
				}
				else
				{
					GRAPHICS_Sphere(&pos[j],RadiusBall,PURPLE);
				}
            }*/
			
			//glLoadIdentity();
			if(eye==1)
				DrawForTheEye(eye, texture1, 0);
			else
				DrawForTheEye(eye, texture2, eyeOffset);	
		}

        //GRAPHICS_Sphere(&targ,RadiusTarget,YELLOW,0.6);

		// Print score
		/*sprintf(text,"Score: %d",hitcnt);
		GraphicsDisplayText(text,0.4,textpos);

		if(display_postext)
		{
			sprintf(postext,"X: %.3f, Y:%.3f, Z:%.3f",rpos(1,1),rpos(2,1),rpos(3,1));
			GraphicsDisplayText(postext,postext_size,postext_pos);
		}*/
		
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

    //swapbuffers();
	glutSwapBuffers();
}

/******************************************************************************/


void GlutDisplay( void )
{
	//"hack" to prevent application from going into fullscreen
	// and thus be able to span it across multiple displays"
	//TODO: Find an apropriate Position for this hack, replace the hardcoded vars by params
	//Find options to get rid of (most of the) menubar
	/*static int first_try = 0;   
	if(first_try == 0)
	{
		//bring fullscreen application back into windowed mode
		glutReshapeWindow(2560, 1000);
		glutPositionWindow(0,24);
		first_try ++; //only do this once
	}*/

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

    if( (robot_id=WROBOT_Open(RobotName,robot_type)) == ROBOT_INVALID )
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
    	
	
	GRAPHICS_OpenGL(GRAPHICS_FLAG_LIGHTING | GRAPHICS_FLAG_TRANSPARENCY,TURQUOISE);
	//glDisable(GL_LIGHT0);
	/*GLfloat light0_position[] = { 30.0, 5.5, 30.0, 0.01 };
	GLfloat light0_specular[] = { 0, 0, 1, 0.5 };
    GLfloat light0_brightness[] = { 1, 1, 1, 1.0 };
	GLfloat light0_ambient[] = { 1.0, 0.0, 1.0, 1.0 };
	GLfloat mat_shininess[] = { 50.0 };

	glShadeModel (GL_SMOOTH);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light0_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glMaterialfv(GL_FRONT, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	//glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    //glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_brightness);
   // glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);

	glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);*/
	glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

	GLfloat light0_position[] = { 0.0, 0.0, 30.0, 0.01 };
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

	GLfloat no_mat[] = { 0.0, 0.0, 0.0, 1.0 };
   GLfloat mat_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
   GLfloat mat_ambient_color[] = { 0.8, 0.8, 0.8, 1.0 };
   GLfloat mat_diffuse[] = { 0.1, 0.1, 0.1, 1.0 };
   GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
   GLfloat no_shininess[] = { 0.0 };
   GLfloat low_shininess[] = { 5.0 };
   GLfloat high_shininess[] = { 100.0 };
   GLfloat mat_emission[] = {0.3, 0.2, 0.2, 0.0};

   //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glShadeModel (GL_SMOOTH);

   glMaterialfv(GL_BACK, GL_AMBIENT, mat_ambient);
   glMaterialfv(GL_BACK, GL_DIFFUSE, mat_diffuse);
   glMaterialfv(GL_BACK, GL_SPECULAR, mat_specular);
   glMaterialfv(GL_BACK, GL_SHININESS, high_shininess);
   glMaterialfv(GL_BACK, GL_EMISSION, no_mat);


    Robot2D = ROBOT_2D();
    Zpos = OffsetZ + GRAPHICS_CalibCentrePOMX(3,1); // GRAPHICS_FocalPlane;

    // Work-space dimensions...
    vmin(1,1) = PercentX * GRAPHICS_CalibRange[GRAPHICS_X][GRAPHICS_MIN];
    vmin(2,1) = OffsetY + (PercentY * GRAPHICS_CalibRange[GRAPHICS_Y][GRAPHICS_MIN]);
    vmin(3,1) = GRAPHICS_CalibRange[GRAPHICS_Z][GRAPHICS_MIN];

    vmax(1,1) = PercentX * GRAPHICS_CalibRange[GRAPHICS_X][GRAPHICS_MAX];
    vmax(2,1) = OffsetY + (PercentY * GRAPHICS_CalibRange[GRAPHICS_Y][GRAPHICS_MAX]);
    vmax(3,1) = GRAPHICS_CalibRange[GRAPHICS_Z][GRAPHICS_MAX];

	printf("vmin: %f, %f, %f\n", vmin(1,1),vmin(2,1),vmin(3,1));
	printf("vmax: %f, %f, %f\n", vmax(1,1),vmax(2,1),vmax(3,1));

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

	//position for text display
	textpos(1,1) = -8;
	textpos(2,1) = 17;	
	textpos(3,1) = -2;
	postext_pos(1,1) = 0;
	postext_pos(2,1) = 4;
	postext_pos(3,1) = 0;


    reset_all();

    if( !WROBOT_Start(robot_id,OPERATOR_forces,robot_type) )
    {
        printf("Cannot start robot.\n");
        WROBOT_Close(robot_id, robot_type);
        ExitProgram();
    }

	//glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	//GLFWwindow *window = glfwCreateWindow(640, 480, "Hello Triangle!", NULL, NULL);

	
		printf("Initialising...\n");
		//glutInitWindowSize(700, 700);
		//glutInitWindowPosition(0, 0);
		//glutCreateWindow("SOIL Texture Test");
	

	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  
	isImageLoaded = LoadGLTexture("right3.bmp", texture1);
	isImageLoaded = LoadGLTexture("left3.bmp", texture2);
	if(isImageLoaded){
    glutDisplayFunc(GlutDisplay);
    glutIdleFunc(GlutIdle);
	glutKeyboardFunc(key_callback);
	//glutMouseFunc(mouse_button_callback);
	glutSpecialFunc(special_key_callback);
	
	glutReshapeWindow(800, 600);        /* Restore us */
    glutPositionWindow(0,0);

    glutMainLoop(); 
	}
	return;
}

/******************************************************************************/

