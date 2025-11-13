/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  EV3 Version 2.0 - Updated Jul. 2022 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <stdbool.h>

extern int sx;              // Get access to the image size from the imageCapture module
extern int sy;
int laggy=0;

// declare static functions
static void soccer_mode(struct RoboAI *ai, struct blob *blobs);
static void penalty_mode(struct RoboAI *ai, double smx, double smy);
static void chase_mode(struct RoboAI *ai, struct blob *blobs);

// Tuning knobs for penalty routine
enum {
    FACE_THRESH_DEG   = 8,    // tweak
    ALIGN_THRESH_DEG  = 20,   // tweak
    TARGET_BALL_DIST  = 100,   // pixels; tweak to your scale
    CLOSE_BALL_SLACK  = 5,    // +/-
    BEHIND_BALL_GAP   = 10    // min px robot should be "behind" ball wrt goal
};

// Helpers (predicates)
static inline double deg_wrap(double d){
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

static bool is_facing_ball(struct RoboAI *ai, double smx, double smy) {
    double e = compute_angle_error_to_ball(ai, smx, smy);
    fprintf(stderr, "Angle error to ball: %.2f deg\n", e);
    return !isnan(e) && fabs(e) <= FACE_THRESH_DEG;
}

static bool is_close_to_ball(struct RoboAI *ai) {
    double de = 0, dd = 0;
    double d = compute_distance_error(ai, TARGET_BALL_DIST, &de, &dd);
    return !isnan(d) && d <= (TARGET_BALL_DIST + CLOSE_BALL_SLACK);
}

// are we behind the ball and pointing so that a straight push sends the ball toward the opponent goal?
static bool is_aligned_to_goal_for_shot(struct RoboAI *ai) {
    if (!ai || !ai->st.self || !ai->st.ball) return false;

    // Vector robot->ball
    double rbx = ai->st.ball->cx - ai->st.self->cx;
    double rby = ai->st.ball->cy - ai->st.self->cy;
    double rbn = hypot(rbx, rby);
    if (rbn < 1e-6) return false;
    rbx /= rbn; rby /= rbn;

    // "Goal direction" unit vector in field coordinates.
    // By your rotate_to_goal(): side==0 -> face 180° (negative X), else 0° (positive X)
    double gx = (ai->st.side == 0) ? -1.0 : 1.0;
    double gy = 0.0;

    // angle between robot->ball and goal dir
    double dot = rbx*gx + rby*gy;
    double ang_deg = acos(fmax(-1.0, fmin(1.0, dot))) * 180.0 / M_PI;

    // "behind ball" condition so we push *through* the ball toward goal
    bool behind =
        (ai->st.side == 0) ? (ai->st.self->cx >= ai->st.ball->cx + BEHIND_BALL_GAP)
                           : (ai->st.self->cx <= ai->st.ball->cx - BEHIND_BALL_GAP);

    return (ang_deg <= ALIGN_THRESH_DEG) && behind;
}

/**************************************************************
 * Display List Management
 * 
 * The display list head is kept as a pointer inside the A.I. 
 * data structure. Initially NULL (of course). It works like
 * any other linked list - anytime you add a graphical marker
 * it's added to the list, the imageCapture code loops over
 * the list and draws any items in there.
 * 
 * The list IS NOT CLEARED between frames (so you can display
 * things like motion paths that go over mutiple frames).
 * Your code will need to call clearDP() when you want this
 * list cleared.
 * 
 * ***********************************************************/
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type=0;
  newNode->x1=x;
  newNode->y1=y;
  newNode->x2=-1;
  newNode->y2=-1;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  
  newNode->next=head;
  return(newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x2;
  newNode->y2=y2;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);  
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B)
{
  struct displayList *newNode;
  double l;
  
  l=sqrt((dx*dx)+(dy*dy));
  dx=dx/l;
  dy=dy/l;
  
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addVector(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x1+(length*dx);
  newNode->y2=y1+(length*dy);
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x-length;
  newNode->y1=y;
  newNode->x2=x+length;
  newNode->y2=y;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  head=newNode;

  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x;
  newNode->y1=y-length;
  newNode->x2=x;
  newNode->y2=y+length;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *clearDP(struct displayList *head)
{
  struct displayList *q;
  while(head)
  {
      q=head->next;
      free(head);
      head=q;
  }
  return(NULL);
}

/**************************************************************
 * End of Display List Management
 * ***********************************************************/

/*************************************************************
 * Blob identification and tracking
 * ***********************************************************/

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // This function looks for and identifies a blob with the specified colour.
 // It uses the hue and saturation values computed for each blob and tries to
 // select the blob that is most like the expected colour (red, green, or blue)
 //
 // If you find that tracking of blobs is not working as well as you'd like,
 // you can try to improve the matching criteria used in this function.
 // Remember you also have access to shape data and orientation axes for blobs.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> Blue bot
 //                   1 -> Red bot
 //                   2 -> Yellow ball
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double vr_x,vr_y,maxfit,mincos,dp;
 double vb_x,vb_y,fit;
 double maxsize=0;
 double maxgray;
 int grayness;
 int i;
 static double Mh[4]={-1,-1,-1,-1};
 static double mx0,my0,mx1,my1,mx2,my2;
 FILE *f;
 
 // Import calibration data from file - this will contain the colour values selected by
 // the user in the U.I.
 if (Mh[0]==-1)
 {
  f=fopen("colours.dat","r");
  if (f!=NULL)
  {
   fread(&Mh[0],4*sizeof(double),1,f);
   fclose(f);
   mx0=cos(Mh[0]);
   my0=sin(Mh[0]);
   mx1=cos(Mh[1]);
   my1=sin(Mh[1]);
   mx2=cos(Mh[2]);
   my2=sin(Mh[2]);
  }
 }

 if (Mh[0]==-1)
 {
     fprintf(stderr,"roboAI.c :: id_coloured_blob2(): No colour calibration data, can not ID blobs. Please capture colour calibration data on the U.I. first\n");
     return NULL;
 }
 
 maxfit=.025;                                             // Minimum fitness threshold
 mincos=.9;                                               // Threshold on colour angle similarity
 maxgray=.25;                                             // Maximum allowed difference in colour
                                                          // to be considered gray-ish (as a percentage
                                                          // of intensity)

 // The reference colours here are in the HSV colourspace, we look at the hue component, which is a
 // defined within a colour-wheel that contains all possible colours. Hence, the hue component
 // is a value in [0 360] degrees, or [0 2*pi] radians, indicating the colour's location on the
 // colour wheel. If we want to detect a different colour, all we need to do is figure out its
 // location in the colour wheel and then set the angles below (in radians) to that colour's
 // angle within the wheel.
 // For reference: Red is at 0 degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  // Agent IDs are as follows: 0 : blue bot,  1 : red bot, 2 : yellow ball
  if (col==0) {vr_x=mx0; vr_y=my0;}                                                    
  else if (col==1) {vr_x=mx1; vr_y=my1;}
  else if (col==2) {vr_x=mx2; vr_y=my2;}

 // In what follows, colours are represented by a unit-length vector in the direction of the
 // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
 // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
 // If the dot product is 1 the colours are identical (their vectors perfectly aligned), 
 // from there, the dot product decreases as the colour vectors start to point in different
 // directions. Two colours that are opposite will result in a dot product of -1.
 
 p=blobs;
 while (p!=NULL)
 { 
  if (p->size>maxsize) maxsize=p->size;
  p=p->next;
 }

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  // Normalization and range extension
  vb_x=cos(p->H);
  vb_y=sin(p->H);

  dp=(vb_x*vr_x)+(vb_y*vr_y);                                       // Dot product between the reference color vector, and the
                                                                    // blob's color vector.

  fit=dp*p->S*p->S*(p->size/maxsize);                               // <<< --- This is the critical matching criterion.
                                                                    // * THe dot product with the reference direction,
                                                                    // * Saturation squared
                                                                    // * And blob size (in pixels, not from bounding box)
                                                                    // You can try to fine tune this if you feel you can
                                                                    // improve tracking stability by changing this fitness
                                                                    // computation

  // Check for a gray-ish blob - they tend to give trouble
  grayness=0;
  if (fabs(p->R-p->G)/p->R<maxgray&&fabs(p->R-p->G)/p->G<maxgray&&fabs(p->R-p->B)/p->R<maxgray&&fabs(p->R-p->B)/p->B<maxgray&&\
      fabs(p->G-p->B)/p->G<maxgray&&fabs(p->G-p->B)/p->B<maxgray) grayness=1;
  
  if (fit>maxfit&&dp>mincos&&grayness==0)
  {
   fnd=p;
   maxfit=fit;
  }
  
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Motion direction vector. Not valid
 //   while rotating - possibly valid while turning
 // - Heading direction - vector obtained from the blob shape, it is
 //   correct up to a factor of (-1) (i.e. it may point backward w.r.t.
 //   the direction your bot is facing). This vector remains valid
 //   under rotation.
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // NOTE: If a particular agent is not found, the corresponding field in
 //       the AI data structure (ai->st.ball, ai->st.self, ai->st.opp)
 //       will remain NULL. Make sure you check for this before you 
 //       try to access an agent's blob data! 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 //
 // You can change this function if you feel the tracking is not stable.
 // First, though, be sure to completely understand what it's doing.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 
 // Reset ID flags and agent blob pointers
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;
 
 // Find the ball
 p=id_coloured_blob2(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;
  ai->st.bdx=p->dx;
  ai->st.bdy=p->dy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute motion direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  else
  {
    ai->st.bmx=0;
    ai->st.bmy=0;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot - the colour is set from commane line, 0=Blue, 1=Red
 p=id_coloured_blob2(ai,blobs,ai->st.botCol);
 if (p!=NULL&&p!=ai->st.ball)
 {
  ai->st.self=p;			// Update pointer to self-blob
  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;
  ai->st.sdx=p->dx;
  ai->st.sdy=p->dy;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
//  printf("--->    Track agents(): d=[%lf, %lf], [x,y]=[%3.3lf, %3.3lf], old=[%3.3lf, %3.3lf], v=[%2.3lf, %2.3lf], motion=[%2.3lf, %2.3lf]\n",ai->st.sdx,ai->st.sdy,ai->st.self->cx,ai->st.self->cy,ai->st.old_scx,ai->st.old_scy,vx,vy,vx/mg,vy/mg);
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }
  else
  {
   ai->st.smx=0;
   ai->st.smy=0;
  }
  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;
 }
 else ai->st.self=NULL;

 // ID our opponent - whatever colour is not botCol
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,1);
 else p=id_coloured_blob2(ai,blobs,0);
 if (p!=NULL&&p!=ai->st.ball&&p!=ai->st.self)
 {
  ai->st.opp=p;	
  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;
  ai->st.odx=p->dx;
  ai->st.ody=p->dy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  else
  {
   ai->st.omx=0;
   ai->st.omy=0;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 static double oldX,oldY;
 double frame_inc=1.0/5.0;
 double dist;
 
 track_agents(ai,blobs);		// Call the tracking function to find each agent

 BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30, 30);			// Start forward motion to establish heading
                                                // Will move for a few frames.
  
 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  BT_all_stop(0);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}
/*********************************************************************************
 * End of blob ID and tracking code
 * ******************************************************************************/

/*********************************************************************************
 * Routine to initialize the AI 
 * *******************************************************************************/
int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 BT_all_stop(0);			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.sdx=0;
 ai->st.sdy=0;
 ai->st.odx=0;
 ai->st.ody=0;
 ai->st.bdx=0;
 ai->st.bdy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 ai->DPhead=NULL;
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 // The code here just makes sure the image processing loop is constantly
 // tracking the bots while they're placed in the locations required
 // to do the calibration (i.e. you DON'T need to add anything more
 // in this function).
 track_agents(ai,blobs);
}


/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is your robot's state machine.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your EV3 to get damaged!

   IMPORTANT NOTE: There are TWO sources of information about the 
                   location/parameters of each agent
                   1) The 'blob' data structures from the imageCapture module
                   2) The values in the 'ai' data structure.
                      The 'blob' data is incomplete and changes frame to frame
                      The 'ai' data should be more robust and stable
                      BUT in order for the 'ai' data to be updated, you
                      must call the function 'track_agents()' in your code
                      after eah frame!
                      
    DATA STRUCTURE ORGANIZATION:

    'RoboAI' data structure 'ai'
         \    \    \   \--- calibrate()  (pointer to AI_clibrate() )
          \    \    \--- runAI()  (pointer to the function AI_main() )
           \    \------ Display List head pointer 
            \_________ 'ai_data' data structure 'st'
                         \  \   \------- AI state variable and other flags
                          \  \---------- pointers to 3 'blob' data structures
                           \             (one per agent)
                            \------------ parameters for the 3 agents
                              
  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

  static double ux,uy,len,mmx,mmy,tx,ty,x1,y1,x2,y2;
  double angDif;
  char line[1024];
  static int count=0;
  static double old_dx=0, old_dy=0;

  static double stored_smx = 0, stored_smy = 0;
      
  /************************************************************
   * Standard initialization routine for starter code,
   * from state **0 performs agent detection and initializes
   * directions, motion vectors, and locations
   * Triggered by toggling the AI on.
   * - Modified now (not in starter code!) to have local
   *   but STATIC data structures to keep track of robot
   *   parameters across frames (blob parameters change
   *   frame to frame, memoryless).
   ************************************************************/
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	  // The id_bot() routine will change the AI state to initial state + 1
  {				                 // if robot identification is successful.
      
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;         // This sets the side the bot thinks as its own side 0->left, 1->right
   BT_all_stop(0);
   
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.sdx,ai->st.sdy,ai->st.state);
   stored_smx = ai->st.smx;
   stored_smy = ai->st.smy;
   
   if (ai->st.self!=NULL)
   {
       // This checks that the motion vector and the blob direction vector
       // are pointing in the same direction. If they are not (the dot product
       // is less than 0) it inverts the blob direction vector so it points
       // in the same direction as the motion vector.
       if (((ai->st.smx*ai->st.sdx)+(ai->st.smy*ai->st.sdy))<0)
       {
           ai->st.self->dx*=-1.0;
           ai->st.self->dy*=-1.0;
           ai->st.sdx*=-1;
           ai->st.sdy*=-1;
       }
       old_dx=ai->st.sdx;
       old_dy=ai->st.sdy;
   }
  
   if (ai->st.opp!=NULL)
   {
       // Checks motion vector and blob direction for opponent. See above.
       if (((ai->st.omx*ai->st.odx)+(ai->st.omy*ai->st.ody))<0)
       {
           ai->st.opp->dx*=-1;
           ai->st.opp->dy*=-1;
           ai->st.odx*=-1;
           ai->st.ody*=-1;
       }       
   }

         
  }
  
  // Initialize BotInfo structures
   
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
 fprintf(stderr,"Just trackin with state'!\n", ai->st.state);	// bot, opponent, and ball.
  track_agents(ai,blobs);
  
  // get current state and call appropriate function
  int state = ai->st.state;

  if (state >= 0 && state < 100) {
      // SOCCER mode
      soccer_mode(ai, blobs);
  } else if (state >= 100 && state < 200) {
      // PENALTY mode
      penalty_mode(ai, stored_smx, stored_smy);
  } else if (state >= 200 && state < 300) {
      // CHASE mode
      chase_mode(ai, blobs);
  } else {
      fprintf(stderr, "Unknown AI state: %d\n", state);
  }
}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/

}

static void soccer_mode(struct RoboAI *ai, struct blob *blobs) {
}

// TODOO: more detailed implementation
static void penalty_mode(struct RoboAI *ai, double stored_smx, double stored_smy) {
  fprintf(stderr, "In PENALTY mode, current state: %d\n", ai->st.state);
  int state = ai->st.state;

  // TODOO: add more transitions (lost track, reset, still moving etc)
  // now only consider the main flow
  switch (state) {
    case ST_PENALTY_ROTATE_TO_BALL:
      // if (!is_facing_ball(ai)) {
      //   fprintf(stderr, "Rotating to face ball in PENALTY mode\n");
      //   rotate_to_blob(ai);
      // } else {
      //   fprintf(stderr, "Facing ball achieved in PENALTY mode\n");
      //   ai->st.state = ST_PENALTY_DONE;
      //   BT_all_stop(0);
      // }

      // test moving to ball and stop when close
      // assume already rotated to face ball
      // print the distance to ball
      fprintf(stderr, "Distance to ball: %f\n", compute_distance_error(ai, 40.0, NULL, NULL));
      if (is_close_to_ball(ai)) {
        ai->st.state = ST_PENALTY_DONE;
        break;
      }
    case ST_PENALTY_MOVE_TO_BALL:
      move_to_blob(ai, stored_smx, stored_smy);
      if (!is_facing_ball(ai, stored_smx, stored_smy)) {
        ai->st.state = ST_PENALTY_ROTATE_TO_BALL;
        BT_all_stop(0);
        break;
      } else if (is_close_to_ball(ai)) {
        ai->st.state = ST_PENALTY_ALIGN_TO_GOAL;
        BT_all_stop(0);
      }
      break;
    case ST_PENALTY_ALIGN_TO_GOAL:
      align_to_goal_with_ball(ai, stored_smx, stored_smy);
      if (!is_facing_ball(ai, stored_smx, stored_smy)) {
        ai->st.state = ST_PENALTY_ROTATE_TO_BALL;
        BT_all_stop(0);
        break;
      } else if (is_aligned_to_goal_for_shot(ai) && is_close_to_ball(ai)) {
        ai->st.state = ST_PENALTY_KICK_BALL;
        BT_all_stop(0);
      }
      break;
    case ST_PENALTY_KICK_BALL:
      kick_ball(ai);
      ai->st.state = ST_PENALTY_DONE;
      break;
    case ST_PENALTY_DONE:
      BT_all_stop(0);
      break;
    default:
      fprintf(stderr, "Unknown PENALTY state: %d\n", state);
      ai->st.state = ST_PENALTY_ROTATE_TO_BALL;
      break;
  }
}

static void chase_mode(struct RoboAI *ai, struct blob *blobs) {
}

// TODOO: implement the four functions below
void rotate_to_blob(struct RoboAI *ai, double smx, double smy) {
  // we always rotate to the ball in penalty; keep signature for symmetry
  // fast, blocking snap using gyro
  quick_face_to_ball(ai, smx, smy);
  // non-blocking fallback (if quick_face was within threshold it returns immediately)
  // nothing else needed here
}

void move_to_blob(struct RoboAI *ai, double smx, double smy) {
    // frame-driven PD approach; stops itself when close
    approach_to_ball(ai, smx, smy);
}

void align_to_goal_with_ball(struct RoboAI *ai, double smx, double smy) {
    // ensure we keep the ball centered while we drift into a "behind the ball" pose
    if (!is_facing_ball(ai, smx, smy)) {
        // small corrective snap toward the ball
        quick_face_to_ball(ai, smx, smy);
        return;
    }

    if (!is_aligned_to_goal_for_shot(ai)) {
        // Not behind or not well aligned.
        // Simple heuristic: circle slightly around the ball toward the required side.
        // Positive step if we need to move "upfield", negative otherwise.
        double step = (ai->st.side == 0) ? +12.0 : -12.0; // tweak
        rotate_step_blocking(step);
        // Then take a tiny approach step to settle the arc
        approach_to_ball(ai, smx, smy);
        return;
    }

    // At this point we are behind and oriented; do nothing here.
}

// blocking
void kick_ball(struct RoboAI *ai)
{
    // temporary simple kick logic - drive forward fast for 1 second
    // replace with kick motor control if available
    BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 80, 80);
    sleep(1);
    BT_all_stop(0);
}

// TODOO: need functions to check status (i.e. facing ball, close to ball, aligned to goal)
// at top of file


/////////////////////////////////////////////////////////////////////////////////
// temporary logic
// without test yet !
// 有八百个参数可能要调
/////////////////////////////////////////////////////////////////////////////////

// 这个用作计算机器人当前朝向和球的角度差的helper func
// 返回值是度数(degrees)，正负表示方向, normalized to [-180, 180]
// smx, smy 是机器人的运动向量，不要用ai里面的，传入一个固定的最新的
double compute_angle_error_to_ball(struct RoboAI *ai, double smx, double smy)
{
    if (!ai || !ai->st.self || !ai->st.ball) return NAN;

    // position deltas
    double dx = ai->st.ball->cx - ai->st.self->cx;
    double dy = ai->st.ball->cy - ai->st.self->cy;
    double ang_to_ball = atan2(dy, dx);

    // normalize ball direction vector
    double bn = sqrt(dx*dx + dy*dy);
    if (bn < 1e-3) return NAN;
    double bnx = bx / bn;
    double bny = by / bn;

    double hdx = ai->st.sdx;
    double hdy = ai->st.sdy;
  // use motion vector to disambiguate heading direction
  // if dot product < 0, reverse heading direction
    double dot_heading_motion = hdx*smx + hdy*smy;  // 身体 vs 运动方向
    double dot_motion_ball    = smx*bnx + smy*bny;  // 运动 vs 球方向
    if (dot_heading_motion < 0 && dot_motion_ball > 0) {
        hdx = -hdx;
        hdy = -hdy;
    }

    double ang_bot = atan2(hdy, hdx);

    // angle error
    double ang_err = ang_to_ball - ang_bot;
    // normalized to [-pi, pi]
    while (ang_err >  M_PI) ang_err -= 2*M_PI;
    while (ang_err < -M_PI) ang_err += 2*M_PI;
    // convert to degrees
    double ang_err_deg = ang_err * (180.0 / M_PI);
    return ang_err_deg;
}

// 计算机器人和球的距离误差 的helper func
// target_dist 是目标距离(e.g. 距离球40)， dist_err返回当前距离和目标距离的差值， d_dist返回距离的变化率
// 返回当前距离
double compute_distance_error(struct RoboAI *ai,
                              double target_dist,
                              double *dist_err,
                              double *d_dist)
{
    if (!ai || !ai->st.self || !ai->st.ball) return NAN;

    // current distance 
    double dx = ai->st.ball->cx - ai->st.self->cx;
    double dy = ai->st.ball->cy - ai->st.self->cy;
    double dist = hypot(dx, dy);

    // prev distance
    double old_dx = ai->st.old_bcx - ai->st.old_scx;
    double old_dy = ai->st.old_bcy - ai->st.old_scy;
    double old_dist = hypot(old_dx, old_dy);

    // distance difference 
    if (dist_err) *dist_err = dist - target_dist;  // distance error to target
    if (d_dist)   *d_dist   = old_dist - dist;  // distance change rate

    return dist;
}

// 调用前后检查角度！ 如果面向球角度差小于10度则 --> 进入下一个state
// 这是使用gryo的阻塞的！快速转向函数
// 调用后机器人应当面向球, 但此时的机器人位置可能有偏差(并不是精准朝向球的位置)（不管，依靠approach的pd调整！）
void quick_face_to_ball(struct RoboAI *ai, double smx, double smy)
{
    if (ai->st.ball == NULL || ai->st.self == NULL) return;

    // init current gyro reading
    int gyro_angle = 0, gyro_rate = 0;
    BT_read_gyro(GYRO_PORT, 1, &gyro_angle, &gyro_rate);
    double curr_deg = (double)gyro_angle;

    // compute angle error to ball
    double ang_err_deg = compute_angle_error_to_ball(ai, smx, smy);
    if (isnan(ang_err_deg)) return;
    if (fabs(ang_err_deg) < ALIGN_THRESH_DEG) return; 

    double target_deg = curr_deg + ang_err_deg;
    const double THRESH = ALIGN_THRESH_DEG;
    const double SPEED = 12.0;

    // blocking turn to target using gyro
    int rotate_flag = -1; // -1: not rotating, 0: to right , 1: to left
    while (1)
    {
        BT_read_gyro(GYRO_PORT, 0, &gyro_angle, &gyro_rate);
        curr_deg = (double)gyro_angle;
        double err = target_deg - curr_deg;
        while (err > 180.0) err -= 360.0;
        while (err < -180.0) err += 360.0;

        if (fabs(err) < THRESH)
        {
            fprintf(stderr, "quick_face_to_ball: aligned to ball within %.2f degrees\n", THRESH);
            BT_motor_port_stop(LEFT_MOTOR, 1);
            BT_motor_port_stop(RIGHT_MOTOR, 1);
            break;
        }
        if (err > 0 && rotate_flag != 0)
        {
            rotate_flag = 0;
            BT_drive(LEFT_MOTOR, RIGHT_MOTOR, (char)(-SPEED*1.1), (char)(SPEED));  // turn right
            fprintf(stderr, "quick_face_to_ball: turning right with angle %.2f and target angle %.2f\n", curr_deg, target_deg);
        }
        else if (err < 0 && rotate_flag != 1)
        {
            rotate_flag = 1;
            BT_drive(LEFT_MOTOR, RIGHT_MOTOR, (char)(SPEED*1.1), (char)(-SPEED));  // turn left
            fprintf(stderr, "quick_face_to_ball: turning left with angle %.2f and target angle %.2f\n", curr_deg, target_deg);
        }
        usleep(10000); // 10ms
    }
    BT_motor_port_stop(LEFT_MOTOR, 1);
    BT_motor_port_stop(RIGHT_MOTOR, 1);
}

// assume 已经朝向了球， 如果没有朝向球（角度差 > ?? 12度 ）返回上一个state(使用gryo 旋转)
// 先init gryo sensor 在第一次调用前
// non- blocking & frame - driven
// 根据机器人和球的位置动态调整左右motor，是机器人可以更精准地接近球
void approach_to_ball(struct RoboAI *ai, double smx, double smy)
{
    if (!ai || !ai->st.self || !ai->st.ball) return;

    // angle error to ball as P term
    double ang_err = compute_angle_error_to_ball(ai, smx, smy);
    if (isnan(ang_err)) return;

    // rate of angle change from gyro as D term
    int g_angle = 0, g_rate = 0;
    BT_read_gyro(GYRO_PORT, 0, &g_angle, &g_rate);
    double gyro_rate_scaled = ((double)g_rate) / 60.0; // scale 值要调，不确定要不要

    // // use static variable to store previous angle error for D term image自身的d项，有需要再加吧
    // static double prev_ang_err = 0.0;
    // double d_err_vis = ang_err - prev_ang_err;
    // prev_ang_err = ang_err;

    // turn PD control
    const double Kp_turn = 1.6; // 要调参
    const double Kd_g = 7.5;// 要调参
   // const double Kd_v = 0.0;// 要调参
    double turn = Kp_turn * ang_err
                - Kd_g * gyro_rate_scaled;
               // + Kd_v * d_err_vis; // pd
    // turn limits
    if (turn > 12) turn = 12;
    if (turn < -12) turn = -12;

    // distance to ball
    double target_dist = 40.0;  // target distance to ball // 要调参
    double dist_err = 0.0, d_dist = 0.0;
    double dist = compute_distance_error(ai, target_dist, &dist_err, &d_dist);

    // forward PD control --> 接近时减速
    const double Kp_fwd = 0.3; // 要调参
    const double Kd_fwd = 1.2;// 要调参
    double forward_speed = Kp_fwd * dist_err - Kd_fwd * d_dist; // pd

    // speed limits
    if (forward_speed > 30) forward_speed = 30;
    if (forward_speed < 15) forward_speed = 15;

    // compute left/right motor speeds
    double left  = (forward_speed - turn) * 1.1; // 左轮稍微快一点补偿左右轮偏差， 补偿偏差的参数要调！
    double right = forward_speed + turn;

    // deadband - ensure minimum speed to overcome friction
    if (fabs(left)  < 8) left  = (left>=0?8:-8);
    if (fabs(right) < 8) right = (right>=0?8:-8);

    // stop condition
    // 可以之后增加连续停止的判定，防止误停？
    if (dist < target_dist + 5.0) {
        BT_all_stop(0);
        return;
    }

    BT_drive(LEFT_MOTOR, RIGHT_MOTOR, left, right);
}

/// 使机器人面向对方球门
// blocking！
void rotate_to_goal(struct RoboAI *ai)
{
    if (!ai || !ai->st.self) return;
    struct blob *self = ai->st.self;

    // normalize to unit vector
    double sdx = self->dx;
    double sdy = self->dy;
    double s_norm = sqrt(sdx*sdx + sdy*sdy);
    if (s_norm < 1e-3) return;
    sdx /= s_norm; sdy /= s_norm;

    // compute current field direction in degrees
    double curr_field_deg = atan2(sdy, sdx) * 180.0 / M_PI;
    if (curr_field_deg < 0) curr_field_deg += 360.0;

    // compute target field direction based on side (0 for left or 180 for right)
    double target_field_deg = (ai->st.side == 0) ? 180.0 : 0.0;

    // compute angle difference relative to field direction and normalize to [-180, 180]
    double delta_field = target_field_deg - curr_field_deg;
    while (delta_field > 180) delta_field -= 360;
    while (delta_field < -180) delta_field += 360;

    // init gyro reading and use our computed angle diff
    int gyro_angle = 0, gyro_rate = 0;
    BT_read_gyro(GYRO_PORT, 1, &gyro_angle, &gyro_rate);  // reset // 最好不要一直reset，在最开始reset// todo
    double gyro_start = (double)gyro_angle;
    double target_deg_gyro = gyro_start + delta_field;

    
    const double THRESH = 10.0;
    const double SPEED  = 30.0;
    // blocking turn to target using gyro
    while (1)
    {
        BT_read_gyro(GYRO_PORT, 0, &gyro_angle, &gyro_rate);
        double curr_deg = (double)gyro_angle;
        double err = target_deg_gyro - curr_deg;
        while (err > 180.0) err -= 360.0;
        while (err < -180.0) err += 360.0;

        if (fabs(err) < THRESH) {
            BT_all_stop(0);
            break;
        }

        if (err > 0)
            BT_drive(LEFT_MOTOR, RIGHT_MOTOR, -SPEED * 1.1, SPEED);  
        else
            BT_drive(LEFT_MOTOR, RIGHT_MOTOR, SPEED * 1.1, -SPEED ); 

        usleep(10000);
    }
    BT_all_stop(0);
}

// blocking rotate by step_deg degrees 旋转固定角度
// positive for CCW, negative for CW
void rotate_step_blocking(double step_deg)
{
    int gyro_angle = 0, gyro_rate = 0;
    BT_read_gyro(GYRO_PORT, 1, &gyro_angle, &gyro_rate); // reset gryo
    double curr_deg = (double)gyro_angle;

    double target_deg = curr_deg + step_deg;   
    const double THRESH = 2.0;                 // 可调
    const double SPEED = 30.0;       // 可调

    

    while (1)
    {
        BT_read_gyro(GYRO_PORT, 0, &gyro_angle, &gyro_rate);
        curr_deg = (double)gyro_angle;

        double err = target_deg - curr_deg;
        while (err > 180.0) err -= 360.0;
        while (err < -180.0) err += 360.0;

        if (fabs(err) < THRESH) {
            BT_all_stop(0);
            break;
        }

        if (err > 0)
            BT_drive(LEFT_MOTOR, RIGHT_MOTOR, (char)(-SPEED), (char)(SPEED)); // 左
        else
            BT_drive(LEFT_MOTOR, RIGHT_MOTOR, (char)(SPEED), (char)(-SPEED)); // 右

        usleep(10000); // 10 ms
    }

    BT_all_stop(0);
}

// 每帧调用
// 如果没有面向球，则朝球的方向旋转10度（可能的step degreee）
void chase_rotate(struct RoboAI *ai, double smx, double smy)
{
    if (!ai || !ai->st.self || !ai->st.ball) return;

    // compute angle error to ball
    double ang_err_deg = compute_angle_error_to_ball(ai, smx, smy);
    if (isnan(ang_err_deg)) return;

    //参数设定 --> 可调
    const double ALIGN_THRESH = 8.0;  
   // const double STEP_DEG = 30.0; // 先尝试固定step 吧
    const double STEP_MAX = 30.0;      // 最大步长
    const double STEP_MIN = 5.0;       // 最小步长

    // P 根据ang difference 控制旋转步长
    // 可pd？
    const double Kp = 0.4;            // 比例系数（角度越大旋转越多）
    double step = Kp * ang_err_deg;   // P 控制输出
    // limit step size
    if (step > STEP_MAX) step = STEP_MAX;
    if (step < -STEP_MAX) step = -STEP_MAX;
    if (fabs(step) < STEP_MIN) step = (step > 0 ? STEP_MIN : -STEP_MIN); // 最小步长

    // 如果需要旋转，则旋转一个step
    if (fabs(ang_err_deg) > ALIGN_THRESH)
    {
        // 旋转方向！朝球的方向旋转
        // double step = (ang_err_deg > 0 ? STEP_DEG : -STEP_DEG);

        // this is blocking！
        rotate_step_blocking(step);
    }
    else
    {
       
        BT_all_stop(0);
    }
}