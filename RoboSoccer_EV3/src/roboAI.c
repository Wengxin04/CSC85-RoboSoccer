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

#include "roboAI.h" // <--- Look at this header file!
#include <math.h>
#include <unistd.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// include imagecapture/imageCapture.h to get access to the blob data structure
#include "imagecapture/imageCapture.h"

#include <stdbool.h>

extern int sx; // Get access to the image size from the imageCapture module
extern int sy;
int laggy = 0;

// global variable for rotating
// rotate_flag: -1 not rotating, 0 rotating right, 1 rotating leftint
// rotate_flag = -1; // global variable to indicate rotation statusint
// rotating_angle = 0; // global variable to store the angle rotated from
// gryodouble target_angle = 0.0; // global variable to store the target angle
// from angle difference computation
int rotate_flag = -1;   // global variable to indicate rotation status
int rotating_angle = 0; // global variable to store the angle rotated from gry
double target_angle = 0.0; // global variable to store the target angle from
                           // angle difference computation
int move_flag =
    -2; // global variable to indicate move status, start at -2 (not moving)

int edge_flag = -1; // global variable to indicate edge attack status

int backing_count = 0;

////////////////////////////////////
// Denosing data
////////////////////////////////////

struct TrackingHistory trackHist = {0};

// corrects direction vector
void correct_direction(struct BlobHistory *h) {
  if (h->count < 2)
    return; // need at least 2 samples

  double dot = h->dx[0] * h->dx[1] + h->dy[0] * h->dy[1];
  double mag0 = sqrt(h->dx[0] * h->dx[0] + h->dy[0] * h->dy[0]);
  double mag1 = sqrt(h->dx[1] * h->dx[1] + h->dy[1] * h->dy[1]);
  if (mag0 == 0 || mag1 == 0)
    return;

  double cos_angle = dot / (mag0 * mag1);
  if (cos_angle < 0) { // angle > 90 deg
    // fprintf(stderr, "correcting dir? %f %f\n", h->dx[0], h->dy[0]);
    h->dx[0] *= -1;
    h->dy[0] *= -1;
  }
}

void update_blob_history(struct BlobHistory *h, struct blob *b) {

  if (b != NULL) { // ball detected
    // Reset missed-frame counter
    h->missed_frames = 0;
    h->is_active = 1;

    // Shift old history
    for (int i = HISTORY_LEN - 1; i > 0; --i) {
      h->cx[i] = h->cx[i - 1];
      h->cy[i] = h->cy[i - 1];
      h->vx[i] = h->vx[i - 1];
      h->vy[i] = h->vy[i - 1];
      h->mx[i] = h->mx[i - 1];
      h->my[i] = h->my[i - 1];
      h->dx[i] = h->dx[i - 1];
      h->dy[i] = h->dy[i - 1];
    }

    // Store new sample
    h->cx[0] = b->cx;
    h->cy[0] = b->cy;

    if (b->vx == 0.0 || b->vy == 0.0) {
      // if 0.0, make it so that the entire history is now invalid
      for (int i = h->count - 1; i >= 0; --i) {
        h->mx[i] = 0.0;
        h->my[i] = 0.0;
      }
    } else {
      h->mx[0] = b->mx;
      h->my[0] = b->my;
    }

    h->vx[0] = b->vx;
    h->vy[0] = b->vy;
    h->dx[0] = b->dx;
    h->dy[0] = b->dy;

    if (h->count < HISTORY_LEN)
      h->count++;

    correct_direction(h);
  } else {
    // Blob missing this frame
    h->missed_frames++;
    if (h->missed_frames > MAX_MISSED_FRAMES) {
      h->is_active = 0; // officially lost
      h->count = 0;     // optionally clear history
      for (int i = 0; i < HISTORY_LEN; ++i) {
        h->cx[i] = 0.0;
        h->cy[i] = 0.0;
        h->vx[i] = 0.0;
        h->vy[i] = 0.0;
        h->mx[i] = 0.0;
        h->my[i] = 0.0;
        h->dx[i] = 0.0;
        h->dy[i] = 0.0;
      }
    }
  }
}

// Exponential smoothing denoiser
int denoise_exp(struct BlobHistory *h, double *cx, double *cy, double *vx,
                double *vy, double *dx, double *dy, double *mx, double *my) {

  const double alpha = 0.5; // smoothing factor
  if (h->missed_frames > MAX_MISSED_FRAMES || h->count == 0)
    return -1; // blob lost
  if (cx == NULL || cy == NULL || vx == NULL || vy == NULL || dx == NULL ||
      dy == NULL || mx == NULL || my == NULL)
    return -1; // invalid pointers

  // Start with oldest valid sample
  int start = h->count - 1;
  double scx = h->cx[start];
  double scy = h->cy[start];
  double svx = h->vx[start];
  double svy = h->vy[start];
  double sdx = h->dx[start];
  double sdy = h->dy[start];
  double smx = h->mx[start];
  double smy = h->my[start];

  // Iteratively apply EWMA from older to newer
  for (int i = start - 1; i >= 0; --i) {
    scx = alpha * h->cx[i] + (1 - alpha) * scx;
    scy = alpha * h->cy[i] + (1 - alpha) * scy;
    svx = alpha * h->vx[i] + (1 - alpha) * svx;
    svy = alpha * h->vy[i] + (1 - alpha) * svy;
    sdx = alpha * h->dx[i] + (1 - alpha) * sdx;
    sdy = alpha * h->dy[i] + (1 - alpha) * sdy;
    smx = alpha * h->mx[i] + (1 - alpha) * smx;
    smy = alpha * h->my[i] + (1 - alpha) * smy;
  }

  // printf("Noised: cx=%.2f, cy=%.2f, vx=%.2f, vy=%.2f, dx=%.2f, dy=%.2f\n",
  //    *cx, *cy, *vx, *vy, *dx, *dy);
  // printf("Denoised: cx=%.2f, cy=%.2f, vx=%.2f, vy=%.2f, dx=%.2f, dy=%.2f\n",
  //        scx, scy, svx, svy, sdx, sdy);

  *cx = scx;
  *cy = scy;
  *vx = svx;
  *vy = svy;
  *dx = sdx;
  *dy = sdy;
  *mx = smx;
  *my = smy;

  return 1; // valid smoothed output
}

////////////////////////////////////
// End of denoising data
////////////////////////////////////

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
struct displayList *addPoint(struct displayList *head, int x, int y, double R,
                             double G, double B) {
  struct displayList *newNode;
  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL) {
    fprintf(stderr, "addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type = 0;
  newNode->x1 = x;
  newNode->y1 = y;
  newNode->x2 = -1;
  newNode->y2 = -1;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;

  newNode->next = head;
  return (newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2,
                            int y2, double R, double G, double B) {
  struct displayList *newNode;
  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL) {
    fprintf(stderr, "addLine(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x1;
  newNode->y1 = y1;
  newNode->x2 = x2;
  newNode->y2 = y2;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  return (newNode);
}

struct displayList *addVector(struct displayList *head, int x1, int y1,
                              double dx, double dy, int length, double R,
                              double G, double B) {
  struct displayList *newNode;
  double l;

  l = sqrt((dx * dx) + (dy * dy));
  dx = dx / l;
  dy = dy / l;

  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL) {
    fprintf(stderr, "addVector(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x1;
  newNode->y1 = y1;
  newNode->x2 = x1 + (length * dx);
  newNode->y2 = y1 + (length * dy);
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  return (newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length,
                             double R, double G, double B) {
  struct displayList *newNode;
  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL) {
    fprintf(stderr, "addLine(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x - length;
  newNode->y1 = y;
  newNode->x2 = x + length;
  newNode->y2 = y;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  head = newNode;

  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL) {
    fprintf(stderr, "addLine(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x;
  newNode->y1 = y - length;
  newNode->x2 = x;
  newNode->y2 = y + length;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  return (newNode);
}

struct displayList *clearDP(struct displayList *head) {
  struct displayList *q;
  while (head) {
    q = head->next;
    free(head);
    head = q;
  }
  return (NULL);
}

/**************************************************************
 * End of Display List Management
 * ***********************************************************/

/*************************************************************
 * Blob identification and tracking
 * ***********************************************************/

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col) {
  /////////////////////////////////////////////////////////////////////////////
  // This function looks for and identifies a blob with the specified colour.
  // It uses the hue and saturation values computed for each blob and tries to
  // select the blob that is most like the expected colour (red, green, or blue)
  //
  // If you find that tracking of blobs is not working as well as you'd like,
  // you can try to improve the matching criteria used in this function.
  // Remember you also have access to shape data and orientation axes for blobs.
  //
  // Inputs: The robot's AI data structure, a list of blobs, and a colour
  // target: Colour parameter: 0 -> Blue bot
  //                   1 -> Red bot
  //                   2 -> Yellow ball
  // Returns: Pointer to the blob with the desired colour, or NULL if no such
  // 	     blob can be found.
  /////////////////////////////////////////////////////////////////////////////

  struct blob *p, *fnd;
  double vr_x, vr_y, maxfit, mincos, dp;
  double vb_x, vb_y, fit;
  double maxsize = 0;
  double maxgray;
  int grayness;
  int i;
  static double Mh[4] = {-1, -1, -1, -1};
  static double mx0, my0, mx1, my1, mx2, my2;
  FILE *f;

  // Import calibration data from file - this will contain the colour values
  // selected by the user in the U.I.
  if (Mh[0] == -1) {
    f = fopen("colours.dat", "r");
    if (f != NULL) {
      fread(&Mh[0], 4 * sizeof(double), 1, f);
      fclose(f);
      mx0 = cos(Mh[0]);
      my0 = sin(Mh[0]);
      mx1 = cos(Mh[1]);
      my1 = sin(Mh[1]);
      mx2 = cos(Mh[2]);
      my2 = sin(Mh[2]);
    }
  }

  if (Mh[0] == -1) {
    fprintf(
        stderr,
        "roboAI.c :: id_coloured_blob2(): No colour calibration data, can not "
        "ID blobs. Please capture colour calibration data on the U.I. first\n");
    return NULL;
  }

  maxfit = .025; // Minimum fitness threshold
  mincos = .9;   // Threshold on colour angle similarity
  maxgray = .25; // Maximum allowed difference in colour
                 // to be considered gray-ish (as a percentage
                 // of intensity)

  // The reference colours here are in the HSV colourspace, we look at the hue
  // component, which is a defined within a colour-wheel that contains all
  // possible colours. Hence, the hue component is a value in [0 360] degrees,
  // or [0 2*pi] radians, indicating the colour's location on the colour wheel.
  // If we want to detect a different colour, all we need to do is figure out
  // its location in the colour wheel and then set the angles below (in radians)
  // to that colour's angle within the wheel. For reference: Red is at 0
  // degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  // Agent IDs are as follows: 0 : blue bot,  1 : red bot, 2 : yellow ball
  if (col == 0) {
    vr_x = mx0;
    vr_y = my0;
  } else if (col == 1) {
    vr_x = mx1;
    vr_y = my1;
  } else if (col == 2) {
    vr_x = mx2;
    vr_y = my2;
  }

  // In what follows, colours are represented by a unit-length vector in the
  // direction of the hue for that colour. Similarity between two colours (e.g.
  // a reference above, and a pixel's or blob's colour) is measured as the
  // dot-product between the corresponding colour vectors. If the dot product is
  // 1 the colours are identical (their vectors perfectly aligned), from there,
  // the dot product decreases as the colour vectors start to point in different
  // directions. Two colours that are opposite will result in a dot product of
  // -1.

  p = blobs;
  while (p != NULL) {
    if (p->size > maxsize)
      maxsize = p->size;
    p = p->next;
  }

  p = blobs;
  fnd = NULL;
  while (p != NULL) {
    // Normalization and range extension
    vb_x = cos(p->H);
    vb_y = sin(p->H);

    dp = (vb_x * vr_x) +
         (vb_y * vr_y); // Dot product between the reference color vector, and
                        // the blob's color vector.

    fit =
        dp * p->S * p->S *
        (p->size / maxsize); // <<< --- This is the critical matching criterion.
                             // * THe dot product with the reference direction,
                             // * Saturation squared
                             // * And blob size (in pixels, not from bounding
                             // box) You can try to fine tune this if you feel
                             // you can improve tracking stability by changing
                             // this fitness computation

    // Check for a gray-ish blob - they tend to give trouble
    grayness = 0;
    if (fabs(p->R - p->G) / p->R < maxgray &&
        fabs(p->R - p->G) / p->G < maxgray &&
        fabs(p->R - p->B) / p->R < maxgray &&
        fabs(p->R - p->B) / p->B < maxgray &&
        fabs(p->G - p->B) / p->G < maxgray &&
        fabs(p->G - p->B) / p->B < maxgray)
      grayness = 1;

    if (fit > maxfit && dp > mincos && grayness == 0) {
      fnd = p;
      maxfit = fit;
    }

    p = p->next;
  }

  return (fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs) {
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
  double mg, vx, vy, pink, doff, dmin, dmax, adj;

  // Reset ID flags and agent blob pointers
  ai->st.ballID = 0;
  ai->st.selfID = 0;
  ai->st.oppID = 0;
  ai->st.ball = NULL; // Be sure you check these are not NULL before
  ai->st.self = NULL; // trying to access data for the ball/self/opponent!
  ai->st.opp = NULL;

  // Find the ball
  p = id_coloured_blob2(ai, blobs, 2);
  if (p) {
    ai->st.ball = p;   // New pointer to ball
    ai->st.ballID = 1; // Set ID flag for ball (we found it!)
    ai->st.bvx =
        p->cx -
        ai->st
            .old_bcx; // Update ball velocity in ai structure and blob structure
    ai->st.bvy = p->cy - ai->st.old_bcy;
    ai->st.ball->vx = ai->st.bvx;
    ai->st.ball->vy = ai->st.bvy;
    ai->st.bdx = p->dx;
    ai->st.bdy = p->dy;

    ai->st.old_bcx = p->cx; // Update old position for next frame's computation
    ai->st.old_bcy = p->cy;
    ai->st.ball->idtype = 3;

    vx = ai->st.bvx; // Compute motion direction (normalized motion vector)
    vy = ai->st.bvy;
    mg = sqrt((vx * vx) + (vy * vy));
    if (mg > NOISE_VAR) // Update heading vector if meaningful motion detected
    {
      vx /= mg;
      vy /= mg;
      ai->st.bmx = vx;
      ai->st.bmy = vy;
    } else {
      ai->st.bmx = 0;
      ai->st.bmy = 0;
    }
    ai->st.ball->mx = ai->st.bmx;
    ai->st.ball->my = ai->st.bmy;
  } else {
    ai->st.ball = NULL;
  }

  // ID our bot - the colour is set from commane line, 0=Blue, 1=Red
  p = id_coloured_blob2(ai, blobs, ai->st.botCol);
  if (p != NULL && p != ai->st.ball) {
    ai->st.self = p; // Update pointer to self-blob
    ai->st.selfID = 1;
    ai->st.svx = p->cx - ai->st.old_scx;
    ai->st.svy = p->cy - ai->st.old_scy;
    ai->st.self->vx = ai->st.svx;
    ai->st.self->vy = ai->st.svy;
    ai->st.sdx = p->dx;
    ai->st.sdy = p->dy;

    vx = ai->st.svx;
    vy = ai->st.svy;
    mg = sqrt((vx * vx) + (vy * vy));
    //  printf("--->    Track agents(): d=[%lf, %lf], [x,y]=[%3.3lf, %3.3lf],
    //  old=[%3.3lf, %3.3lf], v=[%2.3lf, %2.3lf], motion=[%2.3lf,
    //  %2.3lf]\n",ai->st.sdx,ai->st.sdy,ai->st.self->cx,ai->st.self->cy,ai->st.old_scx,ai->st.old_scy,vx,vy,vx/mg,vy/mg);
    if (mg > NOISE_VAR) {
      vx /= mg;
      vy /= mg;
      ai->st.smx = vx;
      ai->st.smy = vy;
    } else {
      ai->st.smx = 0;
      ai->st.smy = 0;
    }
    ai->st.self->mx = ai->st.smx;
    ai->st.self->my = ai->st.smy;
    ai->st.old_scx = p->cx;
    ai->st.old_scy = p->cy;
    ai->st.self->idtype = 1;
  } else
    ai->st.self = NULL;

  // ID our opponent - whatever colour is not botCol
  if (ai->st.botCol == 0)
    p = id_coloured_blob2(ai, blobs, 1);
  else
    p = id_coloured_blob2(ai, blobs, 0);
  if (p != NULL && p != ai->st.ball && p != ai->st.self) {
    ai->st.opp = p;
    ai->st.oppID = 1;
    ai->st.ovx = p->cx - ai->st.old_ocx;
    ai->st.ovy = p->cy - ai->st.old_ocy;
    ai->st.opp->vx = ai->st.ovx;
    ai->st.opp->vy = ai->st.ovy;
    ai->st.odx = p->dx;
    ai->st.ody = p->dy;

    ai->st.old_ocx = p->cx;
    ai->st.old_ocy = p->cy;
    ai->st.opp->idtype = 2;

    vx = ai->st.ovx;
    vy = ai->st.ovy;
    mg = sqrt((vx * vx) + (vy * vy));
    if (mg > NOISE_VAR) {
      vx /= mg;
      vy /= mg;
      ai->st.omx = vx;
      ai->st.omy = vy;
    } else {
      ai->st.omx = 0;
      ai->st.omy = 0;
    }
    ai->st.opp->mx = ai->st.omx;
    ai->st.opp->my = ai->st.omy;
  } else
    ai->st.opp = NULL;
}

void id_bot(struct RoboAI *ai, struct blob *blobs) {
  ///////////////////////////////////////////////////////////////////////////////
  // ** DO NOT CHANGE THIS FUNCTION **
  // This routine calls track_agents() to identify the blobs corresponding to
  // the robots and the ball. It commands the bot to move forward slowly so
  // heading can be established from blob-tracking.
  //
  // NOTE 1: All heading estimates, velocity vectors, position, and orientation
  //         are noisy. Remember what you have learned about noise management.
  //
  // NOTE 2: Heading and velocity estimates are not valid while the robot is
  //         rotating in place (and the final heading vector is not valid
  //         either). To re-establish heading, forward/backward motion is
  //         needed.
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
  static double stepID = 0;
  static double oldX, oldY;
  double frame_inc = 1.0 / 5.0;
  double dist;

  track_agents(ai, blobs); // Call the tracking function to find each agent

  BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30,
           30); // Start forward motion to establish heading
                // Will move for a few frames.

  if (ai->st.selfID == 1 && ai->st.self != NULL)
    fprintf(stderr, "Successfully identified self blob at (%f,%f)\n",
            ai->st.self->cx, ai->st.self->cy);
  if (ai->st.oppID == 1 && ai->st.opp != NULL)
    fprintf(stderr, "Successfully identified opponent blob at (%f,%f)\n",
            ai->st.opp->cx, ai->st.opp->cy);
  if (ai->st.ballID == 1 && ai->st.ball != NULL)
    fprintf(stderr, "Successfully identified ball blob at (%f,%f)\n",
            ai->st.ball->cx, ai->st.ball->cy);

  stepID += frame_inc;
  if (stepID >= 1 &&
      ai->st.selfID == 1) // Stop after a suitable number of frames.
  {
    ai->st.state += 1;
    stepID = 0;
    BT_all_stop(0);
  } else if (stepID >= 1)
    stepID = 0;

  // At each point, each agent currently in the field should have been
  // identified.
  return;
}
/*********************************************************************************
 * End of blob ID and tracking code
 * ******************************************************************************/

/*********************************************************************************
 * Routine to initialize the AI
 * *******************************************************************************/
int setupAI(int mode, int own_col, struct RoboAI *ai) {
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
    fprintf(stderr, "Standard Robo-Soccer mode requested\n");
    ai->st.state = ST_SOCCER_INIT;
    break;
  case AI_PENALTY:
    fprintf(stderr, "Penalty mode! let's kick it!\n");
    ai->st.state = ST_PENALTY_INIT;
    break;
  case AI_CHASE:
    fprintf(stderr, "Chasing the ball...\n");
    ai->st.state = ST_CHASE_INIT;
    break;
  default:
    fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n",
            mode);
    ai->st.state = ST_SOCCER_INIT;
  }

  BT_all_stop(0);      // Stop bot,
  ai->runAI = AI_main; // and initialize all remaining AI data
  ai->calibrate = AI_calibrate;
  ai->st.ball = NULL;
  ai->st.self = NULL;
  ai->st.opp = NULL;
  ai->st.side = 0;
  ai->st.botCol = own_col;
  ai->st.old_bcx = 0;
  ai->st.old_bcy = 0;
  ai->st.old_scx = 0;
  ai->st.old_scy = 0;
  ai->st.old_ocx = 0;
  ai->st.old_ocy = 0;
  ai->st.bvx = 0;
  ai->st.bvy = 0;
  ai->st.svx = 0;
  ai->st.svy = 0;
  ai->st.ovx = 0;
  ai->st.ovy = 0;
  ai->st.sdx = 0;
  ai->st.sdy = 0;
  ai->st.odx = 0;
  ai->st.ody = 0;
  ai->st.bdx = 0;
  ai->st.bdy = 0;
  ai->st.selfID = 0;
  ai->st.oppID = 0;
  ai->st.ballID = 0;
  ai->DPhead = NULL;
  fprintf(stderr, "Initialized!\n");

  return (1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs) {
  // Basic colour blob tracking loop for calibration of vertical offset
  // See the handout for the sequence of steps needed to achieve calibration.
  // The code here just makes sure the image processing loop is constantly
  // tracking the bots while they're placed in the locations required
  // to do the calibration (i.e. you DON'T need to add anything more
  // in this function).
  track_agents(ai, blobs);
}

/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state) {
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

  static double ux, uy, len, mmx, mmy, tx, ty, x1, y1, x2, y2;
  double angDif;
  char line[1024];
  static int count = 0;
  static double old_dx = 0, old_dy = 0;

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
  if (ai->st.state == ST_SOCCER_INIT || ai->st.state == ST_PENALTY_INIT ||
      ai->st.state == ST_CHASE_INIT) // Initial set up - find own, ball, and opponent blobs
  {
    // Carry out self id process.
    fprintf(stderr, "Initial state, self-id in progress...\n");

    id_bot(ai, blobs);
    if ((ai->st.state % 100) !=
        0) // The id_bot() routine will change the AI state to initial state + 1
    {      // if robot identification is successful.

      if (ai->st.self->cx >= 512)
        ai->st.side = 1;
      else
        ai->st.side = 0; // This sets the side the bot thinks as its own side
                         // 0->left, 1->right
      BT_all_stop(0);

      fprintf(stderr,
              "Self-ID complete. Current position: (%f,%f), current heading: "
              "[%f, %f], blob direction=[%f, %f], AI state=%d\n",
              ai->st.self->cx, ai->st.self->cy, ai->st.smx, ai->st.smy,
              ai->st.sdx, ai->st.sdy, ai->st.state);
      stored_smx = ai->st.smx;
      stored_smy = ai->st.smy;

      rotate_flag = -1;
      target_angle = 0;
      rotating_angle = 0;

      if (ai->st.self != NULL) {
        // This checks that the motion vector and the blob direction vector
        // are pointing in the same direction. If they are not (the dot product
        // is less than 0) it inverts the blob direction vector so it points
        // in the same direction as the motion vector.
        if (((ai->st.smx * ai->st.sdx) + (ai->st.smy * ai->st.sdy)) < 0) {
          ai->st.self->dx *= -1.0;
          ai->st.self->dy *= -1.0;
          ai->st.sdx *= -1;
          ai->st.sdy *= -1;
        }
        old_dx = ai->st.sdx;
        old_dy = ai->st.sdy;
      }

      if (ai->st.opp != NULL) {
        // Checks motion vector and blob direction for opponent. See above.
        if (((ai->st.omx * ai->st.odx) + (ai->st.omy * ai->st.ody)) < 0) {
          ai->st.opp->dx *= -1;
          ai->st.opp->dy *= -1;
          ai->st.odx *= -1;
          ai->st.ody *= -1;
        }
      }
    }

    // Initialize BotInfo structures

  } else {
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

     Please note that in this function you should add appropriate functions
    below to handle each state's processing, and the code here should mostly
    deal with state transitions and with calling the appropriate function based
    on what the bot is supposed to be doing.
    *****************************************************************************/
    fprintf(stderr, "Just trackin with state: %d!\n",
         ai->st.state);
    track_agents(ai, blobs);

    // get current state and call appropriate function
    int state = ai->st.state;

    if (state >= 0 && state < 100) {
      soccer_mode(ai, &stored_smx, &stored_smy);
    } else if (state >= 100 && state < 200) {
      penalty_mode(ai, &stored_smx, &stored_smy);
    } else if (state >= 200 && state < 300) {
      chase_mode(ai, &stored_smx, &stored_smy);
    } else {
      fprintf(stderr, "Unknown AI state: %d\n", state);
    }
  }

  /**********************************************************************************
   TO DO:

   Add the rest of your game playing logic below. Create appropriate functions
  to handle different states (be sure to name the states/functions in a
  meaningful way), and do any processing required in the space below.

   AI_main() should *NOT* do any heavy lifting. It should only call appropriate
   functions based on the current AI state.

   You will lose marks if AI_main() is cluttered with code that doesn't belong
   there.
  **********************************************************************************/
}

// Behaviour definitions
#define ESCAPE_BEHAVIOR 1
#define NORMAL_ATTACK_BEHAVIOR 2
#define EDGE_ATTACK_BEHAVIOR 3
#define DEFEND_BEHAVIOR 4
#define BEHAVIOR_NOT_CHANGE 0
#define BALL_LOST_BEHAVIOR 5

// Tuning parameters
enum {
  FACE_THRESH_DEG = 20,        // degrees to consider "facing" target
  TARGET_BALL_DIST = 120,      // pixels to consider "close" to ball
  TARGET_TARGET_DIST = 100,     // pixels to consider "close" to target
  DEFENSE_THRESHOLD = 300,     // pixels for opp to ball to consider defense
  OPP_FACE_THRESH_DEG = 45,    // degrees for opp to ball/goal to consider defense
  ESCAPE_THRESHOLD = 150,      // pixels for opp to self to consider escape
  ESCAPE_ANGLE_THRESH_DEG = 60, // degrees for opp to self to consider escape
  EDGE_PLAY_THRESHOLD_Y = 150,  // pixels to consider close to side edges
  EDGE_PLAY_THRESHOLD_X = 100,  // pixels to consider close to top/bottom edges
  EDGE_DELTA_GOAL = 200,       // pixels to consider cloese to goal when doing edge play
  DELTA_TO_TARGET = 150,      // pixels to consider "close" to target position
  DELTA_TO_OPP = 300,         // pixels to consider "close" to opponent
  TARGET_DEFENSE_DIST = 200,   // pixels to consider "close" to defense position
};

// return true if facing target within threshold FACE_THRESH_DEG
bool is_facing_target(struct RoboAI *ai, double smx, double smy,
                      double target_cx, double target_cy) {
  double e = compute_angle_error_to_target(ai, smx, smy, target_cx, target_cy);
  // fprintf(stderr, "Angle error to target: %.2f deg\n", e);
  return !isnan(e) && fabs(e) <= FACE_THRESH_DEG;
}

// return true if close enough to ball within TARGET_BALL_DIST
bool is_close_to_ball(struct RoboAI *ai, double ball_cx, double ball_cy) {
  double de = 0, dd = 0;
  double d =
      compute_distance_error(ai, TARGET_BALL_DIST, &de, &dd, ball_cx, ball_cy);
  // fprintf(stderr, "Distance to ball: %.2f px (err %.2f, d %.2f)\n", d, de, dd);
  return !isnan(d) && d <= (TARGET_BALL_DIST);
}

// return true if close enough to target within TARGET_TARGET_DIST
bool is_close_to_target(struct RoboAI *ai, double target_cx, double target_cy) {
  if (!ai || !ai->st.self)
    return false;
  double dx = target_cx - ai->st.self->cx;
  double dy = target_cy - ai->st.self->cy;
  double dist = hypot(dx, dy);
  // fprintf(stderr, "Distance to target: %.2f px\n", dist);
  return dist <= TARGET_TARGET_DIST;
}

// return true if need defense: opp is close to ball (within DEFENSE_THRESHOLD) and facing ball and goal (within OPP_FACE_THRESH_DEG)
bool need_defense(struct RoboAI *ai) {
  double opp_ball_dist =
      compute_opp_distance_to_target(ai, ai->st.ball->cx, ai->st.ball->cy);
  double opp_ball_angle =
      compute_opp_angle_diff_to_target(ai, ai->st.ball->cx, ai->st.ball->cy);
  double gx, gy;
  compute_goal_center1(1 - ai->st.side, &gx, &gy);
  double opp_goal_angle = compute_opp_angle_diff_to_target(ai, gx, gy);
  return fabs(opp_ball_dist) < DEFENSE_THRESHOLD &&
         fabs(opp_ball_angle) < OPP_FACE_THRESH_DEG &&
         fabs(opp_goal_angle) < OPP_FACE_THRESH_DEG;
}

// return true if need escape: opp is close to self (within ESCAPE_THRESHOLD) and facing self (within ESCAPE_ANGLE_THRESH_DEG)
bool need_escape(struct RoboAI *ai, double *smx, double *smy) {
  double dist =
      compute_opp_distance_to_target(ai, ai->st.self->cx, ai->st.self->cy);
  double angle = compute_angle_error_to_target(ai, *smx, *smy, ai->st.opp->cx,
                                               ai->st.opp->cy);
  return fabs(dist) < ESCAPE_THRESHOLD && fabs(angle) < ESCAPE_ANGLE_THRESH_DEG;
}

// return true if need edge play: ball is close to edge but not close to goal
bool need_edge_play(struct RoboAI *ai) {
  double gx, gy;
  compute_goal_center1(1-ai->st.side, &gx, &gy);
  bool close_to_goal = (sqrt((ai->st.ball->cx - gx) * (ai->st.ball->cx - gx) +
                                (ai->st.ball->cy - gy) * (ai->st.ball->cy - gy)) <
                         EDGE_DELTA_GOAL);
  bool close_to_edge = (ai->st.ball->cy < EDGE_PLAY_THRESHOLD_Y ||
          (sy - ai->st.ball->cy) < EDGE_PLAY_THRESHOLD_Y) ||
          (ai->st.ball->cx < EDGE_PLAY_THRESHOLD_X ||
           (sx - ai->st.ball->cx) < EDGE_PLAY_THRESHOLD_X);
  return close_to_edge && !close_to_goal; 
}

/**********************************
 *
 * motion control functions
 *
 ************************************/
// rotate to blob at (target_x, target_y)
void rotate_to_blob(struct RoboAI *ai, double smx, double smy, double target_x,
                    double target_y) {
  // before rotation, initialize
  double angle_delta = 0.0;
  static double prev_ang_err = 0.0;
  static double prev_5_err_ang[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  static double prev_speed = 40.0;
  int g_rate = 0;

  if (rotate_flag == -1) {
    // fprintf(stderr, "Starting rotation to target blob at (%.2f, %.2f)\n",
    //         target_x, target_y);
    target_angle =
        compute_angle_error_to_target(ai, smx, smy, target_x, target_y);
    // init gyro
    int g_angle = 0;
    // init gryo to 0
    BT_read_gyro(GYRO_PORT, 1, &g_angle, &g_rate);
    rotating_angle = g_angle;
    for (int i = 0; i < 5; i++) {
      prev_5_err_ang[i] = angle_delta;
    }
    prev_ang_err = angle_delta;
  } else {
    // this is during rotation
    // read gyro as current rotated angle
    int cur_angle = 0;
    BT_read_gyro(GYRO_PORT, 0, &cur_angle, &g_rate);
    rotating_angle = cur_angle;
  }

  angle_delta = target_angle - rotating_angle;

  while (angle_delta > 180.0)
    angle_delta -= 360.0;
  while (angle_delta < -180.0)
    angle_delta += 360.0;

  bool rotate_limit = fabs(rotating_angle) > 200.0;
  bool left_done = (target_angle < 0) && (angle_delta >= -5.0);
  bool right_done = (target_angle > 0) && (angle_delta <= 5.0);

  // finished rotation
  if (left_done || right_done || rotate_limit) { // within 5 degrees
    // stop
    // fprintf(stderr,
    //         "Rotation to target blob completed. Target angle: %.2f, Rotated "
    //         "angle: %d\n",
    //         target_angle, rotating_angle);
    // BT_motor_port_stop(LEFT_MOTOR, 0);
    // BT_motor_port_stop(RIGHT_MOTOR, 0);
    // reset and correct
    rotate_flag = -2; // reset flag
    rotating_angle = 0;
    prev_ang_err = 0.0;
    for (int i = 0; i < 5; i++) {
      prev_5_err_ang[i] = 0.0;
    }
    prev_speed = 0.0;
    // correct_motion_vector(&ai->st.smx, &ai->st.smy, target_angle);
    // target_angle = 0.0;
    return;
  }

  // D
  double ang_diff = angle_delta - prev_ang_err;
  // ang_diff = g_rate;

  // I
  static int err_index = 0;
  prev_5_err_ang[err_index] = angle_delta;
  err_index = (err_index + 1) % 5;
  double ang_intg = 0.0;
  for (int i = 0; i < 5; i++) {
    ang_intg += prev_5_err_ang[i];
  }

  // turn PID control for angle
  const double Kp_ang = 0.7;
  const double Kd_ang = 2.5;
  const double Ki_ang = 0.1;

  double up_ang = Kp_ang * angle_delta;
  double ud_ang = Kd_ang * ang_diff;
  double ui_ang = Ki_ang * ang_intg;

  double speed = up_ang + ud_ang + ui_ang; // pid

  // turn limits
  if (speed > 100)
    speed = 100;
  if (speed < -100)
    speed = -100;
  if (speed < 25 && speed > 0)
    speed = 25;
  if (speed > -25 && speed < 0)
    speed = -25;

  // prevent sudden speed change
  if (speed > prev_speed + 10) {
    speed = prev_speed + 10;
  } else if (speed < prev_speed - 10) {
    speed = prev_speed - 10;
  }

  // fprintf(stderr,
  //         "angle_delta: %.2f, prev_ang_err: %.2f, up_ang: %.2f, ud_ang: %.2f,
  //         " "ui_ang: %.2f, speed: %.2f\n", angle_delta, prev_ang_err, up_ang,
  //         ud_ang, ui_ang, speed);

  prev_speed = speed;
  prev_ang_err = angle_delta;

  rotate_flag = 0;
  BT_drive(LEFT_MOTOR, RIGHT_MOTOR, speed, -speed);
}

// move to blob at (target_x, target_y) until within target_dist
void move_to_blob(struct RoboAI *ai, double smx, double smy, double target_x,
                  double target_y, double target_dist) {
  if (!ai || !ai->st.self)
    return;

  // P for distance
  double dist_err = 0.0, d_dist = 0.0;
  double dist = compute_distance_error(ai, target_dist, &dist_err, &d_dist,
                                       target_x, target_y);

  static double prev_ang_err = 0.0;
  static double prev_5_err_ang[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  static double prev_dist_err = 0.0;
  static double prev_5_err_dist[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

  // stop condition
  if (dist < target_dist + 5.0) {
    move_flag = -2; // reached target
    // fprintf(stderr,
    //         "Reached target distance to blob. Current distance: %.2f, Target "
    //         "distance: %.2f\n",
    //         dist, target_dist);
    BT_all_stop(0);
    prev_ang_err = 0.0;
    for (int i = 0; i < 5; i++) {
      prev_5_err_ang[i] = 0.0;
      prev_5_err_dist[i] = 0.0;
    }
    prev_dist_err = 0.0;
    return;
  }

  // angle error to ball as P term
  double ang_err =
      compute_angle_error_to_target(ai, smx, smy, target_x, target_y);
  if (isnan(ang_err))
    return;

  ang_err = fmod(ang_err + 180.0, 360.0) - 180.0; // wrap to [-180, 180]

  if (move_flag == -2) {
    // fprintf(stderr,
    //         "Starting move to blob at (%.2f, %.2f) with target distance %.2f\n",
    //         target_x, target_y, target_dist);
    move_flag = 0; // moving
    for (int i = 0; i < 5; i++) {
      prev_5_err_ang[i] = ang_err;
      prev_5_err_dist[i] = dist_err;
    }
    prev_ang_err = ang_err;
    prev_dist_err = dist_err;
  }

  // rate of angle change from gyro as D term
  int g_angle = 0, g_rate = 0;
  BT_read_gyro(GYRO_PORT, 0, &g_angle, &g_rate);
  double gyro_rate_scaled =
      ((double)g_rate) / 60.0;

  // use static variable to store previous angle error for D term
  // D
  double ang_diff = ang_err - prev_ang_err;
  // ang_diff = g_rate;

  // I
  static int err_index = 0;
  prev_5_err_ang[err_index] = ang_err;
  err_index = (err_index + 1) % 5;
  double ang_intg = 0.0;
  for (int i = 0; i < 5; i++) {
    ang_intg += prev_5_err_ang[i];
  }

  // turn PID control for angle
  const double Kp_ang = 1.0;
  const double Kd_ang = 2.5;
  const double Ki_ang = 0.1;

  double up_ang = Kp_ang * ang_err;
  double ud_ang = Kd_ang * ang_diff;
  double ui_ang = Ki_ang * ang_intg;

  double turn = up_ang + ud_ang + ui_ang; // pid
  prev_ang_err = ang_err;

  // D
  double dist_diff = dist_err - prev_dist_err;

  // I
  static int dist_err_index = 0;
  prev_5_err_dist[dist_err_index] = prev_dist_err;
  dist_err_index = (dist_err_index + 1) % 5;
  double dist_intg = 0.0;
  for (int i = 0; i < 5; i++) {
    dist_intg += prev_5_err_dist[i];
  }

  // turn PID control for distance
  const double Kp_dist = 0.6;
  const double Kd_dist = 0.3;
  const double Ki_dist = 0.0;

  double up_dist = Kp_dist * dist_err;
  double ud_dist = Kd_dist * dist_diff;
  double ui_dist = Ki_dist * dist_intg;

  double forward_speed = up_dist + ud_dist + ui_dist; // pid

  prev_dist_err = dist_err;

  if (forward_speed > 100)
    forward_speed = 100;

  int left = (forward_speed +
              turn); // left motor needs more power to maintain straight line
  int right = (forward_speed - turn); // right motor needs less power

  // slow rate to prevent sudden changes
  static int prev_left = 30, prev_right = 30;
  if (left < prev_left - 10)
    left = prev_left - 10;
  if (left > prev_left + 10)
    left = prev_left + 10;
  if (right < prev_right - 10)
    right = prev_right - 10;
  if (right > prev_right + 10)
    right = prev_right + 10;

  // deadband - ensure minimum speed to overcome friction
  if (left < -80)
    left = -80;
  if (left > 80)
    left = 80;
  if (right < -80)
    right = -80;
  if (right > 80)
    right = 80;
  if (left > 0 && left < 20)
    left = 20;
  if (right > 0 && right < 20)
    right = 20;
  if (left < 0 && left > -20)
    left = -20;
  if (right < 0 && right > -20)
    right = -20;

  prev_left = left;
  prev_right = right;

  // fprintf(stderr,"approach_to_target: dist %.2f (err %.2f, d %.2f), fwd %.2f, %.2f,%.2f,turn %.2f, %.2f, %.2f, %.2f, left %d, right %d, ang_err%.2f\n", dist, dist_err, d_dist, forward_speed, up_dist, ud_dist, turn,
  //     up_ang, ud_ang, ui_ang, left, right, ang_err);

  BT_drive(LEFT_MOTOR, RIGHT_MOTOR, left, right);
}

// kick the ball (blocking function)
void kick_ball(struct RoboAI *ai) {
  // rush forward a bit
  fprintf(stderr, "Kicking the ball!\n");
  BT_timed_motor_port_start(RIGHT_MOTOR, 100, 100, 800, 100);
  BT_timed_motor_port_start(LEFT_MOTOR, 100, 100, 800, 100);
  usleep(300 * 1000);
  // kick
  BT_timed_motor_port_start(KICK_MOTOR, 100, 100, 200, 100);
  usleep(800 * 1000);
  BT_timed_motor_port_start(KICK_MOTOR, -100, 100, 200, 100);
  usleep(500 * 1000);
}

// rotate left and kick (blocking function)
// used in edge play to get ball out of corner
void rotate_left_kick(struct RoboAI *ai) {
  // rush forward a bit
  BT_timed_motor_port_start(RIGHT_MOTOR, 70, 50, 800, 50);
  BT_timed_motor_port_start(LEFT_MOTOR, 70, 50, 800, 50);
  usleep(500 * 1000);
  // rotate left
  BT_timed_motor_port_start(RIGHT_MOTOR, 100, 100, 500, 100);
  BT_timed_motor_port_start(LEFT_MOTOR, -100, 100, 500, 100);
  usleep(350 * 1000);
  // kick
  BT_timed_motor_port_start(KICK_MOTOR, 100, 0, 100, 100);
  usleep(600 * 1000);
  BT_timed_motor_port_start(KICK_MOTOR, -80, 100, 200, 100);
  usleep(300 * 1000);
}

// rotate right and kick (blocking function)
// used in edge play to get ball out of corner
void rotate_right_kick(struct RoboAI *ai) {
  // rush forward a bit
  BT_timed_motor_port_start(RIGHT_MOTOR, 100, 50, 800, 50);
  BT_timed_motor_port_start(LEFT_MOTOR, 100, 50, 800, 50);
  usleep(500 * 1000);
  // rotate right
  BT_timed_motor_port_start(RIGHT_MOTOR, -100, 100, 500, 100);
  BT_timed_motor_port_start(LEFT_MOTOR, 100, 100, 500, 100);
  usleep(350 * 1000);
  // kick
  BT_timed_motor_port_start(KICK_MOTOR, 100, 0, 100, 100);
  usleep(600 * 1000);
  BT_timed_motor_port_start(KICK_MOTOR, -80, 100, 200, 100);
  usleep(300 * 1000);
}

void self_not_found_backing(struct RoboAI *ai) {
  static int lost_count = 0;

  if (!ai || backing_count > 3) return;
  if (ai && !ai->st.self){
    fprintf(stderr, "Self lost! backing count %d and lost count %d\n", backing_count, lost_count);
    lost_count++;
  }  
  if (lost_count > 5){
    BT_drive(LEFT_MOTOR, RIGHT_MOTOR, -55, -50); // move backward
    sleep(1);                                    // move for 0.5 second
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    backing_count++;
  }

}

// TODOO: need functions to check status (i.e. facing ball, close to ball,
// aligned to goal) at top of file

/***********************************
 *
 * computations functions
 *
 ************************************/

// a function that computes the gcx, gcy coordinate of the goal center based on
// which side we are on
// this computes the opponent goal center
void compute_goal_center(struct RoboAI *ai, double *gcx, double *gcy) {
  if (!ai || !ai->st.self || !gcx || !gcy)
    return;

  // left goal center is: (0, sy/2)
  // right goal center is: (sx, sy/2)

  int our_side = ai->st.side; // 0 for left, 1 for right
  if (our_side == 0) {
    // left side, so opponent goal is right
    *gcx = sx; // right edge
    *gcy = sy / 2.0;
  } else {
    // right side, so opponent goal is left
    *gcx = 0.0; // left edge
    *gcy = sy / 2.0;
  }
}

// compute target position to approach the ball from a better angle to the goal
void compute_target_position(struct RoboAI *ai, double *target_cx,
                                    double *target_cy) {
  // use ball's position as target for simplicity
  if (!ai || !ai->st.self || !ai->st.ball || !target_cx || !target_cy)
    return;

  double bx = ai->st.ball->cx;
  double by = ai->st.ball->cy;
  double gx, gy;
  compute_goal_center(ai, &gx, &gy);

  double dx = bx - gx;
  double dy = by - gy;
  double L = sqrt(dx * dx + dy * dy);

  double x = DELTA_TO_TARGET * fabs(dx) / L;
  ;
  double y = DELTA_TO_TARGET * fabs(dy) / L;

  // determine target_cy based on ball position relative to goal
  if (by < gy) {
    // ball is at top left quarter
    *target_cy = by - y;
  } else {
    // ball is at bottom left quarter
    *target_cy = by + y;
  }

  // determine target_cx based on goal position
  if (gx < bx) {
    // goal is at left
    *target_cx = bx + x;
  } else {
    // goal is at right
    *target_cx = bx - x;
  }
}

// compute the angle error between robot heading and target position
// return value in degrees, positive means target is to the right, negative means to the left, normalized to [-180, 180]
double compute_angle_error_to_target(struct RoboAI *ai, double smx, double smy,
                                     double target_x, double target_y) {
  if (!ai || !ai->st.self || !ai->st.ball)
    return NAN;

  // position deltas
  double dx = target_x - ai->st.self->cx;
  double dy = target_y - ai->st.self->cy;
  double ang_to_target = atan2(dy, dx);

  // normalize target direction vector
  double bn = sqrt(dx * dx + dy * dy);
  if (bn < 1e-3)
    return NAN;
  double bnx = dx / bn;
  double bny = dy / bn;

  double hdx = ai->st.sdx;
  double hdy = ai->st.sdy;
  // use motion vector to disambiguate heading direction
  // if dot product < 0, reverse heading direction
  double dot_heading_motion = hdx * smx + hdy * smy;
  // fprintf(stderr,
  //         "compute_angle_error_to_target: direction vectors: heading (%.2f, "
  //         "%.2f), motion (%.2f, %.2f), dot %.2f and target(%.2f, %.2f) and "
  //         "target dot %.2f\n",
  //         hdx, hdy, smx, smy, dot_heading_motion, bnx, bny,
  //         hdx * bnx + hdy * bny);
  if ((smx != 0 || smy != 0) && dot_heading_motion < 0) {
    // correct heading direction based on motion vector
    hdx = -hdx;
    hdy = -hdy;
  }

  double ang_bot = atan2(hdy, hdx);

  // angle error
  double ang_err = ang_to_target - ang_bot;
  // then ang_err < 0 -> target is to the left of heading --> need turn left
  //      ang_err > 0 -> target is to the right of heading --> need turn right

  // normalized to [-pi, pi]
  while (ang_err > M_PI)
    ang_err -= 2 * M_PI;
  while (ang_err < -M_PI)
    ang_err += 2 * M_PI;
  // convert to degrees
  double ang_err_deg = ang_err * (180.0 / M_PI);

  // fprintf(stderr,
  //         "compute_angle_error_to_target: angle error %.2f degrees with motion "
  //         "vector (%.2f, %.2f) and corrected heading (%.2f, %.2f)\n",
  //         ang_err_deg, smx, smy, hdx, hdy);
  return ang_err_deg;
}

// correct motion vector (smx, smy) by rotate_angle_deg
// return the new motion vector magnitude
double correct_motion_vector(double *smx, double *smy,
                             double rotate_angle_deg) {
  double rad = rotate_angle_deg * M_PI / 180.0;
  double cos_rad = cos(rad);
  double sin_rad = sin(rad);

  double new_smx = (*smx) * cos_rad - (*smy) * sin_rad; // -sinx
  double new_smy = (*smx) * sin_rad + (*smy) * cos_rad; // +sinx

  *smx = new_smx;
  *smy = new_smy;
  return hypot(new_smx, new_smy);
}

// compute current distance dist and distance error dist_err to target position (target_cx,
// target_cy)
double compute_distance_error(struct RoboAI *ai, double target_dist,
                              double *dist_err, double *d_ball_dist,
                              double target_cx, double target_cy) {
  if (!ai || !ai->st.self || !ai->st.ball)
    return NAN;

  // current distance
  double dx = target_cx - ai->st.self->cx;
  double dy = target_cy - ai->st.self->cy;
  double dist = hypot(dx, dy);

  // distance change rate to ball
  if (d_ball_dist) {
    double vx_rel = ai->st.bvx - ai->st.svx;
    double vy_rel = ai->st.bvy - ai->st.svy;
    double denom = dist > 1e-3 ? dist : 1e-3;
    double rate = (dx * vx_rel + dy * vy_rel) / denom;
    *d_ball_dist = -rate; // make positive = approaching
  }

  // distance difference
  if (dist_err)
    *dist_err = dist - target_dist; // distance error to target

  return dist;
}

// check if any of the blobs (self, ball, opp) is lost
bool check_anything_lost(struct RoboAI *ai) {
  if (!ai || !ai->st.self || !ai->st.ball || !ai->st.opp)
    return true;
  return false;
}

// check if ball or self is lost
bool check_ball_self_lost(struct RoboAI *ai) {
  if (!ai || !ai->st.self || !ai->st.ball)
    return true;
  return false;
}

/***********************************
 *
 * AI mode functions
 *
 ***********************************/

// called when in PENALTY mode
void penalty_mode(struct RoboAI *ai, double *stored_smx, double *stored_smy) {
  // fprintf(stderr, "In PENALTY mode, current state: %d\n", ai->st.state);
  int state = ai->st.state;

  // denoise check for all blobs
  struct blob *aiBlob[] = {ai->st.ball, ai->st.self, ai->st.opp};
  struct BlobHistory *aiBlobHist[] = {&trackHist.ball, &trackHist.self,
                                      &trackHist.opp};

    for (int i = 0; i < 3; i++) {
      struct blob *b = aiBlob[i];
      struct BlobHistory *hist = aiBlobHist[i];
      if (!b || !hist)
        continue;

      update_blob_history(hist, b);
      int valid = denoise_exp(hist, &b->cx, &b->cy, &b->vx, &b->vy, &b->dx,
                              &b->dy, &b->mx, &b->my);
      if (valid < 0) {
        fprintf(stderr, "Lost track of a blob, back to 101\n");
        ai->st.state = ST_PENALTY_ROTATE_TO_TARGET;
      }
    }

    if (check_ball_self_lost(ai)) {
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    ai->st.state = ST_PENALTY_DONE;
    return;
  }

  switch (state) {
  case ST_PENALTY_ROTATE_TO_TARGET: {

    double target_cx, target_cy;
    compute_target_position(ai, &target_cx, &target_cy);

    if (is_close_to_target(ai, target_cx, target_cy)) {
      ai->st.state = ST_PENALTY_ROTATE_TO_BALL;
      move_flag = -2;
      break;
    }
    if (is_facing_target(ai, *stored_smx, *stored_smy, target_cx, target_cy) &&
        rotate_flag == -1) {
      ai->st.state = ST_PENALTY_MOVE_TO_TARGET; // facing target
                                                // target_angle = 0;
      break;
    }

    // non-blocking rotate to target
    rotate_to_blob(ai, *stored_smx, *stored_smy, target_cx, target_cy);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing target achieved in PENALTY mode\n");
      ai->st.state = ST_PENALTY_MOVE_TO_TARGET;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(stored_smx, stored_smy, target_angle);
      target_angle = 0;
    }
    break;
  }

  case ST_PENALTY_MOVE_TO_TARGET: {
    // calculate target position
    double tgt_cx, tgt_cy;
    compute_target_position(ai, &tgt_cx, &tgt_cy);

    if (is_close_to_target(ai, tgt_cx, tgt_cy)) {
      ai->st.state = ST_PENALTY_ROTATE_TO_BALL;
      double de = 0, dd = 0;
      double d = compute_distance_error(ai, TARGET_BALL_DIST, &de, &dd, tgt_cx,
                                        tgt_cy);
      move_flag = -2;
      break;
    }

    if (!is_facing_target(ai, *stored_smx, *stored_smy, tgt_cx, tgt_cy)) {
      ai->st.state = ST_PENALTY_ROTATE_TO_TARGET;
      break;
    }

    move_to_blob(ai, *stored_smx, *stored_smy, tgt_cx, tgt_cy,
                 TARGET_TARGET_DIST-20);
    break;
  }

  case ST_PENALTY_ROTATE_TO_BALL: {
    // ball position
    double ball_cx = ai->st.ball->cx;
    double ball_cy = ai->st.ball->cy;
    // double angle_error = compute_angle_error_to_target(ai, *stored_smx,
    // *stored_smy, ball_cx, ball_cy);
    if (is_facing_target(ai, *stored_smx, *stored_smy, ball_cx, ball_cy) &&
        rotate_flag == -1) {
      // rotate_flag = -1;
      ai->st.state = ST_PENALTY_MOVE_TO_BALL; // facing target
                                              // target_angle = 0;
      break;
    }

    // non-blocking rotate to target
    rotate_to_blob(ai, *stored_smx, *stored_smy, ball_cx, ball_cy);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing ball achieved in PENALTY mode\n");
      ai->st.state = ST_PENALTY_MOVE_TO_BALL;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(stored_smx, stored_smy, target_angle);
      target_angle = 0;
    }
    break;
  }

  case ST_PENALTY_MOVE_TO_BALL: {
    // ball position
    double b_cx = ai->st.ball->cx;
    double b_cy = ai->st.ball->cy;

    if (is_close_to_ball(ai, b_cx, b_cy)) {
      ai->st.state = ST_PENALTY_KICK_BALL;
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      break;
    }

    if (!is_facing_target(ai, *stored_smx, *stored_smy, b_cx, b_cy)) {
      ai->st.state = ST_PENALTY_ROTATE_TO_BALL;
      move_flag = -2;
      break;
    }

    move_to_blob(ai, *stored_smx, *stored_smy, b_cx, b_cy, TARGET_BALL_DIST-20);
    break;
  }

  case ST_PENALTY_KICK_BALL: {
    kick_ball(ai);
    ai->st.state = ST_PENALTY_DONE;
    break;
  }

  case ST_PENALTY_DONE: {
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    break;
  }

  default: {
    fprintf(stderr, "Unknown PENALTY state: %d\n", state);
    ai->st.state = ST_PENALTY_ROTATE_TO_TARGET;
    break;
  }
  }
}

// called when in CHASE mode
void chase_mode(struct RoboAI *ai, double *stored_smx, double *stored_smy) {
  // fprintf(stderr, "In CHASE mode, current state: %d\n", ai->st.state);
  int state = ai->st.state;

  if (check_ball_self_lost(ai)) {
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    ai->st.state = ST_CHASE_DONE;
    return;
  }

  switch (state) {
  case ST_CHASE_ROTATE_TO_BALL:
  {
    // ball position
    double ball_cx = ai->st.ball->cx;
    double ball_cy = ai->st.ball->cy;
    // double angle_error = compute_angle_error_to_target(ai, *stored_smx,
    // *stored_smy, ball_cx, ball_cy);
    if (is_facing_target(ai, *stored_smx, *stored_smy, ball_cx, ball_cy) &&
        rotate_flag == -1) {
      // rotate_flag = -1;
      ai->st.state = ST_CHASE_MOVE_TO_BALL; // facing target
                                              // target_angle = 0;
      break;
    }

    // non-blocking rotate to target
    rotate_to_blob(ai, *stored_smx, *stored_smy, ball_cx, ball_cy);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing ball achieved in PENALTY mode\n");
      ai->st.state = ST_CHASE_MOVE_TO_BALL;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(stored_smx, stored_smy, target_angle);
      target_angle = 0;
    }
    break;
  }

  case ST_CHASE_MOVE_TO_BALL:
  {
    // ball position
    double b_cx = ai->st.ball->cx;
    double b_cy = ai->st.ball->cy;

    if (is_close_to_ball(ai, b_cx, b_cy)) {
      ai->st.state = ST_CHASE_KICK_BALL;
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      break;
    }

    if (!is_facing_target(ai, *stored_smx, *stored_smy, b_cx, b_cy)) {
      ai->st.state = ST_CHASE_ROTATE_TO_BALL;
      move_flag = -2;
      break;
    }

    move_to_blob(ai, *stored_smx, *stored_smy, b_cx, b_cy, TARGET_BALL_DIST);
    break;
  }  

  case ST_CHASE_KICK_BALL:
  {
    kick_ball(ai);
    ai->st.state = ST_CHASE_DONE;
    break;
  }

  case ST_CHASE_DONE:
    {
    if (check_ball_self_lost(ai)) {
      fprintf(stderr, "Ball lost after kick, rotating to search\n");
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      self_not_found_backing(ai);
      usleep(500 * 1000); // wait for a second
      break;
    } else {
      fprintf(stderr, "Ball found after kick, resuming chase\n");
      ai->st.state = ST_CHASE_ROTATE_TO_BALL;
      backing_count = 0;
      rotate_flag = -1; // reset rotate flag
    }
    usleep(10 * 1000);
    break;
  }

  default:
    fprintf(stderr, "Unknown CHASE state: %d\n", state);
    ai->st.state = ST_CHASE_ROTATE_TO_BALL;
    break;
  }
}

/***********************************
 *
 * soccer mode and sub-modes
 *
 ***********************************/

// soccer mode helper functions
// compute angle difference between opp heading and opp-to-target vector
double compute_opp_angle_diff_to_target(struct RoboAI *ai, double target_x,
                                        double target_y) {
  if (!ai || !ai->st.opp)
    return 0.0;

  // position deltas
  double dx = target_x - ai->st.opp->cx;
  double dy = target_y - ai->st.opp->cy;
  double ang_to_target = atan2(dy, dx);

  double hdx = ai->st.odx;
  double hdy = ai->st.ody;

  double ang_opp = atan2(hdy, hdx);

  // angle error
  double ang_err = ang_to_target - ang_opp;

  // normalized to [-pi/2, pi/2]
  while (ang_err > M_PI)
    ang_err -= 2 * M_PI;
  while (ang_err < -M_PI)
    ang_err += 2 * M_PI;
  // reduce to [-pi/2, pi/2] because direction and its opposite represent the
  // same line
  if (ang_err > M_PI / 2)
    ang_err -= M_PI;
  else if (ang_err < -M_PI / 2)
    ang_err += M_PI;
  // convert to degrees
  return ang_err * (180.0 / M_PI);
}

// compute distance between opp and target position
double compute_opp_distance_to_target(struct RoboAI *ai, double target_cx,
                                      double target_cy) {
  if (!ai || !ai->st.opp)
    return NAN;

  // position deltas
  double dx = target_cx - ai->st.opp->cx;
  double dy = target_cy - ai->st.opp->cy;
  double dist = hypot(dx, dy);
  return dist;
}

// compute either own goal center or opponent goal center based on side
void compute_goal_center1(int side, double *gcx, double *gcy) {
  // left goal center is: (0, sy/2)
  // right goal center is: (sx, sy/2)

  if (side == 0) {
    // left side, so opponent goal is right
    *gcx = sx; // right edge
    *gcy = sy / 2.0;
  } else {
    // right side, so opponent goal is left
    *gcx = 0.0; // left edge
    *gcy = sy / 2.0;
  }
}

// general target position computation helper
// can be used for defense and other modes: (gx, gy) can be any target point to connect with ball
void compute_target_pos_general(struct RoboAI *ai, double gx, double gy,
                                double delta, double *target_cx,
                                double *target_cy) {
  // use ball's position as target for simplicity
  if (!ai || !ai->st.self || !ai->st.ball || !target_cx || !target_cy)
    return;

  double bx = ai->st.ball->cx;
  double by = ai->st.ball->cy;

  double dx = bx - gx;
  double dy = by - gy;
  double L = sqrt(dx * dx + dy * dy);

  double x = delta * fabs(dx) / L;
  ;
  double y = delta * fabs(dy) / L;

  // determine target_cy based on ball position relative to goal
  if (by < gy) {
    // ball is at top left quarter
    *target_cy = by - y;
  } else {
    // ball is at bottom left quarter
    *target_cy = by + y;
  }

  // determine target_cx based on goal position
  if (gx < bx) {
    // goal is at left
    *target_cx = bx + x;
  } else {
    // goal is at right
    *target_cx = bx - x;
  }
}

// compute defense target position
void compute_defense_target(struct RoboAI *ai, double *target_cx,
                            double *target_cy) {
  compute_target_pos_general(ai, ai->st.opp->cx, ai->st.opp->cy, DELTA_TO_OPP,
                             target_cx, target_cy);
}

// helper function for escape mode
double compute_target_x(double target_y, double line_slope,
                        double line_intercept) {
  // line equation: y = mx + b  --> x = (y - b) / m
  if (fabs(line_slope) < 1e-6) {
    // vertical line case, slope is infinite
    return NAN;
  }
  return (target_y - line_intercept) / line_slope;
}

// compute escape rotate target position
void compute_escape_rotate_target(struct RoboAI *ai, double *target_x,
                                  double *target_y) {
  double line_slope = 0.0;
  double line_intercept = 0.0;
  double x1 = ai->st.self->cx;
  double y1 = ai->st.self->cy;
  double x2, y2 = 0.0;
  compute_goal_center1(1 - ai->st.side, &x2, &y2);

  line_slope = (y2 - y1) / (x2 - x1);
  line_intercept = y1 - line_slope * x1;

  double temp_x = compute_target_x(0, line_slope, line_intercept);
  if (temp_x > 0 && temp_x < sx) {
    *target_x = temp_x;
    *target_y = 0;
  } else {
    temp_x = compute_target_x(sy, line_slope, line_intercept);
    *target_x = temp_x;
    *target_y = sy;
  }
}

// compute nearest edge to ball
// return edge id: 0 - up, 1 - down, 2 - left, 3 - right
int compute_ball_nearest_edge(struct RoboAI *ai, double *edge_x,
                              double *edge_y) {
  if (!ai || !ai->st.ball)
    return -1;

  double bx = ai->st.ball->cx;
  double by = ai->st.ball->cy;

  double dist_up = by;                 // distance to top (y=0)
  double dist_down = (double)sy - by;  // distance to bottom (y=sy)
  double dist_left = bx;               // distance to left (x=0)
  double dist_right = (double)sx - bx; // distance to right (x=sx)

  double min_dist = dist_up;
  int edge_id = 0; // 0: up

  if (dist_down < min_dist) {
    min_dist = dist_down;
    edge_id = 1;
  }
  if (dist_left < min_dist) {
    min_dist = dist_left;
    edge_id = 2;
  }
  if (dist_right < min_dist) {
    min_dist = dist_right;
    edge_id = 3;
  }

  if (edge_x) {
    if (edge_id == 2)
      *edge_x = 0.0; // left edge
    else if (edge_id == 3)
      *edge_x = (double)sx; // right edge
    else
      *edge_x = bx; // same x as ball
  }

  if (edge_y) {
    if (edge_id == 0)
      *edge_y = 0.0; // top edge
    else if (edge_id == 1)
      *edge_y = (double)sy; // bottom edge
    else
      *edge_y = by; // same y as ball
  }

  return edge_id;
}

// check if target position is valid
bool check_target_validation(struct RoboAI *ai, double target_cx,
                             double target_cy) {
  if (target_cx < 50 || target_cx > sx-50 || target_cy < 50 || target_cy > sy-50) {
    return false;
  }

  if (!ai || !ai->st.self) return false;
  double self_x = ai->st.self->cx;
  double self_y = ai->st.self->cy;
  double own_gx, own_gy;
  compute_goal_center1(1-ai->st.side, &own_gx, &own_gy);

  if (ai->st.side == 0) {
    // left side
    if (target_cx < self_x - 50) {
      return false;
    }
  } else {
    // right side
    if (target_cx > self_x + 50) {
      return false;
    }
  }

  return true;
}

// checker for soccer behavior
int check_soccer_state_behavior(struct RoboAI *ai, double *smx, double *smy) {
  // determine whether the ai should escape, defend, or attack (normal attack or
  // edge attack)

  bool edge_attack = false;

  if (check_ball_self_lost(ai) || rotate_flag > 0) {
    // case ball or self lost
    return BEHAVIOR_NOT_CHANGE; // lost track, do not change behavior
  }else if (!ai->st.opp){
    // case only opp lost
    edge_attack = need_edge_play(ai);
    if (edge_attack || edge_flag > 0) {
      return EDGE_ATTACK_BEHAVIOR;
    }else{
      return NORMAL_ATTACK_BEHAVIOR;
    }
  }

  bool escape = need_escape(ai, smx, smy);
  if (escape) {
    edge_flag = -1; // reset edge flag when escaping
    return ESCAPE_BEHAVIOR;
  }

  bool defend = need_defense(ai);
  if (defend) {
    edge_flag = -1; // reset edge flag when defending
    return DEFEND_BEHAVIOR;
  }

  edge_attack = need_edge_play(ai);
  if (edge_attack || edge_flag > 0) {
    return EDGE_ATTACK_BEHAVIOR;
  }

  // now don't consider edge attack, only do normal attack
  return NORMAL_ATTACK_BEHAVIOR;
}

// called when in SOCCER mode
void soccer_mode(struct RoboAI *ai, double *smx, double *smy) {
  int state = ai->st.state;

  struct blob *aiBlob[] = {ai->st.ball, ai->st.self, ai->st.opp};
  struct BlobHistory *aiBlobHist[] = {&trackHist.ball, &trackHist.self,
                                      &trackHist.opp};


    for (int i = 0; i < 3; i++) {
      struct blob *b = aiBlob[i];
      struct BlobHistory *hist = aiBlobHist[i];
      if (!b || !hist)
        continue;

      update_blob_history(hist, b);
      int valid = denoise_exp(hist, &b->cx, &b->cy, &b->vx, &b->vy, &b->dx,
                              &b->dy, &b->mx, &b->my);
      if (valid < 0) {
        fprintf(stderr, "Lost track of a blob, back to 101\n");
        ai->st.state = ST_SOCCER_ROTATE_TO_TARGET;
      }
    }

  // normal play states: 10-19
  // edge play states: 20-29
  // defense states: 30-39
  // escape states: 40-49
  if (state >= 40 && state < 50) {
    // fprintf(stderr, "Escaping!\n");
    soccer_escape_mode(ai, smx, smy);
  } else if (state >= 30 && state < 40) {
    // fprintf(stderr, "Defending goal\n");
    soccer_defense_mode(ai, smx, smy);
  } else if (state >= 10 && state < 20) {
    // fprintf(stderr, "Normal attack mode\n");
    soccer_normal_play_mode(ai, smx, smy);
  } else if (state >= 20 && state < 30) {
    // fprintf(stderr, "Edge attack mode\n");
    soccer_edge_play_mode(ai, smx, smy);
    return;
  } else {
    // fprintf(stderr, "In SOCCER test mode, current state: %d\n", state);
    ai->st.state = ST_SOCCER_ROTATE_TO_TARGET; // set to normal play mode
    soccer_normal_play_mode(ai, smx, smy);
  }
}

/****************************
 * soccer sub-behavior modes
 ****************************/
// normal play mode
void soccer_normal_play_mode(struct RoboAI *ai, double *stored_smx,
                             double *stored_smy) {
  int state = ai->st.state;

  int behavior = check_soccer_state_behavior(ai, stored_smx, stored_smy);
  if (behavior == NORMAL_ATTACK_BEHAVIOR || behavior == BEHAVIOR_NOT_CHANGE) {
    // do nothing, continue normal play
  } else if (behavior == ESCAPE_BEHAVIOR) {
    // fprintf(stderr, "Escaping in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_ESCAPE_ROTATE;
    return;
  } else if (behavior == DEFEND_BEHAVIOR) {
    // fprintf(stderr, "Defending in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_DEFEND_ROTATE;
    return;
  } else if (behavior == EDGE_ATTACK_BEHAVIOR) {
    // fprintf(stderr, "Edge attacking in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_EDGE_ROTATE_TARGET;
    return;
  }

  if (check_ball_self_lost(ai)) {
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    ai->st.state = ST_SOCCER_NORMAL_PLAY_DONE;
    return;
  }

  switch (state) {
  case ST_SOCCER_CHECK_BEHAVIOR:
    ai->st.state = ST_SOCCER_ROTATE_TO_TARGET;
    break;

  case ST_SOCCER_ROTATE_TO_TARGET: {
    double target_cx, target_cy;
    compute_target_position(ai, &target_cx, &target_cy);
    if (!check_target_validation(ai, target_cx, target_cy)) {
      edge_flag = 1; // force edge play
      break;
    }

    if (is_close_to_target(ai, target_cx, target_cy)) {
      // fprintf(stderr, "Already reached target in SOCCER mode, stopping\n");
      ai->st.state = ST_SOCCER_ROTATE_TO_BALL;
      move_flag = -2;
      break;
    }

    if (is_facing_target(ai, *stored_smx, *stored_smy, target_cx, target_cy) &&
        rotate_flag == -1) {
      ai->st.state = ST_SOCCER_MOVE_TO_TARGET; // facing target
      break;
    }
    // fprintf(stderr,
    //         "[state 20]Rotating to face target in SOCCER mode and check "
    //         "rotate-flag = %d\n",
    //         rotate_flag);
    // non-blocking rotate to target
    rotate_to_blob(ai, *stored_smx, *stored_smy, target_cx, target_cy);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing target achieved in SOCCER mode\n");
      ai->st.state = ST_SOCCER_MOVE_TO_TARGET;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(stored_smx, stored_smy, target_angle);
      target_angle = 0;
    }
    break;
  }

  case ST_SOCCER_MOVE_TO_TARGET: {
    double target_cx, target_cy;
    compute_target_position(ai, &target_cx, &target_cy);
    if (!check_target_validation(ai, target_cx, target_cy)) {
      edge_flag = 1; // force edge play
      break;
    }

    if (is_close_to_target(ai, target_cx, target_cy)) {
      // fprintf(stderr, "Already reached target in SOCCER mode, stopping\n");
      ai->st.state = ST_SOCCER_ROTATE_TO_BALL;
      move_flag = -2;
      break;
    }
    if (!is_facing_target(ai, *stored_smx, *stored_smy, target_cx, target_cy)) {
      // fprintf(stderr, "Lost facing target in SOCCER mode, rotating to face\n");
      ai->st.state = ST_SOCCER_ROTATE_TO_TARGET;
      move_flag = -2;
      break;
    }

    move_to_blob(ai, *stored_smx, *stored_smy, target_cx, target_cy,
                 TARGET_TARGET_DIST);
    break;
  }
  case ST_SOCCER_ROTATE_TO_BALL: {
    if (check_ball_self_lost(ai)) {
      // fprintf(stderr, "Something lost, rotating to search in SOCCER mode\n");
      ai->st.state = ST_SOCCER_NORMAL_PLAY_DONE;
      break;
    }
    // ball position
    double ball_cx = ai->st.ball->cx;
    double ball_cy = ai->st.ball->cy;
    // double angle_error = compute_angle_error_to_target(ai, *stored_smx,
    // *stored_smy, ball_cx, ball_cy);
    if (is_facing_target(ai, *stored_smx, *stored_smy, ball_cx, ball_cy) &&
        rotate_flag == -1) {
      // fprintf(stderr, "Rotating to face target in PENALTY mode\n");
      ai->st.state = ST_SOCCER_MOVE_TO_BALL; // facing target
      break;
    }

    // non-blocking rotate to target
    rotate_to_blob(ai, *stored_smx, *stored_smy, ball_cx, ball_cy);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing target achieved in PENALTY mode\n");
      ai->st.state = ST_SOCCER_MOVE_TO_BALL;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(stored_smx, stored_smy, target_angle);
      target_angle = 0;
    }
    break;
  }

  case ST_SOCCER_MOVE_TO_BALL: {

    if (is_close_to_ball(ai, ai->st.ball->cx, ai->st.ball->cy)) {
      // fprintf(stderr, "Reached ball in SOCCER mode, kicking\n");
      ai->st.state = ST_SOCCER_KICK_BALL;
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      move_flag = -2;
      break;
    }
    if (!is_facing_target(ai, *stored_smx, *stored_smy, ai->st.ball->cx,
                          ai->st.ball->cy)) {
      // fprintf(stderr, "Lost facing ball in SOCCER mode, rotating to face\n");
      ai->st.state = ST_SOCCER_ROTATE_TO_BALL;
      move_flag = -2;
      break;
    }
    move_to_blob(ai, *stored_smx, *stored_smy, ai->st.ball->cx, ai->st.ball->cy,
                 TARGET_BALL_DIST);
    break;
  }
  case ST_SOCCER_KICK_BALL: {
    // fprintf(stderr, "Kicking ball in SOCCER mode\n");
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    kick_ball(ai);
    ai->st.state = ST_SOCCER_NORMAL_PLAY_DONE;
    break;
  }
  case ST_SOCCER_NORMAL_PLAY_DONE: {
    if (check_ball_self_lost(ai)) {
      fprintf(stderr, "Ball lost after kick, rotating to search\n");
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      ai->st.state = ST_SOCCER_NORMAL_PLAY_DONE;
      self_not_found_backing(ai);
      usleep(500 * 1000); // wait for a second
      break;
    } else {
      fprintf(stderr, "Ball found after kick, resuming chase\n");
      ai->st.state = ST_SOCCER_ROTATE_TO_TARGET;
      backing_count = 0;
      rotate_flag = -1; // reset rotate flag
    }
    usleep(10 * 1000);
    break;
  }
  }
}

// defense mode
void soccer_defense_mode(struct RoboAI *ai, double *smx, double *smy) {
  int state = ai->st.state;
  // fprintf(stderr, "In SOCCER[DEFENSE] mode, current state: %d\n", state);

  int behavior = check_soccer_state_behavior(ai, smx, smy);
  if (behavior == DEFEND_BEHAVIOR || behavior == BEHAVIOR_NOT_CHANGE) {
    // do nothing, continue defense
  } else if (behavior == ESCAPE_BEHAVIOR) {
    // fprintf(stderr, "Escaping in soccer defense mode\n");
    ai->st.state = ST_SOCCER_ESCAPE_ROTATE;
    return;
  } else if (behavior == NORMAL_ATTACK_BEHAVIOR) {
    // fprintf(stderr, "Normal attack in soccer defense mode\n");
    ai->st.state = ST_SOCCER_ROTATE_TO_TARGET;
    return;
  } else if (behavior == EDGE_ATTACK_BEHAVIOR) {
    // fprintf(stderr, "Edge attacking in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_EDGE_ROTATE_TARGET;
    return;
  }

  double target_cx, target_cy;

  if (check_anything_lost(ai)) {
    fprintf(stderr, "Something lost, rotating to search in SOCCER mode\n");
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    ai->st.state = ST_SOCCER_DEFEND_DONE;
    return;
  }

  compute_defense_target(ai, &target_cx, &target_cy);

  switch (state) {
  case ST_SOCCER_CHECK_BEHAVIOR:
    ai->st.state = ST_SOCCER_DEFEND_ROTATE;
    break;

  case ST_SOCCER_DEFEND_ROTATE: {

    if (is_facing_target(ai, *smx, *smy, target_cx, target_cy) &&
        rotate_flag == -1) {
      ai->st.state = ST_SOCCER_DEFEND_MOVE; // facing target
      break;
    }
    // non-blocking rotate to target
    rotate_to_blob(ai, *smx, *smy, target_cx, target_cy);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing target achieved in SOCCER mode\n");
      ai->st.state = ST_SOCCER_DEFEND_MOVE;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(smx, smy, target_angle);
      target_angle = 0;
    }
    break;
  }

  case ST_SOCCER_DEFEND_MOVE: {

    if (is_close_to_target(ai, target_cx, target_cy)) {
      // fprintf(stderr, "Reached defense target, holding position\n");
      ai->st.state = ST_SOCCER_DEFEND_DONE;
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      move_flag = -2;
      break;
    }

    if (!is_facing_target(ai, *smx, *smy, target_cx, target_cy)) {
      // fprintf(stderr, "Lost facing defense target, rotating to face\n");
      ai->st.state = ST_SOCCER_DEFEND_ROTATE;
      move_flag = -2;
      break;
    }
    move_to_blob(ai, *smx, *smy, target_cx, target_cy, TARGET_DEFENSE_DIST);
    break;
  }

  case ST_SOCCER_DEFEND_DONE: {
    if (ai == NULL || ai->st.ball == NULL || ai->st.self == NULL ||
        ai->st.opp == NULL) {
      // fprintf(stderr, "Ball lost after kick, rotating to search\n");
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      self_not_found_backing(ai);
      usleep(500 * 1000); // wait for a second
      break;
    } else {
      fprintf(stderr, "Ball found after kick, resuming chase\n");
      backing_count = 0;
      ai->st.state = ST_SOCCER_DEFEND_ROTATE;
    }
    usleep(10 * 1000);
    break;
  }
  }
}

// escape mode
void soccer_escape_mode(struct RoboAI *ai, double *smx, double *smy) {
  int state = ai->st.state;
  // fprintf(stderr, "In SOCCER[ESCAPE] mode, current state: %d\n", state);

  int behavior = check_soccer_state_behavior(ai, smx, smy);
  if (behavior == ESCAPE_BEHAVIOR || behavior == BEHAVIOR_NOT_CHANGE) {
    // do nothing, continue escape
  } else if (behavior == DEFEND_BEHAVIOR) {
    // fprintf(stderr, "Defending in soccer escape mode\n");
    ai->st.state = ST_SOCCER_DEFEND_ROTATE;
    return;
  } else if (behavior == NORMAL_ATTACK_BEHAVIOR) {
    // fprintf(stderr, "Normal attack in soccer escape mode\n");
    ai->st.state = ST_SOCCER_ROTATE_TO_TARGET;
    return;
  } else if (behavior == EDGE_ATTACK_BEHAVIOR) {
    // fprintf(stderr, "Edge attacking in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_EDGE_ROTATE_TARGET;
    return;
  }

  if (check_anything_lost(ai)) {
    fprintf(stderr, "Something lost, rotating to search in SOCCER mode\n");
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    ai->st.state = ST_SOCCER_ESCAPE_DONE;
    return;
  }

  switch (state) {
  case ST_SOCCER_CHECK_BEHAVIOR:
    ai->st.state = ST_SOCCER_ESCAPE_ROTATE;
    break;

  case ST_SOCCER_ESCAPE_ROTATE: {

    double target_x, target_y;
    compute_escape_rotate_target(ai, &target_x, &target_y);
    if (is_facing_target(ai, *smx, *smy, target_x, target_y) &&
        rotate_flag == -1) {
      ai->st.state = ST_SOCCER_ESCAPE_MOVE; // facing target
      break;
    }
    // fprintf(stderr, "[state 20]Rotating to face target in SOCCER mode\n");
    // non-blocking rotate to target
    rotate_to_blob(ai, *smx, *smy, target_x, target_y);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing target achieved in SOCCER mode\n");
      ai->st.state = ST_SOCCER_ESCAPE_MOVE;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(smx, smy, target_angle);
      target_angle = 0;
    }
    break;
  }
  case ST_SOCCER_ESCAPE_MOVE: {

    BT_drive(LEFT_MOTOR, RIGHT_MOTOR, -55, -50); // move backward
    sleep(1);                                    // move for 0.5 second
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    ai->st.state = ST_SOCCER_ESCAPE_DONE;
    break;
  }
  case ST_SOCCER_ESCAPE_DONE: {
    if (ai == NULL || ai->st.ball == NULL || ai->st.self == NULL ||
        ai->st.opp == NULL) {
      fprintf(stderr, "Ball lost after escape, rotating to search\n");
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      ai->st.state = ST_SOCCER_ESCAPE_DONE;
      self_not_found_backing(ai);
      usleep(500 * 1000); // wait for a second
      break;
    } else {
      fprintf(stderr, "Ball found after escape, resuming chase\n");
      backing_count = 0;
      ai->st.state = ST_SOCCER_ESCAPE_ROTATE;
      rotate_flag = -1; // reset rotate flag
    }
    sleep(1);
    break;
  }
  }
}

// edge play mode
void soccer_edge_play_mode(struct RoboAI *ai, double *smx, double *smy) {
  int state = ai->st.state;

  int behavior = check_soccer_state_behavior(ai, smx, smy);
  if (behavior == EDGE_ATTACK_BEHAVIOR || behavior == BEHAVIOR_NOT_CHANGE) {
    // do nothing, continue normal play
  } else if (behavior == ESCAPE_BEHAVIOR) {
    // fprintf(stderr, "Escaping in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_ESCAPE_ROTATE;
    return;
  } else if (behavior == DEFEND_BEHAVIOR) {
    // fprintf(stderr, "Defending in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_DEFEND_ROTATE;
    return;
  } else if (behavior == NORMAL_ATTACK_BEHAVIOR) {
    // fprintf(stderr, "Normal attacking in soccer normal play mode\n");
    ai->st.state = ST_SOCCER_ROTATE_TO_TARGET;
    return;
  }

  if (check_ball_self_lost(ai)) {
    fprintf(stderr, "Something lost, rotating to search in SOCCER mode\n");
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    ai->st.state = ST_SOCCER_EDGE_DONE;
    return;
  }

  double edge_x, edge_y;
  int edge_id = compute_ball_nearest_edge(ai, &edge_x, &edge_y);
  // fprintf(stderr,
  //         "In SOCCER[EDGE] mode, current state: %d, nearest edge id: %d with "
  //         "edge (%.2f, %.2f)\n",
  //         state, edge_id, edge_x, edge_y);

  switch (state) {
  case ST_SOCCER_EDGE_ROTATE_TARGET: {
    double target_cx, target_cy;
    if (edge_id == 0 || edge_id == 1) {
      // top or bottom edge, keep x same as ball, move y by delta
      target_cx = ai->st.ball->cx;
      target_cy =
          ai->st.ball->cy + (edge_id == 0 ? DELTA_TO_TARGET : -DELTA_TO_TARGET);
    } else {
      target_cx =
          ai->st.ball->cx + (edge_id == 2 ? DELTA_TO_TARGET : -DELTA_TO_TARGET);
      target_cy = ai->st.ball->cy;
    }
    if (is_close_to_target(ai, target_cx, target_cy)) {
      ai->st.state = ST_SOCCER_EDGE_ROTATE_BALL;
      break;
    }

    if (is_facing_target(ai, *smx, *smy, target_cx, target_cy) &&
        rotate_flag == -1) {
      ai->st.state = ST_SOCCER_EDGE_MOVE_TARGET; // facing target
      break;
    }
    rotate_to_blob(ai, *smx, *smy, target_cx, target_cy);

    if (rotate_flag == -2) {
      // fprintf(stderr, "Facing target achieved in SOCCER EDGE mode\n");
      ai->st.state = ST_SOCCER_EDGE_MOVE_TARGET;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(smx, smy, target_angle);
      target_angle = 0.0;
      rotating_angle = 0.0;
    }
    break;
  }

  case ST_SOCCER_EDGE_MOVE_TARGET: {
    double target_cx, target_cy;
    if (edge_id == 0 || edge_id == 1) {
      // top or bottom edge, keep x same as ball, move y by delta
      target_cx = ai->st.ball->cx;
      target_cy =
          ai->st.ball->cy + (edge_id == 0 ? DELTA_TO_TARGET : -DELTA_TO_TARGET);
    } else {
      target_cx =
          ai->st.ball->cx + (edge_id == 2 ? DELTA_TO_TARGET : -DELTA_TO_TARGET);
      target_cy = ai->st.ball->cy;
    }
    if (is_close_to_target(ai, target_cx, target_cy)) {
      ai->st.state = ST_SOCCER_EDGE_ROTATE_BALL;
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      break;
    }

    if (!is_facing_target(ai, *smx, *smy, target_cx, target_cy)) {
      // fprintf(stderr,
      //         "state[31]: check rotate_flag = %d, rotating_angle = %.2f, "
      //         "target_angle = %.2f\n",
      //         rotate_flag, rotating_angle, target_angle);
      rotate_flag = -1; // reset rotate flag
      rotating_angle = 0;
      target_angle = 0.0;
      ai->st.state = ST_SOCCER_EDGE_ROTATE_TARGET;
      move_flag = -2;
      break;
    }
    if (!is_close_to_target(ai, target_cx, target_cy)) {
      move_to_blob(ai, *smx, *smy, target_cx, target_cy, 50);
    }
    break;
  }

  case ST_SOCCER_EDGE_ROTATE_BALL: {
    double target_cx = ai->st.ball->cx;
    double target_cy = ai->st.ball->cy;

    if (is_facing_target(ai, *smx, *smy, target_cx, target_cy) &&
        rotate_flag == -1) {
      // fprintf(stderr, "Rotating to face target in PENALTY mode\n");
      ai->st.state = ST_SOCCER_EDGE_MOVE_BALL; // facing target
      break;
    }

    // non-blocking rotate to target
    rotate_to_blob(ai, *smx, *smy, target_cx, target_cy);

    if (rotate_flag == -2) {
      ai->st.state = ST_SOCCER_EDGE_MOVE_BALL;
      rotate_flag = -1; // reset rotate flag
      correct_motion_vector(smx, smy, target_angle);
      target_angle = 0;
    }
    break;
  }

  case ST_SOCCER_EDGE_MOVE_BALL: {
    if (is_close_to_ball(ai, ai->st.ball->cx, ai->st.ball->cy)) {
      ai->st.state = ST_SOCCER_EDGE_KICK;
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      move_flag = -2;
      break;
    }
    if (!is_facing_target(ai, *smx, *smy, ai->st.ball->cx, ai->st.ball->cy)) {
      ai->st.state = ST_SOCCER_EDGE_ROTATE_BALL;
      move_flag = -2;
      break;
    }
    move_to_blob(ai, *smx, *smy, ai->st.ball->cx, ai->st.ball->cy, 0);
    break;
  }

  case ST_SOCCER_EDGE_KICK: {
    double goal_x, goal_y;
    double ball_x = ai->st.ball->cx;
    double ball_y = ai->st.ball->cy;
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    compute_goal_center(ai, &goal_x, &goal_y);
    int g_angle = 0, g_rate = 0;
    BT_read_gyro(GYRO_PORT, 1, &g_angle, &g_rate);
    if (edge_id == 2) {
      bool goal_is_left = (goal_x == 0);
      bool ball_on_top = (ball_y < sy * 0.5);

      if (goal_is_left) {
        // Goal is on the same side (left), avoid kicking toward own goal
        if (ball_on_top) {
          rotate_right_kick(ai); // top-left: push to right side
        } else {
          rotate_left_kick(ai); // bottom-left: push slightly upward-left
        }
      } else {
        if (ball_on_top) {
          rotate_left_kick(ai); // top-left: push to right side
        } else {
          rotate_right_kick(ai); // bottom-left: push slightly upward-left
        }
      }
    } else if (edge_id == 3) { // ball near right edge
      bool goal_is_right = (goal_x == sx);
      bool ball_on_top = (ball_y < sy * 0.5);
      if (goal_is_right) {
        // Goal is on the same side (left), avoid kicking toward own goal
        if (ball_on_top) {
          rotate_right_kick(ai); // top-left: push to right side
        } else {
          rotate_left_kick(ai); // bottom-left: push slightly upward-left
        }
      } else {
        if (ball_on_top) {
          rotate_left_kick(ai); // top-left: push to right side
        } else {
          rotate_right_kick(ai); // bottom-left: push slightly upward-left
        }
      }
    } else if (edge_id == 0) { // ball near top edge
      bool goal_is_left = (goal_x == 0);
      if (goal_is_left) {
        rotate_left_kick(ai); // top-left: push to right side
      } else {
        rotate_right_kick(ai); // bottom-left: push slightly upward-left
      }
    } else if (edge_id == 1) { // ball near bottom edge
      bool goal_is_left = (goal_x == 0);
      if (goal_is_left) {
        rotate_right_kick(ai); // top-left: push to right side
      } else {
        rotate_left_kick(ai); // bottom-left: push slightly upward-left
      }
    }
    BT_motor_port_stop(LEFT_MOTOR, 0);
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    BT_read_gyro(GYRO_PORT, 0, &g_angle, &g_rate);
    correct_motion_vector(smx, smy, g_angle);
    // after kick, move to done state
    ai->st.state = ST_SOCCER_EDGE_DONE;
    edge_flag = -1; // reset edge flag after kick
    break;
  }

  case ST_SOCCER_EDGE_DONE: {
    if (ai == NULL || ai->st.ball == NULL || ai->st.self == NULL) {
      fprintf(stderr, "Ball lost after kick, rotating to search\n");
      BT_motor_port_stop(LEFT_MOTOR, 0);
      BT_motor_port_stop(RIGHT_MOTOR, 0);
      self_not_found_backing(ai);
      usleep(500 * 1000); // wait for a second
      break;
    } else {
      fprintf(stderr, "Ball found after kick, resuming chase\n");
      backing_count = 0;
      ai->st.state = ST_SOCCER_EDGE_ROTATE_TARGET;
    }
    usleep(10 * 1000);
    break;
  }

  default:
    break;
  }
}
