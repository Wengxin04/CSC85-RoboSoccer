/***************************************************
 CSC C85 - UTSC RoboSoccer AI core
 
 This file contains the definition of the AI data
 structure which holds the state of your bot's AI.

 You must become familiar with this structure and
 its contents. 

 You will need to modify this file to add headers
 for any functions you added to implemet the 
 soccer playing functionality of your bot.

 Be sure to document everything you do thoroughly.

 AI scaffold: Parker-Lee-Estrada, Summer 2013
 Updated by F. Estrada, Jul. 2022

***************************************************/

#ifndef _ROBO_AI_H
#define _ROBO_AI_H

#include "imagecapture/imageCapture.h"
#include "API/btcomm.h"
#include <stdio.h>
#include <stdlib.h>

// Change this to match the ports your bots motors are connected to
// TODOO: change it to the actual motor ports
#define LEFT_MOTOR MOTOR_C
#define RIGHT_MOTOR MOTOR_D
#define KICK_MOTOR MOTOR_A
#define GYRO_PORT PORT_4

#define AI_SOCCER 0 	// Play soccer!
#define AI_PENALTY 1    // Go score some goals!
#define AI_CHASE 2 	    // Kick the ball around and chase it!

#define NOISE_VAR 5.0                   // Minimum amount of displacement considered NOT noise (in pixels).

// TOSEE:
// Penalty kick mode state definitions (100-199)
#define ST_PENALTY_INIT 100           // Initial state
#define ST_PENALTY_ROTATE_TO_BALL 101 // Rotate to face the ball
#define ST_PENALTY_MOVE_TO_BALL 102   // Move to the ball
#define ST_PENALTY_ALIGN_TO_GOAL 103  // Align to face the goal
#define ST_PENALTY_KICK_BALL 104      // Kick the ball
#define ST_PENALTY_DONE 199           // Penalty kick done

struct AI_data{
	// This data structure is used to hold all data relevant to the state of the AI.
	// This includes, of course, the current state, as well as the status of
	// our own bot, the opponent (if present), and the ball (if present).
	// For each agent in the game we keep a pointer to the blob that corresponds
	// to the agent (see the blob data structure in imageCapture.h), and data
	// about its old position, as well as current velocity and heading vectors.
	//
	// MIND THE NOISE.

	// Robot's playfield side id (w.r.t. the viepoint of the camera).
	int side;		// side=0 implies the robot's own side is the left side
                    // side=1 implies the robot's own side is the right side
                    // This is set based on the robot's initial position
                    // on the field
    int botCol;		// Own bot's colour. 0 - green, 1 - red

	int state;		// Current AI state

	// Object ID status for self, opponent, and ball. Just boolean 
        // values indicating whether blobs have been found for each of these
	// entities.
	int selfID;
	int oppID;
	int ballID;

	// Blob track data. Ball likely needs to be detected at each frame
	// separately. So we keep old location to estimate v
	struct blob *ball;		       // Current ball blob *NULL* if ball is not visible/found
	double old_bcx, old_bcy;	   // Previous ball cx,cy
	double bvx,bvy;			       // Ball velocity vector
	double bmx,bmy;			       // Ball motion vector
	double bdx,bdy;                // Ball heading direction (from blob shape)

	// Self track data. Done separately each frame
    struct blob *self;		       // Current self blob *NULL* if not visible/found
	double old_scx, old_scy;	   // Previous self (cx,cy)
	double svx,svy;			       // Current self [vx vy]
	double smx,smy;			       // Self motion vector
	double sdx,sdy;                // Self heading direction (from blob shape)

	// Opponent track data. Done separately each frame
    struct blob *opp;		       // Current opponent blob *NULL* if not visible/found
	double old_ocx, old_ocy;	   // Previous opponent (cx,cy)
	double ovx,ovy;			       // Current opponent [vx vy]
	double omx,omy;			       // Opponent motion vector
	double odx,ody;                // Opponent heading direction (from blob shape)
};

struct RoboAI {
	// Main AI data container. It allows us to specify which function
	// will handle the AI, and sets up a data structure to store the
	// AI's data (see above).
	void (* runAI)(struct RoboAI *ai, struct blob *, void *state);
	void (* calibrate)(struct RoboAI *ai, struct blob *);
	struct AI_data st;
    struct displayList *DPhead;
};

/**
 * \brief Set up an AI structure for playing roboSoccer
 *
 * Set up an AI structure for playing roboSoccer. Must be
 * called before using the AI structure during gameplay.
 * \param[in] mode The operational mode for the AI
 * \param[out] ai A structure containing data necessary for
 * 		AI algorithms
 * \pre ai is uninitialized
 * \post ai is set up for use, and must be cleaned up using
 * 		cleanupAI
 */
int setupAI(int mode, int own_col, struct RoboAI *ai);

/**
 * \brief Top-level AI loop.
 * 
 * Decides based on current state and blob configuration what
 * the bot should do next, and calls the appropriate behaviour
 * function.
 *
 * \param[in] ai, pointer to the data structure for the running AI
 * \param[in] blobs, pointer to the current list of tracked blobs
 * \param[out] void, but the state description in the AI structure may have changed
 * \pre ai is not NULL, blobs is not NULL
 * \post ai is not NULL, blobs is not NULL
 */
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state);

// Calibration stub
void AI_calibrate(struct RoboAI *ai, struct blob *blobs);

/* PaCode - just the function headers - see the functions for descriptions */
void id_bot(struct RoboAI *ai, struct blob *blobs);
struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col);
void track_agents(struct RoboAI *ai, struct blob *blobs);

// Display List functions
// the AI data structure provides a way for you to add graphical markers on-screen,
// the functions below add points or lines at a specified location and with the
// colour you want. Items you add will remain there until cleared. Do not mess
// with the list directly, use the functions below!
// Colours are specified as floating point values in [0,255], black is [0,0,0]
// white is [255,255,255].
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B);
struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B);
struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B);
struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B);
struct displayList *clearDP(struct displayList *head);

/****************************************************************************
 TO DO:
   Add headers for your own functions implementing the bot's soccer
   playing functionality below.
*****************************************************************************/

// TOSEE (edit as needed):
// basic, reusable soccer behaviours
void rotate_to_blob(struct RoboAI *ai, struct blob *target_blob);
void move_to_blob(struct RoboAI *ai, struct blob *target_blob);
void align_to_goal_with_ball(struct RoboAI *ai, double smx, double smy);
void kick_ball(struct RoboAI *ai);

// other helper functions declarations
double compute_angle_error_to_ball(struct RoboAI *ai, double smx, double smy);
double compute_distance_error(struct RoboAI *ai, double target_dist, double *dist_err, double *d_dist);
void quick_face_to_ball(struct RoboAI *ai, double smx, double smy);
void approach_to_ball(struct RoboAI *ai, double smx, double smy);
void rotate_to_goal(struct RoboAI *ai);
void rotate_step_blocking(double step_deg);
void chase_rotate(struct RoboAI *ai, double smx, double smy);

#endif
