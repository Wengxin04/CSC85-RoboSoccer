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
#include <stdbool.h>

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
#define ST_PENALTY_ROTATE_TO_TARGET 101 // Rotate to face the target
#define ST_PENALTY_MOVE_TO_TARGET 102    // Move to the target
#define ST_PENALTY_ROTATE_TO_BALL 103  // Rotate to face the ball
#define ST_PENALTY_MOVE_TO_BALL 104   // Move to the ball
#define ST_PENALTY_KICK_BALL 105      // Kick the ball
#define ST_PENALTY_DONE 199           // Penalty kick done

#define ST_MOTION_UPDATE1 110
#define ST_MOTION_UPDATE2 111

#define ST_PENALTY_EMPTY1 120
#define ST_PENALTY_EMPTY2 121

// Chase mode state definitions (200-299)
#define ST_CHASE_INIT 200              // Initial state
#define ST_CHASE_ROTATE_TO_BALL 201    // Rotate to face the ball
#define ST_CHASE_MOVE_TO_BALL 202       // Move to the ball
#define ST_CHASE_KICK_BALL 203        // Kick the ball
#define ST_CHASE_DONE 299              // Chase done

// Soccer mode state definitions (0-99)
#define ST_SOCCER_INIT 0              // Initial state
#define ST_SOCCER_ESCAPE_FROM_OPP 1   // Escape from opponent
#define ST_SOCCER_NORMAL_PLAY 2      // Normal play state
#define ST_SOCCER_EDGE_PLAY 3       // Edge play state
#define ST_SOCCER_DEFEND_GOAL 4		// Defend state - need a high priority

// escape from opponent substates
#define ST_SOCCER_ESCAPE_ROTATE 10
#define ST_SOCCER_ESCAPE_MOVE 11
#define ST_SOCCER_ESCAPE_DONE 12

// normal play substates: if the ball is kickable
#define ST_SOCCER_ROTATE_TO_TARGET 20
#define ST_SOCCER_MOVE_TO_TARGET 21
#define ST_SOCCER_ROTATE_TO_BALL 22
#define ST_SOCCER_MOVE_TO_BALL 23
#define ST_SOCCER_DRIBBLE_BALL 24 // to add
#define ST_SOCCER_SWERVE_OBSTACLE 25 // to add
#define ST_SOCCER_KICK_BALL 26
#define ST_SOCCER_NORMAL_PLAY_EMPTY1      27
#define ST_SOCCER_NORMAL_PLAY_EMPTY2      28
#define ST_SOCCER_NORMAL_PLAY_DONE        29

// edge play substates: if the ball is near the edge of the field (exit edge play when the ball is back to normal play area)
#define ST_SOCCER_ROTATE_TO_EDGE 30
#define ST_SOCCER_MOVE_TO_EDGE 31
#define ST_SOCCER_ROTATE_TO_KICK 32

// defend goal substates
#define ST_SOCCER_DEFEND_ROTATE 40
#define ST_SOCCER_DEFEND_MOVE 41
#define ST_SOCCER_DEFEND_DONE 42
#define ST_SOCCER_DEFEND_EMPTY 43

void soccer_normal_play_mode(struct RoboAI *ai, double *smx, double *smy);

#define ST_SOCCER_DONE 99             // Soccer done

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
void rotate_to_blob(struct RoboAI *ai, double smx, double smy, double target_x, double target_y);
void move_to_blob(struct RoboAI *ai, double smx, double smy, double t_cx, double t_cy, double target_dist);
void align_to_goal_with_ball(struct RoboAI *ai, double smx, double smy);
void kick_ball(struct RoboAI *ai);

// other helper functions declarations
double compute_angle_error_to_target(struct RoboAI *ai, double smx, double smy, double target_cx, double target_cy);
void compute_target_position(struct RoboAI *ai, double *target_cx, double *target_cy);
double compute_distance_error(struct RoboAI *ai, double target_dist, double *dist_err, double *d_dist, double target_cx, double target_cy);
void rotate_to_goal(struct RoboAI *ai);
void rotate_step_blocking(double step_deg);
void chase_rotate(struct RoboAI *ai, double smx, double smy);
void compute_goal_center(struct RoboAI *ai, double *gcx, double *gcy);
void compute_target_position_soccer(struct RoboAI *ai, double *target_cx, double *target_cy);
void compute_target_pos_general(struct RoboAI *ai, double gx, double gy, double delta, double *target_cx, double *target_cy);
void dribble_ball_towards_goal(struct RoboAI *ai);
int is_in_kicking_position(struct RoboAI *ai);
void swerve_around_obstacle(struct RoboAI *ai, struct blob *obstacle);
void defend_goal(struct RoboAI *ai);
double correct_motion_vector(double* smx, double*smy, double rotate_angle_deg);

bool need_escape(struct RoboAI *ai, double *smx, double *smy);
bool need_defense(struct RoboAI *ai);


#endif
