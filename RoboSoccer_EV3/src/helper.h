

#ifndef _HELPER_H
#define _HELPER_H

#include "imagecapture/imageCapture.h"
#include "API/btcomm.h"
#include <stdio.h>
#include <stdlib.h>
#include "roboAI.h"
#include <stdbool.h>

#define LEFT_MOTOR MOTOR_C
#define RIGHT_MOTOR MOTOR_D
#define KICK_MOTOR MOTOR_A
#define GYRO_PORT PORT_4

bool is_facing_target(struct RoboAI *ai, double smx, double smy, double target_cx, double target_cy);
bool is_close_to_target(struct RoboAI *ai, double target_cx, double target_cy);
bool is_close_to_ball(struct RoboAI *ai, double ball_cx, double ball_cy);

double compute_angle_error_to_target(struct RoboAI *ai, double smx, double smy, double target_cx, double target_cy);
void compute_target_position(struct RoboAI *ai, double *target_cx, double *target_cy);
double compute_distance_error(struct RoboAI *ai, double target_dist, double *dist_err, double *d_dist, double target_cx, double target_cy);
void compute_goal_center(struct RoboAI *ai, double *gcx, double *gcy);
void compute_target_position_soccer(struct RoboAI *ai, double *target_cx, double *target_cy);

double correct_motion_vector(double* smx, double*smy, double rotate_angle_deg);


#endif