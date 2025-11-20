#include <unistd.h>
#include <math.h>
#include "imagecapture/imageCapture.h"

#include <stdbool.h>

// Tuning knobs for penalty routine
enum {
    FACE_THRESH_DEG   = 15,    // tweak
    ALIGN_THRESH_DEG  = 15,   // tweak
    TARGET_BALL_DIST  = 100,   // pixels; tweak to your scale
    TARGET_TARGET_DIST= 20,   // pixels; tweak to your scale
    CLOSE_BALL_SLACK  = 15,    // +/-
    BEHIND_BALL_GAP   = 10,    // min px robot should be "behind" ball wrt goal
    DELTA_TO_TARGET   = 350,    // temporary
    DEFEND_GOAL_THRESHOLD = 450, // temporary
};

bool is_facing_target(struct RoboAI *ai, double smx, double smy, double target_cx, double target_cy) {
    double e = compute_angle_error_to_target(ai, smx, smy, target_cx, target_cy);
    fprintf(stderr, "Angle error to target: %.2f deg\n", e);
    return !isnan(e) && fabs(e) <= FACE_THRESH_DEG;
}
bool is_close_to_target(struct RoboAI *ai, double target_cx, double target_cy) {
    if (!ai || !ai->st.self) return false;
    double dx = target_cx - ai->st.self->cx;
    double dy = target_cy - ai->st.self->cy;
    double dist = hypot(dx, dy);
    fprintf(stderr, "Distance to target: %.2f px\n", dist);
    return dist <= TARGET_TARGET_DIST;
}

bool is_close_to_ball(struct RoboAI *ai, double ball_cx, double ball_cy) {
    double de = 0, dd = 0;
    double d = compute_distance_error(ai, TARGET_BALL_DIST, &de, &dd, ball_cx, ball_cy);
    fprintf(stderr, "Distance to ball: %.2f px (err %.2f, d %.2f)\n", d, de, dd);
    return !isnan(d) && d <= (TARGET_BALL_DIST + CLOSE_BALL_SLACK);
}

// compute functions 

// 新增！

// side = 0 --> give right goal
// side = 1 --> give left goal
// default case can use ai->st.side
void compute_goal_center(int side, double *gcx, double *gcy)
{
  if (!ai || !ai->st.self || !gcx || !gcy) return;

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

void get_out_goal(double *gcx, double *gcy)
{
    compute_goal_center(ai->st.side, gcx, gcy);
}

// for goal
void compute_target_position_soccer(struct RoboAI *ai, double *target_cx, double *target_cy){
    double gx, gy;
    get_out_goal(&gx, &gy);
    compute_target_pos_general(ai, gx, gy,  DELTA_TO_TARGET, target_cx, target_cy);
}

#define DELTA_TO_OPP 500
// for obstacle avoidance in soccer mode
void compute_obstacle_avoidance_target(struct RoboAI *ai, double *target_cx, double *target_cy){
   compute_target_pos_general(ai, ai->st.opp->cx, ai->st.opp->cy,  DELTA_TO_OPP, target_cx, target_cy);
}

// default side using ai->st.side for 球门
void compute_target_pos_general(struct RoboAI *ai, double gx, double gy, double delta, double *target_cx, double *target_cy)
{
  // use ball's position as target for simplicity
  if (!ai || !ai->st.self || !ai->st.ball || !target_cx || !target_cy) return;
  
  double bx = ai->st.ball->cx;
  double by = ai->st.ball->cy;

  double dx = bx - gx;
  double dy = by - gy;
  double L = sqrt(dx*dx + dy*dy);

  double x = delta * fabs(dx) / L;;
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


////new things for opp compute

double compute_opp_angle_diff_to_target(struct RoboAI *ai, double target_x, double target_y)
{
    if (!ai || !ai->st.opp) return;

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
    // 我们只关心opp direction vector 所在直线和opp-target vector 的夹角
    // first normalize to [-pi, pi]
    while (ang_err >  M_PI) ang_err -= 2*M_PI;
    while (ang_err < -M_PI) ang_err += 2*M_PI;
    // reduce to [-pi/2, pi/2] because direction and its opposite represent the same line
    if (ang_err >  M_PI/2) ang_err -= M_PI;
    else if (ang_err < -M_PI/2) ang_err += M_PI;
    // convert to degrees
    return ang_err * (180.0 / M_PI);
}

double compute_opp_distance_to_target(struct RoboAI *ai, double target_cx, double target_cy)
{
    if (!ai || !ai->st.opp) return NAN;

    // position deltas
    double dx = target_cx - ai->st.opp->cx;
    double dy = target_cy - ai->st.opp->cy;
    double dist = hypot(dx, dy);
    return dist;
}

double compute_target_x(double target_y, double line_slope, double line_intercept){
    // line equation: y = mx + b  --> x = (y - b) / m
    if (fabs(line_slope) < 1e-6) {
        // vertical line case, slope is infinite
        return NAN; // or some error value
    }
    return (target_y - line_intercept) / line_slope;
}

void compute_escape_rotate_target(struct RoboAI *ai, double* target_x, double* target_y){
    double line_slope = 0.0;
    double line_intercept = 0.0;
    double x1 = ai->st.self->cx;;
    double y1 = ai->st.self->cy;
    double x2, y2 = 0.0;
    // get 自己球门位置
    compute_goal_center(1-ai->st.side, &x2, &y2);

    // 暂时不处理slope == infinite的情况 --> vertical line
    // 不太可能有这种情况出现

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

#define DEFENSE_THRESHOLD 300
#define OPP_FACE_THRESH_DEG 20
// 暂时不做更精准的判断
// 这里其实可以做更精准的判断
bool need_defense(struct RoboAI *ai){
    // opp 离球很近
    double opp_ball_dist = compute_opp_distance_to_target(ai, ai->st.ball->cx, ai->st.ball->cy);
    // opp 朝球的方向对齐
    double opp_ball_angle = compute_opp_angle_diff_to_target(ai, ai->st.ball->cx, ai->st.ball->cy);
    // opp 朝向我方球门的方向对齐
    double gx, gy;
    compute_goal_center(1 - ai->st.side, &gx, &gy);
    double opp_goal_angle = compute_opp_angle_diff_to_target(ai, gx, gy);
    return fabs(opp_ball_dist) < DEFENSE_THRESHOLD &&
           fabs(opp_ball_angle) < OPP_FACE_THRESH_DEG &&
           fabs(opp_goal_angle) < OPP_FACE_THRESH_DEG;
}

#define ESCAPE_THRESHOLD 300
// 暂时不做更精准的判断
// 这里其实可以做更精准的判断 --> OPP 的 blob里面的边框方向
bool need_escape(struct RoboAI *ai){
    return compute_opp_distance_to_target(ai, ai->st.self->cx, ai->st.self->cy) < ESCAPE_THRESHOLD;
}

////////// old things

void compute_target_position(struct RoboAI *ai, double *target_cx, double *target_cy)
{
  // use self's x and ball's y as target for simplicity
  if (!ai || !ai->st.self || !ai->st.ball || !target_cx || !target_cy) return;
  *target_cx = ai->st.self->cx;
  *target_cy = ai->st.ball->cy;
}


// 这个用作计算机器人当前朝向和球的角度差的helper func
// 返回值是度数(degrees)，正负表示方向, normalized to [-180, 180]
// smx, smy 是机器人的运动向量，不要用ai里面的，传入一个固定的最新的
double compute_angle_error_to_target(struct RoboAI *ai, double smx, double smy, double target_x, double target_y)
{
    if (!ai || !ai->st.self || !ai->st.ball) return NAN;

    // position deltas
    double dx = target_x - ai->st.self->cx;
    double dy = target_y - ai->st.self->cy;
    double ang_to_target = atan2(dy, dx);

    // normalize target direction vector
    double bn = sqrt(dx*dx + dy*dy);
    if (bn < 1e-3) return NAN;
    double bnx = dx / bn;
    double bny = dy / bn;

    double hdx = ai->st.sdx;
    double hdy = ai->st.sdy;
  // use motion vector to disambiguate heading direction
  // if dot product < 0, reverse heading direction
    double dot_heading_motion = hdx*smx + hdy*smy;  // 身体 vs 运动方向
    fprintf(stderr, "compute_angle_error_to_target: direction vectors: heading (%.2f, %.2f), motion (%.2f, %.2f), dot %.2f and target(%.2f, %.2f) and target dot %.2f\n",
            hdx, hdy, smx, smy, dot_heading_motion, bnx, bny, hdx*bnx + hdy*bny);
    // to do: fix 当机器人背对着球
    if ((smx != 0 || smy!= 0) && dot_heading_motion < -0.5) {
        // 校准robot heading 方向， 根据motion vector
        // 运动方向 == 头方向
        hdx = -hdx;
        hdy = -hdy;
        fprintf(stderr, "compute_angle_error_to_target: correcting heading direction based on motion vector\n");

    }

    double ang_bot = atan2(hdy, hdx);

    // 服了我自己了现实就分不清左右.....这个原本partial success只是概率吗？？
    // angle error
    double ang_err = ang_to_target - ang_bot;
    // then ang_err < 0 -> target is to the left of heading --> need turn left
    //      ang_err > 0 -> target is to the right of heading --> need turn right
    //坐标轴以top left 为原点，x向右，y向下 为positive

      
    // normalized to [-pi, pi]
    while (ang_err >  M_PI) ang_err -= 2*M_PI;
    while (ang_err < -M_PI) ang_err += 2*M_PI;
    // convert to degrees
    double ang_err_deg = ang_err * (180.0 / M_PI);

    fprintf(stderr, "compute_angle_error_to_target: angle error %.2f degrees with motion vector (%.2f, %.2f) and corrected heading (%.2f, %.2f)\n", ang_err_deg, smx, smy, hdx, hdy);
    return ang_err_deg;
}

// 计算机器人和球的距离误差 的helper func
// target_dist 是目标距离(e.g. 距离球40)， dist_err返回当前距离和目标距离的差值， d_dist返回距离的变化率
// 返回当前距离
double compute_distance_error(struct RoboAI *ai,
                              double target_dist,
                              double *dist_err,
                              double *d_ball_dist, double target_cx, double target_cy)
{
    if (!ai || !ai->st.self || !ai->st.ball) return NAN;

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
    if (dist_err) *dist_err = dist - target_dist;  // distance error to target

    // to do: check whether d_dist  works as expected
    // fprintf(stderr, "compute_distance_error1: current distance %.2f, distance error %.2f, distance change rate %.2f \n",
    //         dist,
    //         dist_err ? *dist_err : NAN,
    //         d_dist ? *d_dist : NAN);

    // fprintf(stderr,"compute_distance_error2: check cx cy: ball (%.2f, %.2f) self (%.2f, %.2f) and old self old cx cy (%.2f, %.2f)\n",
    //         ai->st.ball->cx, ai->st.ball->cy,
    //         ai->st.self->cx, ai->st.self->cy,
    //         ai->st.old_scx, ai->st.old_scy);

    return dist;
}

// correction function

double correct_motion_vector(double* smx, double*smy, double rotate_angle_deg){
  double rad = rotate_angle_deg * M_PI / 180.0;  
  double cos_rad = cos(rad);
  double sin_rad = sin(rad);

  double new_smx = (*smx) * cos_rad - (*smy) * sin_rad; // -sinx
  double new_smy = (*smx) * sin_rad + (*smy) * cos_rad; // +sinx

  *smx = new_smx;
  *smy = new_smy;
  return hypot(new_smx, new_smy);

}


// for soccer mode

#define BALL_IN_ATTACK_ZONE 1
#define BALL_NOT_IN_ATTACK_ZONE 0
#define BALL_VERY_CLOSE_TO_GOAL 2

#define OBSTACLE_DETECTED 1
#define NO_OBSTACLE 0

static int last_state = -1; // use to restore state after obstacle avoidance

static int check_ball_position(struct RoboAI *ai) {
  // TODOO: implement function to check ball position
  // return BALL_IN_ATTACK_ZONE or BALL_NOT_IN_ATTACK_ZONE or BALL_VERY_CLOSE_TO_GOAL
  double bx, by;
  bx = ai->st.ball->cx;
  by = ai->st.ball->cy;

  // ??? Need to reconsider when to defend, right now just consider when ball is very close to goal
  // check whether ball is very close to our goal
  double gx, gy; // goal position
  compute_goal_center(ai, &gx, &gy);
  double dist_to_goal = sqrt((bx - gx) * (bx - gx) + (by - gy) * (by - gy));
  if (dist_to_goal < DEFEND_GOAL_THRESHOLD) {
    return BALL_VERY_CLOSE_TO_GOAL;
  }

  // check whether ball is in edge or attack zone
  // sx and sy are length and width of the field respectively
  // random values for now: 200 pixels from edge for y, 400 pixels from edge for x
  if (by < 200 || by > (sy - 200) || bx < 400 || bx > (sx - 400)) {
    return BALL_NOT_IN_ATTACK_ZONE; // ball is in edge
  } else {
    return BALL_IN_ATTACK_ZONE; // ball is in attack zone
  }
}

// TODO: Consider obstacle case, in what cases we consider obstacle detected?
static int detect_obstacle(struct RoboAI *ai) {
  // TODOO: implement function to detect obstacles
  // return OBSTACLE_DETECTED or NO_OBSTACLE
  // for now, just return NO_OBSTACLE
  return NO_OBSTACLE;
}


