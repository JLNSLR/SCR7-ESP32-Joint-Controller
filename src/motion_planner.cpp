#include <motion_planner.h>
#include <cmath>
#include <cstdlib>
#include <stdint.h>
#include <Arduino.h>
#include <drive_system_settings.h>

//#define MOTION_PLAN_DEBUG



MotionPlanner::MotionPlanner(float max_vel, float max_accel, float max_deccel, float max_jerk, float max_jerk_deccel) {
    constraints.max_vel = max_vel;
    constraints.max_accel = max_accel;
    constraints.max_jerk = max_jerk;
    constraints.max_jerk_deccel = max_jerk_deccel;
    constraints.max_deccel = max_deccel;


    executing_traj_flag = false;

}


bool MotionPlanner::planMotion(float start_pos, float end_pos) {
    return planMotion(start_pos, end_pos, this->constraints);
}
bool MotionPlanner::planMotion(float start_pos, float end_pos, float max_vel, float max_accel, float max_jerk) {

    motion_constraints target_constraints;
    if (max_vel > 0) {
        target_constraints.max_vel = max_vel;
    }
    else {
        target_constraints.max_vel = this->constraints.max_vel;
    }
    if (max_accel > 0) {
        target_constraints.max_accel = max_accel;
        target_constraints.max_deccel = max_accel;
    }
    else {
        target_constraints.max_accel = this->constraints.max_accel;
        target_constraints.max_deccel = this->constraints.max_deccel;
    }
    if (max_jerk > 0) {
        target_constraints.max_jerk = max_jerk;
        target_constraints.max_jerk_deccel = max_jerk;
    }
    else {
        target_constraints.max_jerk = this->constraints.max_jerk;
        target_constraints.max_jerk_deccel = this->constraints.max_jerk_deccel;
    }

    return planMotion(start_pos, end_pos, target_constraints);
}
bool MotionPlanner::planMotion(float start, float target, motion_constraints constraints_new) {

    if (executing_traj_flag) {
        return false;
    }

    if (abs(start - target) < 0.05 * DEG2RAD) {
        return false;
    }
    feasible_next = constraints_new;
    feasible_next = gen_feasible_constraints(start, target, feasible_next);

    float jerk_k1;
    float jerk_k2;
    float t_phases[7];
    float t_phase_end_times[7];
    trajectory_point phase_points[8];
    float dir = 1.0;

    st_curve_parameters curve_params;
    curve_params.feasible = feasible_next;

    // Calculate Timing of ST-Curve

    float delta_pos;
    if (start > target) {
        dir = -1.0;
        delta_pos = start - target;
    }
    else {
        delta_pos = target - start;
    }


    delta_pos = abs(delta_pos);

    float inv_jerk = 1.0 / feasible_next.max_jerk;
    float inv_acc = 1.0 / feasible_next.max_accel;

    float inv_jerk_d = 1.0 / feasible_next.max_jerk_deccel;
    float inv_decc = 1.0 / feasible_next.max_deccel;

#ifdef MOTION_PLAN_DEBUG
    Serial.println(feasible_next.max_accel);
    Serial.println(feasible_next.max_deccel);

    Serial.println(feasible_next.max_vel);
#endif //




    float t_1 = 1.5 * feasible_next.max_accel * inv_jerk; //t1_ first accel phase
    float t_2 = feasible_next.max_vel * inv_acc - t_1;
    float t_4 = 1.5 * feasible_next.max_deccel * inv_jerk_d; //t4
    float t_5 = feasible_next.max_vel * inv_decc - t_4;

    float t_3 = (delta_pos / feasible_next.max_vel) - 0.5 * t_1 - feasible_next.max_vel * 0.5 * inv_acc - feasible_next.max_vel * 0.5 * inv_decc - t_4 * 0.5;

    t_phases[0] = t_1;
    t_phases[1] = t_2;
    t_phases[2] = t_1;
    t_phases[3] = t_3;
    t_phases[4] = t_4;
    t_phases[5] = t_5;
    t_phases[6] = t_4;



#ifdef MOTION_PLAN_DEBUG
    Serial.print("Phase Duration: ");
    Serial.print(t_1);
    Serial.print(" ");
    Serial.print(t_2);
    Serial.print(" ");
    Serial.print(t_1);
    Serial.print(" ");
    Serial.print(t_3);
    Serial.print(" ");
    Serial.print(t_4);
    Serial.print(" ");
    Serial.print(t_5);
    Serial.print(" ");
    Serial.println(t_4);

    Serial.print("Phase End times: ");
#endif // MOTION_PLAN_DEBUG

    t_phase_end_times[0] = t_phases[0];

#ifdef MOTION_PLAN_DEBUG
    Serial.print(t_phase_end_times[0]);
    Serial.print(" ");
#endif // MOTION_PLAN_DEBUG

    for (int i = 1; i < 7; i++) {
        t_phase_end_times[i] = t_phases[i] + t_phase_end_times[i - 1];
#ifdef MOTION_PLAN_DEBUG
        Serial.print(t_phase_end_times[i]);
        Serial.print(" ");
#endif // MOTION_PLAN_DEBUG

    }
#ifdef MOTION_PLAN_DEBUG
    Serial.println("");
#endif // MOTION_PLAN_DEBUG

    float jerk_common_divisor = (t_5 + 2 * t_4 + 2 * t_3 + 2 * t_1 + t_2);
    jerk_k1 = delta_pos * 12.0 / (t_1 * t_1 * t_1 * (t_1 + t_2) * jerk_common_divisor);
    jerk_k2 = delta_pos * 12.0 / (t_4 * t_4 * t_4 * (t_4 + t_5) * jerk_common_divisor);


    curve_params.dir = dir;
    curve_params.jerk_k1 = jerk_k1;
    curve_params.jerk_k2 = jerk_k2;
    curve_params.execution_time = t_phase_end_times[6];

    for (int i = 0; i < 7; i++) {
        curve_params.t_phase_end_times[i] = t_phase_end_times[i];
        curve_params.t_phases[i] = t_phases[i];


    }


    curve_params.feasible = feasible_next;

    // Calculate Phase points;
    phase_points[0].position = start;
    phase_points[0].velocity = 0;
    phase_points[0].acceleration = 0;
    phase_points[1] = phase1_acceleration_buildup(t_1, phase_points[0], curve_params);
    phase_points[2] = phase2_upper_acc_limit(t_2, phase_points[1], curve_params);
    phase_points[3] = phase3_acceleration_ramp_down(t_1, phase_points[2], curve_params);
    phase_points[4] = phase4_const_velocity(t_3, phase_points[3], curve_params);
    phase_points[5] = phase5_deceleration_buildup(t_4, phase_points[4], curve_params);
    phase_points[6] = phase6_low_deceleration_limit(t_5, phase_points[5], curve_params);
    phase_points[7].position = target;
    phase_points[7].velocity = 0;
    phase_points[7].acceleration = 0;

#ifdef MOTION_PLAN_DEBUG
    for (int i = 0; i < 8; i++) {
        Serial.print(phase_points[i].acceleration);
        Serial.print(" ");
        Serial.print(phase_points[i].velocity);
        Serial.print(" ");
        Serial.print(phase_points[i].position);
        Serial.println("  ");
    }

    Serial.println("dir");
    Serial.println(dir);
#endif // MOTION_PLAN_DEBUG


    curve_params.dir = dir;
    curve_params.jerk_k1 = jerk_k1;
    curve_params.jerk_k2 = jerk_k2;
    curve_params.execution_time = t_phase_end_times[6];

    for (int i = 0; i < 8; i++) {
        curve_params.phase_points[i] = phase_points[i];
    }


    current_curve = curve_params;
    internal_time = 0.0;
    executing_traj_flag = true;

    return true;



}


trajectory_point MotionPlanner::sequenceMotion(float delta_t) {

    float t_epsilon = -0.0;

    trajectory_point traj_point;

    internal_time += delta_t;

    if (executing_traj_flag == false) {
        traj_point = current_point;
    }

    if (internal_time < current_curve.t_phase_end_times[0]) {
        traj_point = phase1_acceleration_buildup(internal_time, current_curve.phase_points[0], current_curve);
        executing_traj_flag = true;
    }
    if ((internal_time - t_epsilon >= current_curve.t_phase_end_times[0] && internal_time < current_curve.t_phase_end_times[1] && current_curve.t_phases[0] > 0)) {

        traj_point = phase2_upper_acc_limit(internal_time - current_curve.t_phase_end_times[0], current_curve.phase_points[1], current_curve);
    }
    if (internal_time - t_epsilon >= current_curve.t_phase_end_times[1] && internal_time < current_curve.t_phase_end_times[2]) {

        traj_point = phase3_acceleration_ramp_down(internal_time - current_curve.t_phase_end_times[1], current_curve.phase_points[2], current_curve);
    }
    if (internal_time - t_epsilon >= current_curve.t_phase_end_times[2] && internal_time < current_curve.t_phase_end_times[3]) {
        traj_point = phase4_const_velocity(internal_time - current_curve.t_phase_end_times[2], current_curve.phase_points[3], current_curve);
    }
    if (internal_time - t_epsilon >= current_curve.t_phase_end_times[3] && internal_time < current_curve.t_phase_end_times[4]) {
        traj_point = phase5_deceleration_buildup(internal_time - current_curve.t_phase_end_times[3], current_curve.phase_points[4], current_curve);
    }
    if (internal_time - t_epsilon >= current_curve.t_phase_end_times[4] && internal_time < current_curve.t_phase_end_times[5]) {
        traj_point = phase6_low_deceleration_limit(internal_time - current_curve.t_phase_end_times[4], current_curve.phase_points[5], current_curve);
    }
    if (internal_time - t_epsilon >= current_curve.t_phase_end_times[5] && internal_time < current_curve.t_phase_end_times[6]) {
        traj_point = phase7_deceleration_ramp_down(internal_time - current_curve.t_phase_end_times[5], current_curve.phase_points[6], current_curve);
    }
    if (internal_time - t_epsilon >= current_curve.t_phase_end_times[6]) {
        executing_traj_flag = false;
        traj_point = current_curve.phase_points[7];
    }

    return traj_point;

}

trajectory_point MotionPlanner::phase1_acceleration_buildup(float t, trajectory_point start, st_curve_parameters curve) {

    trajectory_point point;

    float t1 = curve.t_phases[0];

    float jerk_k1 = curve.jerk_k1;
    float dir = curve.dir;

    point.acceleration = dir * jerk_k1 * (t1 * t * t * 0.5 - t * t * t * 1.0 / 3.0) + start.acceleration;
    point.velocity = dir * jerk_k1 * (t1 * t * t * t * 0.1666 - t * t * t * t * 0.08333) + start.velocity + start.acceleration * t;
    point.position = dir * jerk_k1 * (t1 * t * t * t * t * 0.04166 - t * t * t * t * t * 0.0166) + start.position + start.velocity * t + start.acceleration * 0.5 * t * t;
    current_point = point;
    return point;


}
trajectory_point MotionPlanner::phase2_upper_acc_limit(float t, trajectory_point start, st_curve_parameters curve) {

    trajectory_point point;
    float dir = curve.dir;
    point.acceleration = dir * curve.feasible.max_accel;
    point.velocity = point.acceleration * t + start.velocity;
    point.position = start.velocity * t + point.acceleration * t * t * 0.5 + start.position;
    current_point = point;

    return point;

}
trajectory_point MotionPlanner::phase3_acceleration_ramp_down(float t, trajectory_point start, st_curve_parameters curve) {

    trajectory_point point;
    float t1 = curve.t_phases[2];
    float jerk_k1 = curve.jerk_k1;
    float dir = curve.dir;
    point.acceleration = dir * -jerk_k1 * (t1 * t * t * 0.5 - t * t * t * 1.0 / 3.0) + dir * curve.feasible.max_accel;
    point.velocity = dir * -jerk_k1 * (t1 * t * t * t * 0.1666 - t * t * t * t * 0.08333) + start.velocity + dir * curve.feasible.max_accel * t;
    point.position = dir * -jerk_k1 * (t1 * t * t * t * t * 0.04166 - t * t * t * t * t * 0.0166) + start.position + start.velocity * t + dir * 0.5 * t * t * curve.feasible.max_accel;
    current_point = point;

    return point;

}
trajectory_point MotionPlanner::phase4_const_velocity(float t, trajectory_point start, st_curve_parameters curve) {
    trajectory_point point;
    float dir = curve.dir;
    point.acceleration = 0.0;
    point.velocity = dir * curve.feasible.max_vel;
    point.position = point.velocity * t + start.position;
    current_point = point;
    return point;

}
trajectory_point MotionPlanner::phase5_deceleration_buildup(float t, trajectory_point start, st_curve_parameters curve) {
    trajectory_point point;
    float t1 = curve.t_phases[4];
    float jerk_k2 = curve.jerk_k2;
    float dir = curve.dir;


    point.acceleration = dir * -jerk_k2 * (t1 * t * t * 0.5 - t * t * t * 1.0 / 3.0);
    point.velocity = dir * -jerk_k2 * (t1 * t * t * t * 0.1666 - t * t * t * t * 0.08333) + start.velocity;
    point.position = dir * -jerk_k2 * (t1 * t * t * t * t * 0.04166 - t * t * t * t * t * 0.0166) + start.position + start.velocity * t;
    current_point = point;
    return point;

}
trajectory_point MotionPlanner::phase6_low_deceleration_limit(float t, trajectory_point start, st_curve_parameters curve) {
    trajectory_point point;
    float dir = curve.dir;


    point.acceleration = -curve.feasible.max_deccel * dir;
    point.velocity = point.acceleration * t + start.velocity;
    point.position = start.velocity * t + point.acceleration * t * t * 0.5 + start.position;
    current_point = point;
    return point;
}
trajectory_point MotionPlanner::phase7_deceleration_ramp_down(float t, trajectory_point start, st_curve_parameters curve) {
    trajectory_point point;
    float t1 = curve.t_phases[6];

    float jerk_k2 = curve.jerk_k2;
    float dir = curve.dir;
    point.acceleration = dir * jerk_k2 * (t1 * t * t * 0.5 - t * t * t * 1.0 / 3.0) - dir * curve.feasible.max_deccel;
    point.velocity = dir * jerk_k2 * (t1 * t * t * t * 0.1666 - t * t * t * t * 0.08333) + start.velocity - dir * curve.feasible.max_deccel * t;
    point.position = dir * jerk_k2 * (t1 * t * t * t * t * 0.04166 - t * t * t * t * t * 0.0166) + start.position + start.velocity * t - curve.feasible.max_deccel * 0.5 * t * t * dir;
    current_point = point;
    return point;
}



motion_constraints MotionPlanner::gen_feasible_constraints(float start, float end, motion_constraints constraints) {

    // initial constraints

    motion_constraints feasible = constraints;

    float epsilon = 0.5;


    float dir = 1.0;

    float delta_pos;

    if (start > end) {
        dir = -1.0;
        delta_pos = start - end;

    }
    else {
        delta_pos = end - start;
    }
    delta_pos = abs(delta_pos);

    // Check Acceleration

    float curve_vel = (1.5 * feasible.max_accel * feasible.max_accel) / feasible.max_jerk;


    if (curve_vel > feasible.max_vel) {

        feasible.max_accel = sqrt(feasible.max_jerk * feasible.max_vel * 0.666f) * (1 - epsilon);
    }

    // Check Decceleration
    curve_vel = 1.5 * feasible.max_deccel * feasible.max_deccel / feasible.max_jerk_deccel;

    if (curve_vel > feasible.max_vel) {

        feasible.max_deccel = sqrt(feasible.max_jerk_deccel * feasible.max_vel * 0.666f) * (1 - epsilon);
    }


    // Check Velocity



    float t_curve_acc = 0.75 * abs(feasible.max_accel * dir) / feasible.max_jerk + 0.75 * feasible.max_deccel / feasible.max_jerk_deccel;

    float t_pos_min = delta_pos / feasible.max_vel;

    float t_curve = t_curve_acc + feasible.max_vel * 0.5 / (feasible.max_accel) + feasible.max_vel * 0.5 / feasible.max_deccel;


    while (t_pos_min < t_curve) {

        //reduce v_feasible until minimum time to reach position is large enough for S-Curve
        feasible.max_vel = (abs((t_pos_min / t_curve))) * feasible.max_vel * 0.9;
        t_pos_min = delta_pos / feasible.max_vel;
        t_curve = t_curve_acc + feasible.max_vel * 0.5 / (feasible.max_accel) + feasible.max_vel * 0.5 / feasible.max_deccel;

    }
    // Check Acceleration

    curve_vel = 1.5 * feasible.max_accel * feasible.max_accel / feasible.max_jerk;


    if (curve_vel > feasible.max_vel) {

        feasible.max_accel = sqrt(feasible.max_jerk * feasible.max_vel * 0.666f) * (1 - epsilon);
    }

    // Check Decceleration
    curve_vel = 1.5 * feasible.max_deccel * feasible.max_deccel / feasible.max_jerk_deccel;

    if (curve_vel > feasible.max_vel) {

        feasible.max_deccel = sqrt(feasible.max_jerk_deccel * feasible.max_vel * 0.666f) * (1 - epsilon);
    }


    return feasible;


}


