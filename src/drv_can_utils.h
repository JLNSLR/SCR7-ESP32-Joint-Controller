#ifndef DRV_CAN_UTILS
#define DRV_CAN_UTILS

#define DRVSYS_VERSION
#include<stdint.h>


#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

#include <robot_global_def.h>
#include <drive_system_types.h>


union portable_float_type {
    float value;
    uint32_t binary;
};

int32_t convert_float_to_integer(const unsigned int n_bits, const float max_val, const float value);

float convert_integer_to_float(const unsigned int n_bits, const float max_val, const int32_t value);


/* CAN-ID used to identify origin and type of a CAN message.
11 bit id: 0x000 - 0x7ff
first three bits  - robot sys id -> default value: 000 - 0x0
bits 3 - 6: robot joint id -> values 0x0 -> 0x6
bits 7-10: message type: values: 0x0 -> 0xF

*/

/* message type id: 4 bit value 0x0 -> 0xF */
enum drv_can_msg_type_id {
    drive_state_msg = 0x0, drive_traj_target = 0x1, go_to_target = 0x2, controller_state_msg = 0x3, mode_command = 0x4,
    drive_param_command = 0x5, drive_sys_light_command = 0x6, drive_direct_torque_command = 0x7, drive_load_state_msg = 0x8,
    drive_constants = 0x9, drive_gain_data = 0xA, drive_controller_command = 0xB
};

enum drv_can_motionCmdType { traj_command, direct_command, goto_command };


struct drvComm_CANID {
    union {
        uint16_t msg_id; //11bit id 0x7FF
        struct {
            int sys_id : 3;
            int joint_id : 4;
            drv_can_msg_type_id msg_type : 4;
        };
    };
};

#define DRVCOMM_CONTROLLER_CMD_LENGTH 2
struct drvComm_modeCmd {
    bool start : 1;
    bool stop : 1;
    drvSys_controlMode mode : 8;
};


enum drvComm_parameterType {
    torque_limit, pos_limit_high, pos_limit_low, max_vel, torque_sensor_int_offset,
    torque_sensor_ext_offset, torque_sensor_conversion_factor, nn_pid_learning_rate, nn_pid_max_learning_rate, nn_pid_gain_penalty
};

#define DRVCOMM_PARAMS_CMD_LENGTH 5
struct drvComm_paramsCmd {
    drvComm_parameterType type : 8;
    uint32_t param_data : 32;
};

enum drvComm_controllerCmdType {
    reset, delete_calibration, save_nn, reset_nn, set_torque_sensor_calibrated,
    reset_torque_sensor, set_nn_ff_active, set_nn_ff_inactive, set_position_control_active, set_position_control_inactive
};
struct drvComm_controllerCmd {
    drvComm_controllerCmdType type : 8;
};

#define DRVCOMM_DRIVE_STATE_LENGTH 7
struct drvComm_DriveStatePacket {
    int pos : 14;
    int vel : 14;
    int acc : 14;
    int m_torque : 9;
};
#define DRVCOMM_LOAD_MSG_LENGTH 7
struct drvComm_loadStatePacket {
    int torque_sensor_val : 24;
    int position_delta : 14;
    bool torque_sensor_calibrated : 1;

};

enum drvComm_gainType { pos_gain, vel_gain, pos_step_gain, ff_gain };
#define DRVCOMM_GAIN_LENGTH 8
struct drvComm_gains_Packet {
    drvComm_gainType type : 2;
    unsigned int val0 : 16;
    unsigned int val1 : 16;
    unsigned int val2 : 16;
};

struct drvComm_gain_cmd {
    drvComm_gainType type;
    float val0;
    float val1;
    float val2;

};

#define DRVCOMM_CONTROLLER_STATE_LENGTH 6
struct drvComm_ControllerState {
    int stateFlag : 3;
    int mode : 3;
    int calibrated : 1;
    int position_control : 1;
    int feed_forward_control : 1;
    int hit_endstop : 2;
    int temperature : 12;
    int overtemperature : 1;
    int overtemp_warn : 1;
    int torque_limit : 8;
    int fan_level : 4;
    int neural_control : 1;
};

#define DRVCOMM_LIGHT_CMD_LENGTH 5
struct drvComm_LightCmd {
    int rgb_hsv : 1;
    int rh : 8;
    int gs : 8;
    int bv : 8;
    int period_ms : 12;
};


/* Motion command Types:
    0 - new targets (pos, vel_ff, torque_ff)
    1 - go to new pos (interpolate to pos)
    2 - direct mode
    3 - new targets hybrid (pos, vel_ff, torque_ff, joint_torque)
*/
#define DRVCOMM_TRAJ_CMD_LENGTH 8
struct drvComm_MotionCmd_traj_target {
    int pos : 16;
    int vel : 14;
    int acc : 13;
    int torque_ff : 9;
    int position_control : 1;
    int velocity_control : 1;
    int feedforward : 1;
};
#define DRVCOMM_GOTO_CMD_LENGTH 6
struct drvComm_MotionCmd_goTo_target_packet {
    int pos : 14;
    int vel_max : 14;
    int acc_max : 13;
};


#define DRVCOMM_DIRECT_TORQUE_CMD_LENGTH 2
struct drvComm_MotionCmd_direct_motor_torque_target {
    int m_torque_target : 9;
};

struct drvComm_DriveState {
    float pos;
    float vel;
    float acc;
    float motor_torque;
    float joint_torque;
};

struct drvComm_goTo_target {
    float target_pos;
    float vel_max;
    float acc_max;
};

struct drvComm_load_state {
    float torque_sensor_val_Nm;
    float delta_pos;
    int torque_sensor_raw;
    bool calibrated;
};

//Drive State Data
drvComm_DriveStatePacket drvComm_pack_drive_state(drvSys_driveState state, const  int joint_id);
drvSys_driveState drvComm_unpack_drive_state(drvComm_DriveStatePacket state_packet, const  int joint_id);

// Load State Data
drvComm_loadStatePacket  drvComm_pack_load_state(float torque_sensor_val, float delta_pos, bool calibrated, const int joint_id);
drvComm_load_state drvComm_unpack_load_state(drvComm_loadStatePacket load_state_packet, int joint_id);

// Trajectory Target Command
drvSys_driveControlTargets drvComm_unpack_traj_command(drvComm_MotionCmd_traj_target targets, const int joint_id);
drvComm_MotionCmd_traj_target drvComm_pack_traj_command(drvSys_driveTargets targets, const int joint_id, bool position_control = true,
    bool velocity_control = true, bool feedforward = true);

//PID Parameter update
drvComm_gain_cmd drvComm_unpack_gain_packet(drvComm_gains_Packet gains_packet);
drvComm_gains_Packet drvComm_pack_gain_packet(drvComm_gain_cmd gains);

//Direct torque command
drvComm_MotionCmd_direct_motor_torque_target drvComm_pack_direct_torque_command(float target_torque, const int joint_id);
float drvComm_unpack_direct_torque_command(drvComm_MotionCmd_direct_motor_torque_target torque_val, const int joint_id);

// Controller State Message
drvComm_ControllerState drvComm_pack_controllerState(drvSys_controllerCondition controller_state, const int joint_id, float torque_limit);
drvSys_controllerCondition drvComm_unpack_contollerState(drvComm_ControllerState state_data, const int joint_id);

// Go to command
drvComm_goTo_target drvComm_unpack_go_to_command(drvComm_MotionCmd_goTo_target_packet goto_packet);
drvComm_MotionCmd_goTo_target_packet drvComm_pack_go_to_command(drvComm_goTo_target goto_target);

// Control mode command
drvComm_modeCmd drvComm_gen_controller_command(bool start = true, bool stop = false, drvSys_controlMode control_mode = closed_loop_foc);

// Parameter command generation
drvComm_paramsCmd drvComm_gen_params_command(drvComm_parameterType type, float param_value);
//Lightcommand
drvComm_LightCmd drvComm_gen_light_command(bool rgb, uint8_t rh, uint8_t gs, uint8_t gv, int period_ms);




#endif // ! DRV_CAN_UTILS
