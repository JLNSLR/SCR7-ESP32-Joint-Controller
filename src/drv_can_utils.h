#ifndef DRV_CAN_UTILS
#define DRV_CAN_UTILS

#define DRVSYS_VERSION

#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

#ifdef DRVSYS_VERSION
#include <drive_system.h>
#include <motion_interface.h>
#endif //DRVSYS_VERSION

#define MAX_VEL_DATA_DEG_PER_S 240 // (°/s)
#define MAX_POS_DATA_DEG 180 // °
#define MAX_ACC_DATA_DEG 1250
#define MAX_TORQUE_DATA_NM 100

#define MAX_POS_FACTOR  0.00549316
#define MAX_POS_FACTOR_PACK  182.044444
#define MAX_VEL_FACTOR  0.012207
#define MAX_VEL_FACTOR_PACK  81.92
#define MAX_ACC_FACTOR  0.0610351
#define MAX_ACC_FACTOR_PACK  16.384

#define MAX_MOTOR_TORQUE_VALUE 2.4 //Nm

#define MAX_MOTOR_TEMP 150.0

#define MAX_PID_GAIN_VAL 1000.0



const float pos_data_14bit_to_val = MAX_POS_DATA_DEG * DEG2RAD * (1.0 / 8192);
const float vel_data_13bit_to_val = MAX_VEL_DATA_DEG_PER_S * DEG2RAD * (1.0 / 4096);
const float acc_data_13bit_to_val = MAX_ACC_DATA_DEG * DEG2RAD * (1.0 / 4096);

const float pos_to_14bit_data = 1.0 / pos_data_14bit_to_val;
const float vel_to_13bit_data = 1.0 / vel_data_13bit_to_val;
const float acc_to_13bit_data = 1.0 / acc_data_13bit_to_val;

const float joint_torque_data_15bit_to_val = float(MAX_TORQUE_DATA_NM) / float(16384);
const float joint_torque_to_15bit_data = 1.0 / joint_torque_data_15bit_to_val;

const float motor_torque_data_9bit_to_val = DRVSYS_TORQUE_LIMIT / 255;
const float motor_torque_to_9bit_data = 1 / motor_torque_data_9bit_to_val;

#ifdef DRVSYS_VERSION
const float max_motor_torque_12bit_data_to_val = MAX_MOTOR_TORQUE_VALUE / 4096;
const float max_motor_torque_val_to_12bit_data = 1.0 / max_motor_torque_12bit_data_to_val;
#endif

const float motor_temp_12bit_to_val = MAX_MOTOR_TEMP / 4096;
const float motor_temp_val_to_12bit = 1.0 / motor_temp_12bit_to_val;

const float pid_gain_data_20bit_to_val = MAX_PID_GAIN_VAL * (1.0 / 1048576);
const float pid_gain_val_to_20bit = 1.0 / pid_gain_data_20bit_to_val;




/* CAN-ID used to identify origin and type of a CAN message.
11 bit id: 0x000 - 0x7ff
first three bits  - robot sys id -> default value: 000 - 0x0
bits 3 - 6: robot joint id -> values 0x0 -> 0x6
bits 7-10: message type: values: 0x0 -> 0xF

*/

/* message type id: 4 bit value 0x0 -> 0xF */
enum drv_can_msg_type_id {
    drive_state_msg = 0x0, drive_traj_target = 0x1, go_to_target = 0x2, controller_state_msg = 0x3, mode_command = 0x4,
    drive_param_command = 0x5, drive_sys_light_command = 0x6, drive_direct_torque_command = 0x7,
    reset_command = 0x8, drive_constants = 0x9, drive_pid_data = 0xA

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
struct drvComm_controllerCmd {
    bool start : 1;
    bool stop : 1;
    drvSys_controlMode mode : 8;
};


enum drvComm_parameterType { torque_limit, pos_limit_high, pos_limit_low, max_vel, max_current_mA };

#define DRVCOMM_PARAMS_CMD_LENGTH 5
struct drvComm_paramsCmd {
    drvComm_parameterType type : 8;
    int parameter : 32;
};

#define DRVCOMM_DRIVE_STATE_LENGTH 8
struct drvComm_DriveStatePaket {
    int pos : 14;
    int vel : 13;
    int acc : 13;
    int m_torque : 9;
    int joint_torque : 15;
};

#define DRVCOMM_PID_GAIN_LENGTH 8
struct drvComm_PID_Gains_Paket {
    int type : 1;
    int p : 20;
    int i : 20;
    int d : 20;
};

#define DRVCOMM_CONTROLLER_STATE_LENGTH 5
struct drvComm_ControllerState {
    int stateFlag : 3;
    int mode : 3;
    int calibrated : 1;
    int hit_endstop : 2;
    int temperature : 12;
    int overtemperature : 1;
    int overtemp_warn : 1;
    int max_motor_torque : 12;
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
    int pos : 14;
    int vel : 13;
    int acc : 13;
    int torque_ff : 9;
    int ref_torque : 15;
};
#define DRVCOMM_GOTO_CMD_LENGTH 5
struct drvComm_MotionCmd_goTo_target_packet {
    int pos : 14;
    int vel_max : 13;
    int acc_max : 13;
};

struct drvComm_goTo_target {
    float target_pos;
    float vel_max;
    float acc_max;
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

#ifdef DRVSYS_VERSION
drvComm_DriveStatePaket drvComm_pack_drive_state(drvSys_driveState state);

drvSys_driveState drvComm_unpack_drive_state(drvComm_DriveStatePaket state_packet);

drvSys_driveState drvComm_unpack_drive_state(drvComm_DriveStatePaket state_packet, float max_motor_torque);

drvSys_PID_Gains drvComm_unpack_pid_packet(drvComm_PID_Gains_Paket pid_packet);

drvComm_PID_Gains_Paket drvComm_pack_pid_packet(drvSys_PID_Gains gains, bool pos_controller);

drvSys_driveTargets drvComm_unpack_traj_command(drvComm_MotionCmd_traj_target targets);

drvSys_driveTargets drvComm_pack_traj_command(drvComm_MotionCmd_traj_target targets);

drvSys_driveTargets drvComm_pack_traj_command(drvComm_MotionCmd_traj_target targets, float max_motor_torque);

drvComm_MotionCmd_direct_motor_torque_target drvComm_pack_direct_torque_command(float target_torque, float max_torque);

drvComm_ControllerState drvComm_pack_controllerState(drvSys_controllerCondition controller_state, float max_torque);

drvSys_controllerCondition drvComm_unpack_contollerState(drvComm_ControllerState state_data);

#endif


drvComm_goTo_target drvComm_unpack_go_to_command(drvComm_MotionCmd_goTo_target_packet goto_packet);

drvComm_MotionCmd_goTo_target_packet drvComm_pack_go_to_command(drvComm_goTo_target goto_target);

float drvComm_unpack_direct_torque_command(drvComm_MotionCmd_direct_motor_torque_target torque_val);

drvComm_controllerCmd drvComm_gen_controller_command(bool start = true, bool stop = false, drvSys_controlMode control_mode = closed_loop_foc);

drvComm_paramsCmd drvComm_gen_params_command(drvComm_parameterType type, float param_value);




#endif // ! DRV_CAN_UTILS
