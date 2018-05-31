#ifndef YUMI_SUTURE_DEF_H
#define YUMI_SUTURE_DEF_H

#include <cstdint>
#include <stdint.h>

/**
 * Suturing device
 */
enum SUTURE_CTRL{
    SUTURE_ENABLE_MOTOR = 1 << 0,
    SUTURE_DISABLE_MOTOR = 1 << 1,
    SUTURE_RUN_STITCH = 1 << 2,
    SUTURE_SPEED_PLUS = 1 << 3,
    SUTURE_SPEED_MINUS = 1 << 4,
    SUTURE_RUN_PIERCE = 1 << 5
};
typedef uint32_t SUTURE_CTRL_CMD;

#define Motor0 0
#define Motor1 1
#define ERROR_INDEX   -100000000
#define TIMEOUT_INDEX -200000000
#define TIMEOUT 800
#define NODENO 2

struct deviceInfomation
{
    double msrPos[NODENO];
    double cmdPos[NODENO];
    double msrTem[NODENO];
    double msrCur[NODENO];
    bool   connectSig;
    bool   enableSig;
    int    status;
    double speedScale;
    double absLockUpPos;
    double absLockDownPos;
    double absOpenPos;
    double absClosePos;
};


/**
 * Visual servoing
 */

/// yumi_gui publish
enum SERVO_CTRL{
    SERVO_START = 1 << 0,
    SERVO_DMOVE_MODE = 1 << 1,
    SERVO_SELECT_MODE = 1 << 2,
    SERVO_RESET = 1 << 3,

    SERVO_DTRANS_XP = 1 << 4,
    SERVO_DTRANS_XN = 1 << 5,
    SERVO_DTRANS_YP = 1 << 6,
    SERVO_DTRANS_YN = 1 << 7,
    SERVO_DTRANS_ZP = 1 << 8,
    SERVO_DTRANS_ZN = 1 << 9,
    SERVO_DROT_XP = 1 << 10,
    SERVO_DROT_XN = 1 << 11,
    SERVO_DROT_YP = 1 << 12,
    SERVO_DROT_YN = 1 << 13,
    SERVO_DROT_ZP = 1 << 14,
    SERVO_DROT_ZN = 1 << 15,

    SERVO_ADD_PT = 1 << 16,
    SERVO_RM_PT = 1 << 17,

    SERVO_NEXT_STEP = 1 << 18
};
typedef uint32_t SERVO_CTRL_CMD;

/// yumi_vis_servo publish
enum TASK_STATE{
    STANDBY = 0,
//    SERVO_RUNNING = 1,
            PLAN_TRAJ = 1,
    GOTO_HOME_VIS = 2,
    GOTO_TARGET = 3,
    RUN_STITCH = 4,
    RUN_PIERCE = 5,
    RUN_ROTATION = 6,
    PULL_THREAD = 7,
    GOTO_HOME_KIN = 8
};
typedef uint32_t SUTURE_TASK_STATE;

#endif  // YUMI_SUTURE_DEF_H
