#ifndef __SHM_STRUCT_H__
#define __SHM_STRUCT_H__

#define MAX_JOINT_NUM 64
#define MAX_IMU_NUM 2
#define MAX_FSENSOR_NUM 4

#define MAX_JOINT_NAME_LENGTH 64
struct shm_struct {
  float ref_angle[MAX_JOINT_NUM];
  float act_angle[MAX_JOINT_NUM];
  //float pgain[MAX_JOINT_NUM];
  //float dgain[MAX_JOINT_NUM];
  float ref_velocity[MAX_JOINT_NUM];
  float act_velocity[MAX_JOINT_NUM];

  char servo_on[MAX_JOINT_NUM];
  char servo_off[MAX_JOINT_NUM];
  char loopback[MAX_JOINT_NUM];
  char joint_enable[MAX_JOINT_NUM];

  int joint_offset[MAX_JOINT_NUM];
  int servo_state[MAX_JOINT_NUM];

  int16_t ref_stroke[MAX_JOINT_NUM]; // reference stroke
  int16_t snt_stroke[MAX_JOINT_NUM]; // sent stroke
  int16_t act_stroke[MAX_JOINT_NUM]; // current stroke

  float body_omega[MAX_IMU_NUM][3];
  float body_acc[MAX_IMU_NUM][3];
  float body_posture[MAX_IMU_NUM][4]; //Quaternion
  float zero_acc[MAX_IMU_NUM][3];

  float force_sensor[MAX_FSENSOR_NUM][6];
  float force_sensor_offset[MAX_FSENSOR_NUM][6];

  long frame;
  float jitter;
  // char joint_names[MAX_JOINT_NUM][MAX_JOINT_NAME_LENGTH];
};

#endif
