#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>
#include <algorithm>
#include <stdexcept>

#include <time.h>
#include <sched.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <signal.h>

#include <stdint.h>
#include <unistd.h>
//#include <boost/format.hpp>
//#include <boost/program_options.hpp>
//#include <boost/circular_buffer.hpp>

#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AeroControllers.hh"

#include "aero_hardware_interface/AngleJointNames.hh"
#include "aero_hardware_interface/Stroke2Angle.hh"
#include "aero_hardware_interface/Angle2Stroke.hh"
#include "aero_hardware_interface/UnusedAngle2Stroke.hh"

#include "aero_move_base/AeroBaseConfig.hh"

#include <aero_startup/controller_shm.h>

using namespace aero;
using namespace controller;

static struct shm_struct *shm;

static void initialize_shm()
{
  shm = (struct shm_struct *)set_shared_memory(5555, sizeof(struct shm_struct));

  if (shm == NULL) {
    fprintf(stderr, "shared memory initialize error\n");
    exit(1);
  }

  for(int i = 0; i < MAX_JOINT_NUM; i++) {
    shm->servo_on[i] = 0;
    shm->servo_off[i] = 0;
    shm->loopback[i] = 0;
    shm->joint_enable[i] = 1;
    shm->joint_offset[i] = 0;
  }
}

#define DISPLAY_THREAD_PERIOD  250000 //250ms
#define MAIN_THREAD_PERIOD      50000 //50ms (20Hz)
#define OVERWRAP_SCALE         2.8 // 1.0 := no overwrap
#define NSEC_PER_SEC    1000000000L

static sig_atomic_t  stop_flag = 0;
static std::vector< std::string> joint_list;
static std::vector< std::string> stroke_list;
static std::vector< std::string> wheel_names;
namespace {
  // スレッド間共有コンテキスト
  volatile unsigned long long frame_counter = 0;
  double jitter;
  double max_interval;
  double ave_interval;
  // 描画 スレッド
  void* display_thread_fun(void *arg) {
    while(!stop_flag) {
      fprintf(stderr, "max_interval: %6.4f [ms] / average: %6.4f [ms]\n", max_interval/1000.0, ave_interval/1000.0);
      {
        int size = joint_list.size();
        int jnm_cnt = 0;
        int ref_cnt = 0;
        int act_cnt = 0;
        for(int j = 0; j < 5; j++) {
          if (jnm_cnt >= size) break;
          for(int i = 0; i < 8; i++) {
            if (jnm_cnt < size) fprintf(stderr, "%14.14s ", joint_list[jnm_cnt++].c_str());
          }
          fprintf(stderr, "\n");
          for(int i = 0; i < 8; i++) {
            if (ref_cnt < size) fprintf(stderr, "%14.4f ", shm->ref_angle[ref_cnt++]);
          }
          fprintf(stderr, " ref_angle\n");
          for(int i = 0; i < 8; i++) {
            if (act_cnt < size) fprintf(stderr, "%14.4f ", shm->act_angle[act_cnt++]);
          }
          fprintf(stderr, " act_angle\n\n");
        }
      }
      fprintf(stderr, "\n");
      {
        int size = stroke_list.size();
        int snm_cnt = 0;
        int ref_cnt = 0;
        int act_cnt = 0;
        int snt_cnt = 0;
        for(int j = 0; j < 5; j++) {
          if (snm_cnt >= size) break;
          for(int i = 0; i < 8; i++) {
            if (snm_cnt < size) fprintf(stderr, "%14.14s ", stroke_list[snm_cnt++].c_str());
          }
          fprintf(stderr, "\n");
          for(int i = 0; i < 8; i++) {
            if (ref_cnt < size) fprintf(stderr, "%14d ", shm->ref_stroke[ref_cnt++]);
          }
          fprintf(stderr, " ref_stroke\n");
          for(int i = 0; i < 8; i++) {
            if (snt_cnt < size) fprintf(stderr, "%14d ", shm->snt_stroke[snt_cnt++]);
          }
          fprintf(stderr, " sent_stroke\n");
          for(int i = 0; i < 8; i++) {
            if (act_cnt < size) fprintf(stderr, "%14d ", shm->act_stroke[act_cnt++]);
          }
          fprintf(stderr, " act_stroke\n\n");
        }
      }
      fprintf(stderr, "\n");
      {
        int size = wheel_names.size();
        for(int i = 0; i < size; i++) {
          fprintf(stderr, "%14.14s ", wheel_names[i].c_str());
        }
        fprintf(stderr, "\n");
        for(int i = 0; i < size; i++) {
          fprintf(stderr, "%14d ", shm->wheel_velocity[i]);
        }
      }
      struct timespec tm;
      tm.tv_sec = 0;
      tm.tv_nsec = DISPLAY_THREAD_PERIOD*1000;
      nanosleep(&tm, NULL);

      //// clear screen
      fprintf(stderr, "\x1b[2J");
      fprintf(stderr, "\x1b[1;1H");
    }
  }
}

void cchandler(int) { stop_flag = 1; }

int main(int argc, char** argv)
{
  signal( SIGINT, &cchandler );

  // initialize shared memory
  initialize_shm();
  shm->frame = -1;
  // TODO: read from command-line args
  std::string port_upper("/dev/aero_upper");
  std::string port_lower("/dev/aero_lower");

  bool debug = false; // TODO: read from command-line args
  bool no_display = false;

  std::cerr << "initializing ctrl with "
            << port_upper << " and " << port_lower
            << std::endl;

  AeroUpperController controller_upper(port_upper);
  AeroLowerController controller_lower(port_lower);

  // number of whole body joints
  int number_of_angle_joints =
    controller_upper.get_number_of_angle_joints() +
    controller_lower.get_number_of_angle_joints();

  // make joint_list // not required ...
  // std::vector< std::string > joint_list;
  joint_list.resize(number_of_angle_joints);
  for(int i = 0; i < number_of_angle_joints; i++) {
    std::string name;
    if(controller_upper.get_joint_name(i, name)) {
      joint_list[i] = name;
    } else if (controller_lower.get_joint_name(i, name)) {
      joint_list[i] = name;
    } else {
      std::cerr << "[WARN] name of joint " << i << "can not find!" << std::endl;
    }
  }

  int number_of_strokes =
    controller_upper.get_number_of_strokes() +
    controller_lower.get_number_of_strokes();
  stroke_list.resize(0);
  for(int i = 0; i < controller_upper.get_number_of_strokes(); i++) {
    std::string name = controller_upper.get_stroke_joint_name(i);
    stroke_list.push_back(name);
  }
  for(int i = 0; i < controller_lower.get_number_of_strokes(); i++) {
    std::string name = controller_lower.get_stroke_joint_name(i);
    stroke_list.push_back(name);
  }

  aero::navigation::AeroBaseConfig config;
  config.get_wheel_names(wheel_names);

  std::cerr << "num of joints: " << joint_list.size() << std::endl;
  std::cerr << "num of strokes: " << stroke_list.size() << std::endl;
  std::cerr << "num of wheels: "  << wheel_names.size() << std::endl;
#if 0
  for(int i = 0; i < joint_list.size(); i++) {
    std::cerr << joint_list[i] << std::endl;
  }
  for(int i = 0; i < stroke_list.size(); i++) {
    std::cerr << stroke_list[i] << std::endl;
  }
  exit(1);
#endif

#if 0
  for(int i = 0; i < joint_list.size(); i++) {
    int nmlen = joint_list[i].size() + 1;
    if (nmlen > MAX_JOINT_NAME_LENGTH) {
      std::cerr << "[WARN] name of joint: " << joint_list[i] << " is longer than "
                << MAX_JOINT_NAME_LENGTH << std::endl;
      nmlen = MAX_JOINT_NAME_LENGTH;
    }
    char *cstr = shm->joint_names[i];
    std::char_traits<char>::copy(cstr, joint_list[i].c_str(), nmlen);
  }
  if (joint_list.size() < MAX_JOINT_NUM) {
    shm->joint_names[joint_list.size()-1][0] = '\0';
  }
#endif

  // display スレッド開始
#if 1
  pthread_t display_thread;
  if (!no_display) {
    int args[4];
    args[0] = number_of_angle_joints;
    if ( pthread_create( &display_thread, NULL, display_thread_fun, (void *)args) ) {
      fprintf(stderr, "pthread_create (display)");
      exit(1);
    }
  }
#endif

  // main loop 開始
  std::vector<double > prev_ref_positions(number_of_angle_joints);
  bool initialized_flag = false;
  bool wheel_off = false;
  max_interval = 0.0;
  ave_interval = 0.0;
  int cntr = 0;
  static timespec m_t;
  clock_gettime( CLOCK_MONOTONIC, &m_t );
  while(!stop_flag) {
    ////// read strokes and convert strokes to positions
    //
    controller_upper.update_position();
    controller_lower.update_position();

    // get upper actual positions
    std::vector<int16_t> upper_act_strokes =
      controller_upper.get_actual_stroke_vector();
    // get lower actual positions
    std::vector<int16_t> lower_act_strokes =
      controller_lower.get_actual_stroke_vector();

    // whole body strokes
    std::vector<int16_t> act_strokes (AERO_DOF_UPPER + AERO_DOF_LOWER);
    if (upper_act_strokes.size() < AERO_DOF_UPPER) {
      for (size_t i = 0; i < AERO_DOF_UPPER; ++i) {
        act_strokes[i] = 0;
      }
    } else { // usually should enter else, enters if when port is not activated
      for (size_t i = 0; i < AERO_DOF_UPPER; ++i) {
        act_strokes[i] = upper_act_strokes[i];
      }
    }
    if ( lower_act_strokes.size() < AERO_DOF_LOWER ) {
      for (size_t i = 0; i < AERO_DOF_LOWER; ++i) {
        act_strokes[i + AERO_DOF_UPPER] = 0; //??
      }
    } else { // usually should enter else, enters if when port is not activated
      for (size_t i = 0; i < AERO_DOF_LOWER; ++i) {
        act_strokes[i + AERO_DOF_UPPER] = lower_act_strokes[i];
      }
    }

    // whole body positions from strokes
    std::vector<double> act_positions;
    act_positions.resize(number_of_angle_joints);
    common::Stroke2Angle(act_positions, act_strokes);
    //
    for(int i = 0; i < number_of_angle_joints; i++) {
      shm->act_angle[i]  = act_positions[i];
      shm->act_stroke[i] = act_strokes[i];
    }

    ////// convert poitions to strokes and write strokes
    std::vector<double > ref_positions(number_of_angle_joints);
    if (!initialized_flag) { // first one time
      for(int i = 0; i < number_of_angle_joints; i++) {
        prev_ref_positions[i] = shm->ref_angle[i] = shm->act_angle[i];
      }

      std::vector<int16_t> wheel_vector(controller_lower.get_reference_wheel_vector().size());
      for(int i = 0; i < wheel_vector.size(); i++) {
            wheel_vector[i] = 0;
            shm->wheel_velocity[i] = 0;
      }
      uint16_t time_csec = static_cast<uint16_t>((OVERWRAP_SCALE*MAIN_THREAD_PERIOD)/(1000*10));
      controller_lower.set_wheel_velocity(wheel_vector, time_csec);
      controller_lower.wheel_only_off();
      wheel_off = true;
      initialized_flag = true;
    } else {
      //std::fill(ref_positions.begin(), ref_positions.end(), 0.0);
      for(int i = 0; i < number_of_angle_joints; i++) {
        ref_positions[i] = shm->ref_angle[i];
      }

      std::vector<bool > mask_positions(number_of_angle_joints);
      std::fill(mask_positions.begin(), mask_positions.end(), true); // send if true

      for(int i = 0; i < number_of_angle_joints; i++) {
        double tmp = ref_positions[i];
        if (tmp == prev_ref_positions[i]) {
          mask_positions[i] = false;
        }
        prev_ref_positions[i] = tmp;
      }

      std::vector<int16_t> ref_strokes(AERO_DOF);
      common::Angle2Stroke(ref_strokes, ref_positions);
      std::vector<int16_t> snt_strokes(ref_strokes);
      common::UnusedAngle2Stroke(snt_strokes, mask_positions);

      // split strokes into upper and lower
      std::vector<int16_t> upper_strokes(
        snt_strokes.begin(), snt_strokes.begin() + AERO_DOF_UPPER);
      std::vector<int16_t> lower_strokes(
        snt_strokes.begin() + AERO_DOF_UPPER, snt_strokes.end());

      uint16_t time_csec = static_cast<uint16_t>((OVERWRAP_SCALE*MAIN_THREAD_PERIOD)/(1000*10));
      controller_upper.set_position(upper_strokes, time_csec);
      controller_lower.set_position(lower_strokes, time_csec);

      for(int i = 0; i < number_of_angle_joints; i++) {
        shm->ref_stroke[i] = ref_strokes[i];
        shm->snt_stroke[i] = snt_strokes[i];
      }

      shm->frame++;
    }

    //// Base
    {
      static int zero_count = 1;
      bool all_zero = true;
      for (int i = 0; i < wheel_names.size(); i++) {
        if (shm->wheel_velocity[i] != 0.0) {
          all_zero = false;
          break;
        }
      }
      if (all_zero) {
        // stop controller
        zero_count++;
        if (zero_count > 4 && !wheel_off) {
          std::vector<int16_t> wheel_vector(controller_lower.get_reference_wheel_vector().size());
          for(int i = 0; i < wheel_vector.size(); i++) {
            wheel_vector[i] = 0;
          }
          uint16_t time_csec = static_cast<uint16_t>((OVERWRAP_SCALE*MAIN_THREAD_PERIOD)/(1000*10));
          controller_lower.set_wheel_velocity(wheel_vector, time_csec);
          controller_lower.wheel_only_off();
          wheel_off = true;
        }
      } else {
        if(wheel_off) {
          controller_lower.wheel_on();
          wheel_off = false;
        }
        zero_count = 0;
        std::vector<int16_t> wheel_vector;
        std::vector<int16_t>& ref_vector = controller_lower.get_reference_wheel_vector();
        wheel_vector.assign(ref_vector.begin(), ref_vector.end());

        std::vector<int32_t> joint_to_wheel_indices(AERO_DOF_WHEEL);
        for (size_t i = 0; i < wheel_names.size(); ++i) {
          joint_to_wheel_indices[i] = controller_lower.get_wheel_id(wheel_names[i]);
        }
        for(int i = 0; i < wheel_vector.size(); i++) {
          if (joint_to_wheel_indices[i] >= 0) {
            wheel_vector[static_cast<size_t>(joint_to_wheel_indices[i])]
              = shm->wheel_velocity[i];
          }
        }
        uint16_t time_csec = static_cast<uint16_t>((OVERWRAP_SCALE*MAIN_THREAD_PERIOD)/(1000*10));
        controller_lower.set_wheel_velocity(wheel_vector, time_csec);
      }
    }

    {
      struct timespec tm;
      tm.tv_nsec = m_t.tv_nsec;
      tm.tv_sec  = m_t.tv_sec;
      tm.tv_nsec += MAIN_THREAD_PERIOD*1000;
      while( tm.tv_nsec >= NSEC_PER_SEC ){
        tm.tv_nsec -= NSEC_PER_SEC;
        tm.tv_sec++;
      }
      clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &tm, NULL );
    }

    if(cntr > 100) {
      cntr = 0;
      max_interval = 0.0;
    }
    static timespec n_t;
    clock_gettime( CLOCK_MONOTONIC, &n_t );
    const double measured_interval = ((n_t.tv_sec - m_t.tv_sec)*NSEC_PER_SEC + (n_t.tv_nsec - m_t.tv_nsec))/1000.0;
    if (measured_interval > max_interval) max_interval = measured_interval;
    if(ave_interval == 0.0) {
      ave_interval = measured_interval;
    } else {
      ave_interval = (measured_interval + (100 - 1)*ave_interval)/100.0;
    }
    m_t.tv_sec = n_t.tv_sec;
    m_t.tv_nsec = n_t.tv_nsec;
    cntr++;
  }

  if (!no_display) {
    // 終了スレッドの結合
    if ( pthread_join ( display_thread, NULL ) == 0) {
      fprintf(stderr, "display_thread_join\n");
      exit(1);
    }
  }

  return 0;
}
