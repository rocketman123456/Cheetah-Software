/*
 * Added by EHL, 4DAGE, Zhuhai
 * developerleen@gmail.com
*/

#ifndef FSM_STATE_RECOVERY_SIT_DOWN_H
#define FSM_STATE_RECOVERY_SIT_DOWN_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_RecoverySitDown : public FSM_State<T> {
 public:
  FSM_State_RecoverySitDown(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

 private:
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  static constexpr int SitDown = 0;

  unsigned long long _state_iter;

  // JPos
  Vec3<T> sit_jpos[4];
  Vec3<T> initial_jpos[4];
  Vec3<T> zero_vec3;

  Vec3<T> f_ff;

  // iteration setup

  const int standup_ramp_iter = 1000;
  const int standup_settle_iter = 500;

  void _SitDown(const int & iter);

  bool _UpsideDown();
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

};

#endif  // FSM_STATE_RECOVERY_SIT_DOWN_H
