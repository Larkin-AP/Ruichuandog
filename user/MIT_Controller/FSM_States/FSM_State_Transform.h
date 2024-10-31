#ifndef FSM_STATE_TRANSFORM_H
#define FSM_STATE_TRANSFORM_H

#include "FSM_State.h"
#include <robot/include/rt/Dr_Empower_can.h>

/**
 *
 */
template <typename T>
class FSM_State_Transform : public FSM_State<T> {
 public:
  FSM_State_Transform(ControlFSMData<T>* _controlFSMData);

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
  MotorController motorController;  // MotorController 成员变量
  // Keep track of the control iterations
  int iter = 0;
  std::vector< Vec3<T> > _ini_foot_pos;
};

#endif  
