/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_Transform.h"


/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Transform<T>::FSM_State_Transform(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::TRANSFORM, "TRANSFORM"),
    motorController("/dev/serial/by-id/usb-Dr-Tech_DR-USB_CAN_828595630E53-if00", 115200),
_ini_foot_pos(4){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Transform<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;

  for(size_t leg(0); leg<4; ++leg){
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Transform<T>::run() {
  if (iter == 1) 
  {
  
  // MotorController motorController("/dev/serial/by-id/usb-Dr-Tech_DR-USB_CAN_828595630E53-if00", 115200);


  // }
    // 示例：设置电机角度
  int motor_id = 1;          // 电机ID
  float angle = 400.0f;       // 目标角度
  float speed = 300.0f;       // 转动速度
  float param = 300.0f;       // 角度模式参数
  int mode = 1;              // 控制模式：0=轨迹跟踪模式, 1=梯形轨迹模式, 2=前馈控制模式
  

  bool success = motorController.setAngle(motor_id, angle, speed, param, mode);

  // 检查函数是否成功
  if (success) {
      std::cout << "Angle set successfully!" << std::endl;
  } else {
      std::cout << "Failed to set angle." << std::endl;
  }
  }

}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Transform<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_TRANSFORM:
      break;
    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;


    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_TRANSFORM << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Transform<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Transform<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_Transform<float>;
