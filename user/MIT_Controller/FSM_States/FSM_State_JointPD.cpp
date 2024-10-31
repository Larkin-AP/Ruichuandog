/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_JointPD.h"
#include <Configuration.h>


template <typename T>
FSM_State_JointPD<T>::FSM_State_JointPD(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::JOINT_PD, "JOINT_PD"),
_ini_jpos(cheetah::num_act_joint){

  zero_vec3.setZero();
//stand up
  motion_jpos[0][0] << 0.f, -.8f, 1.6f;
  motion_jpos[0][1] << 0.f, -.8f, 1.6f;
  motion_jpos[0][2] << 0.f, -.8f, 1.6f;
  motion_jpos[0][3] << 0.f, -.8f, 1.6f;  

  for (int i=1;i<=2;++i)  for (int j=0;j<4;++j)
    motion_jpos[i][j]=motion_jpos[0][j];

  float bc;
  float ang1,ang2;


  motion_jpos[2][0] << 0.f, -1.531, 2.871;
  motion_jpos[2][1] << 0.f, -1.531, 2.871;
  motion_jpos[2][2] << 0.f, 0.487, 1.181;
  motion_jpos[2][3] << 0.f, 0.487, 1.181;


  ang1=asin((halfBodyLen-L2/2)/L1);

//初始蹲
  motion_jpos[3][0] << 0.f,-1.126, 1.234;
  motion_jpos[3][1] << 0.f,-1.126, 1.234;
  motion_jpos[3][2] << 0.f, -0.066,1.181;//-0.6, 1.39;//-0.64, 1.41;//-0.385, 1.256;
  motion_jpos[3][3] << 0.f, -0.066,1.181;//-0.6, 1.39;

//半起，慢
  motion_jpos[4][0] << 0.f,-1.126, 2.234;
  motion_jpos[4][1] << 0.f,-1.126, 2.234;
  motion_jpos[4][2] << 0.f, -3.14*50/180,1.181;//-0.6, 1.39;//-0.64, 1.41;//-0.385, 1.256;
  motion_jpos[4][3] << 0.f, -3.14*50/180,1.181;//-0.6, 1.39;


 std::cout<<"*******************************************************\n";
  printf("ang2_1=%.2f\n",(M_PI/2-ang1)*180/M_PI);
  bc=sqrt(L1*L1+halfBodyLen*halfBodyLen-2*L1*halfBodyLen*sin(ang1));
  ang2=M_PI+((ang1<-M_PI/2)*2-1)*acos((L1-halfBodyLen*sin(ang1))/(bc+0.00000001))-acos(L2/2 /bc);
  printf("ang2_2=%.2f  bc=%.2f\n",ang2*180/M_PI,bc);


//直起，快
  Vec3<T> pos;
  ang1=-asin((L2/2-0.03) /L1)-M_PI/2;
  pos<<-.0,0,-.2;
  motion_jpos[5][0] =convertAngle(pos);
  motion_jpos[5][1] =convertAngle(pos);
  motion_jpos[5][2] << 0.f, ang1, -ang1;
  motion_jpos[5][3] << 0.f, ang1, -ang1;


  pos<<-.2,0,-.2;
  motion_jpos[6][0] =convertAngle(pos);
  pos<<-.2,-.28,-.2;
  motion_jpos[6][1] =convertAngle(pos);
  motion_jpos[6][2] << 0.f, ang1, -ang1;
  motion_jpos[6][3] << 0.f, ang1, -ang1;

  pos<<-.0,0,-.2;
  motion_jpos[7][1] =convertAngle(pos);
  pos<<-.0,.28,-.2;
  motion_jpos[7][0] =convertAngle(pos);
  motion_jpos[7][2] << 0.f, ang1, -ang1;
  motion_jpos[7][3] << 0.f, ang1, -ang1;

  pos<<-.2,0,-.2;
  motion_jpos[8][1] =convertAngle(pos);
  pos<<-.2,.28,-.2;
  motion_jpos[8][0] =convertAngle(pos);
  motion_jpos[8][2] << 0.f, ang1, -ang1;
  motion_jpos[8][3] << 0.f, ang1, -ang1;

  for (int i=9;i<=13-4;++i)  for (int j=0;j<4;++j)
    motion_jpos[i][j]=motion_jpos[i-4][j];

  motion_jpos[13-4][0] << 0.f, -1.2f, 1.9f;
  motion_jpos[13-4][1] << 0.f, -1.2f, 1.9f;

  motion_jpos[14-4][0] << 0.f, -.6f, 1.2f;
  motion_jpos[14-4][1] << 0.f, -.6f, 1.2f;
  motion_jpos[14-4][2] << 0,-.5,1.53;
  motion_jpos[14-4][3] << 0,-.5,1.53;

  motion_jpos[15-4][0] << 0.f, -.8f, 1.6f;
  motion_jpos[15-4][1] << 0.f, -.8f, 1.6f;
  motion_jpos[15-4][2] << 0.f, -.8f, 1.6f;
  motion_jpos[15-4][3] << 0.f, -.8f, 1.6f;


}

template <typename T>
void FSM_State_JointPD<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  iter = 0;

  _state_iter = 0;
  for(size_t i(0); i < 4; ++i) {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }

  _flag = 3;

  _motion_start_iter = 0;

}

template <typename T>
void FSM_State_JointPD<T>::run() {
  _doMotion(_state_iter - _motion_start_iter,_flag);
  ++_state_iter;

}


template <typename T>
void FSM_State_JointPD<T>::_doMotion(const int & curr_iter,int index){
  for(size_t i(0); i<4; ++i){
    _SetJPosInterPts(curr_iter, motion_ramp_iter[index], i, 
        initial_jpos[i], motion_jpos[index][i]);
  }

 
  if(curr_iter >= motion_ramp_iter[index] + motion_settle_iter[index]){ //when compelete
    _flag = index +1;  if (_flag==104) _flag++; if (_flag>15-4) _flag=11;

    // printf("_flag %d\n",_flag);
    for(size_t i(0); i<4; ++i){
      initial_jpos[i] = this->_data->_legController->datas[i].q; 
    }
    _motion_start_iter = _state_iter + 1;
  }
}

template <typename T>
void FSM_State_JointPD<T>::_SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg, 
    const Vec3<T> & ini, const Vec3<T> & fin){

    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if(curr_iter <= max_iter) {
      b = (float)curr_iter/(float)max_iter;
      a = 1.f - b;
    }

    // compute setpoints
    Vec3<T> inter_pos = a * ini + b * fin;

    if ((_flag==5)&&(leg>1)){       //when body up, calc ang2 

      this->jointPDControl(leg, inter_pos, zero_vec3, 300);
    }else if (_flag==3 ||_flag==4){

      this->jointPDControl(leg, inter_pos, zero_vec3, 380);    
    }else if (_flag<2){
      this->jointPDControl(leg, inter_pos, zero_vec3, 80);  
    }else if (_flag==2){
      this->jointPDControl(leg, inter_pos, zero_vec3, 200);  
    }else if (_flag==14-4){
      this->jointPDControl(leg, inter_pos, zero_vec3, 25);
    }else if (_flag==15-4){
      this->jointPDControl(leg, inter_pos, zero_vec3, 110);  
    }else{
      this->jointPDControl(leg, inter_pos, zero_vec3, 180);
    }
}

template <typename T>
void FSM_State_JointPD<T>::_StandUp(const int & curr_iter){

  for(size_t leg(0); leg<4; ++leg){
    _SetJPosInterPts(curr_iter, standup_ramp_iter, 
        leg, initial_jpos[leg], stand_jpos[leg]);
  }
  if(curr_iter >= fold_ramp_iter + fold_settle_iter){
    _flag = FoldLegs;   //next action
    for(size_t i(0); i<4; ++i){
      initial_jpos[i] = stand_jpos[i];    
    }
    _motion_start_iter = _state_iter + 1;
  }
}

template <typename T>
void FSM_State_JointPD<T>::_FoldLegs(const int & curr_iter){

  for(size_t i(0); i<4; ++i){
    _SetJPosInterPts(curr_iter, fold_ramp_iter, i, 
        initial_jpos[i], fold_jpos[i]);
  }

  if(curr_iter >= fold_ramp_iter + fold_settle_iter){
    _flag = StandUp;   //next action
    for(size_t i(0); i<4; ++i){
      initial_jpos[i] = fold_jpos[i];    
    }
    _motion_start_iter = _state_iter + 1;
  }
}


template <typename T>
Vec3<T> FSM_State_JointPD<T>::convertAngle(Vec3<T> pos){
    Vec3<T> ang;
    float hipx=pos(0);
    float hipy=pos(1);
    float hipz=pos(2);
    if (hipz>=0) {ang(0)=0;}else{ang(0)=atan2(hipy,-hipz);}
    float L12=sqrt((hipz)*(hipz)+hipx*hipx);
    if(L12>0.31) L12=0.31;//max
    if(L12<0.08) L12=0.08;//min
    float fai=acos((L1*L1+L12*L12-L2*L2)/2.0/L1/L12);
    ang(1)=atan2(hipx,-hipz)-fai;
    ang(2)=M_PI-acos((L1*L1+L2*L2-L12*L12)/2.0/L1/L2);
    return ang;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_JointPD<T>::checkTransition() {
  this->nextStateName = this->stateName;

  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_JOINT_PD:
      // Normal operation for state based transitions
      break;

    case K_IMPEDANCE_CONTROL:
      // Requested change to impedance control
      this->nextStateName = FSM_StateName::IMPEDANCE_CONTROL;

      // Transition time is 1 second
      this->transitionDuration = 1.0;
      break;

    case K_STAND_UP:
      // Requested change to impedance control
      this->nextStateName = FSM_StateName::STAND_UP;

      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;

    case K_BALANCE_STAND:
      // Requested change to balance stand
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.0;
      break;

    case K_PASSIVE:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::PASSIVE;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_JOINT_PD << " to "
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
TransitionData<T> FSM_State_JointPD<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::IMPEDANCE_CONTROL:

      iter++;
      if (iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
      } else {
        this->transitionData.done = false;
      }
      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;


    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;

      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_JOINT_PD << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  // Finish transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}



/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_JointPD<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_JointPD<double>;
template class FSM_State_JointPD<float>;



