#ifndef FSM_STATE_JOINTPD_H
#define FSM_STATE_JOINTPD_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_JointPD : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_JointPD(ControlFSMData<T>* _controlFSMData);

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

 private:
  // Keep track of the control iterations
  int iter = 0;
  DVec<T> _ini_jpos;


  float L1=0.205;
  float L2=0.205;
  float halfBodyLen=0.23; 
  float maxL=.31;
  static constexpr int StandUp = 0;
  static constexpr int FoldLegs = 1;

  unsigned long long _state_iter;
  int _motion_start_iter = 0;
  int _flag = FoldLegs;
  // JPos
  Vec3<T> initial_jpos[4];
  Vec3<T> zero_vec3;

  Vec3<T> fold_jpos[4];
  Vec3<T> stand_jpos[4];

  const int fold_ramp_iter = 1000;
  const int fold_settle_iter = 1000;

  const int standup_ramp_iter = 500;
  const int standup_settle_iter = 500;

  
  void _StandUp(const int & iter);
  void _FoldLegs(const int & iter);
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);
  Vec3<T> convertAngle(Vec3<T> pos);
//common motion action perform  
  Vec3<T> motion_jpos[16-4][4];
  int motion_ramp_iter[16-4]={600,100,1300,600,    590,260,   100,100,100,/*500,500,500,*/100,300,900};  
  int motion_settle_iter[16-4]={100,100,500,800,   0,400, 100,100,100,/*500,500,500,*/50,50,2500};  
  void _doMotion(const int & iter,int index);  
};

#endif  // FSM_STATE_JOINTPD_H
