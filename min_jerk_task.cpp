/*============================================================================
==============================================================================
                      
                              min_jerk_task.c
 
==============================================================================
Remarks:

      sekeleton to create the sample task

============================================================================*/

// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

// defines

// local variables
static double      start_time = 0.0;
static SL_DJstate  target[N_DOFS+1];
static double      delta_t = 0.01;
static double      duration = 1.0;
static double      time_to_go;
static int	   my_times = 0;


// global functions 
extern "C" void
add_min_jerk_task( void );

// local functions
static int  init_min_jerk_task(void);
static int  run_min_jerk_task(void);
static int  change_min_jerk_task(void);

static int 
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
		    double t_togo, double dt,
		    double *x_next, double *xd_next, double *xdd_next);


/*****************************************************************************
******************************************************************************
Function Name	: add_min_jerk_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_min_jerk_task( void )
{
  int i, j;
  
  addTask("Min Jerk Task", init_min_jerk_task, 
	  run_min_jerk_task, change_min_jerk_task);
}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_min_jerk_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_min_jerk_task(void)
{
  int j, i;
  int ans;
  static int firsttime = TRUE;
  
  my_times = 0;
  if (firsttime){
    firsttime = FALSE;
  }

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target)) 
    return FALSE;

  // re-use the variable target for our min-jerk movement: only the right arm moves
/*
  target[R_SFE].th += 0.4;
  target[R_SAA].th -= 0.4;
  target[R_EB].th  += 0.5;
*/

  target[R_SFE].th += 0.5;

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;

  start_time = task_servo_time;
  printf("start time = %.3f, task_servo_time = %.3f\n", 
	 start_time, task_servo_time);

  // start data collection
  scd();

  // time to go
  time_to_go = duration;

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_min_jerk_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_min_jerk_task(void)
{
  int j, i;
  double task_time;

  // ******************************************
  // NOTE: all array indices start with 1 in SL
  // ******************************************
  task_time = task_servo_time - start_time;

  // compute the update for the desired states
  for (i=1; i<=N_DOFS; ++i) {
    min_jerk_next_step(joint_des_state[i].th,
		       joint_des_state[i].thd,
		       joint_des_state[i].thdd,
		       target[i].th,
		       target[i].thd,
		       target[i].thdd,
		       time_to_go,
		       delta_t,
		       &(joint_des_state[i].th),
		       &(joint_des_state[i].thd),
		       &(joint_des_state[i].thdd));
  }

  // compute inverse dynamics torques
  SL_InvDynNE(joint_state,joint_des_state,endeff,&base_state,&base_orient);
  
  // decrement time to go
  time_to_go -= delta_t;
  if (time_to_go <= 0) {
    my_times++;
    if (my_times == 1) {
  	target[R_SAA].th -= 0.6;
  	time_to_go = duration;
	run_min_jerk_task();
    } else if (my_times == 2) {
	target[R_SAA].th -= 0.1;
  	target[R_SFE].th -= 0.6;
  	time_to_go = duration;
	run_min_jerk_task();
    } else if (my_times == 3) {
  	for (i=1; i<=N_DOFS; i++)
    	    target[i] = joint_default_state[i];
  	time_to_go = duration;
	run_min_jerk_task();
    } else {
      freeze();
    }
/*
    my_times++;
    if (my_times == 1) {
  	target[R_SFE].th += 1;
  	target[R_SAA].th -= 10;
  	//target[R_EB].th  += 0.5;
  	time_to_go = duration;
	run_min_jerk_task();
    } else if (my_times == 2) {
  	target[R_SFE].th += 10;
  	target[R_SAA].th -= 10;
  	//target[R_EB].th  += 0.5;
  	time_to_go = duration;
	run_min_jerk_task();
    } else if (my_times == 3) {
  	for (i=1; i<=N_DOFS; i++)
    	    target[i] = joint_default_state[i];
  	time_to_go = duration;
	run_min_jerk_task();
    } else {
*/
    //  freeze();
  //  }
  }

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_min_jerk_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_min_jerk_task(void)
{
  int    ivar;
  double dvar;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  min_jerk_next_step
\date  April 2014
   
\remarks 

Given the time to go, the current state is updated to the next state
using min jerk splines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          x,xd,xdd : the current state, vel, acceleration
 \param[in]          t,td,tdd : the target state, vel, acceleration
 \param[in]          t_togo   : time to go until target is reached
 \param[in]          dt       : time increment
 \param[in]          x_next,xd_next,xdd_next : the next state after dt

 ******************************************************************************/
static int 
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
		    double t_togo, double dt,
		    double *x_next, double *xd_next, double *xdd_next)

{

  // your code goes here ...
  *x_next = ((6*t)/pow(t_togo,5) - (3*td)/pow(t_togo,4) + tdd/(2*pow(t_togo,3)) - (6*x)/pow(t_togo,5) - (3*xd)/pow(t_togo,4) - xdd/(2*pow(t_togo,3)))*pow(dt,5) + ((7*td)/pow(t_togo,3) - (15*t)/pow(t_togo,4) - tdd/pow(t_togo,2) + (15*x)/pow(t_togo,4) + (8*xd)/pow(t_togo,3) + (3*xdd)/(2*pow(t_togo,2)))*pow(dt,4) + ((10*t)/pow(t_togo,3) - (4*td)/pow(t_togo,2) + tdd/(2*t_togo) - (10*x)/pow(t_togo,3) - (6*xd)/pow(t_togo,2) - (3*xdd)/(2*t_togo))*pow(dt,3) + (xdd*pow(dt,2))/2 + xd*dt + x;
  
  *xd_next = ((30*t)/pow(t_togo, 5) - (15*td)/pow(t_togo, 4) + (5*tdd)/(2*pow(t_togo, 3)) - (30*x)/pow(t_togo, 5) - (15*xd)/pow(t_togo, 4) - (5*xdd)/(2*pow(t_togo, 3)))*pow(dt, 4) + ((28*td)/pow(t_togo, 3) - (60*t)/pow(t_togo, 4) - (4*tdd)/pow(t_togo, 2) + (60*x)/pow(t_togo, 4) + (32*xd)/pow(t_togo, 3) + (6*xdd)/pow(t_togo, 2))*pow(dt, 3) + ((30*t)/pow(t_togo, 3) - (12*td)/pow(t_togo, 2) + (3*tdd)/(2*t_togo) - (30*x)/pow(t_togo, 3) - (18*xd)/pow(t_togo, 2) - (9*xdd)/(2*t_togo))*pow(dt, 2) + xdd*dt + xd;

  *xdd_next = ((120*t)/pow(t_togo, 5) - (60*td)/pow(t_togo, 4) + (10*tdd)/pow(t_togo, 3) - (120*x)/pow(t_togo, 5) - (60*xd)/pow(t_togo, 4) - (10*xdd)/pow(t_togo, 3))*pow(dt, 3) + ((84*td)/pow(t_togo, 3) - (180*t)/pow(t_togo, 4) - (12*tdd)/pow(t_togo, 2) + (180*x)/pow(t_togo, 4) + (96*xd)/pow(t_togo, 3) + (18*xdd)/pow(t_togo, 2))*pow(dt, 2) + ((60*t)/pow(t_togo, 3) - (24*td)/pow(t_togo, 2) + (3*tdd)/t_togo - (60*x)/pow(t_togo, 3) - (36*xd)/pow(t_togo, 2) - (9*xdd)/t_togo)*dt + xdd;
  
  return TRUE;
}

