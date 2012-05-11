#ifndef ROBOTAGENT_H_
#define ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/

class CRobotAgent;

#include "common.h"
#include "agent.h"
#include "behavior.h"
#include "crminrobotagent.h"

/******************************************************************************/
/******************************************************************************/

class CRobotAgent : public CAgent
{
public: 
    CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments, TBehaviorVector vec_behaviors);
    virtual ~CRobotAgent();
    
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStep(unsigned int n_step_number);    
    
    virtual EAgentType   GetType();   

    CRMinRobotAgent* crminAgent;

protected:
    TBehaviorVector     m_vecBehaviors;
};

/******************************************************************************/
/******************************************************************************/

#endif // ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/
