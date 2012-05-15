#ifndef ROBOTAGENT_H_
#define ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/

class CRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgent;

/******************************************************************************/
/******************************************************************************/

#include "common.h"
#include "agent.h"
#include "behavior.h"
#include "crminrobotagent.h"

/******************************************************************************/
/******************************************************************************/

class CRobotAgent : public CAgent
{
public: 
    CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_agent_arguments, CArguments* pc_crm_arguments, TBehaviorVector vec_behaviors);
    virtual ~CRobotAgent();
    
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStepUpdatePosition();    
    
    virtual EAgentType   GetType();   

    // Gets the number of feature vectors of different types
    // into m_punFeaturesSensed. Returns the range used to sense the feature vectors
    virtual double GetFeaturesSensed(unsigned int*  m_punFeaturesSensed);

    virtual CRobotAgent* TryToConnectToRandomRobotAgentWithWeights();

    virtual CRMinRobotAgent* GetCRMinRobotAgent();

protected:
    TBehaviorVector     m_vecBehaviors;
    CRMinRobotAgent*    crminAgent;
};

/******************************************************************************/
/******************************************************************************/

#endif // ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/
