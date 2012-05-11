#include "robotagent.h"

/******************************************************************************/
/******************************************************************************/

CRobotAgent::CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments, TBehaviorVector vec_behaviors) :
    CAgent(pch_name, un_identification, pc_arguments), m_vecBehaviors(vec_behaviors)
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }

    crminAgent = new CRMinRobotAgent(m_crmArguments);
}

/******************************************************************************/
/******************************************************************************/

CRobotAgent::~CRobotAgent() 
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        delete (*i);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SimulationStep(unsigned int n_step_number)
{
    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SimulationStep();
        if (!bControlTaken) 
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
                (*i)->Action();
        } else {
            (*i)->Suppress();
        }          
    }

    SimulationStepUpdatePosition();
}

/******************************************************************************/
/******************************************************************************/

EAgentType CRobotAgent::GetType()
{
    return ROBOT;
}

/******************************************************************************/
/******************************************************************************/
