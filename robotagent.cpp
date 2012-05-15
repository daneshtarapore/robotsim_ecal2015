#include "robotagent.h"

/******************************************************************************/
/******************************************************************************/

CRobotAgent::CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments, CArguments* pc_crm_arguments, TBehaviorVector vec_behaviors) :
    CAgent(pch_name, un_identification, pc_arguments), m_vecBehaviors(vec_behaviors)
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }

    crminAgent = new CRMinRobotAgent(this, pc_crm_arguments);
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

    // Update the T-cells of the CRM instance for this robot
    crminAgent->SimulationStepUpdatePosition();

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

double CRobotAgent::GetFeaturesSensed(unsigned int*  m_punFeaturesSensed)
{

}

/******************************************************************************/
/******************************************************************************/

CRobotAgent* CRobotAgent::TryToConnectToRandomRobotAgentWithWeights()
{

}

/******************************************************************************/
/******************************************************************************/

CRMinRobotAgent* CRobotAgent::GetCRMinRobotAgent()
{
    return crminAgent;
}

/******************************************************************************/
/******************************************************************************/

