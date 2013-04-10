#include "stopwhenclosetootheragentbehavior.h"

/******************************************************************************/
/******************************************************************************/

CStopWhenCloseToOtherAgentBehavior::CStopWhenCloseToOtherAgentBehavior(double f_distance) : m_fDistance(f_distance)
{
}

/******************************************************************************/
/******************************************************************************/

bool CStopWhenCloseToOtherAgentBehavior::TakeControl()
{
//    if (m_pcAgent->CountAgents(m_fDistance, ROBOT) == 0 && m_pcAgent->GetClosestAgent(m_fDistance, ANY) != NULL) {
//        printf("Something is very wrong\n");
//    }

//    return m_pcAgent->CountAgents(m_fDistance, ROBOT) > 0;
    return false;
}

/******************************************************************************/
/******************************************************************************/

void CStopWhenCloseToOtherAgentBehavior::Action()
{
    TVector2d newvelocity;
    newvelocity.x = 0.0;
    newvelocity.y = 0.0;

    m_pcAgent->SetVelocity(&newvelocity);
}

/******************************************************************************/
/******************************************************************************/

void CStopWhenCloseToOtherAgentBehavior::SetAgent(CAgent* pc_agent)
{
    CBehavior::SetAgent(pc_agent);
}

/******************************************************************************/
/******************************************************************************/

