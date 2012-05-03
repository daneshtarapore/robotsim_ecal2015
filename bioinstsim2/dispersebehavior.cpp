#include "dispersebehavior.h"

/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior(double f_sensory_radius) :
    m_fSensoryRadius(f_sensory_radius) 
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CDisperseBehavior::TakeControl() 
{
    return m_pcAgent->CountAgents(m_fSensoryRadius, ROBOT) > 0;
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::SimulationStep() 
{
    m_tCenterOfMass = m_pcAgent->GetCenterOfMassOfSurroundingAgents(m_fSensoryRadius, ROBOT);
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::Action()
{
    m_tCenterOfMass.x = -m_tCenterOfMass.x;
    m_tCenterOfMass.y = -m_tCenterOfMass.y;
   
    m_pcAgent->MoveTowards(m_tCenterOfMass, m_pcAgent->GetMaximumSpeed());
}

/******************************************************************************/
/******************************************************************************/
