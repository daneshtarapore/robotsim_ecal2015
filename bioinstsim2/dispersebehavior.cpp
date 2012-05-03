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
    return m_tCenterOfMass.x != 0.0 && m_tCenterOfMass.y != 0.0;
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::SimulationStep() 
{
    m_tCenterOfMass = m_pcAgent->GetCenterOfMassOfSurroundingAgents(m_fSensoryRadius, ANY);
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::Action()
{
    if (m_tCenterOfMass.x != 0.0 && m_tCenterOfMass.y != 0.0) 
    {
        m_tCenterOfMass.x = -m_tCenterOfMass.x;
        m_tCenterOfMass.y = -m_tCenterOfMass.y;

        m_pcAgent->MoveTowards(m_tCenterOfMass, m_pcAgent->GetMaximumSpeed());
    }
}

/******************************************************************************/
/******************************************************************************/
