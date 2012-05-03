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
    return m_tCenterOfMass.m_fX != 0.0 && m_tCenterOfMass.m_fY != 0.0;
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
    if (m_tCenterOfMass.m_fX != 0.0 && m_tCenterOfMass.m_fY != 0.0) 
    {
        m_tCenterOfMass.m_fX = -m_tCenterOfMass.m_fX;
        m_tCenterOfMass.m_fY = -m_tCenterOfMass.m_fY;

        m_pcAgent->MoveTowards(m_tCenterOfMass, m_pcAgent->GetMaximumSpeed());
    }
}

/******************************************************************************/
/******************************************************************************/
