#include "flockbehavior.h"

/******************************************************************************/
/******************************************************************************/

CFlockBehavior::CFlockBehavior(double f_sensory_radius) :
    m_fSensoryRadius(f_sensory_radius) 
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CFlockBehavior::TakeControl() 
{
    return m_pcAgent->CountAgents(m_fSensoryRadius, ROBOT) > 0;
}

/******************************************************************************/
/******************************************************************************/

void CFlockBehavior::SimulationStep() 
{
    m_tVelocity = m_pcAgent->GetAverageVelocityOfSurroundingAgents(m_fSensoryRadius, ROBOT);
}

/******************************************************************************/
/******************************************************************************/

void CFlockBehavior::Action()
{    
    double fVelocityLength = Vec2dLength(m_tVelocity);
    if (fVelocityLength > 0.000001)
    {
        Vec2dMultiplyScalar(m_tVelocity, m_pcAgent->GetMaximumSpeed() / fVelocityLength);
        m_pcAgent->SetVelocity(&m_tVelocity);

    }
}

/******************************************************************************/
/******************************************************************************/
