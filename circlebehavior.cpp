#include "circlebehavior.h"


/******************************************************************************/
/******************************************************************************/

CCircleBehavior::CCircleBehavior(double f_radius) :
    m_fradius(f_radius)
{
    m_ftheta = 0.0;

    pt_circlecenter.x = m_pcAgent->GetPosition()->x - f_radius;
    pt_circlecenter.y = m_pcAgent->GetPosition()->y;
}

/******************************************************************************/
/******************************************************************************/

bool CCircleBehavior::TakeControl()
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CCircleBehavior::Action()
{
    m_ftheta += 3.142*2.0 / 1000.0;

    if (m_ftheta > 3.142*2.0)
        m_ftheta = 0.0;

    TVector2d newpos;
    newpos.x = pt_circlecenter.x + m_fradius*cos(m_ftheta);
    newpos.y = pt_circlecenter.y + m_fradius*sin(m_ftheta);

    m_pcAgent->MoveTowards(newpos, m_pcAgent->GetMaximumSpeed());

    //if (m_fChangeDirectionProbability >= Random::nextDouble())
    //    m_pcAgent->SetRandomVelocity();
}

/******************************************************************************/
/******************************************************************************/

