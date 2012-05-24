#ifndef CIRCLEBEHAVIOR_H
#define CIRCLEBEHAVIOR_H

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CCircleBehavior : public CBehavior
{
public:
    CCircleBehavior(double f_radius);

    virtual bool TakeControl();
    virtual void Action();
    virtual void SetAgent(CAgent* pc_agent);


protected:
    double m_fradius;
    double m_ftheta;
    TVector2d pt_circlecenter;
};

/******************************************************************************/
/******************************************************************************/


#endif // CIRCLEBEHAVIOR_H
