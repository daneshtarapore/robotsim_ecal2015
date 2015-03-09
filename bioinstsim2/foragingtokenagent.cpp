#include "foragingtokenagent.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

CForagingTokenAgent::CForagingTokenAgent(const char* pch_name, unsigned int un_identification, unsigned int un_numResourcesInToken, CArguments* pc_arguments) :
        CAgent(pch_name, un_identification, pc_arguments)
{
    un_numResourcesInToken = un_numResourcesInToken;
}

/******************************************************************************/
/******************************************************************************/

CForagingTokenAgent::~CForagingTokenAgent()
{
}

/******************************************************************************/
/******************************************************************************/

void CForagingTokenAgent::SimulationStepUpdatePosition()
{
    CForagingExperiment* foragingexpt = (CForagingExperiment*)CSimulator::GetInstance()->GetExperiment();
    unsigned int un_numForagingTokens = foragingexpt->GetNumForagingTokens();

    assert(GetIdentification() > 0);

    const TVector2d* tOldPosition;
    tOldPosition = GetPosition();

    //double f_angle =  (GetIdentification()-1) * (2*M_PI/un_numForagingTokens);
    //double f_radius = 20.0;

    TVector2d tNewPosition;
    tNewPosition.x = tOldPosition->x;//f_radius * cos(f_angle);
    tNewPosition.y = tOldPosition->y;//f_radius * sin(f_angle);

    SetPosition(&tNewPosition);

    // foraging tokens are stationary
    TVector2d tNewVelocity;
    tNewVelocity.x=0;
    tNewVelocity.y=0;
    SetVelocity(&tNewVelocity);

    CAgent::SimulationStepUpdatePosition();
}

/******************************************************************************/
/******************************************************************************/
