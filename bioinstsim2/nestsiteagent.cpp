#include "nestsiteagent.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

CNestSiteAgent::CNestSiteAgent(const char* pch_name, unsigned int un_identification, float f_numTokensCollected, CArguments* pc_arguments) :
        CAgent(pch_name, un_identification, pc_arguments)
{
    this->f_numTokensCollected = f_numTokensCollected;
}

/******************************************************************************/
/******************************************************************************/

CNestSiteAgent::~CNestSiteAgent()
{
}

/******************************************************************************/
/******************************************************************************/

void CNestSiteAgent::SimulationStepUpdatePosition()
{
    CForagingExperiment* foragingexpt = (CForagingExperiment*)CSimulator::GetInstance()->GetExperiment();
    foragingexpt->SetNumTokensCollected(un_numTokensCollected);

    assert(GetIdentification() > 0);

    TVector2d tNewPosition;
    tNewPosition.x = 0.0;
    tNewPosition.y = 0.0;

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
