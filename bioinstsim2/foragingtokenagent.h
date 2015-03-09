#ifndef FORAGINGTOKENAGENT_H_
#define FORAGINGTOKENAGENT_H_

/******************************************************************************/
/******************************************************************************/

class CForagingTokenAgent;

/******************************************************************************/
/******************************************************************************/

#include <list>
#include "common.h"
#include "agent.h"
#include "foragingexperiment.h"

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CForagingTokenAgent : public CAgent
{
public: 
    CForagingTokenAgent(const char* pch_name, unsigned int un_identification, unsigned int un_numResourcesInToken, CArguments *pc_arguments);

    virtual ~CForagingTokenAgent();
    
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStepUpdatePosition();
    
    virtual inline EAgentType   GetType() {return FORAGINGTOKEN;}

    //virtual void Sense();
    virtual inline unsigned int GetColor() {return YELLOW;}

protected:
    unsigned int un_numResourcesInToken;

};

/******************************************************************************/
/******************************************************************************/

#endif // FORAGINGTOKENAGENT_H_

/******************************************************************************/
/******************************************************************************/
