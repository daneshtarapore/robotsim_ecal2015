#ifndef NESTSITEAGENT_H_
#define NESTSITEAGENT_H_

/******************************************************************************/
/******************************************************************************/

class CNestSiteAgent;

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

class CNestSiteAgent : public CAgent
{
public: 
    CNestSiteAgent(const char* pch_name, unsigned int un_identification, float f_numTokensInNest, CArguments *pc_arguments);

    virtual ~CNestSiteAgent();
    
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStepUpdatePosition();
    
    virtual inline EAgentType   GetType() {return NESTSITE;}

    //virtual void Sense();
    virtual inline unsigned int GetColor() {return GREY;}

    inline float GetNumTokensCollected() {return f_numTokensInNest;}

protected:
    float f_numTokensInNest;

};

/******************************************************************************/
/******************************************************************************/

#endif // NESTSITEAGENT_H_

/******************************************************************************/
/******************************************************************************/
