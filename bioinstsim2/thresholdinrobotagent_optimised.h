#ifndef THRESHOLDINROBOTAGENTOPTIMISED_H
#define THRESHOLDINROBOTAGENTOPTIMISED_H

/******************************************************************************/
/******************************************************************************/
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <list>
#include "arguments.h"
#include "featurevector.h"
#include "random.h"
#include "robotagent_optimised.h"

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

class ThresholdinRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

class CRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

class ThresholdinRobotAgentOptimised
{
public:
    ThresholdinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent, CArguments* m_crmArguments);

    virtual ~ThresholdinRobotAgentOptimised();

    virtual void SimulationStepUpdatePosition();

protected:

    CRobotAgentOptimised* robotAgent;

    virtual void UpdateState();

    unsigned m_uThreshold;

    unsigned m_uTolerableHD; // Tolerable hamming distance THD (in bits). FVs THD apart are not classified separately
};


/******************************************************************************/
/******************************************************************************/

#endif // THRESHOLDINROBOTAGENTOPTIMISED_H
