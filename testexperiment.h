#ifndef TESTEXPERIMENT_H_
#define TESTEXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"
#include "robotagent.h"

/******************************************************************************/
/******************************************************************************/

enum ESwarmBehavType
{
    AGGREGATION,
    DISPERSION,
    FLOCKING,
    HOMING1,
    HOMING2
};

enum EErrorBehavType
{
    STRAIGHTLINE,
    RANDOMWK,
    CIRCLE,
    STOP,
    NOERR
};

class CTestExperiment : public CExperiment
{
public:
    CTestExperiment(CArguments* pc_experiment_arguments,
                    CArguments* pc_arena_arguments,
                    CArguments* pc_agent_arguments,
                    CArguments* pc_crm_arguments);
    
    virtual CAgent*     CreateAgent();
    virtual void SimulationStep(unsigned int un_step_number);


protected:
    ESwarmBehavType m_eswarmbehavType;
    EErrorBehavType m_eerrorbehavType;

    unsigned int    m_unMisbehaveStep;
    CRobotAgent*    m_pcMisbehaveAgent;
};

/******************************************************************************/
/******************************************************************************/

#endif
