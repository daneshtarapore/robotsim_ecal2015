#ifndef TESTEXPERIMENT_H_
#define TESTEXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

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

class CTestExperiment : public CExperiment
{
public:
    CTestExperiment(CArguments* pc_experiment_arguments,
                    CArguments* pc_arena_arguments,
                    CArguments* pc_agent_arguments,
                    CArguments* pc_crm_arguments);
    
    virtual CAgent*     CreateAgent();

protected:
    ESwarmBehavType m_eswarmbehavType;

    char swarmbehav[40];

};

/******************************************************************************/
/******************************************************************************/

#endif
