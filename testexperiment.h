#ifndef TESTEXPERIMENT_H_
#define TESTEXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

class CTestExperiment : public CExperiment
{
public:
    CTestExperiment(CArguments* pc_experiment_arguments,
                    CArguments* pc_arena_arguments,
                    CArguments* pc_agent_arguments,
                    CArguments* pc_crm_arguments);
    
    virtual CAgent*     CreateAgent();
};

/******************************************************************************/
/******************************************************************************/

#endif
