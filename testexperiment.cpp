#include <vector>
#include "testexperiment.h"
#include "robotagent.h"
#include "aggregatebehavior.h"

/******************************************************************************/
/******************************************************************************/

CTestExperiment::CTestExperiment(CArguments* pc_experiment_arguments,
                                 CArguments* pc_arena_arguments,
                                 CArguments* pc_agent_arguments) :
    CExperiment(pc_experiment_arguments, pc_arena_arguments, pc_agent_arguments)
{}

/******************************************************************************/
/******************************************************************************/

CAgent* CTestExperiment::CreateAgent() 
{
    static unsigned int id = 0;

    CAggregateBehavior* pcAggregationBehavior = new CAggregateBehavior();
    vector<CBehavior*> vecBehaviors;
    vecBehaviors.push_back(pcAggregationBehavior);
    return new CRobotAgent("robot", id++, m_pcAgentArguments, vecBehaviors);

}

/******************************************************************************/
/******************************************************************************/


