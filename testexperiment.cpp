#include <vector>
#include "testexperiment.h"
#include "robotagent.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"

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
    vector<CBehavior*> vecBehaviors;

    CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(2);
    vecBehaviors.push_back(pcAggregateBehavior);
//    CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior();
//    vecBehaviors.push_back(pcDisperseBehavior);

    return new CRobotAgent("robot", id++, m_pcAgentArguments, vecBehaviors);

}

/******************************************************************************/
/******************************************************************************/


