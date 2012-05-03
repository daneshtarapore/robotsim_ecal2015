#include <vector>
#include "testexperiment.h"
#include "robotagent.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "flockbehavior.h"

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
    CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(1);
    vecBehaviors.push_back(pcDisperseBehavior);
//    CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(3);
//    vecBehaviors.push_back(pcAggregateBehavior);
    CFlockBehavior* pcFlockBehavior = new CFlockBehavior(5);
    vecBehaviors.push_back(pcFlockBehavior);

    return new CRobotAgent("robot", id++, m_pcAgentArguments, vecBehaviors);
}

/******************************************************************************/
/******************************************************************************/


