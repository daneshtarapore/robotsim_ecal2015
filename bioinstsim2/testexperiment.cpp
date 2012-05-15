#include <vector>
#include "testexperiment.h"
#include "robotagent.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "flockbehavior.h"
#include "homingbehavior.h"
#include "randomwalkbehavior.h"

/******************************************************************************/
/******************************************************************************/

CTestExperiment::CTestExperiment(CArguments* pc_experiment_arguments,
                                 CArguments* pc_arena_arguments,
                                 CArguments* pc_agent_arguments,
                                 CArguments* pc_crm_arguments) :
    CExperiment(pc_experiment_arguments, pc_arena_arguments, pc_agent_arguments, pc_crm_arguments)
{    
}

/******************************************************************************/
/******************************************************************************/

CAgent* CTestExperiment::CreateAgent() 
{
    static unsigned int id = 0;
    static CAgent* pcPreviousAgent = NULL;

    vector<CBehavior*> vecBehaviors;
    CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(1);
    vecBehaviors.push_back(pcDisperseBehavior);

    CHomingBehavior* pcHomingBehavior = new CHomingBehavior(1000, pcPreviousAgent);
    vecBehaviors.push_back(pcHomingBehavior);

    CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
    vecBehaviors.push_back(pcRandomWalkBehavior);

//    CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(1);
//    vecBehaviors.push_back(pcAggregateBehavior);
//    CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(2);
//    vecBehaviors.push_back(pcDisperseBehavior2);
//    CFlockBehavior* pcFlockBehavior = new CFlockBehavior(2);
//    vecBehaviors.push_back(pcFlockBehavior);
//    CFlockBehavior* pcFlockBehavior = new CFlockBehavior(3);
//    vecBehaviors.push_back(pcFlockBehavior);
    
    CAgent* pcAgent = new CRobotAgent("robot", id++, m_pcAgentArguments, m_pcCRMArguments, vecBehaviors);
//a    if (pcPreviousAgent == NULL)
        pcPreviousAgent = pcAgent;
    return pcAgent; //pcPreviousAgent;
}

/******************************************************************************/
/******************************************************************************/


