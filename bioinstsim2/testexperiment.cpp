//bioinstsim2 -a sizex=100,sizey=100,resx=50,resy=50,help -e name=TEST,help -T maxspeed=0.1,count=50,fvsenserange=10,featuresenserange=5,bitflipprob=0.0,help -M,numberoffeatures=4,exchangeprob=0.0,cross-affinity=0.4,help -s 111,help -n 10000,help


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


    // Dispersion behavior
    // Parameters: Dispersion range (d)
    // Robots disperse with the distance between them decided by dispersion range d in CDisperseBehavior(d)
    // Note that for large value of d (e.g. 50), the robots collapse into each other!
    /*{
        CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(10);
        vecBehaviors.push_back(pcDisperseBehavior2);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }*/


    // Aggregation behavior: formation of clusters of robots
    // Parameters: Dispersion range (d), Aggregation range (a)
    // The number of robots in each cluster seems to be proportional to d/a

    /*{
        CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(5); //1
        vecBehaviors.push_back(pcDisperseBehavior2);
        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(10);
        vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }*/


    // Homing behavior 1: Ind. robots follow the robot that was placed immediately before them or the first robot
    // Parameters: Dispersion range (d), Homing range (h)
    // The dispersion range d > 0, else robots collapse into their individual leaders
    // If dispersion range d is too high (e.g. 5), other robots besides the leader disrupt the homing behavior - resulting in slowly moving clusters of robots
    /*{
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(1);
        vecBehaviors.push_back(pcDisperseBehavior);
        CHomingBehavior* pcHomingBehavior = new CHomingBehavior(100, pcPreviousAgent);
        vecBehaviors.push_back(pcHomingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }*/

    // Homing behavior 2: All other robots follow a single leader bot
    // The robots form a single cluster around the leader-bot. The bahavior is quite similar to the aggregation behav.
    // The dispersion range d > 0 else robots collapse into leader.
    // Increasing d increases the area occupied by the cluster
    /*{
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(5);
        vecBehaviors.push_back(pcDisperseBehavior);
        CHomingBehavior* pcHomingBehavior = new CHomingBehavior(100, pcPreviousAgent);
        vecBehaviors.push_back(pcHomingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }*/


    // Flocking behavior:
    // Parameters: Dispersion range (d), Flocking range (f)
    // Dispersion range d is proportional to the size of individual flock of robots
    // Flocking range f influences how fast a flock is formed. Also f > d to initiate flocking
    {
        CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(5);
        vecBehaviors.push_back(pcDisperseBehavior2);
        CFlockBehavior* pcFlockBehavior = new CFlockBehavior(10);
        vecBehaviors.push_back(pcFlockBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    //CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(1);
    //vecBehaviors.push_back(pcAggregateBehavior);
    //CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(2);
    //vecBehaviors.push_back(pcDisperseBehavior2);
    //CFlockBehavior* pcFlockBehavior = new CFlockBehavior(2);
    //vecBehaviors.push_back(pcFlockBehavior);
    //CFlockBehavior* pcFlockBehavior = new CFlockBehavior(3);
    //vecBehaviors.push_back(pcFlockBehavior);


    CAgent* pcAgent = new CRobotAgent("robot", id++, m_pcAgentArguments, m_pcCRMArguments, vecBehaviors);


   // if condition enabled, all robots follow a single leader
   // if (pcPreviousAgent == NULL)
        pcPreviousAgent = pcAgent;


    return pcAgent; //pcPreviousAgent;
}

/******************************************************************************/
/******************************************************************************/


