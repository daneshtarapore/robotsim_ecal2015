//bioinstsim2 -a sizex=100,sizey=100,resx=50,resy=50,help -e name=TEST,swarmbehav=AGGREGATION,help -T maxspeed=0.1,count=50,fvsenserange=10,featuresenserange=6,bitflipprob=0.0,help -M,numberoffeatures=8,exchangeprob=0.0,cross-affinity=0.4,help -s 111,help -n 10000,help

#include <vector>
#include "testexperiment.h"
#include "robotagent.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "flockbehavior.h"
#include "homingbehavior.h"
#include "randomwalkbehavior.h"
#include "circlebehavior.h"
#include "stopbehavior.h"

/******************************************************************************/
/******************************************************************************/

CTestExperiment::CTestExperiment(CArguments* pc_experiment_arguments,
                                 CArguments* pc_arena_arguments,
                                 CArguments* pc_agent_arguments,
                                 CArguments* pc_crm_arguments) :
    CExperiment(pc_experiment_arguments, pc_arena_arguments, pc_agent_arguments, pc_crm_arguments)
{    
    static bool bHelpDisplayed = false;

    const char* swarmbehav = pc_experiment_arguments->GetArgumentAsStringOr("swarmbehav", "FLOCKING");

    if (strcmp(swarmbehav, "AGGREGATION") == 0)
    {
        m_eswarmbehavType = AGGREGATION;
    }
    else if (strcmp(swarmbehav, "DISPERSION") == 0)
    {
        m_eswarmbehavType = DISPERSION;
    }
    else if (strcmp(swarmbehav, "FLOCKING") == 0)
    {
        m_eswarmbehavType = FLOCKING;
    }
    else if (strcmp(swarmbehav, "HOMING1") == 0)
    {
        m_eswarmbehavType = HOMING1;
    }
    else if (strcmp(swarmbehav, "HOMING2") == 0)
    {
        m_eswarmbehavType = HOMING2;
    }

    const char* errorbehav = pc_experiment_arguments->GetArgumentAsStringOr("errorbehav", "");
    if (strcmp(errorbehav, "STRLN") == 0)
    {
        m_eerrorbehavType = STRAIGHTLINE;
    }
    else if (strcmp(errorbehav, "RNDWK") == 0)
    {
        m_eerrorbehavType = RANDOMWK;
    }
    else if (strcmp(errorbehav, "CIRCLE") == 0)
    {
        m_eerrorbehavType = CIRCLE;
    }
    else if (strcmp(errorbehav, "STOP") == 0)
    {
        m_eerrorbehavType = STOP;
    }
    else
    {
        m_eerrorbehavType = NOERR;
    }

    m_unMisbehaveStep = pc_experiment_arguments->GetArgumentAsIntOr("misbehavestep", 0);

    m_unNormalAgentToTrack   = pc_experiment_arguments->GetArgumentAsIntOr("tracknormalagent", 0);
    m_unAbnormalAgentToTrack = pc_experiment_arguments->GetArgumentAsIntOr("trackabnormalagent",15);
    m_unNumAbnormalAgents    = pc_experiment_arguments->GetArgumentAsIntOr("numabnormalagents",1);


    if (pc_experiment_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        char* pchSwarmBehavior = "INVALID";
        switch (m_eswarmbehavType) {
        case AGGREGATION : pchSwarmBehavior = "AGGREGATION"; break;
        case DISPERSION  : pchSwarmBehavior = "DISPERSION"; break;
        case FLOCKING    : pchSwarmBehavior = "FLOCKING"; break;
        case HOMING1     : pchSwarmBehavior = "HOMING1"; break;
        case HOMING2     : pchSwarmBehavior = "HOMING2"; break;
        default:
            pchSwarmBehavior = "UNKNOWN"; 
        }

        char* pchErrorBehavior = "INVALID";
        switch (m_eerrorbehavType) {
        case STRAIGHTLINE  : pchErrorBehavior = "STRAIGHTLINE"; break;
        case RANDOMWK      : pchErrorBehavior = "RANDOMWK"; break;
        case CIRCLE        : pchErrorBehavior = "CIRCLE"; break;
        case STOP          : pchErrorBehavior = "STOP"; break;
        case NOERR         : pchErrorBehavior = "NOERR"; break;
        default:
            pchErrorBehavior = "UNKNOWN"; 
        }

        printf("swarmbehav=[AGGREGATION,DISPERSION,FLOCKING,HOMING1,HOMING2] -- behavior selected: %s\n", pchSwarmBehavior);
        printf("errorbehav=[STRLN,RNDWK,CIRCLE,STOP ]                        -- behavior selected: %s\n", pchErrorBehavior);
        printf("misbehavestep=#     Step when agent starts misbehaving [%d]\n",m_unMisbehaveStep);
        printf("tracknormalagent=#    Id of normal agent to track [%d]\n",  m_unNormalAgentToTrack);
        printf("trackabnormalagent=#  Id of abnormal agent to track [%d]\n",m_unAbnormalAgentToTrack);
        printf("numabnormalagents=#  Number of abnormal agents [%d]\n",m_unNumAbnormalAgents);

        bHelpDisplayed = true;
    }

    for(int i = 0; i < 20; i++)
    {
        m_pcMisbehaveAgent[i]     = NULL;
    }
    m_pcNormalAgentToTrack = NULL;
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
    if(m_eswarmbehavType == DISPERSION)
    {
        CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(3);
        vecBehaviors.push_back(pcDisperseBehavior2);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Aggregation behavior: formation of clusters of robots
    // Parameters: Dispersion range (d), Aggregation range (a)
    // The number of robots in each cluster seems to be proportional to d/a
    if(m_eswarmbehavType == AGGREGATION)
    {
        CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(3); //1
        vecBehaviors.push_back(pcDisperseBehavior2);
        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(10);
        vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }


    // Homing behavior 2: Ind. robots follow the robot that was placed immediately before them or the first robot
    // Parameters: Dispersion range (d), Homing range (h)
    // The dispersion range d > 0, else robots collapse into their individual leaders
    // If dispersion range d is too high (e.g. 5), other robots besides the leader disrupt the homing behavior - resulting in slowly moving clusters of robots
    if(m_eswarmbehavType == HOMING2)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(3);
        vecBehaviors.push_back(pcDisperseBehavior);
        CHomingBehavior* pcHomingBehavior = new CHomingBehavior(10000, pcPreviousAgent);
        vecBehaviors.push_back(pcHomingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Homing behavior 1: All other robots follow a single leader bot
    // The robots form a single cluster around the leader-bot. The bahavior is quite similar to the aggregation behav.
    // The dispersion range d > 0 else robots collapse into leader.
    // Increasing d increases the area occupied by the cluster
    if(m_eswarmbehavType == HOMING1)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(3);
        vecBehaviors.push_back(pcDisperseBehavior);
        CHomingBehavior* pcHomingBehavior = new CHomingBehavior(100, pcPreviousAgent);
        vecBehaviors.push_back(pcHomingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }


    // Flocking behavior:
    // Parameters: Dispersion range (d), Flocking range (f)
    // Dispersion range d is proportional to the size of individual flock of robots
    // Flocking range f influences how fast a flock is formed. Also f > d to initiate flocking
    if(m_eswarmbehavType == FLOCKING)
    {
        //behav. inserted is decreasing order of priority to take control of the agent

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(1); //5
        vecBehaviors.push_back(pcDisperseBehavior);
        CFlockBehavior* pcFlockBehavior = new CFlockBehavior(3); //10
        vecBehaviors.push_back(pcFlockBehavior);
        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(5);
        vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);

//            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(4);
//            vecBehaviors.push_back(pcDisperseBehavior);
//            CFlockBehavior* pcFlockBehavior = new CFlockBehavior(5);
//            vecBehaviors.push_back(pcFlockBehavior);
//            CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(10);
//            vecBehaviors.push_back(pcAggregateBehavior);
        //           CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        //           vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    CAgent* pcAgent = new CRobotAgent("robot", id++, m_pcAgentArguments, m_pcCRMArguments, vecBehaviors);

    for(int i = 0; i < m_unNumAbnormalAgents; i++)
    {
        if ((id - 1 + i) == m_unAbnormalAgentToTrack)
        {
            m_pcMisbehaveAgent[i] = (CRobotAgent*) pcAgent;
        }
    }

    if ((id - 1) == m_unNormalAgentToTrack)
    {
        m_pcNormalAgentToTrack = (CRobotAgent*) pcAgent;
    }

    if(m_eswarmbehavType == HOMING1)
    {
        if (pcPreviousAgent == NULL)
            pcPreviousAgent = pcAgent;
    }
    else
    {
        pcPreviousAgent = pcAgent;
    }

    return pcAgent; //pcPreviousAgent;
}

/******************************************************************************/
/******************************************************************************/

void CTestExperiment::PrintVelocityDifference(CAgent* pc_agent, double f_range) 
{
    // Velocity magnitude and direction wrt. surrounding agents:
    TVector2d tTemp    = pc_agent->GetAverageVelocityOfSurroundingAgents(f_range, ROBOT);
    
    float dir_relativeagentvelocity = 0;
    float tmp_agentvelocity         = Vec2dLength((*pc_agent->GetVelocity()));
    float mag_relativeagentvelocity = tmp_agentvelocity - Vec2dLength((tTemp));

    if (Vec2dLength((*pc_agent->GetVelocity())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
        dir_relativeagentvelocity = Vec2dAngle((*pc_agent->GetVelocity()), tTemp);
    else
        dir_relativeagentvelocity = 0.0;

    printf("%f:[ mag: %f, dir: %f ]", f_range, mag_relativeagentvelocity, dir_relativeagentvelocity);
}

/******************************************************************************/
/******************************************************************************/


void CTestExperiment::PrintStatsForAgent(CAgent* pc_agent)
{
    unsigned int unAgentsWithIn2            = pc_agent->CountAgents(2.0, ROBOT);
    unsigned int unAgentsWithIn4            = pc_agent->CountAgents(4.0, ROBOT);
    unsigned int unAgentsWithIn6            = pc_agent->CountAgents(6.0, ROBOT);
    unsigned int unAgentsWithIn8            = pc_agent->CountAgents(8.0, ROBOT);
    unsigned int unAgentsWithIn10           = pc_agent->CountAgents(10.0, ROBOT);
    
    printf("AgentsWithin - 2: %d, 4: %d, 6: %d, 8: %d, 10: %d, ", unAgentsWithIn2, unAgentsWithIn4, unAgentsWithIn6, unAgentsWithIn8, unAgentsWithIn10);

    double fAverageDistanceToAgentsWithin2  = pc_agent->GetAverageDistanceToSurroundingAgents(2.0, ROBOT);
    double fAverageDistanceToAgentsWithin4  = pc_agent->GetAverageDistanceToSurroundingAgents(4.0, ROBOT);
    double fAverageDistanceToAgentsWithin6  = pc_agent->GetAverageDistanceToSurroundingAgents(6.0, ROBOT);
    double fAverageDistanceToAgentsWithin8  = pc_agent->GetAverageDistanceToSurroundingAgents(8.0, ROBOT);
    double fAverageDistanceToAgentsWithin10 = pc_agent->GetAverageDistanceToSurroundingAgents(10.0, ROBOT);

    printf("AverageDistanceToAgentsWithin - 2: %f, 4: %f, 6: %f, 8: %f, 10: %f, ", fAverageDistanceToAgentsWithin2, fAverageDistanceToAgentsWithin4,
           fAverageDistanceToAgentsWithin6, fAverageDistanceToAgentsWithin8, fAverageDistanceToAgentsWithin10);

    printf("VelocityDifference: "); 
    PrintVelocityDifference(pc_agent, 2);
    printf(", ");
    PrintVelocityDifference(pc_agent, 4);
    printf(", ");
    PrintVelocityDifference(pc_agent, 6);
    printf(", ");
    PrintVelocityDifference(pc_agent, 8);
    printf(", ");
    PrintVelocityDifference(pc_agent, 10);
    printf(", ");
       
    printf("\n");

}


/******************************************************************************/
/******************************************************************************/


void CTestExperiment::SimulationStep(unsigned int un_step_number)
{
    if (m_pcMisbehaveAgent[0] && un_step_number > CRMSTARTTIME)
    {
        unsigned int unToleraters  = 0;
        unsigned int unAttackers   = 0;
        unsigned int unUnConverged = 0;

        m_pcMisbehaveAgent[0]->CheckNeighborsReponseToMyFV(&unToleraters, &unAttackers, &unUnConverged);
        printf("\nStep: %d, MisbehavingAgentResponse: tol: %d, att: %d, unconvg: %d", un_step_number, unToleraters, unAttackers, unUnConverged);
        printf("\nMisbehavingAgentFeatureVector: %d\n\n", m_pcMisbehaveAgent[0]->GetFeatureVector()->GetValue());
        printf("\nMisbehavingAgentStats: ");
        PrintStatsForAgent(m_pcMisbehaveAgent[0]);
    }

    if (m_pcNormalAgentToTrack && un_step_number > CRMSTARTTIME)
    {
        unsigned int unToleraters = 0;
        unsigned int unAttackers  = 0;
        unsigned int unUnConverged = 0;

        m_pcNormalAgentToTrack->CheckNeighborsReponseToMyFV(&unToleraters, &unAttackers, &unUnConverged);
        printf("\nStep: %d, NormalAgentResponse: tol: %d, att: %d, unconvg: %d", un_step_number, unToleraters, unAttackers, unUnConverged);
        printf("\nNormalAgentFeatureVector: %d\n\n", m_pcNormalAgentToTrack->GetFeatureVector()->GetValue());
        printf("\nNormalAgentStats: ");
        PrintStatsForAgent(m_pcNormalAgentToTrack);
    }


    if (un_step_number == m_unMisbehaveStep)
    {
        for(int i = 0; i < m_unNumAbnormalAgents; i++)
        {
            vector<CBehavior*> vecBehaviors;
            if(m_eerrorbehavType == STRAIGHTLINE)
            {
                CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0);
                vecBehaviors.push_back(pcRandomWalkBehavior);
            }
            else if(m_eerrorbehavType == RANDOMWK)
            {
                CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
                vecBehaviors.push_back(pcRandomWalkBehavior);
            }
            else if(m_eerrorbehavType == CIRCLE)
            {
                CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
                vecBehaviors.push_back(pcCircleBehavior);
            }
            else if(m_eerrorbehavType == STOP)
            {
                CStopBehavior* pcStopBehavior = new CStopBehavior();
                vecBehaviors.push_back(pcStopBehavior);
            }
            else
            {
                printf("\n No error behavior");
                exit(-1);
            }

            m_pcMisbehaveAgent[i]->SetBehaviors(vecBehaviors);
        }
    }
}



/******************************************************************************/
/******************************************************************************/


