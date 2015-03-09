
#include <vector>
#include "foragingexperiment.h"
#include <iostream>

#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "flockbehavior.h"
#include "homingbehavior.h"
#include "randomwalkbehavior.h"
#include "circlebehavior.h"
#include "stopbehavior.h"
#include "stopwhenclosetootheragentbehavior.h"


/******************************************************************************/
/******************************************************************************/

CForagingExperiment::CForagingExperiment(CArguments* pc_experiment_arguments,
                                         CArguments* pc_arena_arguments,
                                         CArguments* pc_agent_arguments,
                                         CArguments* pc_model_arguments) :
    CExperiment(pc_experiment_arguments, pc_arena_arguments, pc_agent_arguments, pc_model_arguments)
{    
    static bool bHelpDisplayed = false;

    m_eswarmbehavType = DISPERSION;

    const char* errorbehav = pc_experiment_arguments->GetArgumentAsStringOr("errorbehav", "");
    if (strcmp(errorbehav, "STRLN") == 0)
        m_eerrorbehavType = STRAIGHTLINE;
    else if (strcmp(errorbehav, "RNDWK") == 0)
        m_eerrorbehavType = RANDOMWK;
    else if (strcmp(errorbehav, "CIRCLE") == 0)
        m_eerrorbehavType = CIRCLE;
    else if (strcmp(errorbehav, "STOP") == 0)
        m_eerrorbehavType = STOP;
    else if (strcmp(errorbehav, "AGGREGATION") == 0)
        m_eerrorbehavType = AGGREGATION;
    else if (strcmp(errorbehav, "DISPERSION") == 0)
        m_eerrorbehavType = DISPERSION;
    else if (strcmp(errorbehav, "FLOCKING") == 0)
        m_eerrorbehavType = FLOCKING;
    else if (strcmp(errorbehav, "HOMING1") == 0)
        m_eerrorbehavType = HOMING1;
    else if (strcmp(errorbehav, "HOMING2") == 0)
        m_eerrorbehavType = HOMING2;
    else
        m_eerrorbehavType = NOERR;

    m_unNumRobots         = pc_experiment_arguments->GetArgumentAsIntOr("numrobots", 20);
    m_unNumForagingTokens = pc_experiment_arguments->GetArgumentAsIntOr("numtokens", 10);

    m_unMisbehaveStep = pc_experiment_arguments->GetArgumentAsIntOr("misbehavestep", 0);

    m_unNormalAgentToTrack   = pc_experiment_arguments->GetArgumentAsIntOr("tracknormalagent", 0);
    m_unAbnormalAgentToTrack = pc_experiment_arguments->GetArgumentAsIntOr("trackabnormalagent", 15);
    m_unNumAbnormalAgents    = pc_experiment_arguments->GetArgumentAsIntOr("numabnormalagents", 1);


    if (pc_experiment_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        char* pchErrorBehavior = "INVALID";
        switch (m_eerrorbehavType) {
        case STRAIGHTLINE  : pchErrorBehavior = "STRAIGHTLINE"; break;
        case RANDOMWK      : pchErrorBehavior = "RANDOMWK"; break;
        case CIRCLE        : pchErrorBehavior = "CIRCLE"; break;
        case STOP          : pchErrorBehavior = "STOP"; break;
        case NOERR         : pchErrorBehavior = "NOERR"; break;

        case AGGREGATION   : pchErrorBehavior = "AGGREGATION"; break;
        case DISPERSION    : pchErrorBehavior = "DISPERSION"; break;
        case FLOCKING      : pchErrorBehavior = "FLOCKING"; break;
        case HOMING1       : pchErrorBehavior = "HOMING1"; break;
        case HOMING2       : pchErrorBehavior = "HOMING2"; break;

        default:
            pchErrorBehavior = "UNKNOWN";
        }

        printf("numrobots=#               Number of robots [%d]\n",m_unNumRobots);
        printf("numforagingtokens=#       Number of foraging tokens [%d]\n",m_unNumForagingTokens);
        printf("errorbehav=[STRLN,RNDWK,CIRCLE,STOP,AGGREGATION,DISPERSION,FLOCKING,HOMING1,HOMING2]                        -- behavior selected: %s\n", pchErrorBehavior);
        printf("misbehavestep=#       Step when agent starts misbehaving [%d]\n",m_unMisbehaveStep);
        printf("tracknormalagent=#    Id of normal agent to track [%d]\n",  m_unNormalAgentToTrack);
        printf("trackabnormalagent=#  Id of abnormal agent to track [%d]\n",m_unAbnormalAgentToTrack);
        printf("numabnormalagents=#   Number of abnormal agents [%d]\n",m_unNumAbnormalAgents);

        bHelpDisplayed = true;
    }

    unsigned int m_unNumNestSites = 1;
    assert(m_unNumForagingTokens + m_unNumRobots + m_unNumNestSites == m_unNumberOfAgents);

    for(int i = 0; i < m_unNumRobots; i++)
        m_pcMisbehaveAgent[i]     = NULL;

    m_pcNormalAgentToTrack = NULL;

    m_ppcListRobotsCreated = new CAgent*[m_unNumRobots];
    for(int i = 0; i < m_unNumRobots; i++)
        m_ppcListRobotsCreated[i]     = NULL;

    m_ppcListForagingTokensCreated = new CAgent*[m_unNumForagingTokens];
    for(int i = 0; i < m_unNumForagingTokens; i++)
        m_ppcListForagingTokensCreated[i]     = NULL;

    pcNestSiteAgent = NULL;

}

/******************************************************************************/
/******************************************************************************/

CForagingExperiment::~CForagingExperiment()
{
    delete m_ppcListRobotsCreated;
    delete m_ppcListForagingTokensCreated;
    delete pcNestSiteAgent;
}

/******************************************************************************/
/******************************************************************************/

CAgent* CForagingExperiment::CreateAgent()
{
    static unsigned int id = 0;
    CAgent* pcAgent;

    if(id < m_unNumRobots)
    {
        static CAgent* pcPreviousAgent = NULL;

        vector<CBehavior*> vecBehaviors;
        vecBehaviors = GetAgentBehavior(m_eswarmbehavType, pcPreviousAgent);

        pcAgent = new CRobotAgentOptimised("robot", id++, m_pcAgentArguments, m_pcModelArguments, vecBehaviors);
        pcAgent->SetBehavior(m_eswarmbehavType);

        for(int i = 0; i < m_unNumAbnormalAgents; i++)
            if ((id - 1 + i) == m_unAbnormalAgentToTrack)
            {
                m_pcMisbehaveAgent[i] = (CRobotAgentOptimised*) pcAgent;
                pcAgent->SetBehavIdentification(-1); //abnormal agent
            }

        if ((id - 1) == m_unNormalAgentToTrack)
        {
            m_pcNormalAgentToTrack = (CRobotAgentOptimised*) pcAgent;
            pcAgent->SetBehavIdentification(1); // normal agent
        }

        if(m_eswarmbehavType == HOMING1)
            if (pcPreviousAgent == NULL)
            {
                pcPreviousAgent = pcAgent;
                pcHomeToAgent   = pcAgent;
            }
            else
                pcPreviousAgent = pcAgent;

        m_ppcListRobotsCreated[id-1] = pcAgent;
    }
    else if(id < m_unNumRobots+m_unNumForagingTokens)
    {
        unsigned int un_numResourcesInToken = 10;
        pcAgent = new CForagingTokenAgent("token", id++, un_numResourcesInToken, m_pcAgentArguments);
        m_ppcListForagingTokensCreated[id - (m_unNumRobots+m_unNumForagingTokens)] = pcAgent;
    }
    else
    {
        float f_numTokensCollected = 0.0;
        pcAgent = new CNestSiteAgent("nest", id++, f_numTokensCollected, m_pcAgentArguments);
        pcNestSiteAgent = pcAgent;
    }

    return pcAgent;
}

/******************************************************************************/
/******************************************************************************/

void CForagingExperiment::PrintVelocityDifference(CAgent* pc_agent, double f_range)
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


    if(isnan(dir_relativeagentvelocity))
    {
        double cosinevalue = Vec2dCosAngle(tTemp, (*pc_agent->GetVelocity()));

        if(fabs(cosinevalue - 1.0) < 1.0e-3)
            dir_relativeagentvelocity = 0.0;
        else if(fabs(cosinevalue - -1.0) < 1.0e-3)
            dir_relativeagentvelocity = M_PI;
        else
        {
            dir_relativeagentvelocity = acos(cosinevalue);
        }
    }


    printf("%f:[ mag: %f, dir: %f ]", f_range, mag_relativeagentvelocity, dir_relativeagentvelocity);
}

/******************************************************************************/
/******************************************************************************/


void CForagingExperiment::PrintStatsForAgent(CAgent* pc_agent)
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
    PrintVelocityDifference(pc_agent, 2.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 4.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 6.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 8.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 10.0);
    printf(", ");

    printf("\n");
}


/******************************************************************************/
/******************************************************************************/

unsigned int CForagingExperiment::GetNumForagingTokens()
{
    return m_unNumForagingTokens;
}

/******************************************************************************/
/******************************************************************************/

void CForagingExperiment::SimulationStep(unsigned int un_step_number)
{
    if (un_step_number == m_unMisbehaveStep)
    {
        for(int i = 0; i < m_unNumAbnormalAgents; i++)
        {
            vector<CBehavior*> vecBehaviors;
            vecBehaviors = GetAgentBehavior(m_eerrorbehavType, pcHomeToAgent);
            m_pcMisbehaveAgent[i]->SetBehavior(m_eerrorbehavType);
            m_pcMisbehaveAgent[i]->SetBehaviors(vecBehaviors);
        }
    }


    // detailed log of the responses to abnormal agent
    if (m_pcMisbehaveAgent[0] && un_step_number > MODELSTARTTIME)
    {
        unsigned int unToleraters  = 0;
        unsigned int unAttackers   = 0;
        unsigned int unSuspectors   = 0;
        unsigned int unNbrsInSensoryRange = 0;

#ifdef DEBUGCROSSREGULATIONMODELFLAG
        bool dbgflag = true;
#else
        bool dbgflag = false;
#endif

        m_pcMisbehaveAgent[0]->CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unSuspectors, &unNbrsInSensoryRange, dbgflag);//true

        printf("\nStep: %d, MisbehavingAgentResponse: tol: %d, att: %d, susp: %d, neighboursinsensoryrange: %d", un_step_number, unToleraters, unAttackers, unSuspectors, unNbrsInSensoryRange);
        printf("\nMisbehavingAgentFeatureVector: %d\n\n", m_pcMisbehaveAgent[0]->GetFeatureVector()->GetValue());
    }

    // detailed log of the responses (minimal log) to a single normal agent being tracked
    if (m_pcNormalAgentToTrack && un_step_number > MODELSTARTTIME)
    {
        unsigned int unToleraters = 0;
        unsigned int unAttackers  = 0;
        unsigned int unSuspectors   = 0;
        unsigned int unNbrsInSensoryRange = 0;

#ifdef DEBUGCROSSREGULATIONMODELFLAG
        bool dbgflag = true;
#else
        bool dbgflag = false;
#endif

        m_pcNormalAgentToTrack->CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unSuspectors, &unNbrsInSensoryRange, dbgflag);

        printf("\nStep: %d, NormalAgentResponse: tol: %d, att: %d, neighboursinsensoryrange: %d", un_step_number, unToleraters, unAttackers, unNbrsInSensoryRange);
        printf("\nNormalAgentFeatureVector: %d\n\n", m_pcNormalAgentToTrack->GetFeatureVector()->GetValue());
    }

    TAgentVector* allagents = this->m_pcSimulator->GetAllAgents();
    TAgentVectorIterator i = allagents->begin();
    printf("\nStep: %d, AgentsFeatureVectors: ", un_step_number);
    while (i != allagents->end())
    {
        if ((*i)->GetType() == ROBOT)
        {
            CRobotAgentOptimised* tmp_robotagent  = (CRobotAgentOptimised*) (*i);
            const CFeatureVector* tmp_fv = tmp_robotagent->GetFeatureVector();
            printf("%d %d   ",tmp_robotagent->GetIdentification(), tmp_fv->GetValue());
        }
        i++;
    }

    printf("\n");


    // logging the responses (minimal log) to all the agents
    if(un_step_number > MODELSTARTTIME)
    {
        TAgentVector* allagents = this->m_pcSimulator->GetAllAgents();
        TAgentVectorIterator i = allagents->begin();
        while (i != allagents->end())
        {
            if ((*i)->GetType() == ROBOT)
            {
                CRobotAgentOptimised* tmp_robotagent  = (CRobotAgentOptimised*) (*i);
                const CFeatureVector* tmp_fv = tmp_robotagent->GetFeatureVector();

                unsigned int unToleraters  = 0;
                unsigned int unAttackers   = 0;
                unsigned int unSuspectors   = 0;
                unsigned int unNbrsInSensoryRange = 0;

                bool dbgflag = false;

                tmp_robotagent->CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unSuspectors, &unNbrsInSensoryRange, dbgflag);
                printf("\nResponsestoAllAgents: Step: %d, Id: %d, FV: %d, tol: %d, att: %d, susp: %d, neighboursinsensoryrange: %d", un_step_number, tmp_robotagent->GetIdentification(), tmp_fv->GetValue(), unToleraters, unAttackers, unSuspectors, unNbrsInSensoryRange);
            }
            i++;
        }
        printf("\n");
    }


#ifdef FLOATINGPOINTOPERATIONS
    if(un_step_number + 1 == CSimulator::GetInstance()->GetNumberOfCycles())
    {
        TAgentVector* allagents = m_pcSimulator->GetAllAgents();
        TAgentVectorIterator i = allagents->begin();
        printf("\n\nTimeLogFpOp: %u\t", un_step_number);
        while (i != allagents->end())
        {
            if ((*i)->GetType() == ROBOT)
            {
                CRobotAgentOptimised* tmp_robotagent  = (CRobotAgentOptimised*) (*i);
                printf("%llu  ",tmp_robotagent->GetNumberFloatingPtOperations());
            }
            i++;
        }
        printf("\n");
    }
#endif
}

/******************************************************************************/
/******************************************************************************/

vector<CBehavior*> CForagingExperiment::GetAgentBehavior(ESwarmBehavType swarmbehavType, CAgent*  previousAgent)
{
    vector<CBehavior*> vecBehaviors;

    //behav. pushed is decreasing order of priority to take control of the agent

    // Dispersion behavior
    // Parameters: Dispersion range (d)
    // Robots disperse with the distance between them decided by dispersion range d in CDisperseBehavior(d)
    // Note that for large value of d (e.g. 50), the robots collapse into each other!
    if(swarmbehavType == DISPERSION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(3.0);
        vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Aggregation behavior: formation of clusters of robots
    // Parameters: Dispersion range (d), Aggregation range (a)
    // The number of robots in each cluster seems to be proportional to d/a
    else if(swarmbehavType == AGGREGATION)
    {
        CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(3.0); //1
        vecBehaviors.push_back(pcDisperseBehavior2);
        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(10);
        vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Homing behavior 1: All other robots follow a single leader bot
    // The robots form a single cluster around the leader-bot. The bahavior is quite similar to the aggregation behav.
    // The dispersion range d > 0 else robots collapse into leader.
    // Increasing d increases the area occupied by the cluster
    else if(swarmbehavType == HOMING1)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(3.0);
        vecBehaviors.push_back(pcDisperseBehavior);
        CHomingBehavior* pcHomingBehavior = new CHomingBehavior(100.0, previousAgent);
        vecBehaviors.push_back(pcHomingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Flocking behavior:
    // Parameters: Dispersion range (d), Flocking range (f)
    // Dispersion range d is proportional to the size of individual flock of robots
    // Flocking range f influences how fast a flock is formed. Also f > d to initiate flocking
    else if(swarmbehavType == FLOCKING)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(1.0); //5
        vecBehaviors.push_back(pcDisperseBehavior);
        CFlockBehavior* pcFlockBehavior = new CFlockBehavior(3.0); //10
        vecBehaviors.push_back(pcFlockBehavior);
        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(5.0);
        vecBehaviors.push_back(pcAggregateBehavior);
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }
    else if(swarmbehavType == STRAIGHTLINE)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }
    else if(swarmbehavType == RANDOMWK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }
    else if(swarmbehavType == CIRCLE)
    {
        CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
        vecBehaviors.push_back(pcCircleBehavior);
    }
    else if(swarmbehavType == STOP)
    {
        CStopBehavior* pcStopBehavior = new CStopBehavior();
        vecBehaviors.push_back(pcStopBehavior);
    }
    else
    {
        printf("\n No behavior selected");
        exit(-1);
    }

    return vecBehaviors;
}

/******************************************************************************/
/******************************************************************************/
