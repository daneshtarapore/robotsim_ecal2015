#ifndef FORAGINGEXPERIMENT_H_
#define FORAGINGEXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

#include "robotagent_optimised.h"
#include "foragingtokenagent.h"
#include "nestsiteagent.h"
#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

#define OPTIMISED

/******************************************************************************/
/******************************************************************************/

class CForagingExperiment : public CExperiment
{
public:
    CForagingExperiment(CArguments* pc_experiment_arguments,
                    CArguments* pc_arena_arguments,
                    CArguments* pc_agent_arguments,
                    CArguments* pc_model_arguments);
    ~CForagingExperiment();
    
    virtual CAgent*     CreateAgent();
    virtual void SimulationStep(unsigned int un_step_number);
   
protected:
    virtual void PrintStatsForAgent(CAgent* pc_agent);
    virtual void PrintVelocityDifference(CAgent* pc_agent, double f_range);
    vector<CBehavior*> GetAgentBehavior(ESwarmBehavType swarmbehavType, CAgent*  previousAgent);

    virtual void SpreadBehavior(unsigned int step_number, ESwarmBehavType e_behavior, unsigned int firstswitchat);

    ESwarmBehavType m_eswarmbehavType, m_eerrorbehavType;
    //EErrorBehavType m_eerrorbehavType;

    unsigned int    m_unNumForagingTokens, m_unNumRobots;
    unsigned int    m_unMisbehaveStep;
    unsigned int    m_unNormalAgentToTrack;
    unsigned int    m_unAbnormalAgentToTrack;
    unsigned int    m_unNumAbnormalAgents;
    int             m_iSwitchNormalBehavior;
    unsigned int    m_unDurationofSwitch;

    unsigned int    m_unChaseAbnormalAgents;
    double          m_fSpreadProbability;     
    unsigned int    m_unGradualBehaviorSpreadEnabled;
    unsigned int    m_unFirstSwitchAt;

    CRobotAgentOptimised*    m_pcMisbehaveAgent[20];
    CRobotAgentOptimised*    m_pcNormalAgentToTrack;

    CAgent**        m_ppcListRobotsCreated;
    CAgent**        m_ppcListForagingTokensCreated;
    CAgent*         pcNestSiteAgent;

    CAgent*         pcHomeToAgent;


};

/******************************************************************************/
/******************************************************************************/

#endif
