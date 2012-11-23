#ifndef TESTEXPERIMENT_H_
#define TESTEXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"
#include "robotagent.h"

/******************************************************************************/
/******************************************************************************/

enum ESwarmBehavType
{
    AGGREGATION,
    DISPERSION,
    FLOCKING,
    HOMING1,
    HOMING2,

    STRAIGHTLINE,
    RANDOMWK,
    CIRCLE,
    STOP,

    NOERR
};

/*enum EErrorBehavType
{
    STRAIGHTLINE,
    RANDOMWK,
    CIRCLE,
    STOP,

    STRAIGHTLINE,
    RANDOMWK,
    CIRCLE,
    STOP,

    NOERR
};*/

class CTestExperiment : public CExperiment
{
public:
    CTestExperiment(CArguments* pc_experiment_arguments,
                    CArguments* pc_arena_arguments,
                    CArguments* pc_agent_arguments,
                    CArguments* pc_model_arguments);
    ~CTestExperiment();
    
    virtual CAgent*     CreateAgent();
    virtual void SimulationStep(unsigned int un_step_number);
   
protected:
    virtual void PrintStatsForAgent(CAgent* pc_agent);
    virtual void PrintVelocityDifference(CAgent* pc_agent, double f_range);
    vector<CBehavior*> GetAgentBehavior(ESwarmBehavType swarmbehavType, CAgent*  previousAgent);

    ESwarmBehavType m_eswarmbehavType, m_eerrorbehavType;
    //EErrorBehavType m_eerrorbehavType;

    unsigned int    m_unMisbehaveStep;
    unsigned int    m_unNormalAgentToTrack;
    unsigned int    m_unAbnormalAgentToTrack;
    unsigned int    m_unNumAbnormalAgents;
    int             m_iSwitchNormalBehavior;

    CRobotAgent*    m_pcMisbehaveAgent[20];
    CRobotAgent*    m_pcNormalAgentToTrack;

    CAgent**        m_ppcListAgentsCreated;

    CAgent*         pcHomeToAgent;


};

/******************************************************************************/
/******************************************************************************/

#endif
