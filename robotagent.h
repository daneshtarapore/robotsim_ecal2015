#ifndef ROBOTAGENT_H_
#define ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/

class CRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgent;

/******************************************************************************/
/******************************************************************************/

#include "common.h"
#include "agent.h"
#include "behavior.h"
#include "crminrobotagent.h"

/******************************************************************************/
/******************************************************************************/

class CRobotAgent : public CAgent
{
public: 
    CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_agent_arguments, CArguments* pc_crm_arguments, TBehaviorVector vec_behaviors);
    virtual ~CRobotAgent();
    
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStepUpdatePosition();    
    virtual void SetBehaviors(TBehaviorVector vec_behaviors);
    
    virtual EAgentType   GetType();   

    // Gets the number of feature vectors of different types
    // into m_punFeaturesSensed. Returns the range used to sense the feature vectors
    virtual unsigned int* GetFeaturesSensed() const;

    virtual CRobotAgent* GetRandomRobotWithWeights(double f_range);
    virtual CRobotAgent* GetRandomRobotWithWeights(unsigned int u_nearestnbrs);

    virtual CRMinRobotAgent* GetCRMinRobotAgent();
    virtual void   SetWeight(double f_weight);
    virtual double GetWeight() const; 
    virtual const CFeatureVector* GetFeatureVector() const;
    virtual void Sense(unsigned int u_nearestnbrs);
    virtual double GetFVSenseRange() const;
    virtual unsigned int GetColor();
    virtual unsigned int GetSelectedNumNearestNbrs();

    virtual void SetMostWantedList(unsigned unFeatureVector, unsigned int state);
    
    virtual void CheckNeighborsReponseToMyFV(unsigned int* pun_number_of_toleraters, unsigned int* pun_number_of_attackers, unsigned int* pun_number_of_unconverged);

    virtual unsigned int Attack(CFeatureVector* pc_feature_vector);

protected:
    virtual double CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range);

    double              m_fFVSenseRange;
    CFeatureVector*     m_pcFeatureVector;
    TBehaviorVector     m_vecBehaviors;
    CRMinRobotAgent*    crminAgent;
    double              m_fWeight;
    unsigned int*       m_punFeaturesSensed;
    double              m_fBitflipProbabililty;
    unsigned int*       m_pbMostWantedList;
    double              m_fResponseRange;
    unsigned int        m_uSelectedNumNearestNbrs;

};

/******************************************************************************/
/******************************************************************************/

#endif // ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/
