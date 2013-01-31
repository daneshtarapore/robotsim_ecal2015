#ifndef ROBOTAGENTOPTIMISED_H_
#define ROBOTAGENTOPTIMISED_H_

/******************************************************************************/
/******************************************************************************/

class CRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

struct structFVsSensed
{
    unsigned int uFV;
    double fRobots;
    unsigned int uMostWantedState;

    structFVsSensed(unsigned int fv, double density)
    {
        uFV     = fv;
        fRobots = density;
    }
};

/******************************************************************************/
/******************************************************************************/

#include <list>
#include "common.h"
#include "agent.h"
#include "behavior.h"
#include "crminrobotagent_optimised.h"

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CRobotAgentOptimised : public CAgent
{
public: 
    CRobotAgentOptimised(const char* pch_name, unsigned int un_identification, CArguments* pc_agent_arguments, CArguments* pc_model_arguments, TBehaviorVector vec_behaviors);
    virtual ~CRobotAgentOptimised();
    
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStepUpdatePosition();    
    virtual void SetBehaviors(TBehaviorVector vec_behaviors);
    virtual TBehaviorVector GetBehaviors();
    
    virtual inline EAgentType   GetType() {return ROBOT;}

    virtual CRobotAgentOptimised* GetRandomRobotWithWeights(double f_range);
    virtual CRobotAgentOptimised* GetRandomRobotWithWeights(unsigned int u_nearestnbrs);

    virtual inline CRMinRobotAgentOptimised* GetCRMinRobotAgent() {return crminAgent;}

    virtual void  SetWeight(double f_weight);
    virtual double GetWeight() const;

    virtual inline const CFeatureVector* GetFeatureVector() const {return m_pcFeatureVector;}
    virtual void Sense(unsigned int u_nearestnbrs);
    virtual void UpdateFeatureVectorDistribution(unsigned int fv, double increment);
    virtual inline list<structFVsSensed>* GetFeatureVectorsSensed() {return &listFVsSensed;}
    virtual void PrintFeatureVectorDistribution(unsigned int id);

    virtual inline double GetFVSenseRange() const;

    virtual unsigned int GetColor();

    virtual inline unsigned int GetSelectedNumNearestNbrs() {return m_uSelectedNumNearestNbrs;}

    virtual void SetMostWantedList(list<structFVsSensed>::iterator* it, unsigned int state);
    virtual unsigned int GetMostWantedState(unsigned int fv);

    virtual void CheckNeighborsResponseToMyFV(unsigned int* pun_number_of_toleraters, unsigned int* pun_number_of_attackers, unsigned int* pun_number_of_unconverged, bool b_logs);

    virtual void PrintDecidingAgentDetails(CFeatureVector* m_pcFV,
                                           CRMinRobotAgentOptimised* model_crminagent);

    virtual inline unsigned int Attack(CFeatureVector* pc_feature_vector)
    {return GetMostWantedState(pc_feature_vector->GetValue());}

protected:
    virtual double CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range);

    double                      m_fFVSenseRange;
    CFeatureVector*             m_pcFeatureVector;
    TBehaviorVector             m_vecBehaviors;
    CRMinRobotAgentOptimised*   crminAgent;

    double              m_fWeight;
    double              m_fBitflipProbabililty;

//    float*              m_pfFeaturesSensed; //2^n
//    unsigned int*       m_pbMostWantedList; //2^n
    list<structFVsSensed> listFVsSensed;
//    list<unsigned> vecMostWantedList; // lets just use crminAgent->vecAPCs

    virtual inline void IncIt(list<structFVsSensed>::iterator *it_fvsensed, list<structFVsSensed> *list)
    { (*it_fvsensed) == list->end() ? (*it_fvsensed):++(*it_fvsensed); }

    double              m_fResponseRange;
    unsigned int        m_uSelectedNumNearestNbrs;

};

/******************************************************************************/
/******************************************************************************/

#endif // ROBOTAGENTOPTIMISED_H_

/******************************************************************************/
/******************************************************************************/
