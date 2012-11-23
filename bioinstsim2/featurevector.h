#ifndef FEATUREVECTOR_H_
#define FEATUREVECTOR_H_

/******************************************************************************/
/******************************************************************************/

class CTestExperiment;

/******************************************************************************/
/******************************************************************************/

#include "common.h"
#include "agent.h"
#include "simulator.h"

//#include "testexperiment.h"

#include <string>

/******************************************************************************/
/******************************************************************************/

#define ALTERNATESIXBITFV

/******************************************************************************/
/******************************************************************************/
// the agents with wc in fvs are divided into two (wc on 1 bit), or replicated in both the fvs

//#define WILDCARDINFV 1.0
//#define WILDCARDINFV 0.5

/******************************************************************************/
/******************************************************************************/

class CFeatureVector
{
public:
    CFeatureVector(CAgent* pc_agent);
    virtual ~CFeatureVector();

    static unsigned int NUMBER_OF_FEATURES;
    static unsigned int NUMBER_OF_FEATURE_VECTORS;
    static double       FEATURE_RANGE;

    virtual unsigned int GetValue() const;
    virtual unsigned int GetLength() const;

    void PrintFeatureDetails();

    virtual unsigned int SimulationStep();

    virtual std::string ToString();

#ifdef WILDCARDINFV
    int m_iWildCardBit; // set to -1 if no wildcard bit present in FV
#endif
    
protected:
    virtual void ComputeFeatureValues();

    CAgent*      m_pcAgent;    
    unsigned int m_unValue;
    unsigned int m_unLength;    

    float*         m_pfFeatureValues;
    int*           m_piLastOccuranceEvent;
    int*           m_piLastOccuranceNegEvent;

    float        m_fLowPassFilterParameter;
    float        m_fThresholdOnNumNbrs;
    float        m_fProcessedNumNeighbours;
    int          m_iEventSelectionTimeWindow;

    double       m_fVelocityThreshold;


#ifdef WILDCARDINFV

    /*
    unsigned int m_unSenMotIntCurrQueueIndex;

    unsigned int m_unSumTimeSteps_SenMotIntNbrsInRange_at1;
    unsigned int m_unSumTimeSteps_SenMotIntNbrsInRange_at0;
    unsigned int m_unSumTimeSteps_SenMotIntNbrsInRange_atWC;

    unsigned int m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1;
    unsigned int m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0;
    unsigned int m_unSumTimeSteps_SenMotIntNbrsNotInRange_atWC;

    // for each time-step
    //1: if motors respond to sensory input; 0: if motors donot respond to sensory input; and WC (2): if no sensory input present
    unsigned int* m_punSenMotInt_NbrsInRange;

    //1: if motors respond to absence of sensory input; 0: if motors donot respond to absence of sensory input; and WC (2): if sensory input is present
    unsigned int* m_punSenMotInt_NbrsNotinRange;*/

#endif

#ifdef ALTERNATESIXBITFV

    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    unsigned int m_unNbrsCurrQueueIndex;

    unsigned int m_unSumTimeStepsNbrsRange0to3;
    unsigned int m_unSumTimeStepsNbrsRange3to6;

    unsigned int* m_punNbrsRange0to3AtTimeStep;
    unsigned int* m_punNbrsRange3to6AtTimeStep;



    // keeping track of distance travelled by bot in last 100 time-steps
    int              m_iDistTravelledTimeWindow;

    unsigned int     m_unCoordCurrQueueIndex;

    double           m_fSquaredDistTravelled;
    double           m_fSquaredDistThreshold;

    TVector2d*       m_pvecCoordAtTimeStep;


#endif
};

/******************************************************************************/
/******************************************************************************/


#endif
