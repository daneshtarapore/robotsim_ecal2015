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
    static unsigned int MAX_NUMBER_OF_FEATURES;
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
    float*         m_pfAllFeatureValues;
    int*           m_piLastOccuranceEvent;
    int*           m_piLastOccuranceNegEvent;


    std::vector <unsigned> m_piFeaturesSelected; // the features that are to be introduced into the feature vector
    //m_piFeaturesSelected     = {1, 2, 3, 4, 5, 6}; // the original 6 bit FV used in the BB paper. Features start from 1.

    int          m_iEventSelectionTimeWindow;

    double       m_fVelocityThreshold;
    double       m_fAccelerationThreshold;

    double       m_tAngularVelocityThreshold;
    double       m_tAngularAccelerationThreshold;

    double       m_fRelativeVelocityMagThreshold;
    double       m_fRelativeVelocityDirThreshold;




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


};

/******************************************************************************/
/******************************************************************************/


#endif
