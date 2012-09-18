#ifndef FEATUREVECTOR_H_
#define FEATUREVECTOR_H_

#include "common.h"
#include "agent.h"
#include "simulator.h"

#include <string>

/******************************************************************************/
/******************************************************************************/

#define ALTERNATESIXBITFV

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

    virtual unsigned int SimulationStep();

    virtual std::string ToString();
    
protected:
    virtual void ComputeFeatureValues();

    CAgent*      m_pcAgent;    
    unsigned int m_unValue;
    unsigned int m_unLength;    

    float*         m_pfFeatureValues;
    int*           m_piLastOccuranceEvent;

    float        m_fLowPassFilterParameter;
    float        m_fThresholdOnNumNbrs;
    float        m_fProcessedNumNeighbours;
    int          m_iEventSelectionTimeWindow;

    double       m_dVelocityThreshold;

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

    double           m_dSquaredDistTravelled;
    double           m_dSquaredDistThreshold;

    TVector2d*       m_pvecCoordAtTimeStep;


#endif
};

/******************************************************************************/
/******************************************************************************/


#endif
