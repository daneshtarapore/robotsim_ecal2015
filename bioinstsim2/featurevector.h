#ifndef FEATUREVECTOR_H_
#define FEATUREVECTOR_H_

#include "common.h"
#include "agent.h"
#include "simulator.h"

#include <string>

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
    unsigned int*  m_puLastOccuranceEvent;
    //float*       m_pfThresholds;

    float        m_fLowPassFilterParameter;
    float        m_fThresholdOnNumNbrs;
    float        m_fProcessedNumNeighbours;
    unsigned int m_unEventSelectionTimeWindow;
};

/******************************************************************************/
/******************************************************************************/


#endif
