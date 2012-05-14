#ifndef FEATUREVECTOR_H_
#define FEATUREVECTOR_H_

#include "common.h"
#include "agent.h"

/******************************************************************************/
/******************************************************************************/

class CFeatureVector
{
public:
    CFeatureVector(CAgent* pc_agent);

    static unsigned int NUMBER_OF_FEATURES;

    virtual unsigned int GetValue();
    virtual unsigned int GetLength();

    virtual unsigned int SimulationStep();
    
protected:
    CAgent*      m_pcAgent;    
    unsigned int m_unValue;
    unsigned int m_unLength;    

    float        m_pfFeatureValues[];
    float        m_pfThresholds[];
};

/******************************************************************************/
/******************************************************************************/


#endif
