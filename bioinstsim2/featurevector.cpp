#include "featurevector.h"

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned int CFeatureVector::NUMBER_OF_FEATURE_VECTORS = 0;
double       CFeatureVector::FEATURE_RANGE             = 5.0;

/******************************************************************************/
/******************************************************************************/

CFeatureVector::CFeatureVector(CAgent* pc_agent) : m_pcAgent(pc_agent)
{
    m_unValue  = 0;
    m_unLength = NUMBER_OF_FEATURES;

    NUMBER_OF_FEATURE_VECTORS = 1 << NUMBER_OF_FEATURES;

    m_pfFeatureValues = new float[m_unLength];
    m_pfThresholds    = new float[m_unLength];
}

/******************************************************************************/
/******************************************************************************/

CFeatureVector::~CFeatureVector()
{
    delete m_pfFeatureValues;
    delete m_pfThresholds;   
}

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::GetValue() const
{
    return m_unValue;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::GetLength() const
{
    return m_unLength;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::SimulationStep()
{
    ComputeFeatureValues();
    m_unValue = 0;
    
    for (unsigned int i = 0; i < m_unLength; i++)
    {
        m_unValue += m_pfFeatureValues[i] > m_pfThresholds[i] ? (1 << i) : 0;
    }
}

/******************************************************************************/
/******************************************************************************/

void CFeatureVector::ComputeFeatureValues()
{
    // Number of neighbors:
    m_pfFeatureValues[0] = m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT);

    // Distance to surrounding agents:
    m_pfFeatureValues[1] = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);

    // Velocity magnitude and direction wrt. surrounding agents:
    TVector2d tTemp    = m_pcAgent->GetAverageVelocityOfSurroundingAgents(FEATURE_RANGE, ROBOT);
    
    m_pfFeatureValues[2] = Vec2dLength((*m_pcAgent->GetVelocity())) - Vec2dLength((tTemp));

    if (Vec2dLength((*m_pcAgent->GetVelocity())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
        m_pfFeatureValues[3] = Vec2dAngle((*m_pcAgent->GetVelocity()), tTemp);
    else
        m_pfFeatureValues[3] = 0.0;

    // Acceleration magnitude and direction wrt. surrounding agents:
    tTemp                = m_pcAgent->GetAverageAccelerationOfSurroundingAgents(FEATURE_RANGE, ROBOT);
    m_pfFeatureValues[4] = Vec2dLength((*m_pcAgent->GetAcceleration())) - Vec2dLength((tTemp));
    if (Vec2dLength((*m_pcAgent->GetAcceleration())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
        m_pfFeatureValues[5] = Vec2dAngle((*m_pcAgent->GetAcceleration()), tTemp);
    else
        m_pfFeatureValues[5] = 0.0;
}

/******************************************************************************/
/******************************************************************************/

std::string CFeatureVector::ToString()
{
    char pchTemp[1024];

    sprintf(pchTemp, "Values - "  
                     "nbrs: %3f [%3f]- " 
                     "dist: %5.3f [%5.3f] - "  
                     "vm.: %5.3f [%5.3f] - " 
                     "vd.: %5.3f [%5.3f] - "  
                     "am.: %5.3f [%5.3f] - " 
                     "ad.: %5.3f [%5.3f] - fv: %d", 
            m_pfFeatureValues[0], m_pfThresholds[0],
            m_pfFeatureValues[1], m_pfThresholds[1], 
            m_pfFeatureValues[2], m_pfThresholds[2],
            m_pfFeatureValues[3], m_pfThresholds[3],
            m_pfFeatureValues[4], m_pfThresholds[4],
            m_pfFeatureValues[5], m_pfThresholds[5], 
            m_unValue);


    return string(pchTemp);
}

/******************************************************************************/
/******************************************************************************/

