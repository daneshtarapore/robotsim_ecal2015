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

    m_pfFeatureValues      = new float[m_unLength];
    m_puLastOccuranceEvent = new unsigned int[m_unLength];

    //m_pfThresholds    = new float[m_unLength];

    m_fLowPassFilterParameter    = 0.0001;
    m_fThresholdOnNumNbrs        = 5.0 ;
    m_fProcessedNumNeighbours    = 0.0;

    m_unEventSelectionTimeWindow = 1500U;
}

/******************************************************************************/
/******************************************************************************/

CFeatureVector::~CFeatureVector()
{
    delete m_pfFeatureValues;
    //delete m_pfThresholds;
    delete m_puLastOccuranceEvent;
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
        // m_unValue += m_pfFeatureValues[i] > m_pfThresholds[i] ? (1 << i) : 0;
        m_unValue += (unsigned int)m_pfFeatureValues[i] * (1 << i);
    }
}

/******************************************************************************/
/******************************************************************************/

//void CFeatureVector::ComputeFeatureValues()
//{
//    // Number of neighbors:
//    m_pfFeatureValues[0] = m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT);

//    // Distance to surrounding agents:
//    m_pfFeatureValues[1] = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);

//    // Velocity magnitude and direction wrt. surrounding agents:
//    TVector2d tTemp    = m_pcAgent->GetAverageVelocityOfSurroundingAgents(FEATURE_RANGE, ROBOT);
    
//    float tmp_agentvelocity = Vec2dLength((*m_pcAgent->GetVelocity()));
//    m_pfFeatureValues[2] = tmp_agentvelocity - Vec2dLength((tTemp));

//    if (Vec2dLength((*m_pcAgent->GetVelocity())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
//        m_pfFeatureValues[3] = Vec2dAngle((*m_pcAgent->GetVelocity()), tTemp);
//    else
//        m_pfFeatureValues[3] = 0.0;



//    // Acceleration magnitude and direction wrt. surrounding agents:
//    tTemp                = m_pcAgent->GetAverageAccelerationOfSurroundingAgents(FEATURE_RANGE, ROBOT);

//    float tmp_agentacceleration = Vec2dLength((*m_pcAgent->GetAcceleration()));
//    m_pfFeatureValues[4] = tmp_agentacceleration - Vec2dLength((tTemp));
//    if (Vec2dLength((*m_pcAgent->GetAcceleration())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
//        m_pfFeatureValues[5] = Vec2dAngle((*m_pcAgent->GetAcceleration()), tTemp);
//    else
//        m_pfFeatureValues[5] = 0.0;



//    // Change in velocity direction of individual agent:
//    m_pfFeatureValues[6] = m_pcAgent->GetChangeInOrientation();
//}

/******************************************************************************/
/******************************************************************************/


void CFeatureVector::ComputeFeatureValues()
{
    double dist_nbrsagents        = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);
    double angle_velocity         = m_pcAgent->GetAngularVelocity();
    double angle_acceleration     = m_pcAgent->GetAngularAcceleration();
    //double linear_acceleration    = Vec2dLength((*m_pcAgent->GetAcceleration()));

    /*if(m_pcAgent->GetIdentification() == 25)
        printf("\n Ang. vel. %f, ang. acc. %f", angle_velocity, angle_acceleration);*/

    m_fProcessedNumNeighbours = m_fLowPassFilterParameter * (float)m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT) + (1.0 - m_fLowPassFilterParameter) * m_fProcessedNumNeighbours;

    m_pfFeatureValues[0] = m_fProcessedNumNeighbours > m_fThresholdOnNumNbrs ? 1.0 : 0.0;


    if(dist_nbrsagents <= 3 && angle_acceleration != 0)
    {
        m_puLastOccuranceEvent[1] = CSimulator::GetInstance()->GetSimulationStepNumber();
    }

    if(dist_nbrsagents >  3 && dist_nbrsagents < 6 && angle_acceleration != 0)
    {
        m_puLastOccuranceEvent[2] = CSimulator::GetInstance()->GetSimulationStepNumber();
    }

    if(dist_nbrsagents == 6 && angle_velocity != 0)
    {
        m_puLastOccuranceEvent[3] = CSimulator::GetInstance()->GetSimulationStepNumber();
    }

    unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    for(unsigned int featureindex = 1; featureindex <=3; featureindex++)
    {
        m_pfFeatureValues[featureindex] = ((CurrentStepNumber - m_puLastOccuranceEvent[featureindex]) <= m_unEventSelectionTimeWindow) ? 1.0 : 0.0;
    }

/*    if(linear_acceleration != 0.0)
    {
        m_pfFeatureValues[4] = 1.0;
    }
    else
    {
        m_pfFeatureValues[4] = 0.0;
    }*/

}

/******************************************************************************/
/******************************************************************************/

std::string CFeatureVector::ToString()
{
    char pchTemp[2048];

    sprintf(pchTemp, "Values - "
                     "nbrs: %f - "
                     "dist0to3_angacc: %1.1f - "
                     "dist3to6_angacc: %1.1f - "
                     "dist6_angvelocity: %1.1f - fv: %u",

            m_pfFeatureValues[0],
            m_pfFeatureValues[1],
            m_pfFeatureValues[2],
            m_pfFeatureValues[3],
            m_unValue);

//    sprintf(pchTemp, "Values - "
//                     "nbrs: %3f [%3f]- "
//                     "dist: %5.3e [%5.3f] - "
//                     "vmr.: %5.3e [%5.3f] - "
//                     "vdr.: %5.3e [%5.3f] - "
//                     "amr.: %5.3e [%5.3f] - "
//                     "adr.: %5.3e [%5.3f] -"
//                     "change_ori.: %5.3e [%5.3f] - fv: %d",

//            m_pfFeatureValues[0], m_pfThresholds[0],
//            m_pfFeatureValues[1], m_pfThresholds[1],
//            m_pfFeatureValues[2], m_pfThresholds[2],
//            m_pfFeatureValues[3], m_pfThresholds[3],
//            m_pfFeatureValues[4], m_pfThresholds[4],
//            m_pfFeatureValues[5], m_pfThresholds[5],
//            m_pfFeatureValues[6], m_pfThresholds[6],
//            m_unValue);


    return string(pchTemp);
}

/******************************************************************************/
/******************************************************************************/

