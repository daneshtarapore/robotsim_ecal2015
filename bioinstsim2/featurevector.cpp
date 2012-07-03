#include "featurevector.h"
#include "assert.h"

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::NUMBER_OF_FEATURES        = 4;
unsigned int CFeatureVector::NUMBER_OF_FEATURE_VECTORS = 0;
double       CFeatureVector::FEATURE_RANGE             = 6.0;

/******************************************************************************/
/******************************************************************************/

CFeatureVector::CFeatureVector(CAgent* pc_agent) : m_pcAgent(pc_agent)
{
    m_unValue  = 0;
    m_unLength = NUMBER_OF_FEATURES;

    //assert(NUMBER_OF_FEATURES == 4);
    NUMBER_OF_FEATURE_VECTORS = 1 << NUMBER_OF_FEATURES;

    m_pfFeatureValues      = new float[m_unLength];
    m_piLastOccuranceEvent = new int[m_unLength];

    //m_pfThresholds    = new float[m_unLength];

    m_fLowPassFilterParameter    = 0.01;
    m_fThresholdOnNumNbrs        = 3.99 ;
    m_fProcessedNumNeighbours    = 0.0;

    m_iEventSelectionTimeWindow1 = 1500;
    m_iEventSelectionTimeWindow2 =  500;

    for(unsigned int i = 0; i < NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i] = 0;
    }

    printf(" m_iEventSelectionTimeWindow1 = %d, m_iEventSelectionTimeWindow2 = %d;",
           m_iEventSelectionTimeWindow1, m_iEventSelectionTimeWindow2);
}

/******************************************************************************/
/******************************************************************************/

CFeatureVector::~CFeatureVector()
{
    delete m_pfFeatureValues;
    //delete m_pfThresholds;
    delete m_piLastOccuranceEvent;
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
    double dist_nbrsagents, angle_acceleration, angle_velocity;
    double mag_relativeagentvelocity, dir_relativeagentvelocity,
    mag_relativeagentacceleration, dir_relativeagentacceleration;

    dist_nbrsagents    = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);
    angle_acceleration = m_pcAgent->GetAngularAcceleration();
    angle_velocity     = m_pcAgent->GetAngularVelocity();

    m_fProcessedNumNeighbours = m_fLowPassFilterParameter * (float)m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT) + (1.0 - m_fLowPassFilterParameter) * m_fProcessedNumNeighbours;

    // 1st feature (leftmost position in FV): Processed number of neighbors
    if(m_fProcessedNumNeighbours >= m_fThresholdOnNumNbrs)
    {
        m_pfFeatureValues[0] = 1.0;
    }
    else
    {
        m_pfFeatureValues[0] = 0.0;
    }


    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();

    if((CFeatureVector::NUMBER_OF_FEATURES == 4)||
       (CFeatureVector::NUMBER_OF_FEATURES == 6)||
       (CFeatureVector::NUMBER_OF_FEATURES == 7))
    {
        if(dist_nbrsagents <= 3.0 && angle_acceleration != 0.0)
        {
            m_piLastOccuranceEvent[1] = CurrentStepNumber;
        }

        if(dist_nbrsagents >  3.0 && dist_nbrsagents < 6 && angle_acceleration != 0.0)
        {
            m_piLastOccuranceEvent[2] = CurrentStepNumber;
        }

        if(dist_nbrsagents == 6.0 && angle_velocity != 0.0)
        {
            m_piLastOccuranceEvent[3] = CurrentStepNumber;
        }

        for(unsigned int featureindex = 1; featureindex <=3; featureindex++)
        {
            // Occurance of the following event, atleast once in time window m_iEventSelectionTimeWindow1 (1500)
            // 2nd feature: distance to nbrs [0 3] && change in angular acceleration
            // 3rd feature: distance to nbrs [3 6) && change in angular acceleration
            // 4th feature: No neighbors detected  && change in angular acceleration
            if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow1)
            {
                m_pfFeatureValues[featureindex] = 1.0;
            }
            else
            {
                m_pfFeatureValues[featureindex] =  0.0;
            }
        }

        if(CFeatureVector::NUMBER_OF_FEATURES == 6)
        {
            for(unsigned int featureindex = 4; featureindex <=5; featureindex++)
            {
                // Occurance of the following event, atleast once in time window m_iEventSelectionTimeWindow2 (500)
                // 5th feature: distance to nbrs [0 3] && change in angular acceleration
                // 6th feature: distance to nbrs [3 6) && change in angular acceleration

                if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex-3]) <= m_iEventSelectionTimeWindow2)
                {
                    m_pfFeatureValues[featureindex] = 1.0;
                }
                else
                {
                    m_pfFeatureValues[featureindex] =  0.0;
                }
            }
        }
        else if(CFeatureVector::NUMBER_OF_FEATURES == 7)
        {
            for(unsigned int featureindex = 4; featureindex <=6; featureindex++)
            {
                // Occurance of the following event, atleast once in time window m_iEventSelectionTimeWindow2 (500)
                // 5th feature: distance to nbrs [0 3] && change in angular acceleration
                // 6th feature: distance to nbrs [3 6) && change in angular acceleration
                // 7th feature: No neighbors detected  && change in angular acceleration
                if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex-3]) <= m_iEventSelectionTimeWindow2)
                {
                    m_pfFeatureValues[featureindex] = 1.0;
                }
                else
                {
                    m_pfFeatureValues[featureindex] =  0.0;
                }
            }
        }
    }
    else if(CFeatureVector::NUMBER_OF_FEATURES == 5)
    {
        if(dist_nbrsagents <= 2.0 && angle_acceleration != 0.0)
        {
            m_piLastOccuranceEvent[0] = CurrentStepNumber;
        }

        else if(dist_nbrsagents >  2.0 && dist_nbrsagents <= 4 && angle_acceleration != 0.0)
        {
            m_piLastOccuranceEvent[1] = CurrentStepNumber;
        }

        else if(dist_nbrsagents >  4.0 && dist_nbrsagents < 6  && angle_acceleration != 0.0)
        {
            m_piLastOccuranceEvent[2] = CurrentStepNumber;
        }

        if(dist_nbrsagents == 6.0 && angle_velocity != 0.0)
        {
            m_piLastOccuranceEvent[3] = CurrentStepNumber;
        }

        for(unsigned int featureindex = 0; featureindex <=3; featureindex++)
        {
            // Occurance of the following event, atleast once in time window m_iEventSelectionTimeWindow1 (1500)
            // 2nd feature: distance to nbrs [0 2] && change in angular acceleration
            // 3rd feature: distance to nbrs [2 4] && change in angular acceleration
            // 4th feature: distance to nbrs [4 6) && change in angular acceleration
            // 4th feature: No neighbors detected  && change in angular acceleration
            if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow1)
            {
                m_pfFeatureValues[featureindex+1] = 1.0;
            }
            else
            {
                m_pfFeatureValues[featureindex+1] =  0.0;
            }
        }
    }


    // Velocity magnitude and direction wrt. surrounding agents:
    TVector2d tTemp    = m_pcAgent->GetAverageVelocityOfSurroundingAgents(FEATURE_RANGE, ROBOT);

    float tmp_agentvelocity = Vec2dLength((*m_pcAgent->GetVelocity()));
    mag_relativeagentvelocity = tmp_agentvelocity - Vec2dLength((tTemp));

    if (Vec2dLength((*m_pcAgent->GetVelocity())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
        dir_relativeagentvelocity = Vec2dAngle((*m_pcAgent->GetVelocity()), tTemp);
    else
        dir_relativeagentvelocity = 0.0;


    // Acceleration magnitude and direction wrt. surrounding agents:
    tTemp                = m_pcAgent->GetAverageAccelerationOfSurroundingAgents(FEATURE_RANGE, ROBOT);

    float tmp_agentacceleration = Vec2dLength((*m_pcAgent->GetAcceleration()));
    mag_relativeagentacceleration = tmp_agentacceleration - Vec2dLength((tTemp));
    if (Vec2dLength((*m_pcAgent->GetAcceleration())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
        dir_relativeagentacceleration = Vec2dAngle((*m_pcAgent->GetAcceleration()), tTemp);
    else
        dir_relativeagentacceleration = 0.0;


    if (m_pcAgent->GetIdentification() == 1 && CurrentStepNumber > CRMSTARTTIME)
    {
        printf("\nFV for normal agent %d: #NBRS %d, lpf(#NBRS) %f, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f\n", m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),m_fProcessedNumNeighbours,dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration);
    }

    if (m_pcAgent->GetIdentification() == 15 && CurrentStepNumber > CRMSTARTTIME)
    {
        printf("\nFV for abnormal agent %d: #NBRS %d, lpf(#NBRS) %f, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f\n", m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),m_fProcessedNumNeighbours,dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration);
    }

}

/******************************************************************************/
/******************************************************************************/

std::string CFeatureVector::ToString()
{
    char pchTemp[4096];

    if(CFeatureVector::NUMBER_OF_FEATURES == 4)
    {
        sprintf(pchTemp, "Values - "
                         "nbrs: %f - "
                         "TW1500_dist0to3_angacc: %1.1f - "
                         "TW1500_dist3to6_angacc: %1.1f - "
                         "TW1500_dist6_angvelocity: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_unValue);
    }
    else if(CFeatureVector::NUMBER_OF_FEATURES == 6)
    {
        sprintf(pchTemp, "Values - "
                         "nbrs: %f - "
                         "TW1500_dist0to3_angacc: %1.1f - "
                         "TW1500_dist3to6_angacc: %1.1f - "
                         "TW1500_dist6_angvelocity: %1.1f - "
                         "TW500_dist0to3_angacc: %1.1f - "
                         "TW500_dist3to6_angacc: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],
                m_unValue);
    }
    else if(CFeatureVector::NUMBER_OF_FEATURES == 7)
    {
        sprintf(pchTemp, "Values - "
                         "nbrs: %f - "
                         "TW1500_dist0to3_angacc: %1.1f - "
                         "TW1500_dist3to6_angacc: %1.1f - "
                         "TW1500_dist6_angvelocity: %1.1f - "
                         "TW500_dist0to3_angacc: %1.1f - "
                         "TW500_dist3to6_angacc: %1.1f - "
                         "TW500_dist6_angvelocity: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],
                m_pfFeatureValues[6],
                m_unValue);
    }


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

