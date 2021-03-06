#include "featurevector.h"
#include "assert.h"
#include <iostream>

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned int CFeatureVector::MAX_NUMBER_OF_FEATURES    = 15;
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

    m_pfFeatureValues         = new float[m_unLength];
    m_piLastOccuranceEvent    = new int[m_unLength];
    m_piLastOccuranceNegEvent = new int[m_unLength];


    m_pfAllFeatureValues     = new float[MAX_NUMBER_OF_FEATURES];

    //m_piFeaturesSelected     = {1, 2, 3, 4, 5, 6}; // the original 6 bit FV used in the BB paper.Features start at index 1.
    //m_piFeaturesSelected     = {7, 8, 9, 10, 14, 15}; //already tested
    //m_piFeaturesSelected     = {2, 7, 9, 4, 11, 12};
    //m_piFeaturesSelected     = {1, 8, 3, 10, 6, 13};

    m_piFeaturesSelected     = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    assert(m_piFeaturesSelected.size() >= m_unLength);



    m_iEventSelectionTimeWindow = MODELSTARTTIME; //1500;

    for(unsigned int i = 0; i < NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }

    m_fVelocityThreshold            = 0.05  * (m_pcAgent->GetMaximumSpeed());
    m_fAccelerationThreshold        = 0.05  * (m_pcAgent->GetMaximumSpeed());

    m_tAngularVelocityThreshold     = 0.032  * (m_pcAgent->GetMaximumAngularVelocity());
    m_tAngularAccelerationThreshold = 0.032  * (m_pcAgent->GetMaximumAngularVelocity());

    m_fRelativeVelocityMagThreshold = 0.05 * (m_pcAgent->GetMaximumSpeed());
    m_fRelativeVelocityDirThreshold = 0.05  * (m_pcAgent->GetMaximumAngularVelocity());



#if defined(WILDCARDINFV)

    assert(CFeatureVector::NUMBER_OF_FEATURES == 6);
    m_iWildCardBit = -1; // set to -1 if no wildcard bit present in FV

#endif



    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to3 = 0;
    m_unSumTimeStepsNbrsRange3to6 = 0;

    m_punNbrsRange0to3AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];
    m_punNbrsRange3to6AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];


    // keeping track of distance travelled by bot in last 100 time-steps
    m_iDistTravelledTimeWindow = 100;
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;
    m_fSquaredDistThreshold = (0.05 * (m_pcAgent->GetMaximumSpeed()*(double)m_iDistTravelledTimeWindow)) *
            (0.05 * (m_pcAgent->GetMaximumSpeed()*(double)m_iDistTravelledTimeWindow));

    m_pvecCoordAtTimeStep = new TVector2d[m_iDistTravelledTimeWindow];
}

/******************************************************************************/
/******************************************************************************/

CFeatureVector::~CFeatureVector()
{
    delete m_pfFeatureValues;

    delete m_piLastOccuranceEvent;
    delete m_piLastOccuranceNegEvent;

    delete m_punNbrsRange0to3AtTimeStep;
    delete m_punNbrsRange3to6AtTimeStep;

    delete m_pvecCoordAtTimeStep;
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
        m_unValue += (unsigned int)m_pfFeatureValues[i] * (1 << i);
}

/******************************************************************************/
/******************************************************************************/

/*void CFeatureVector::ComputeFeatureValues()
{
    double dist_nbrsagents, angle_acceleration, angle_velocity, mag_velocity, mag_acceleration;
    double mag_relvelocity, dir_relvelocity;
    unsigned int unCloseRangeNbrCount, unFarRangeNbrCount;

    dist_nbrsagents    = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);
    angle_acceleration = m_pcAgent->GetAngularAcceleration();
    angle_velocity     = m_pcAgent->GetAngularVelocity();
    mag_velocity       = Vec2dLength((*m_pcAgent->GetVelocity()));
    mag_acceleration   = Vec2dLength((*m_pcAgent->GetAcceleration()));

    m_pcAgent->GetRelativeVelocity(&mag_relvelocity, &dir_relvelocity, FEATURE_RANGE);

    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();


    if(NUMBER_OF_FEATURES == 3U)
    {
        // Feature (from LS to MS bits in FV)
        // Sensors
        //1st: set if bot has atleast one neighbor in range 0-3 in the majority of of past X time-steps
        if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
        {
            // decision based on the last X time-steps
            if(m_unSumTimeStepsNbrsRange0to3 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
                m_pfFeatureValues[0] = 1.0;
            else
                m_pfFeatureValues[0] = 0.0;

            // removing the fist entry of the moving time window  from the sum
            m_unSumTimeStepsNbrsRange0to3 -=  m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex];
        }

        // adding new values into the queue
        unCloseRangeNbrCount = m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT);
        if (unCloseRangeNbrCount > 0)
        {
            m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
            m_unSumTimeStepsNbrsRange0to3++;
        }
        else
            m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

        m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;

        // Sensors-motor interactions
        // Set if the occurance of the following event, atleast once in time window X
        // 2nd: distance to nbrs 0-6 && change in angular acceleration

        if(dist_nbrsagents < 6.0 &&
                (angle_acceleration > m_tAngularAccelerationThreshold ||
                 angle_acceleration < -m_tAngularAccelerationThreshold))
            m_piLastOccuranceEvent[0] = CurrentStepNumber;

        if ((CurrentStepNumber - m_piLastOccuranceEvent[0]) <= m_iEventSelectionTimeWindow)
            m_pfFeatureValues[1] = 1.0;
        else
            m_pfFeatureValues[1] = 0.0;

        // Motors
        //3rd: distance travelled by bot in past Y time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
        TVector2d vecAgentPos = *(m_pcAgent->GetPosition());

        if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
        {
            // distance travelled in last 100 time-steps
            m_fSquaredDistTravelled =
                    GetSquaredDistanceBetweenPositions(&vecAgentPos,
                                                       &(m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex]));

            // decision based on distance travelled in the last 100 time-steps
            if(m_fSquaredDistTravelled >= m_fSquaredDistThreshold)
                m_pfFeatureValues[2] = 1.0;
            else
                m_pfFeatureValues[2] = 0.0;
        }

        // adding new coordinate values into the queue
        m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
        m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;

#ifdef DEBUGFEATUREVECTORFLAG
        if(FDMODELTYPE != LINEQ) // lineq - low expected run time; can come back and log more details if needed
            PrintFeatureDetails();
#endif
        return;
    }

    // 6 bit or more feature-vectors
    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-3 in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 3-6 in the majority of of past X time-steps
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last X time-steps
        if(m_unSumTimeStepsNbrsRange0to3 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfFeatureValues[0] = 1.0;
        else
            m_pfFeatureValues[0] = 0.0;

        if(m_unSumTimeStepsNbrsRange3to6 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfFeatureValues[1] = 1.0;
        else
            m_pfFeatureValues[1] = 0.0;

        // removing the fist entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to3 -=  m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange3to6 -=  m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    unCloseRangeNbrCount = m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT);
    if (unCloseRangeNbrCount > 0)
    {
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to3++;
    }
    else
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    unFarRangeNbrCount = (m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT) - unCloseRangeNbrCount);
    if (unFarRangeNbrCount > 0)
    {
        m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange3to6++;
    }
    else
        m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex] = 0;


    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;


#ifdef WILDCARDINFV

    m_iWildCardBit = -1; // set to -1 if no wildcard bit present in FV

    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    if(dist_nbrsagents < 6.0)
    {
        if(angle_acceleration > m_tAngularAccelerationThreshold ||
                angle_acceleration < -m_tAngularAccelerationThreshold)
            m_piLastOccuranceEvent[2]    = CurrentStepNumber;
        else
            m_piLastOccuranceNegEvent[2] = CurrentStepNumber;
    }
    else
    {
        m_iWildCardBit = 2;
    }

    if(dist_nbrsagents == 6.0)
    {
        if(angle_acceleration > m_tAngularAccelerationThreshold ||
                angle_acceleration   < -m_tAngularAccelerationThreshold)
            m_piLastOccuranceEvent[3]    = CurrentStepNumber;
        else
            m_piLastOccuranceNegEvent[3] = CurrentStepNumber;
    }
    else
    {
        m_iWildCardBit = 3;
    }

    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex] = 1.0;

            if (m_iWildCardBit == (int)(featureindex))
                m_iWildCardBit = -1;
        }
        else if ((CurrentStepNumber - m_piLastOccuranceNegEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex] = 0.0;

            if (m_iWildCardBit == (int)(featureindex))
                m_iWildCardBit = -1;
        }
        else
        {
            // default feature value in case of wild card
            m_pfFeatureValues[featureindex] = 0.0;

            if ((featureindex) == 3 && m_iWildCardBit == 2){
                printf("\nBoth bits 2 and 3 cannot have WC");
                exit(-1);}
        }
    }

#else //not WILDCARDINFV

    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    if(dist_nbrsagents < 6.0 &&
            (angle_acceleration > m_tAngularAccelerationThreshold ||
             angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(dist_nbrsagents == 6.0 &&
            (angle_acceleration > m_tAngularAccelerationThreshold ||
             angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex] = 1.0;
        }
        else
        {
            m_pfFeatureValues[featureindex] = 0.0;
        }
    }
#endif //WILDCARDINFV

    // Motors
    //5th: distance travelled by bot in past Y time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    TVector2d vecAgentPos = *(m_pcAgent->GetPosition());

    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // distance travelled in last 100 time-steps
        m_fSquaredDistTravelled =
                GetSquaredDistanceBetweenPositions(&vecAgentPos,
                                                   &(m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex]));


        // decision based on distance travelled in the last 100 time-steps
        if(m_fSquaredDistTravelled >= m_fSquaredDistThreshold)
            m_pfFeatureValues[4] = 1.0;
        else
            m_pfFeatureValues[4] = 0.0;
    }

    // adding new coordinate values into the queue
    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;


    //6th: velocity, higher than 5% of speed is accepted as feature=1
    m_pfFeatureValues[5] = (mag_velocity >= m_fVelocityThreshold) ? 1.0:0.0;
    //    if(mag_velocity >= m_fVelocityThreshold) // higher than 5% of speed is accepted as moving
    //        m_pfFeatureValues[5] = 1.0;
    //    else
    //        m_pfFeatureValues[5] = 0.0;


    if(NUMBER_OF_FEATURES > 6U)
    {
        m_pfFeatureValues[6] = (unCloseRangeNbrCount > 0) ? 1.0:0.0;
        m_pfFeatureValues[7] = (unFarRangeNbrCount   > 0) ? 1.0:0.0;

        if(dist_nbrsagents < 6.0  && (angle_velocity > m_tAngularVelocityThreshold))
            m_piLastOccuranceEvent[0] = CurrentStepNumber;

        if(dist_nbrsagents == 6.0 && (angle_velocity > m_tAngularVelocityThreshold))
            m_piLastOccuranceEvent[1] = CurrentStepNumber;

        if ((CurrentStepNumber - m_piLastOccuranceEvent[0]) <= m_iEventSelectionTimeWindow)
            m_pfFeatureValues[8] = 1.0;
        else
            m_pfFeatureValues[8] = 0.0;

        if(NUMBER_OF_FEATURES > 9U)
        {
            if ((CurrentStepNumber - m_piLastOccuranceEvent[1]) <= m_iEventSelectionTimeWindow)
                m_pfFeatureValues[9] = 1.0;
            else
                m_pfFeatureValues[9] = 0.0;

            m_pfFeatureValues[10] = (mag_acceleration   >= m_fAccelerationThreshold) ||
                    (mag_acceleration   <= -m_fAccelerationThreshold)
                    ? 1.0:0.0;

            m_pfFeatureValues[11] = (angle_velocity     >= m_tAngularVelocityThreshold)    ? 1.0:0.0;

            if(NUMBER_OF_FEATURES > 12U)
            {
                m_pfFeatureValues[12] = (angle_acceleration   >= m_tAngularAccelerationThreshold) ||
                        (angle_acceleration   <= -m_tAngularAccelerationThreshold)
                        ? 1.0:0.0;

                m_pfFeatureValues[13] = (mag_relvelocity   >= m_fRelativeVelocityMagThreshold) ||
                        (mag_relvelocity   <= -m_fRelativeVelocityMagThreshold)
                        ? 1.0:0.0;

                m_pfFeatureValues[14] = (dir_relvelocity   >= m_fRelativeVelocityDirThreshold) ||
                        (dir_relvelocity   <= -m_fRelativeVelocityDirThreshold)
                        ? 1.0:0.0;
            } // 15 bit FV
        } // 12 bit FV
    }     // 9 bit FV


#ifdef DEBUGFEATUREVECTORFLAG
    if(FDMODELTYPE != LINEQ) // lineq - low expected run time; can come back and log more details if needed
        PrintFeatureDetails();
#endif
}*/

/******************************************************************************/
/******************************************************************************/

void CFeatureVector::ComputeFeatureValues()
{
    double dist_nbrsagents, angle_acceleration, angle_velocity, mag_velocity, mag_acceleration;
    double mag_relvelocity, dir_relvelocity;
    unsigned int unCloseRangeNbrCount, unFarRangeNbrCount;

    dist_nbrsagents    = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);
    angle_acceleration = m_pcAgent->GetAngularAcceleration();
    angle_velocity     = m_pcAgent->GetAngularVelocity();
    mag_velocity       = Vec2dLength((*m_pcAgent->GetVelocity()));
    mag_acceleration   = Vec2dLength((*m_pcAgent->GetAcceleration()));

    m_pcAgent->GetRelativeVelocity(&mag_relvelocity, &dir_relvelocity, FEATURE_RANGE);

    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();

    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-3 in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 3-6 in the majority of of past X time-steps
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last X time-steps
        if(m_unSumTimeStepsNbrsRange0to3 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[0] = 1.0;
        else
            m_pfAllFeatureValues[0] = 0.0;

        if(m_unSumTimeStepsNbrsRange3to6 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[1] = 1.0;
        else
            m_pfAllFeatureValues[1] = 0.0;

        // removing the fist entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to3 -=  m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange3to6 -=  m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    unCloseRangeNbrCount = m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT);
    if (unCloseRangeNbrCount > 0)
    {
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to3++;
    }
    else
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    unFarRangeNbrCount = (m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT) - unCloseRangeNbrCount);
    if (unFarRangeNbrCount > 0)
    {
        m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange3to6++;
    }
    else
        m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex] = 0;


    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;


#ifdef WILDCARDINFV

    m_iWildCardBit = -1; // set to -1 if no wildcard bit present in FV

    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    if(dist_nbrsagents < 6.0)
    {
        if(angle_acceleration > m_tAngularAccelerationThreshold ||
                angle_acceleration < -m_tAngularAccelerationThreshold)
            m_piLastOccuranceEvent[2]    = CurrentStepNumber;
        else
            m_piLastOccuranceNegEvent[2] = CurrentStepNumber;
    }
    else
    {
        m_iWildCardBit = 2;
    }

    if(dist_nbrsagents == 6.0)
    {
        if(angle_acceleration > m_tAngularAccelerationThreshold ||
                angle_acceleration   < -m_tAngularAccelerationThreshold)
            m_piLastOccuranceEvent[3]    = CurrentStepNumber;
        else
            m_piLastOccuranceNegEvent[3] = CurrentStepNumber;
    }
    else
    {
        m_iWildCardBit = 3;
    }

    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfAllFeatureValues[featureindex] = 1.0;

            if (m_iWildCardBit == (int)(featureindex))
                m_iWildCardBit = -1;
        }
        else if ((CurrentStepNumber - m_piLastOccuranceNegEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfAllFeatureValues[featureindex] = 0.0;

            if (m_iWildCardBit == (int)(featureindex))
                m_iWildCardBit = -1;
        }
        else
        {
            // default feature value in case of wild card
            m_pfAllFeatureValues[featureindex] = 0.0;

            if ((featureindex) == 3 && m_iWildCardBit == 2){
                printf("\nBoth bits 2 and 3 cannot have WC");
                exit(-1);}
        }
    }

#else //not WILDCARDINFV

    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    if(dist_nbrsagents < 6.0 &&
            (angle_acceleration > m_tAngularAccelerationThreshold ||
             angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(dist_nbrsagents == 6.0 &&
            (angle_acceleration > m_tAngularAccelerationThreshold ||
             angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfAllFeatureValues[featureindex] = 1.0;
        }
        else
        {
            m_pfAllFeatureValues[featureindex] = 0.0;
        }
    }
#endif //WILDCARDINFV

    // Motors
    //5th: distance travelled by bot in past Y time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    TVector2d vecAgentPos = *(m_pcAgent->GetPosition());

    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // distance travelled in last 100 time-steps
        m_fSquaredDistTravelled =
                GetSquaredDistanceBetweenPositions(&vecAgentPos,
                                                   &(m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex]));


        // decision based on distance travelled in the last 100 time-steps
        if(m_fSquaredDistTravelled >= m_fSquaredDistThreshold)
            m_pfAllFeatureValues[4] = 1.0;
        else
            m_pfAllFeatureValues[4] = 0.0;
    }

    // adding new coordinate values into the queue
    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;


    //6th: velocity, higher than 5% of speed is accepted as feature=1
    m_pfAllFeatureValues[5] = (mag_velocity >= m_fVelocityThreshold) ? 1.0:0.0;
    //    if(mag_velocity >= m_fVelocityThreshold) // higher than 5% of speed is accepted as moving
    //        m_pfFeatureValues[5] = 1.0;
    //    else
    //        m_pfFeatureValues[5] = 0.0;


    m_pfAllFeatureValues[6] = (unCloseRangeNbrCount > 0) ? 1.0:0.0;
    m_pfAllFeatureValues[7] = (unFarRangeNbrCount   > 0) ? 1.0:0.0;


    if(dist_nbrsagents < 6.0  && (angle_velocity > m_tAngularVelocityThreshold))
        m_piLastOccuranceEvent[0] = CurrentStepNumber;

    if(dist_nbrsagents == 6.0 && (angle_velocity > m_tAngularVelocityThreshold))
        m_piLastOccuranceEvent[1] = CurrentStepNumber;

    if ((CurrentStepNumber - m_piLastOccuranceEvent[0]) <= m_iEventSelectionTimeWindow)
        m_pfAllFeatureValues[8] = 1.0;
    else
        m_pfAllFeatureValues[8] = 0.0;

    if ((CurrentStepNumber - m_piLastOccuranceEvent[1]) <= m_iEventSelectionTimeWindow)
        m_pfAllFeatureValues[9] = 1.0;
    else
        m_pfAllFeatureValues[9] = 0.0;

    m_pfAllFeatureValues[10] = (mag_acceleration   >= m_fAccelerationThreshold) ||
            (mag_acceleration   <= -m_fAccelerationThreshold)
            ? 1.0:0.0;

    m_pfAllFeatureValues[11] = (angle_velocity     >= m_tAngularVelocityThreshold)    ? 1.0:0.0;

    m_pfAllFeatureValues[12] = (angle_acceleration   >= m_tAngularAccelerationThreshold) ||
            (angle_acceleration   <= -m_tAngularAccelerationThreshold)
            ? 1.0:0.0;

    m_pfAllFeatureValues[13] = (mag_relvelocity   >= m_fRelativeVelocityMagThreshold) ||
            (mag_relvelocity   <= -m_fRelativeVelocityMagThreshold)
            ? 1.0:0.0;

    m_pfAllFeatureValues[14] = (dir_relvelocity   >= m_fRelativeVelocityDirThreshold) ||
            (dir_relvelocity   <= -m_fRelativeVelocityDirThreshold)
            ? 1.0:0.0;


    // adding the selected features into the feature vector
    //for(size_t i = 0; i < m_piFeaturesSelected.size(); ++i)
    for(size_t i = 0; i <  NUMBER_OF_FEATURES; ++i)
        m_pfFeatureValues[i] = m_pfAllFeatureValues[m_piFeaturesSelected[i]-1];

    if (CurrentStepNumber == 0 && m_pcAgent->GetIdentification()==0)
    {
        std::cout << std::endl << "Features selected ";
        for(size_t i = 0; i < m_piFeaturesSelected.size(); ++i)
            std::cout << m_piFeaturesSelected[i] << " ";
        std::cout << std::endl;
    }

#ifdef DEBUGFEATUREVECTORFLAG
    if(FDMODELTYPE != LINEQ) // lineq - low expected run time; can come back and log more details if needed
        PrintFeatureDetails();
#endif
}

/******************************************************************************/
/******************************************************************************/

void CFeatureVector::PrintFeatureDetails()
{
    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();

    double dist_nbrsagents, angle_acceleration, angle_velocity;
    double mag_relativeagentvelocity, dir_relativeagentvelocity,
            mag_relativeagentacceleration, dir_relativeagentacceleration;

    // Velocity magnitude and direction wrt. surrounding agents:
    m_pcAgent->GetRelativeVelocity(&mag_relativeagentvelocity, &dir_relativeagentvelocity, FEATURE_RANGE);
    //    TVector2d tTemp    = m_pcAgent->GetAverageVelocityOfSurroundingAgents(FEATURE_RANGE, ROBOT);

    //    float tmp_agentvelocity = Vec2dLength((*m_pcAgent->GetVelocity()));
    //    mag_relativeagentvelocity = tmp_agentvelocity - Vec2dLength((tTemp));

    //    if (Vec2dLength((*m_pcAgent->GetVelocity())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
    //        dir_relativeagentvelocity = Vec2dAngle((*m_pcAgent->GetVelocity()), tTemp);
    //    else
    //        dir_relativeagentvelocity = 0.0;


    // Acceleration magnitude and direction wrt. surrounding agents:
    m_pcAgent->GetRelativeAcceleration(&mag_relativeagentacceleration, &dir_relativeagentacceleration, FEATURE_RANGE);
    //    tTemp                = m_pcAgent->GetAverageAccelerationOfSurroundingAgents(FEATURE_RANGE, ROBOT);

    //    float tmp_agentacceleration = Vec2dLength((*m_pcAgent->GetAcceleration()));
    //    mag_relativeagentacceleration = tmp_agentacceleration - Vec2dLength((tTemp));
    //    if (Vec2dLength((*m_pcAgent->GetAcceleration())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
    //        dir_relativeagentacceleration = Vec2dAngle((*m_pcAgent->GetAcceleration()), tTemp);
    //    else
    //        dir_relativeagentacceleration = 0.0;


    if (m_pcAgent->GetBehavIdentification() == 1)
    {
        //#ifdef DISABLEMODEL_RETAINRNDCALLS // additional behav stats data
        //        double f_MaxSquareDistTravelled =
        //                (m_pcAgent->GetMaximumSpeed()*(double)m_iDistTravelledTimeWindow)*
        //                (m_pcAgent->GetMaximumSpeed()*(double)m_iDistTravelledTimeWindow);

        //        int sensorymotorinteract    = (m_piLastOccuranceEvent[0]==CurrentStepNumber)?1:0;
        //        int negsensorymotorinteract = (m_piLastOccuranceEvent[1]==CurrentStepNumber)?1:0;

        //        printf("AdditionalNormBehavData: Step: %d, NbrsInRange0to3: %d, NbrsInRange3to6: %d, Sensor-Motor interaction: %d, ~Sensor-Motor interaction: %d, SquaredDistTravelled: %f, MaxSquaredDist: %f\n",
        //               CurrentStepNumber,
        //               m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT),
        //               m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT)-m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT),
        //               sensorymotorinteract, negsensorymotorinteract,
        //               m_fSquaredDistTravelled, f_MaxSquareDistTravelled);
        //#endif //DISABLEMODEL_RETAINRNDCALLS

        /*printf("\nStep: %d. FV for normal agent %d: #NBRS %d, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f, Abs_vel [%f, %f], Abs_acel [%f, %f]\n", CurrentStepNumber, m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration,
               m_pcAgent->GetVelocity()->x, m_pcAgent->GetVelocity()->y,
               m_pcAgent->GetAcceleration()->x , m_pcAgent->GetAcceleration()->y);*/

#ifdef WILDCARDINFV
        printf("Step: %d, Alternate normal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, WildCard: %d\n", CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled, m_fSquaredDistThreshold, m_iWildCardBit);

#else
        printf("Step: %d, Alternate normal FV info, TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f\n", CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled, m_fSquaredDistThreshold);
#endif //WILDCARDINFV
    }

    if (m_pcAgent->GetBehavIdentification() == -1) //&& CurrentStepNumber > MODELSTARTTIME)
    {
        /*printf("\nStep: %d, FV for abnormal agent %d: #NBRS %d, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f, Abs_vel [%f, %f], Abs_acel [%f, %f]\n", CurrentStepNumber, m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration,
               m_pcAgent->GetVelocity()->x, m_pcAgent->GetVelocity()->y,
               m_pcAgent->GetAcceleration()->x , m_pcAgent->GetAcceleration()->y);*/


#ifdef WILDCARDINFV
        printf("Step: %d, Alternate abnormal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, WildCard: %d\n", CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled, m_fSquaredDistThreshold, m_iWildCardBit);

#else
        printf("Step: %d, Alternate abnormal FV info, TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f\n", CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled, m_fSquaredDistThreshold);
#endif //WILDCARDINFV
    }
}

/******************************************************************************/
/******************************************************************************/

std::string CFeatureVector::ToString()
{
    char pchTemp[4096];

    if(NUMBER_OF_FEATURES == 3U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "DistTW100: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_unValue);


    if(NUMBER_OF_FEATURES == 6U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],
                m_unValue);

    if(NUMBER_OF_FEATURES == 9U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - "

                "nbrs:0to3: %1.1f - "
                "nbrs:3to6: %1.1f - "
                "TW450_dist0to6_angvel: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],

                m_pfFeatureValues[6],
                m_pfFeatureValues[7],
                m_pfFeatureValues[8],
                m_unValue);


    if(NUMBER_OF_FEATURES == 12U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - "

                "nbrs:0to3: %1.1f - "
                "nbrs:3to6: %1.1f - "
                "TW450_dist0to6_angvel: %1.1f - "
                "TW450_dist6_angvel: %1.1f - "

                "acceleration: %1.1f - "
                "angvelocity: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],

                m_pfFeatureValues[6],
                m_pfFeatureValues[7],
                m_pfFeatureValues[8],
                m_pfFeatureValues[9],
                m_pfFeatureValues[10],
                m_pfFeatureValues[11],
                m_unValue);



    if(NUMBER_OF_FEATURES == 15U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - "

                "nbrs:0to3: %1.1f - "
                "nbrs:3to6: %1.1f - "
                "TW450_dist0to6_angvel: %1.1f - "
                "TW450_dist6_angvel: %1.1f - "

                "acceleration: %1.1f - "
                "angvelocity: %1.1f - "
                "angacceleration: %1.1f - "
                "relvelocity_mag: %1.1f - "
                "relvelocity_dir: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],

                m_pfFeatureValues[6],
                m_pfFeatureValues[7],
                m_pfFeatureValues[8],
                m_pfFeatureValues[9],
                m_pfFeatureValues[10],
                m_pfFeatureValues[11],

                m_pfFeatureValues[12],
                m_pfFeatureValues[13],
                m_pfFeatureValues[14],
                m_unValue);

    return string(pchTemp);
}

/******************************************************************************/
/******************************************************************************/

