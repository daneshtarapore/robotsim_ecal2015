#include "featurevector.h"
#include "assert.h"

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::NUMBER_OF_FEATURES        = 6;
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

    m_fLowPassFilterParameter    = 0.01;
    m_fThresholdOnNumNbrs        = 3.99 ;
    m_fProcessedNumNeighbours    = 0.0;

    m_iEventSelectionTimeWindow = 1500;//CRMSTARTTIME;

    for(unsigned int i = 0; i < NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;
    }

    m_dVelocityThreshold = 0.05 * (m_pcAgent->GetMaximumSpeed());


#if defined(WILDCARDINFV) && defined(ALTERNATESIXBITFV)

    m_iWildCardBit = -1; // set to -1 if no wildcard bit present in FV

/*    m_unSenMotIntCurrQueueIndex = 0;

    m_unSumTimeSteps_SenMotIntNbrsInRange_at1  = 0;
    m_unSumTimeSteps_SenMotIntNbrsInRange_at0  = 0;
    m_unSumTimeSteps_SenMotIntNbrsInRange_atWC = 0;

    m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1  = 0;
    m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0  = 0;
    m_unSumTimeSteps_SenMotIntNbrsNotInRange_atWC = 0;

    // for each time-step
    //1: if motors respond to sensory input; 0: if motors donot respond to sensory input; and WC (2): if no sensory input present
    m_punSenMotInt_NbrsInRange    = new unsigned int[m_iEventSelectionTimeWindow];

    //1: if motors respond in absence of sensory input; 0: if motors donot respond to absence of sensory input; and WC (2): if sensory input is present
    m_punSenMotInt_NbrsNotinRange = new unsigned int[m_iEventSelectionTimeWindow];*/

#endif


#ifdef ALTERNATESIXBITFV

    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to3 = 0;
    m_unSumTimeStepsNbrsRange3to6 = 0;

    m_punNbrsRange0to3AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];
    m_punNbrsRange3to6AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];


    // keeping track of distance travelled by bot in last 100 time-steps
    m_iDistTravelledTimeWindow = 100;
    m_unCoordCurrQueueIndex    = 0;

    m_dSquaredDistTravelled = 0.0;
    m_dSquaredDistThreshold = (0.05 * (m_pcAgent->GetMaximumSpeed()*m_iDistTravelledTimeWindow)) *
                              (0.05 * (m_pcAgent->GetMaximumSpeed()*m_iDistTravelledTimeWindow));

    m_pvecCoordAtTimeStep = new TVector2d[m_iDistTravelledTimeWindow];
#endif


}

/******************************************************************************/
/******************************************************************************/

CFeatureVector::~CFeatureVector()
{
    delete m_pfFeatureValues;
    delete m_piLastOccuranceEvent;
    delete m_piLastOccuranceNegEvent;

#if defined(WILDCARDINFV) && defined(ALTERNATESIXBITFV)
/*
    delete m_punSenMotInt_NbrsNotinRange;
    delete m_punSenMotInt_NbrsInRange;*/
#endif


#ifdef ALTERNATESIXBITFV

    delete m_punNbrsRange0to3AtTimeStep;
    delete m_punNbrsRange3to6AtTimeStep;
    delete m_pvecCoordAtTimeStep;

#endif
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
        m_unValue += (unsigned int)m_pfFeatureValues[i] * (1 << i);
    }
}

/******************************************************************************/
/******************************************************************************/

void CFeatureVector::ComputeFeatureValues()
{
    double dist_nbrsagents, angle_acceleration, angle_velocity, mag_velocity;
    double mag_relativeagentvelocity, dir_relativeagentvelocity,
    mag_relativeagentacceleration, dir_relativeagentacceleration;


    dist_nbrsagents    = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);
    angle_acceleration = m_pcAgent->GetAngularAcceleration();
    angle_velocity     = m_pcAgent->GetAngularVelocity();
    mag_velocity       = Vec2dLength((*m_pcAgent->GetVelocity()));


#ifdef ALTERNATESIXBITFV
    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();

    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-3 in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 3-6 in the majority of of past X time-steps

    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last 1500 time-steps
        if(m_unSumTimeStepsNbrsRange0to3 > (unsigned)(0.5*m_iEventSelectionTimeWindow))
            m_pfFeatureValues[0] = 1.0;
        else
            m_pfFeatureValues[0] = 0.0;

        if(m_unSumTimeStepsNbrsRange3to6 > (unsigned)(0.5*m_iEventSelectionTimeWindow))
            m_pfFeatureValues[1] = 1.0;
        else
            m_pfFeatureValues[1] = 0.0;

        // removing the fist entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to3 -=  m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange3to6 -=  m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    unsigned int unCloseRangeNbrCount = m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT);
    if (unCloseRangeNbrCount > 0)
    {
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to3++;
    }
    else
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    if ((m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT) - unCloseRangeNbrCount) > 0)
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

    if(dist_nbrsagents < 6)
    {
        if(angle_acceleration > 0.1 || angle_acceleration < -0.1)
            m_piLastOccuranceEvent[0]    = CurrentStepNumber;
        else
            m_piLastOccuranceNegEvent[0] = CurrentStepNumber;
    }
    else
    {
        m_iWildCardBit = 2;
    }

    if(dist_nbrsagents == 6.0)
    {
        if(angle_acceleration > 0.1 || angle_acceleration < -0.1)
            m_piLastOccuranceEvent[1]    = CurrentStepNumber;
        else
            m_piLastOccuranceNegEvent[1] = CurrentStepNumber;
    }
    else
    {
        m_iWildCardBit = 3;
    }

    for(unsigned int featureindex = 0; featureindex <=1; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex+2] = 1.0;
            m_iWildCardBit = -1;
        }
        else if ((CurrentStepNumber - m_piLastOccuranceNegEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex+2] = 0.0;
            m_iWildCardBit = -1;
        }
        else
        {
            // default feature value in case of wild card
            m_pfFeatureValues[featureindex+2] = 0.0;
        }
    }


    /*
    // Sensors-motor interactions
    // Set if the occurance of the following event, MAJORITY OF TIMES in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    m_iWildCardBit = -1; // set to -1 if no wildcard bit present in FV

    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last 1500 (m_iEventSelectionTimeWindow) time-steps
        if(m_unSumTimeSteps_SenMotIntNbrsInRange_at1 > 0 &&
           m_unSumTimeSteps_SenMotIntNbrsInRange_at1 >= m_unSumTimeSteps_SenMotIntNbrsInRange_at0)
        {
            m_pfFeatureValues[2] = 1.0;
        }
        else if(m_unSumTimeSteps_SenMotIntNbrsInRange_at0 > 0 &&
                m_unSumTimeSteps_SenMotIntNbrsInRange_at0 > m_unSumTimeSteps_SenMotIntNbrsInRange_at1)
        {
            m_pfFeatureValues[2] = 0.0;
        }
        else
        {
            m_iWildCardBit = 2; m_pfFeatureValues[2] = 0.0; // default feature value
        }


        if(m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1 > 0 &&
           m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1 >= m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0)
        {
            m_pfFeatureValues[3] = 1.0;
        }
        else if(m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0 > 0 &&
                m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0 >
                m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1)
        {
            m_pfFeatureValues[3] = 0.0;
        }
        else
        {
            m_iWildCardBit = 3; m_pfFeatureValues[3] = 0.0; // default feature value
        }


        // removing the fist entry of the moving time window  from the sum
        if(m_punSenMotInt_NbrsInRange[m_unSenMotIntCurrQueueIndex] == 1)
            m_unSumTimeSteps_SenMotIntNbrsInRange_at1--;
        else if(m_punSenMotInt_NbrsInRange[m_unSenMotIntCurrQueueIndex] == 0)
            m_unSumTimeSteps_SenMotIntNbrsInRange_at0--;
        else
            m_unSumTimeSteps_SenMotIntNbrsInRange_atWC--;


        if(m_punSenMotInt_NbrsNotinRange[m_unSenMotIntCurrQueueIndex] == 1)
            m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1--;
        else if(m_punSenMotInt_NbrsNotinRange[m_unSenMotIntCurrQueueIndex] == 0)
            m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0--;
        else
            m_unSumTimeSteps_SenMotIntNbrsNotInRange_atWC--;

    }


    // adding new values into the queue
    if(dist_nbrsagents < 6)
    {
        if (angle_acceleration > 0.1 || angle_acceleration < -0.1)
        {
            m_punSenMotInt_NbrsInRange[m_unSenMotIntCurrQueueIndex] = 1;
            m_unSumTimeSteps_SenMotIntNbrsInRange_at1++;
        }
        else
        {
            m_punSenMotInt_NbrsInRange[m_unSenMotIntCurrQueueIndex] = 0;
            m_unSumTimeSteps_SenMotIntNbrsInRange_at0++;
        }
    }
    else
    {
        m_punSenMotInt_NbrsInRange[m_unSenMotIntCurrQueueIndex] = 2;
        m_unSumTimeSteps_SenMotIntNbrsInRange_atWC++;
    }

    if(dist_nbrsagents == 6)
    {
        if (angle_acceleration > 0.1 || angle_acceleration < -0.1)
        {
            m_punSenMotInt_NbrsNotinRange[m_unSenMotIntCurrQueueIndex] = 1;
            m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1++;
        }
        else
        {
            m_punSenMotInt_NbrsNotinRange[m_unSenMotIntCurrQueueIndex] = 0;
            m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0++;
        }
    }
    else
    {
        m_punSenMotInt_NbrsNotinRange[m_unSenMotIntCurrQueueIndex] = 2;
        m_unSumTimeSteps_SenMotIntNbrsNotInRange_atWC++;
    }

    m_unSenMotIntCurrQueueIndex = (m_unSenMotIntCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;*/

#else

    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    if(dist_nbrsagents < 6 && (angle_acceleration > 0.1 || angle_acceleration < -0.1))
    {
        m_piLastOccuranceEvent[0] = CurrentStepNumber;
    }

    if(dist_nbrsagents == 6.0 && (angle_acceleration > 0.1 || angle_acceleration < -0.1))
    {
        m_piLastOccuranceEvent[1] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 0; featureindex <=1; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex+2] = 1.0;
        }
        else
        {
            m_pfFeatureValues[featureindex+2] = 0.0;
        }
    }
#endif


    // Motors
    //5th: distance travelled by bot in past Y time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    TVector2d vecAgentPos = *(m_pcAgent->GetPosition());

    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // distance travelled in last 100 time-steps
        m_dSquaredDistTravelled =
                GetSquaredDistanceBetweenPositions(&vecAgentPos,
                                                   &(m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex]));


        // decision based on distance travelled in the last 100 time-steps
        if(m_dSquaredDistTravelled >= m_dSquaredDistThreshold)
            m_pfFeatureValues[4] = 1.0;
        else
            m_pfFeatureValues[4] = 0.0;
    }

    // adding new coordinate values into the queue
    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;



    //6th: velocity, higher than 5% of speed is accepted as feature=1
    if(mag_velocity >= m_dVelocityThreshold) // higher than 5% of speed is accepted as moving
        m_pfFeatureValues[5] = 1.0;
    else
        m_pfFeatureValues[5] = 0.0;

#else

    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();

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


    if(dist_nbrsagents < 2 &&
       (angle_acceleration > 0.1 || angle_acceleration < -0.1))
    {
        m_piLastOccuranceEvent[0] = CurrentStepNumber;
    }

    if((dist_nbrsagents >= 2 && dist_nbrsagents < 4) &&
       (angle_acceleration > 0.1 || angle_acceleration < -0.1))
    {
        m_piLastOccuranceEvent[1] = CurrentStepNumber;
    }

    if((dist_nbrsagents >= 4 && dist_nbrsagents < 6) &&
       (angle_acceleration > 0.1 || angle_acceleration < -0.1))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(dist_nbrsagents == 6.0 && (angle_acceleration > 0.1 || angle_acceleration < -0.1))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 0; featureindex <=3; featureindex++)
    {
        // Occurance of the following event, atleast once in time window m_iEventSelectionTimeWindow (1500)
        // 2nd,3rd,4th feature: distance to nbrs <2,2-4,4-6 && change in angular acceleration
        // 5th feature: No neighbors detected  && change in angular acceleration
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex+1] = 1.0;
        }
        else
        {
            m_pfFeatureValues[featureindex+1] =  0.0;
        }
    }

    //6th feature: velocity
    if(mag_velocity >= 0.1 * m_pcAgent->GetMaximumSpeed()) // higher than 5% of speed is accepted as moving
        m_pfFeatureValues[5] = 1.0;
    else
        m_pfFeatureValues[5] = 0.0;


#endif

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



    if (m_pcAgent->GetIdentification() == 1)// && CurrentStepNumber > CRMSTARTTIME)
    {
        printf("\nFV for normal agent %d: #NBRS %d, lpf(#NBRS) %f, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f, Abs_vel [%f, %f], Abs_acel [%f, %f]\n", m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),m_fProcessedNumNeighbours,dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration,
               m_pcAgent->GetVelocity()->x, m_pcAgent->GetVelocity()->y,
               m_pcAgent->GetAcceleration()->x , m_pcAgent->GetAcceleration()->y);

#ifdef ALTERNATESIXBITFV
#ifdef WILDCARDINFV

/*  printf("Alternate normal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, TimeSteps_Sen-MotIntNbrsInRange_at1: %d, TimeSteps_Sen-MotIntNbrsInRange_at0: %d, TimeSteps_Sen-MotIntNbrsInRange_atWC: %d, TimeSteps_Sen-MotIntNbrsNotInRange_at1: %d, TimeSteps_Sen-MotIntNbrsNotInRange_at0: %d, TimeSteps_Sen-MotIntNbrsNotInRange_atWC: %d\n",

m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_dSquaredDistTravelled, m_dSquaredDistThreshold,

m_unSumTimeSteps_SenMotIntNbrsInRange_at1, m_unSumTimeSteps_SenMotIntNbrsInRange_at0, m_unSumTimeSteps_SenMotIntNbrsInRange_atWC,

m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1, m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0, m_unSumTimeSteps_SenMotIntNbrsNotInRange_atWC);*/

        printf("Alternate normal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, WildCard: %d\n", m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_dSquaredDistTravelled, m_dSquaredDistThreshold, m_iWildCardBit);

#else

        printf("Alternate normal FV info, TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f\n", m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_dSquaredDistTravelled, m_dSquaredDistThreshold);

#endif

#endif
    }

    if (m_pcAgent->GetIdentification() == 15) //&& CurrentStepNumber > CRMSTARTTIME)
    {
        printf("\nFV for abnormal agent %d: #NBRS %d, lpf(#NBRS) %f, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f, Abs_vel [%f, %f], Abs_acel [%f, %f]\n", m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),m_fProcessedNumNeighbours,dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration,
               m_pcAgent->GetVelocity()->x, m_pcAgent->GetVelocity()->y,
               m_pcAgent->GetAcceleration()->x , m_pcAgent->GetAcceleration()->y);

#ifdef ALTERNATESIXBITFV
#ifdef WILDCARDINFV

        /*printf("Alternate abnormal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, TimeSteps_Sen-MotIntNbrsInRange_at1: %d, TimeSteps_Sen-MotIntNbrsInRange_at0: %d, TimeSteps_Sen-MotIntNbrsInRange_atWC: %d, TimeSteps_Sen-MotIntNbrsNotInRange_at1: %d, TimeSteps_Sen-MotIntNbrsNotInRange_at0: %d, TimeSteps_Sen-MotIntNbrsNotInRange_atWC: %d\n",

m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_dSquaredDistTravelled, m_dSquaredDistThreshold,

m_unSumTimeSteps_SenMotIntNbrsInRange_at1, m_unSumTimeSteps_SenMotIntNbrsInRange_at0, m_unSumTimeSteps_SenMotIntNbrsInRange_atWC,

m_unSumTimeSteps_SenMotIntNbrsNotInRange_at1, m_unSumTimeSteps_SenMotIntNbrsNotInRange_at0, m_unSumTimeSteps_SenMotIntNbrsNotInRange_atWC);*/

        printf("Alternate abnormal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, WildCard: %d\n", m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_dSquaredDistTravelled, m_dSquaredDistThreshold, m_iWildCardBit);

#else


        printf("Alternate abnormal FV info, TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f\n", m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_dSquaredDistTravelled, m_dSquaredDistThreshold);
#endif
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

std::string CFeatureVector::ToString()
{
    char pchTemp[4096];


#ifdef ALTERNATESIXBITFV
    sprintf(pchTemp, "Values - "
            "TS_nbrs:0to3: %f - "
            "TS_nbrs:3to6: %f - "
            "TW1500_dist0to6_angacc: %1.1f - "
            "TW1500_dist6_angacc: %1.1f - "
            "DistTW100: %1.1f - "
            "speed: %1.1f - fv: %u",

            m_pfFeatureValues[0],
            m_pfFeatureValues[1],
            m_pfFeatureValues[2],
            m_pfFeatureValues[3],
            m_pfFeatureValues[4],
            m_pfFeatureValues[5],
            m_unValue);

#else
    sprintf(pchTemp, "Values - "
            "nbrs: %f - "
            "TW1500_dist0to2_angacc: %1.1f - "
            "TW1500_dist2to4_angacc: %1.1f - "
            "TW1500_dist4to6_angacc: %1.1f - "
            "TW1500_dist6_angacc: %1.1f - "
            "speed: %1.1f - fv: %u",

            m_pfFeatureValues[0],
            m_pfFeatureValues[1],
            m_pfFeatureValues[2],
            m_pfFeatureValues[3],
            m_pfFeatureValues[4],
            m_pfFeatureValues[5],
            m_unValue);
#endif


    return string(pchTemp);
}

/******************************************************************************/
/******************************************************************************/

