#include "robotagent.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

CRobotAgent::CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments, CArguments* pc_crm_arguments, TBehaviorVector vec_behaviors) :
    CAgent(pch_name, un_identification, pc_arguments), m_vecBehaviors(vec_behaviors)
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }

    crminAgent          = new CRMinRobotAgent(this, pc_crm_arguments);
    m_pcFeatureVector   = new CFeatureVector(this);
    m_punFeaturesSensed = new unsigned int[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];

    m_fWeight = 0;

    static bool bHelpDisplayed = false;

    m_fBitflipProbabililty    = pc_arguments->GetArgumentAsDoubleOr("bitflipprob", 0.0);

    //control at what distances agents can sense one another when FVs have to be communicated
    m_fFVSenseRange               = pc_arguments->GetArgumentAsDoubleOr("fvsenserange", 15.0);

    //at what distances agents are considered neighbors when the individual features are computed
    CFeatureVector::FEATURE_RANGE = pc_arguments->GetArgumentAsDoubleOr("featuresenserange", 6.0);

    m_fResponseRange              = pc_arguments->GetArgumentAsDoubleOr("responserange", m_fFVSenseRange);


    if (pc_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("bitflipprob=#.#               Probability of flipping each bit in sensed feature vectors [%2.5f]\n"
               "fvsenserange=#.#              Range at which other agents' FVs are sensed [%f]\n"
               "featuresenserange=#.#         Range based on which features are computed  [%f]\n"
               "responserange=#.#             Range at which a robot \"reponds\" to other features [%f]\n",
               m_fBitflipProbabililty,
               m_fFVSenseRange,
               CFeatureVector::FEATURE_RANGE,
               m_fResponseRange
            );
    }

    m_pbMostWantedList = new bool[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];

    for (unsigned int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
    {
        m_pbMostWantedList[i] = false;
    }       
}

/******************************************************************************/
/******************************************************************************/

CRobotAgent::~CRobotAgent() 
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        delete (*i);
    }
    delete m_pcFeatureVector;
    delete m_punFeaturesSensed;
    delete m_pbMostWantedList;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SimulationStepUpdatePosition()
{
    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SimulationStep();
        if (!bControlTaken) 
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
                (*i)->Action();
        } else {
            (*i)->Suppress();
        }          
    }

    // Update the T-cells of the CRM instance for this robot
    m_pcFeatureVector->SimulationStep();
    unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    if (m_unIdentification == 1 && CurrentStepNumber > CRMSTARTTIME)
    {
        printf("\nFV for normal agent %d: %s\n", m_unIdentification, m_pcFeatureVector->ToString().c_str());
    }
    if (m_unIdentification == 25 && CurrentStepNumber > CRMSTARTTIME)
    {
        printf("\nFV for abnormal agent %d: %s\n", m_unIdentification, m_pcFeatureVector->ToString().c_str());
    }

    Sense();

    if(CurrentStepNumber > CRMSTARTTIME)
    {
        crminAgent->SimulationStepUpdatePosition();
    }
    CAgent::SimulationStepUpdatePosition();
}

/******************************************************************************/
/******************************************************************************/

const CFeatureVector* CRobotAgent::GetFeatureVector() const
{
    return m_pcFeatureVector;
}

/******************************************************************************/
/******************************************************************************/

EAgentType CRobotAgent::GetType()
{
    return ROBOT;
}

/******************************************************************************/
/******************************************************************************/

unsigned int* CRobotAgent::GetFeaturesSensed() const
{
    return m_punFeaturesSensed;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgent* CRobotAgent::GetRandomRobotWithWeights(double f_range)
{
    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    if (tAgentListList.size() == 0)
    {
        ERROR2("This should never happen - the agent list-list is empty - maybe the position of the agent is wrong (%f,%f)", 
               m_tPosition.x, 
               m_tPosition.y);
    }

    double fWeightSum = CountWeightsInAgentListList(&tAgentListList, f_range);
    if (fWeightSum < 1e-10)
    {
        return NULL;
    } 
    double fSelectedWeight  = Random::nextDouble() * fWeightSum;

    TAgentList* ptAgentList  = NULL;
    TAgentListListIterator i = tAgentListList.begin();
    CAgent* pcAgentSelected  = NULL;

    do
    {        
        while ((*i)->size() == 0) 
        {
            i++;
        }
        TAgentListIterator j = (*i)->begin();

        while (j != (*i)->end() && fSelectedWeight > 0) 
        {
            if ((*j)->m_bTempWithInRange) 
                fSelectedWeight -= ((CRobotAgent*) (*j))->GetWeight();
            if (fSelectedWeight > 0)
                j++;
        }

        if (fSelectedWeight > 0)
            i++;
        else
        {
            if ((*j)->m_bTempWithInRange)
            {
                pcAgentSelected = (*j);
            } else {
                while (j != (*i)->end() && pcAgentSelected == NULL)
                {
                    if ((*j)->m_bTempWithInRange)
                    {
                        pcAgentSelected = (*j);
                    }
                    else
                    {
                        j++;                    
                    }
                }

                if (pcAgentSelected == NULL)
                    i++;
            }

        }
    } while (pcAgentSelected == NULL && i != tAgentListList.end());
    
    if (i == tAgentListList.end())
    {
        ERROR("The random generator seems to be wrong");
    } 
        
    return (CRobotAgent*) pcAgentSelected;
} 

/******************************************************************************/
/******************************************************************************/

double CRobotAgent::CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range)
{
    double fReturn = 0;
    TAgentListListIterator i;

    double fSquareRange = f_range * f_range;

    for (i = ptlist_agent_list_list->begin(); i != ptlist_agent_list_list->end(); i++)
    {
        TAgentListIterator j;
        for (j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->GetType() == ROBOT && (*j) != this)
            {
                (*j)->m_bTempWithInRange = (GetSquaredDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition()) <= fSquareRange);
                if ((*j)->m_bTempWithInRange)
                    fReturn += ((CRobotAgent*) (*j))->GetWeight();
            } else {
                (*j)->m_bTempWithInRange = false;
            }

        }
    }

    return fReturn;
}

/******************************************************************************/
/******************************************************************************/

CRMinRobotAgent* CRobotAgent::GetCRMinRobotAgent()
{
    return crminAgent;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SetWeight(double f_weight)
{
    m_fWeight = f_weight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgent::GetWeight() const
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgent::GetFVSenseRange() const
{
    return m_fFVSenseRange;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::Sense()
{
    for (int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
    {
        m_punFeaturesSensed[i] = 0;
    }

    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), m_fFVSenseRange);
    TAgentListListIterator i;
    double fSenseRangeSquared = m_fFVSenseRange * m_fFVSenseRange;

    for (i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        TAgentListIterator j;
        for (j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->GetType() == ROBOT && GetSquaredDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition()) <= fSenseRangeSquared)
            {
                CRobotAgent* pcRobot = (CRobotAgent*) (*j);
                
                // Apply noise:
                unsigned int unFeatureVector = pcRobot->GetFeatureVector()->GetValue();
                if (m_fBitflipProbabililty > 0.00001) 
                {

                    for (int k = 0; k < CFeatureVector::NUMBER_OF_FEATURES; k++) 
                    {
  
                        if (Random::nextDouble() < m_fBitflipProbabililty)
                        {
                            unsigned int unBitToFlip = k;
                            unFeatureVector ^= 1 << unBitToFlip;
                        }
                    }
                }              

//                unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
//                if(CurrentStepNumber > 3250U)
//                if(this->m_unIdentification == 25U)
//                    if (m_pbMostWantedList[unFeatureVector]) {
//                        printf("Attacking agent number: %d. unFeatureVector: %d\n", (*j)->GetIdentification(),unFeatureVector);
//                    }

                m_punFeaturesSensed[unFeatureVector]++;
            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgent::GetColor()
{
    //return m_unIdentification == TRACKAGENT ? GREEN : RED;
    if(m_unIdentification == 1)
        return GREEN;
    else if (m_unIdentification == 25)
        return RED;
    else
        return BLUE;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SetBehaviors(TBehaviorVector vec_behaviors)
{
    m_vecBehaviors = vec_behaviors;

    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SetMostWantedList(unsigned unFeatureVector, bool state)
{
    m_pbMostWantedList[unFeatureVector] = state;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::CheckNeighborsReponseToMyFV(unsigned int* pun_number_of_toleraters, unsigned int* pun_number_of_attackers, unsigned int* pun_number_of_unconverged)
{
    (*pun_number_of_toleraters)  = 0;
    (*pun_number_of_attackers)   = 0;
    (*pun_number_of_unconverged) = 0;

    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), m_fResponseRange);
    TAgentListListIterator i;
    double fResponseRangeSquared = m_fResponseRange * m_fResponseRange;

    bool m_battackeragentlog=true,m_btolerateragentlog=true;
    for (i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        TAgentListIterator j;
        for (j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->GetType() == ROBOT && 
                GetSquaredDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition()) <= fResponseRangeSquared && 
                (*j) != this)
            {
                CRMinRobotAgent* tmp_crm = ((CRobotAgent*) (*j))->GetCRMinRobotAgent();
                if (((CRobotAgent*) (*j))->Attack(m_pcFeatureVector))
                {
                    (*pun_number_of_attackers)++;

                    if(m_battackeragentlog)
                    {
                        printf("\nAn attacker agent. Convg. error %f    ",tmp_crm->GetConvergenceError());
                        unsigned int* FeatureVectorsSensed;
                        FeatureVectorsSensed = ((CRobotAgent*) (*j))->GetFeaturesSensed();

                        for (int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
                        {
                            if(FeatureVectorsSensed[i] > 0.0)
                            {
                                printf("FV:%d, [APC]:%f, [E]:%f, [R]:%f   ",i,
                                       tmp_crm->GetAPC(i),
                                       tmp_crm->GetCurrE(i),
                                       tmp_crm->GetCurrR(i));
                            }
                        }
                        m_battackeragentlog = false;
                    }
                }
                else
                {
                    (*pun_number_of_toleraters)++;

                    if(m_btolerateragentlog)
                    {
                        printf("\nA tolerator agent. Convg. error %f    ",tmp_crm->GetConvergenceError());
                        unsigned int* FeatureVectorsSensed;
                        FeatureVectorsSensed = ((CRobotAgent*) (*j))->GetFeaturesSensed();
                        for (int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
                        {
                            if(FeatureVectorsSensed[i] > 0.0)
                            {
                                printf("FV:%d, [APC]:%f, [E]:%f, [R]:%f   ",i,
                                       tmp_crm->GetAPC(i),
                                       tmp_crm->GetCurrE(i),
                                       tmp_crm->GetCurrR(i));
                            }
                        }
                        m_btolerateragentlog = false;
                    }
                }

                if (!tmp_crm->GetConvergenceFlag())
                {
                    (*pun_number_of_unconverged)++;
                }

            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/

bool CRobotAgent::Attack(CFeatureVector* pc_feature_vector)
{
    return m_pbMostWantedList[pc_feature_vector->GetValue()];
}

/******************************************************************************/
/******************************************************************************/
