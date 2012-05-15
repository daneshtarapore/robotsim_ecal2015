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


    static bool bHelpDisplayed = false;

    m_fFVSenseRange               = pc_arguments->GetArgumentAsDoubleOr("fvsenserange", 10.0);
    CFeatureVector::FEATURE_RANGE = pc_arguments->GetArgumentAsDoubleOr("featuresenserange", 5.0);

    if (pc_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("fvsenserange=#.#              Range at which other agents' FVs are sensed [%f]\n"
               "featuresenserange=#.#         Range based on which features are computed  [%f]\n",
               m_fFVSenseRange,
               CFeatureVector::FEATURE_RANGE
            );
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
    if (m_unIdentification == 0)
    {
        printf("FV for agent 0: %s\n", m_pcFeatureVector->ToString().c_str());
    }
    Sense();
    crminAgent->SimulationStepUpdatePosition();
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

    for (i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        TAgentListIterator j;
        for (j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->GetType() == ROBOT)
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
                            unsigned int unBitToFlip = k; // Random::nextInt(CFeatureVector::NUMBER_OF_FEATURES);
                            unFeatureVector ^= 1 << unBitToFlip;
                        }
                    }
                }              

                m_punFeaturesSensed[unFeatureVector]++;
            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/
