#include "robotagent_optimised.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised::CRobotAgentOptimised(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments, CArguments* pc_model_arguments, TBehaviorVector vec_behaviors) :
        CAgent(pch_name, un_identification, pc_arguments), m_vecBehaviors(vec_behaviors)
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
        (*i)->SetAgent(this);

    crminAgent = NULL;
    if(FDMODELTYPE == CRM || FDMODELTYPE == CRM_TCELLSINEXCESS)
        crminAgent = new CRMinRobotAgentOptimised(this, pc_model_arguments);
    else if(FDMODELTYPE == THRESHOLDONFVDIST)
        thresholdinAgent = new ThresholdinRobotAgentOptimised(this, pc_model_arguments);
    else { printf("\nUnknown model type"); exit(-1);}

    m_pcFeatureVector   = new CFeatureVector(this);

    m_fWeight = 0.0;

    static bool bHelpDisplayed = false;

    //control at what distances agents can sense one another when FVs have to be communicated
    // now made redundant with selectnumnearestnbrs
    m_fFVSenseRange               = pc_arguments->GetArgumentAsDoubleOr("fvsenserange", 10.0);

    //at what distances agents are considered neighbors when the individual features are computed
    CFeatureVector::FEATURE_RANGE = pc_arguments->GetArgumentAsDoubleOr("featuresenserange", 6.0);

    m_fResponseRange              = pc_arguments->GetArgumentAsDoubleOr("responserange", m_fFVSenseRange);
    m_uSelectedNumNearestNbrs     = pc_arguments->GetArgumentAsIntOr("selectnumnearestnbrs", 10);
    m_uNumVotingNbrs              = pc_arguments->GetArgumentAsIntOr("numvotingnbrs", 10);

#ifdef CRM_ENABLE_SENSORY_HISTORY
    m_fProbForgetFV               = pc_arguments->GetArgumentAsDoubleOr("prob_forget_fv", 1.0);
#endif

    if(pc_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
        printf("fvsenserange=#.#              Range at which other agents' FVs are sensed [%f]\n"
               "featuresenserange=#.#         Range based on which features are computed  [%f]\n"
               "responserange=#.#             Range at which a robot \"reponds\" to other features [%f]\n"
               "selectnumnearestnbrs=#        The number of nearest neighbours for FV sensing and T-cell diffusion (makes fvsenserange redundant) [%d]\n"
               "numvotingnbrs=#               The number of nearest neighbours for voting an agent abnormal) [%d]\n"
#ifdef CRM_ENABLE_SENSORY_HISTORY
               "prob_forget_fv=#             Probability to forget a FV (used if CRM_ENABLE_SENSORY_HISTORY is defined)) [%f]\n"
#endif
               ,
               m_fFVSenseRange,
               CFeatureVector::FEATURE_RANGE,
               m_fResponseRange,
               m_uSelectedNumNearestNbrs,
               m_uNumVotingNbrs
#ifdef CRM_ENABLE_SENSORY_HISTORY
               ,m_fProbForgetFV
#endif

               );


//    m_pfFeaturesSensed  = new float[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];
//    m_pbMostWantedList = new unsigned int[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];
//    for (unsigned int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
//    {
//        m_pbMostWantedList[i] = 0;
//    }

    m_bRobotDeactivated = false;
    m_iDEactivationTime = -1;
    m_unConseqDetectedFaulty = 0;

    m_uNumberFloatingPtOperations = 0;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised::~CRobotAgentOptimised()
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
        delete (*i);

    delete m_pcFeatureVector;
    listFVsSensed.clear();
//    delete m_pfFeaturesSensed;
//    delete m_pbMostWantedList;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SimulationStepUpdatePosition()
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
        } else
            (*i)->Suppress();        
    }

    // Update the model (T-cells of the CRM instance for this robot), CTRNN neuron activations, lineq on fvs
    m_pcFeatureVector->SimulationStep();
    unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();

#ifdef DEBUGFEATUREVECTORFLAG
    if (m_iBehavIdentification == 1)
        printf("\nStep: %d, FV for normal agent %d: %s\n", CurrentStepNumber, m_unIdentification, m_pcFeatureVector->ToString().c_str());

    if (m_iBehavIdentification == -1)
        printf("\nStep: %d, FV for abnormal agent %d: %s\n", CurrentStepNumber, m_unIdentification, m_pcFeatureVector->ToString().c_str());
#endif

    //Sense(GetSelectedNumNearestNbrs());

    if(CurrentStepNumber > MODELSTARTTIME)
    {
        Sense(GetSelectedNumNearestNbrs());
        if(FDMODELTYPE == CRM || FDMODELTYPE == CRM_TCELLSINEXCESS)
            crminAgent->SimulationStepUpdatePosition();
        else if(FDMODELTYPE == THRESHOLDONFVDIST)
            thresholdinAgent->SimulationStepUpdatePosition();
    }

    CAgent::SimulationStepUpdatePosition();
}

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised* CRobotAgentOptimised::GetRandomRobotWithWeights(double f_range)
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
        return NULL;

    double fSelectedWeight  = Random::nextDouble() * fWeightSum;

    TAgentListListIterator i = tAgentListList.begin();
    CAgent* pcAgentSelected  = NULL;

    do
    {        
        while ((*i)->size() == 0) 
        {
            i++;
        }
        TAgentListIterator j = (*i)->begin();

        while (j != (*i)->end() && fSelectedWeight > 0.0)
        {
            if ((*j)->m_bTempWithInRange) 
                fSelectedWeight -= ((CRobotAgentOptimised*) (*j))->GetWeight();
            if (fSelectedWeight > 0.0)
                j++;
        }

        if (fSelectedWeight > 0.0)
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
        ERROR("The random generator seems to be wrong");

    return (CRobotAgentOptimised*) pcAgentSelected;
} 

/******************************************************************************/
/******************************************************************************/

double CRobotAgentOptimised::CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range)
{
    double fReturn = 0.0;
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
                    fReturn += ((CRobotAgentOptimised*) (*j))->GetWeight();
            } else {
                (*j)->m_bTempWithInRange = false;
            }
        }
    }

    return fReturn;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised* CRobotAgentOptimised::GetRandomRobotWithWeights(unsigned int u_nearestnbrs)
{
    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents, ROBOT);

    double fWeightSum = 0.0;
    // 1-11 if u_nearestnbrs is 10, because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        fWeightSum += pcRobot->GetWeight();
    }

    if (fWeightSum < 1e-10)
        return NULL;

    double fSelectedWeight  = Random::nextDouble() * fWeightSum;


    CAgent* pcAgentSelected  = NULL;
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        fSelectedWeight -= pcRobot->GetWeight();

        if(fSelectedWeight <= 0.0)
        {
            pcAgentSelected = pcRobot;
            break;
        }
    }
    return (CRobotAgentOptimised*) pcAgentSelected;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised* CRobotAgentOptimised::GetRandomRobotWithMasterWeights(unsigned int u_nearestnbrs)
{
    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents, ROBOT);

    double fWeightSum = 0.0;
    // 1-11 if u_nearestnbrs is 10, because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        fWeightSum += pcRobot->GetMasterWeight();
    }

    if (fWeightSum < 1e-10)
        return NULL;

    double fSelectedWeight  = Random::nextDouble() * fWeightSum;


    CAgent* pcAgentSelected  = NULL;
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        fSelectedWeight -= pcRobot->GetMasterWeight();

        if(fSelectedWeight <= 0.0)
        {
            pcAgentSelected = pcRobot;
            break;
        }
    }
    return (CRobotAgentOptimised*) pcAgentSelected;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SetWeight(double f_weight)
{
    m_fWeight = f_weight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgentOptimised::GetWeight() const
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/


void CRobotAgentOptimised::SetMasterWeight(double f_weight)
{
    m_fMasterWeight = f_weight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgentOptimised::GetMasterWeight() const
{
    return m_fMasterWeight;
}

/******************************************************************************/
/******************************************************************************/


double CRobotAgentOptimised::GetFVSenseRange() const
{
    return m_fFVSenseRange;
}

/******************************************************************************/
/******************************************************************************/

#ifdef CRM_ENABLE_SENSORY_HISTORY
void CRobotAgentOptimised::Sense(unsigned int u_nearestnbrs)
{  
    list<structFVsSensed>::iterator it, it_history;

    // forget old FVs with probability m_fProbForgetFV
    for (it = listFVsSensed.begin(); it != listFVsSensed.end(); ++it)
        if(Random::nextDouble() <= m_fProbForgetFV)
            it = listFVsSensed.erase(it);

    list<structFVsSensed> tmp_list(listFVsSensed.begin(), listFVsSensed.end());
    listFVsSensed.clear();

    TAgentVector tSortedAgents;
    SortAllAgentsAccordingToDistance(&tSortedAgents, ROBOT);

    // 1-11 because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        UpdateFeatureVectorDistribution(pcRobot->GetFeatureVector()->GetValue(), 1.0);
    }


    // integrate into the current list listFVsSensed the history from tmp_list
    // if FV is the same in the current list, and in the history, - the history for that FV is ignored.
    // else the FV is integrated into the current list
    it = listFVsSensed.begin(); it_history = tmp_list.begin();
    while(it_history != tmp_list.end() && it != listFVsSensed.end())
    {
        if((*it_history).uFV == (*it).uFV)
        {
            ++it_history; ++it;
            continue;
        }

        if((*it_history).uFV < (*it).uFV)
        {
                listFVsSensed.insert(it, structFVsSensed((*it_history).uFV, (*it_history).fRobots, (*it_history).uMostWantedState));
                ++it_history;
        }
        else
             ++it;
   }

    while(it_history != tmp_list.end())
    {
            listFVsSensed.push_back(structFVsSensed((*it_history).uFV, (*it_history).fRobots, (*it_history).uMostWantedState));
            ++it_history;
    }

    //Issue with  intermediary regulatory T-cells (with FV between abnormal and normal FVs) resulting in tolerance of abnromal FVs.
    //Solution1: Normalize so max sum is 0.02
    //Solution2: curtail affinity between T-cells and APCs
    //Normalization does not always solve the problem. So we use solution 2


    //We could also add / forget ind robots fv (seems more biologically intutive way to model history) and then normalize (to make sure abnormal FVs density stay below the bifurcation point), and also apply the curtail on the affinity.
}
#else
void CRobotAgentOptimised::Sense(unsigned int u_nearestnbrs)
{
    listFVsSensed.clear();
    TAgentVector tSortedAgents;
    SortAllAgentsAccordingToDistance(&tSortedAgents, ROBOT);

    // 1-11 because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        UpdateFeatureVectorDistribution(pcRobot->GetFeatureVector()->GetValue(), 1.0);
    }
}
#endif

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::UpdateFeatureVectorDistribution(unsigned int fv, double increment)
{
    list<structFVsSensed>::iterator it;

    // check if fv is in listFVsSensed
    // if so, update the value it holds by increment
    // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
    for (it = listFVsSensed.begin(); it != listFVsSensed.end(); ++it)
    {
        if((*it).uFV == fv)
        {
            // if fv is already present
            (*it).fRobots += increment;
            return;
        }

        if((*it).uFV > fv)
        {   // we assume the list is kept sorted.
            // if fv is absent
            listFVsSensed.insert(it, structFVsSensed(fv, increment));
            return;
        }
    }

    // when the list is empty
    listFVsSensed.push_back(structFVsSensed(fv, increment));
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::PrintFeatureVectorDistribution(unsigned int id)
{
    if(!(GetIdentification() == id))
        return;

    list<structFVsSensed>::iterator it;
    printf("\n====R%d Feature Vector Distribution=====\n",GetIdentification());
    for (it = listFVsSensed.begin(); it != listFVsSensed.end(); ++it)
        printf("FV:%d, Robots:%f, Suspicion:%f ",(*it).uFV, (*it).fRobots, (*it).fSuspicious);
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgentOptimised::GetColor()
{
    unsigned int unToleraters  = 0;
    unsigned int unAttackers   = 0;
    unsigned int unNbrsInSensoryRange = 0;
    unsigned int unSuspectors = 0;

    bool dbgflag = false;

    this->CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unSuspectors, &unNbrsInSensoryRange, dbgflag);

    if(unToleraters > (unAttackers+unSuspectors))
        return GREEN;
    else
        if(unSuspectors >= unAttackers)
            return BLUE;
        else
            return RED;

    //return m_unIdentification == TRACKAGENT ? GREEN : RED;
    /*if(m_unIdentification == 15)
        return RED;
    else
        return BLUE;*/

    /*if(m_iBehavIdentification  == 1)
        return GREEN;
    else if (m_iBehavIdentification  == -1)
        return RED;
    else  if (m_unIdentification == 5) // a supposedly normal agent that seems to take long to join the aggregate
        return YELLOW;
    else*/

    #ifdef TCELLCLONEEXCHANGEANALYSIS
            if(GetIdentification() <= 9) {
                return GREEN;
            }
            else
                return RED;
    #else
//    /*if(GetFeatureVector()->GetValue()==39U)
//        return RED;
//    else*/
//        return BLUE;
    #endif
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SetBehaviors(TBehaviorVector vec_behaviors)
{
    m_vecBehaviors = vec_behaviors;

    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::ClearBehaviors()
{
    m_vecBehaviors.clear();
    assert(m_vecBehaviors.empty());
}

/******************************************************************************/
/******************************************************************************/

inline TBehaviorVector CRobotAgentOptimised::GetBehaviors()
{
    return m_vecBehaviors;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SetMostWantedList(list<structFVsSensed>::iterator* it, unsigned int state)
{
    list<structFVsSensed>::iterator it_fvsensed = (*it);

    (*it_fvsensed).uMostWantedState = state;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgentOptimised::GetMostWantedState(unsigned int fv)
{
    list<structFVsSensed>::iterator it_fvsensed;

    for(it_fvsensed = listFVsSensed.begin(); it_fvsensed != listFVsSensed.end(); ++it_fvsensed)
        if((*it_fvsensed).uFV == fv)
            return (*it_fvsensed).uMostWantedState;

    return 3; // if fv is not in the list (not sensed)
}

void CRobotAgentOptimised::SetSuspicion(list<structFVsSensed>::iterator* it, double state)
{
    list<structFVsSensed>::iterator it_fvsensed = (*it);

    (*it_fvsensed).fSuspicious = state;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::CheckNeighborsResponseToMyFV(unsigned int* pun_number_of_toleraters, unsigned int* pun_number_of_attackers, unsigned int* pun_number_of_suspectors, unsigned int* pun_number_of_neighborsinsensoryrange, bool b_logs)
{
    (*pun_number_of_toleraters)  = 0;
    (*pun_number_of_attackers)   = 0;
    (*pun_number_of_suspectors)  = 0;
    (*pun_number_of_neighborsinsensoryrange) = 0;



    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents, ROBOT);

    // 1-11 because agent at index 0 is ourselves:
    bool m_battackeragentlog=true, m_btolerateragentlog=true, m_bsuspectoragentlog=true;
    for (unsigned int nbrs = 1; nbrs < m_uNumVotingNbrs+1; nbrs++)
    {
        CRobotAgentOptimised* pcRobot         = (CRobotAgentOptimised*) tSortedAgents[nbrs];

        unsigned int fv_status   = pcRobot->Attack(m_pcFeatureVector);
        if (fv_status == 1)
        {
            (*pun_number_of_attackers)++;

            if(m_battackeragentlog && b_logs)
            {
                printf("\nAn attacker agent.");
                //float* FeatureVectorsSensed;
                //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();

                PrintDecidingAgentDetails(m_pcFeatureVector, pcRobot);
                m_battackeragentlog = false;
            }
        }
        else if(fv_status == 2)
        {
            (*pun_number_of_toleraters)++;

            if(m_btolerateragentlog && b_logs)
            {
                printf("\nA tolerator agent.");
                //float* FeatureVectorsSensed;
                //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
                PrintDecidingAgentDetails(m_pcFeatureVector, pcRobot);
                m_btolerateragentlog = false;
            }
        }
        else if(fv_status == 4)
        {
            // FV deemed suspicious
            (*pun_number_of_suspectors)++;
            if(m_bsuspectoragentlog && b_logs)
            {
                printf("\nA suspector agent.");
                PrintDecidingAgentDetails(m_pcFeatureVector, pcRobot);
                m_bsuspectoragentlog = false;
            }
        }


        //float* FeatureVectorsSensed;
        //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
//        if(FeatureVectorsSensed[m_pcFeatureVector->GetValue()] > 0.0)
        if(fv_status != 3)
            (*pun_number_of_neighborsinsensoryrange)++;
        // change to if m_pcFeatureVector->GetValue() is a member of (pcRobot->crminAgent->vecAPCs), then increment (*pun_number_of_neighborsinsensoryrange) by 1

        if(b_logs)
        {
            printf("\nAn agent.");
            //float* FeatureVectorsSensed;
            //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
            PrintDecidingAgentDetails(m_pcFeatureVector, pcRobot);
        }
    }

    if(!m_bRobotDeactivated)
    {
        if((*pun_number_of_attackers) > (*pun_number_of_toleraters))
            m_unConseqDetectedFaulty++; // incremented faulted counter
        else if((*pun_number_of_attackers) <= (*pun_number_of_toleraters))
            m_unConseqDetectedFaulty=0; // reset the counter

        if(m_unConseqDetectedFaulty > DEACTIVATIONTHRESHOLD)
        {
            m_bRobotDeactivated = true;
            m_iDEactivationTime = (int) CSimulator::GetInstance()->GetSimulationStepNumber();
        }
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::PrintDecidingAgentDetails(CFeatureVector* m_pcFV,
                                                     CRobotAgentOptimised* decidingrobot)
{
    if(FDMODELTYPE == THRESHOLDONFVDIST) {
        decidingrobot->PrintFeatureVectorDistribution(decidingrobot->GetIdentification());
        return; }


    CRMinRobotAgentOptimised* model_crminagent     = decidingrobot->GetCRMinRobotAgent();

    printf("  Convg. error %f (%fperc)    ",model_crminagent->GetConvergenceError(), model_crminagent->GetConvergenceError_Perc());
    decidingrobot->PrintFeatureVectorDistribution(decidingrobot->GetIdentification());
    model_crminagent->PrintAPCList(decidingrobot->GetIdentification());
    model_crminagent->PrintTcellList(decidingrobot->GetIdentification());
    model_crminagent->PrintTcellResponseToAPCList(decidingrobot->GetIdentification());
}

/******************************************************************************/
/******************************************************************************/
