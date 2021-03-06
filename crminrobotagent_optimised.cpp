#include "crminrobotagent_optimised.h"

/******************************************************************************/
/******************************************************************************/

#define CELLLOWERBOUND 1.0e-3 //todo: set as percentage instead of absolute value
// note: could result in euler-huen diff at 0, for high error thresholds. In that case, lower this value

#define CONJUGATION_OVERFLOW_LIMIT 1.0e-10  //todo: set as percentage instead of absolute value

/******************************************************************************/
/******************************************************************************/

#define TCELL_UPPERLIMIT_STEPSIZE 500000.0 //10.0 //to give you better control of integration step //500000//todo: could be set as a propotion of the INTEGRATION_TIME
#define TCELL_LOWERLIMIT_STEPSIZE 1.0e-6

#define CONJ_UPPERLIMIT_STEPSIZE 10 //10.0
#define CONJ_LOWERLIMIT_STEPSIZE 1.0e-6 //1.0e-6

#define ERRORALLOWED_TCELL_STEPSIZE 1.0e-2 //todo: set as percentage instead of absolute value
#define ERRORALLOWED_CONJ_STEPSIZE  1.0e-3 //-3//todo: set as percentage instead of absolute value; else will introduce problems when m_fFVtoApcscaling is reduced, and dealing with density of conjugates in order of 1e-6


//#define INTEGRATION_TIME  5.0e+7 // was 1.5e+7 on elephant01a  earlier 1.0e+7
#define FAILSAFE_CONJ_INTEGRATION_TIME  5.0e+5 // a failsafe to prevent endless integrations of a stiff system.
//TODO: Could instead use the differences in the error values (between time-steps), being same over a period of time as a requirement to reduce time-step
#define REDUCESTEPSIZE_CONJ_INTEGRATION_TIME 1.0e+5 // lowers the step size when the conjugation integration has passed this limit, and the error allowed is high (>1e-3).



#define TCELL_CONVERGENCE  1.0e-2 //todo: set as percentage instead of absolute value. Already using the percentage values to break out of integration loop
#define CONJ_CONVERGENCE   1.0e-3 //-3//todo: set as percentage instead of absolute value


/******************************************************************************/
/******************************************************************************/

CRMinRobotAgentOptimised::CRMinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent,
                                                   CArguments* m_crmArguments)
{
    robotAgent = ptr_robotAgent;

    m_fWeight  = 1.0;
    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_crmArguments->GetArgumentAsIntOr("numberoffeatures", 6);
    seedE = m_crmArguments->GetArgumentAsDoubleOr("seedE", 10.0);  // : Density of effector cells
    seedR = m_crmArguments->GetArgumentAsDoubleOr("seedR", 10.0);  // : Density of regulatory cells
    kon   = m_crmArguments->GetArgumentAsDoubleOr("kon", .1);   // : Conjugation rate
    koff  = m_crmArguments->GetArgumentAsDoubleOr("koff", .1);  // : Dissociation rate
    kpe   = m_crmArguments->GetArgumentAsDoubleOr("kpe", 1e-3);   // : Proliferation rate for effector cells
    kde   = m_crmArguments->GetArgumentAsDoubleOr("kde", 1e-6); //1e-6  // : Death rate for effector cells
    kpr   = m_crmArguments->GetArgumentAsDoubleOr("kpr", 0.7e-3);//0.7e-3   // : Proliferation rate for regulatory cells
    kdr   = m_crmArguments->GetArgumentAsDoubleOr("kdr", 1e-6); //1e-6  // : Death rate for regulatory cells

    m_fTryExchangeProbability = m_crmArguments->GetArgumentAsDoubleOr("exchangeprob", 0.0);
    // is now set based on characteristics of robot
    //m_fExchangeRange          = m_crmArguments->GetArgumentAsDoubleOr("exchangerange", 2.0);

    se                        = m_crmArguments->GetArgumentAsDoubleOr("sourceE", seedE); // Source density of E cell generation
    sr                        = m_crmArguments->GetArgumentAsDoubleOr("sourceR", seedR); // Source density of R cell generation

    se_rate                   = m_crmArguments->GetArgumentAsDoubleOr("sourcerateE", 0.0); // Source density of E cell generation
    sr_rate                   = m_crmArguments->GetArgumentAsDoubleOr("sourcerateR", 0.0); // Source density of R cell generation

    m_fcross_affinity         = m_crmArguments->GetArgumentAsDoubleOr("cross-affinity", 0.15);

    m_fApcscalingtype         = m_crmArguments->GetArgumentAsIntOr("fvapcscaling_LinearOrExp", 0); //0: linear, 1: exp
    // linear scaling
    m_fFVtoApcscaling         = m_crmArguments->GetArgumentAsDoubleOr("fvapcscaling", 2.0e-3);
    // exp scaling
    m_fFVtoApcscaling_expbase = m_crmArguments->GetArgumentAsDoubleOr("fvapcscaling_expbase",0.001);
    m_fFVtoApcscaling_exprate = m_crmArguments->GetArgumentAsDoubleOr("fvapcscaling_exprate", 0.95);

    m_uPersistenceThreshold   = m_crmArguments->GetArgumentAsIntOr("perst_th", 0);

    m_fStartExpIntegrationTime= m_crmArguments->GetArgumentAsDoubleOr("startexptintegrationtime", 5.0e+7); // 5.0e+7
    m_fIntegrationTime        = m_crmArguments->GetArgumentAsDoubleOr("integrationtime", m_fStartExpIntegrationTime); // 5.0e+7
    
    m_uSeedfvHdRange          = m_crmArguments->GetArgumentAsIntOr("seedfv-hd-range",
                                                                   CFeatureVector::NUMBER_OF_FEATURES);

#ifndef CRM_ENABLE_SENSORY_HISTORY
    m_uHistoryTcells          = m_crmArguments->GetArgumentAsIntOr("hist_ts", 0);
    m_fSuspicionThreshold     = m_crmArguments->GetArgumentAsDoubleOr("susp_th", 1.0);
    m_fNewFVSuspicionIncr     = m_crmArguments->GetArgumentAsDoubleOr("newfvsusp_incr", 1.0);
#endif

    if(FDMODELTYPE == CRM_TCELLSINEXCESS)
        assert(kon == koff);

    if (m_crmArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#            Number of features in a single FV [%d]\n"
               "seedE=#.#                     Seed density of effector cells [%f]\n"
               "seedR=#.#                     Seed density of regulatory cells [%f]\n"
               "kon=#.#                       Conjugation rate [%f]\n"
               "koff=#.#                      Dissociation rate [%f]\n"
               "kpe=#.#                       Proliferation rate for effector cells [%f]\n"
               "kde=#.#                       Death rate for effector cells [%f]\n"
               "kpr=#.#                       Proliferation rate for regulatory cells [%f]\n"
               "kdr=#.#                       Death rate for regulatory cells [%f]\n"
               "exchangeprob=#.#              Probability of trying to exchange cells with other robots [%f]\n"
               "Source_E=#.#                  Source density of E cells per simulation time-step [%f]\n"
               "Source_R=#.#                  Source density of R cells per simulation time-step [%f]\n"
               "SourceRate_E=#.#              Influx rate of new E cells [%f]\n"
               "SourceRate_R=#.#              Influx rate of new R cells [%f]\n"

               "cross-affinity=#.#            Level of cross-affinity (>0)     [%2.5f]\n"

               "fvapcscalingtype=#            Scaling type (0: linear, and 1: exponential) [%d]\n"
               "fvapcscaling=#.#              Linear scaling parameter of [FV] to [APC] [%e]\n"
               "fvapcscaling_expbase=#.#      Exponential scaling parameter of [FV] to [APC] [%f]\n"
               "fvapcscaling_exprate=#.#      Exponential scaling parameter of [FV] to [APC] [%f]\n"

               "perst_th=#                    Persistence threshold [%d]\n"
               "integrationtime=#.#           CRM integration time [%e]\n"
               "startexptintegrationtime=#.#  CRM integration time at start of experiment [%e]\n"

               "seedfv-hd-range=#             Diversity of seed t-cell population [%d]\n"

       #ifndef CRM_ENABLE_SENSORY_HISTORY
               "hist_ts=#                     T-cell populations recorded for time-steps  [%d]\n"
               "susp_th=#                     Threshold above which a FV is to be tolerated - but deemed suspicious. Range: [0,1]  [%f]\n"
               "newfvsusp_incr=#              Increment to suspicioun counter when encountering a new FV. Range: [0,1]  [%f]\n"
       #endif
               ,CFeatureVector::NUMBER_OF_FEATURES,
               seedE,
               seedR,
               kon,
               koff,
               kpe,
               kde,
               kpr,
               kdr,
               m_fTryExchangeProbability,
               se,
               sr,
               se_rate,
               sr_rate,
               m_fcross_affinity,
               m_fApcscalingtype,
               m_fFVtoApcscaling,
               m_fFVtoApcscaling_expbase, m_fFVtoApcscaling_exprate,
               m_uPersistenceThreshold, m_fIntegrationTime, m_fStartExpIntegrationTime,
               m_uSeedfvHdRange

       #ifndef CRM_ENABLE_SENSORY_HISTORY
               ,m_uHistoryTcells,
               m_fSuspicionThreshold,
               m_fNewFVSuspicionIncr
       #endif
               );
        bHelpDisplayed = true;
    }


    m_unNumberOfReceptors = 1 << (CFeatureVector::NUMBER_OF_FEATURES);

#ifdef TCELLCLONEEXCHANGEANALYSIS
    printf("\nError. The two flock expt has not been optimized");
#endif

    sites = 3U;
    step_h = 1.0;

    assert(sites == 3U);
    assert(m_fcross_affinity > 0.0);
}

/******************************************************************************/
/******************************************************************************/

CRMinRobotAgentOptimised::~CRMinRobotAgentOptimised()
{
    listTcells.clear();

    list<structAPC>::iterator it_apcs;
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        (*it_apcs).listConjugatesonAPC.clear();

    listAPCs.clear();
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::SimulationStepUpdatePosition()
{    
    unsigned PrntRobotId = robotAgent->GetIdentification();

    /*if(PrntRobotId == 8 && CSimulator::GetInstance()->GetSimulationStepNumber() == 5973)
    {
        std::cout << std::endl << " before ";
        robotAgent->PrintFeatureVectorDistribution(PrntRobotId);
        PrintAPCList(PrntRobotId); PrintTcellList(PrntRobotId);
        PrintConjugatestoAPCList(PrntRobotId, CONJ);
        PrintConjugatestoTcellList(PrntRobotId, CONJ);
    }*/

#ifdef DEBUGCROSSREGULATIONMODELFLAG
    robotAgent->PrintFeatureVectorDistribution(PrntRobotId);
#endif

    // Convert the feature vectors of robot agents in the vicinity to APCs for the CRM to work with
    UpdateAPCList(); //O(m-fv + n-apc)
#ifdef DEBUGCROSSREGULATIONMODELFLAG
    PrintAPCList(PrntRobotId);
#endif

#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to regenerate the same sequence of random numbers generated with the normal working of the CRM, so that the same agent behaviors may be obtained when CRM is disabled.
#ifdef DEBUGCROSSREGULATIONMODELFLAG
    PrintTcellList(PrntRobotId);
#endif
    UpdateTcellList(m_uSeedfvHdRange); //O(m-apc + n-tcell)
#ifdef DEBUGCROSSREGULATIONMODELFLAG
    PrintTcellList(PrntRobotId);
#endif

    // allocate memory for the new soon to be conjugates (from t-cells, apcs added), and remove non-existing conjugates (from t-cells, apcs dead)
    UpdateConjugatesToAPCList(); //O(m-apc *(~n-conj + n-tcell))
    //PrintConjugatestoAPCList(PrntRobotId, CONJ);

    UpdateConjugatesToTcellList(); // update (actually recreating) list of pointers to conjugates, allocated to the apcs. //O(m-apc * ~n-conj)
    //PrintConjugatestoTcellList(PrntRobotId, CONJ);


    /*if(PrntRobotId == 8 && CSimulator::GetInstance()->GetSimulationStepNumber() == 5973)
    {
        std::cout << std::endl << " after ";
        robotAgent->PrintFeatureVectorDistribution(PrntRobotId);
        PrintAPCList(PrntRobotId); PrintTcellList(PrntRobotId);
        PrintConjugatestoAPCList(PrntRobotId, CONJ);
        PrintConjugatestoTcellList(PrntRobotId, CONJ);
    }*/



    m_fTCELL_UPPERLIMIT_STEPSIZE = TCELL_UPPERLIMIT_STEPSIZE;
    m_fTCELL_LOWERLIMIT_STEPSIZE = TCELL_LOWERLIMIT_STEPSIZE;


    m_fERRORALLOWED_TCELL_STEPSIZE = ERRORALLOWED_TCELL_STEPSIZE;

    m_fTCELL_CONVERGENCE = TCELL_CONVERGENCE; //todo: set as percentage instead of absolute value. Already using the percentage values to break out of integration loop


    TcellNumericalIntegration_RK2();

#endif

    //if(m_fTryExchangeProbability > 0.0)
    DiffuseTcells(); /* We want the same sequence of random numbers generated, even when m_fTryExchangeProbability = 0.0 */
    //DiffuseMasterTcells();


#ifdef DEBUGCROSSREGULATIONMODELFLAG
    if(this->robotAgent->GetIdentification()==1 || this->robotAgent->GetIdentification()==15)
    {
        printf("\nAllTcellClonesTime: %d, RobotId: %d, ", CSimulator::GetInstance()->GetSimulationStepNumber(), this->robotAgent->GetIdentification());

        list<structTcell>::iterator it_tcells;
        for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
            printf("Clone: %d, A: %f, E: %f, R: %f ",(*it_tcells).uFV,
                   ((*it_tcells).ptrAPCWithAffinity1)==NULL?
                       GetAPC((*it_tcells).uFV):((*it_tcells).ptrAPCWithAffinity1)->fAPC,
                   (*it_tcells).fE, (*it_tcells).fR);
        printf("\n");}
#endif

    UpdateState();
}

/******************************************************************************/
/******************************************************************************/
void CRMinRobotAgentOptimised::DiffuseTcells()
{
    m_fWeight = 0.0;
    list<structTcell>* listRemoteTcells;
    list<structTcell>::iterator it_tcells; list<structTcell>::iterator it_remotetcells;
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        m_fWeight += (*it_tcells).fE + (*it_tcells).fR;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(2);
#endif
    }

    m_fWeight = m_fWeight * m_fWeight;
#ifdef FLOATINGPOINTOPERATIONS
    robotAgent->IncNumberFloatingPtOperations(1);
#endif

    robotAgent->SetWeight(m_fWeight);


    // select the robot from one of the 10 nearest neighbours - but in these expts. comm does not seem to be needed (comment: we dont know this for sure)
    CRobotAgentOptimised* pcRemoteRobotAgent =
            robotAgent->GetRandomRobotWithWeights(robotAgent->GetSelectedNumNearestNbrs());

    if(m_fTryExchangeProbability == 0.0) /* We want the same sequence of random numbers generated, even when m_fTryExchangeProbability = 0.0 */
        return;

    if(pcRemoteRobotAgent == NULL) {
#ifdef DEBUGCROSSREGULATIONMODELFLAG
        printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
               CSimulator::GetInstance()->GetSimulationStepNumber(),
               robotAgent->GetIdentification(), -1);
#endif
        return; }


    CRMinRobotAgentOptimised* crminRemoteRobotAgent =
            pcRemoteRobotAgent->GetCRMinRobotAgent();
    assert(crminRemoteRobotAgent != NULL);

    it_tcells = listTcells.begin(); listRemoteTcells = crminRemoteRobotAgent->GetListTcells();
    it_remotetcells = listRemoteTcells->begin();

    while(it_tcells != listTcells.end() && it_remotetcells != listRemoteTcells->end())
    {
        if((*it_tcells).uFV == (*it_remotetcells).uFV)
        {
            double currEtoSend = (*it_tcells).fE  * m_fTryExchangeProbability;
            double currRtoSend = (*it_tcells).fR  * m_fTryExchangeProbability;

            double currEtoReceive = (*it_remotetcells).fE * m_fTryExchangeProbability;
            double currRtoReceive = (*it_remotetcells).fR * m_fTryExchangeProbability;

            (*it_remotetcells).fE += currEtoSend - currEtoReceive;
            (*it_remotetcells).fR += currRtoSend - currRtoReceive;

            (*it_tcells).fE  += currEtoReceive - currEtoSend;
            (*it_tcells).fR  += currRtoReceive - currRtoSend;

            ++it_tcells; ++it_remotetcells;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(12);
#endif
            continue;
        }

        if((*it_tcells).uFV > (*it_remotetcells).uFV)
        {
            // the serial (instead of simultaneous) execution of the numerical integration and diffusion causes dead clones to be passed to and fro between robots after the numerical intgration has completed.
            // this causes an unnecessary computation burden. we prevent this by checking for dead clonaltypes before receiving them (no needed on actual robot implementation)
            if((*it_remotetcells).fE +(*it_remotetcells).fR <= CELLLOWERBOUND)
            {++it_remotetcells; robotAgent->IncNumberFloatingPtOperations(1); continue;}

            listTcells.insert(it_tcells, structTcell((*it_remotetcells).uFV,
                                                     (*it_remotetcells).fE * m_fTryExchangeProbability,
                                                     (*it_remotetcells).fR * m_fTryExchangeProbability,
                                                     (*it_remotetcells).uHistory,
                                                     NULL));
            (*it_remotetcells).fE -= (*it_remotetcells).fE * m_fTryExchangeProbability;
            (*it_remotetcells).fR -= (*it_remotetcells).fR * m_fTryExchangeProbability;
            ++it_remotetcells;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(6);
#endif
        }
        else
        {
            listRemoteTcells->insert(it_remotetcells, structTcell((*it_tcells).uFV,
                                                                  (*it_tcells).fE * m_fTryExchangeProbability,
                                                                  (*it_tcells).fR * m_fTryExchangeProbability,
                                                                  (*it_tcells).uHistory,
                                                                  NULL));
            (*it_tcells).fE -= (*it_tcells).fE * m_fTryExchangeProbability;
            (*it_tcells).fR -= (*it_tcells).fR * m_fTryExchangeProbability;
            ++it_tcells;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(6);
#endif
        }
    }

    while(it_tcells != listTcells.end()) {
        listRemoteTcells->push_back(structTcell((*it_tcells).uFV,
                                                (*it_tcells).fE * m_fTryExchangeProbability,
                                                (*it_tcells).fR * m_fTryExchangeProbability,
                                                (*it_tcells).uHistory,
                                                NULL));
        (*it_tcells).fE -= (*it_tcells).fE * m_fTryExchangeProbability;
        (*it_tcells).fR -= (*it_tcells).fR * m_fTryExchangeProbability;
        ++it_tcells;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(6);
#endif
    }

    while(it_remotetcells != listRemoteTcells->end()) {
        if((*it_remotetcells).fE + (*it_remotetcells).fR <= CELLLOWERBOUND)
        {++it_remotetcells; robotAgent->IncNumberFloatingPtOperations(1); continue;}

        listTcells.push_back(structTcell((*it_remotetcells).uFV,
                                         (*it_remotetcells).fE * m_fTryExchangeProbability,
                                         (*it_remotetcells).fR * m_fTryExchangeProbability,
                                         (*it_remotetcells).uHistory,
                                         NULL));
        (*it_remotetcells).fE -= (*it_remotetcells).fE * m_fTryExchangeProbability;
        (*it_remotetcells).fR -= (*it_remotetcells).fR * m_fTryExchangeProbability;
        ++it_remotetcells;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(6);
#endif
    }

#ifdef DEBUGCROSSREGULATIONMODELFLAG
    printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
           CSimulator::GetInstance()->GetSimulationStepNumber(),
           robotAgent->GetIdentification(), pcRemoteRobotAgent->GetIdentification());
#endif

}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::DiffuseMasterTcells()
{
    m_fMasterWeight = 0.0;
    list<structTcell>* listRemoteTcells;
    list<structTcell>::iterator it_tcells; list<structTcell>::iterator it_remotetcells;
    for(it_tcells = masterlistTcells.begin(); it_tcells != masterlistTcells.end(); ++it_tcells)
    {
        m_fMasterWeight += (*it_tcells).fE + (*it_tcells).fR;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(2);
#endif
    }

    m_fMasterWeight = m_fMasterWeight * m_fMasterWeight;
#ifdef FLOATINGPOINTOPERATIONS
    robotAgent->IncNumberFloatingPtOperations(1);
#endif

    robotAgent->SetMasterWeight(m_fMasterWeight);


    // select the robot from one of the 10 nearest neighbours - but in these expts. comm does not seem to be needed (comment: we dont know this for sure)
    CRobotAgentOptimised* pcRemoteRobotAgent =
            robotAgent->GetRandomRobotWithMasterWeights(robotAgent->GetSelectedNumNearestNbrs());

    if(m_fTryExchangeProbability == 0.0) /* We want the same sequence of random numbers generated, even when m_fTryExchangeProbability = 0.0 */
        return;

    if(pcRemoteRobotAgent == NULL) {
#ifdef DEBUGCROSSREGULATIONMODELFLAG
        printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
               CSimulator::GetInstance()->GetSimulationStepNumber(),
               robotAgent->GetIdentification(), -1);
#endif
        return; }


    CRMinRobotAgentOptimised* crminRemoteRobotAgent =
            pcRemoteRobotAgent->GetCRMinRobotAgent();
    assert(crminRemoteRobotAgent != NULL);

    it_tcells = masterlistTcells.begin(); listRemoteTcells = crminRemoteRobotAgent->GetMasterListTcells();
    it_remotetcells = listRemoteTcells->begin();

    while(it_tcells != masterlistTcells.end() && it_remotetcells != listRemoteTcells->end())
    {
        if((*it_tcells).uFV == (*it_remotetcells).uFV)
        {
            double currEtoSend = (*it_tcells).fE  * m_fTryExchangeProbability;
            double currRtoSend = (*it_tcells).fR  * m_fTryExchangeProbability;

            double currEtoReceive = (*it_remotetcells).fE * m_fTryExchangeProbability;
            double currRtoReceive = (*it_remotetcells).fR * m_fTryExchangeProbability;

            (*it_remotetcells).fE += currEtoSend - currEtoReceive;
            (*it_remotetcells).fR += currRtoSend - currRtoReceive;

            (*it_tcells).fE  += currEtoReceive - currEtoSend;
            (*it_tcells).fR  += currRtoReceive - currRtoSend;

            ++it_tcells; ++it_remotetcells;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(12);
#endif
            continue;
        }

        if((*it_tcells).uFV > (*it_remotetcells).uFV)
        {
            // the serial (instead of simultaneous) execution of the numerical integration and diffusion causes dead clones to be passed to and fro between robots after the numerical intgration has completed.
            // this causes an unnecessary computation burden. we prevent this by checking for dead clonaltypes before receiving them (no needed on actual robot implementation)
            if((*it_remotetcells).fE +(*it_remotetcells).fR <= CELLLOWERBOUND)
            {++it_remotetcells; robotAgent->IncNumberFloatingPtOperations(1); continue;}

            listTcells.insert(it_tcells, structTcell((*it_remotetcells).uFV,
                                                     (*it_remotetcells).fE * m_fTryExchangeProbability,
                                                     (*it_remotetcells).fR * m_fTryExchangeProbability,
                                                     (*it_remotetcells).uHistory,
                                                     NULL));
            (*it_remotetcells).fE -= (*it_remotetcells).fE * m_fTryExchangeProbability;
            (*it_remotetcells).fR -= (*it_remotetcells).fR * m_fTryExchangeProbability;
            ++it_remotetcells;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(6);
#endif
        }
        else
        {
            listRemoteTcells->insert(it_remotetcells, structTcell((*it_tcells).uFV,
                                                                  (*it_tcells).fE * m_fTryExchangeProbability,
                                                                  (*it_tcells).fR * m_fTryExchangeProbability,
                                                                  (*it_tcells).uHistory,
                                                                  NULL));
            (*it_tcells).fE -= (*it_tcells).fE * m_fTryExchangeProbability;
            (*it_tcells).fR -= (*it_tcells).fR * m_fTryExchangeProbability;
            ++it_tcells;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(6);
#endif
        }
    }

    while(it_tcells != masterlistTcells.end()) {
        listRemoteTcells->push_back(structTcell((*it_tcells).uFV,
                                                (*it_tcells).fE * m_fTryExchangeProbability,
                                                (*it_tcells).fR * m_fTryExchangeProbability,
                                                (*it_tcells).uHistory,
                                                NULL));
        (*it_tcells).fE -= (*it_tcells).fE * m_fTryExchangeProbability;
        (*it_tcells).fR -= (*it_tcells).fR * m_fTryExchangeProbability;
        ++it_tcells;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(6);
#endif
    }

    while(it_remotetcells != listRemoteTcells->end()) {
        if((*it_remotetcells).fE + (*it_remotetcells).fR <= CELLLOWERBOUND)
        {++it_remotetcells; robotAgent->IncNumberFloatingPtOperations(1); continue;}

        listTcells.push_back(structTcell((*it_remotetcells).uFV,
                                         (*it_remotetcells).fE * m_fTryExchangeProbability,
                                         (*it_remotetcells).fR * m_fTryExchangeProbability,
                                         (*it_remotetcells).uHistory,
                                         NULL));
        (*it_remotetcells).fE -= (*it_remotetcells).fE * m_fTryExchangeProbability;
        (*it_remotetcells).fR -= (*it_remotetcells).fR * m_fTryExchangeProbability;
        ++it_remotetcells;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(6);
#endif
    }

#ifdef DEBUGCROSSREGULATIONMODELFLAG
    printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
           CSimulator::GetInstance()->GetSimulationStepNumber(),
           robotAgent->GetIdentification(), pcRemoteRobotAgent->GetIdentification());
#endif

}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgentOptimised::GetIntegrationTime_StepFunction()
{
    unsigned long int persistencethreshold = (unsigned long int)m_uPersistenceThreshold;

    if(persistencethreshold == 0U)
    {
        //printf("\n Returned integration time is %f",m_fIntegrationTime);
        return m_fIntegrationTime;
    }

    //unsigned long int minhistory = 0; unsigned int fv;
    for(list<structTcell>::iterator it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        //if (((*it_tcells).uHistory <= persistencethreshold) && (GetAPC((*it_tcells).uFV) > 0.0))
        if (((*it_tcells).uHistory > 0 && (*it_tcells).uHistory <= persistencethreshold) && (GetAPC((*it_tcells).uFV) > 0.0))
        {
            //double reducedintegration_t = 1.0e+1;
            double reducedintegration_t = 0.0;
            m_fTCELL_UPPERLIMIT_STEPSIZE = 10.0;
            return reducedintegration_t;
            //minhistory = (*it_tcells).uHistory;
            //fv         = (*it_tcells).uFV;
        }
    }

    /*if(minhistory < persistencethreshold)
        return 1.0e+1;
    else*/

    return m_fIntegrationTime;
}

/******************************************************************************/
/******************************************************************************/

// Numerical integration to compute lisTcells  members fE and fR to reflect listApcs member fAPC
void CRMinRobotAgentOptimised::TcellNumericalIntegration_RK2()
{
    double convergence_errormax = -1.0, perc_convergence_errormax;
    double integration_t = 0.0;
    double step_h = 1.0;
    bool b_prevdiff0occurance = false;
    bool b_tcelldeath = false;
    list<structTcell>::iterator it_tcells;

    double integrationtimeofcrm;

    m_fTCELL_UPPERLIMIT_STEPSIZE = TCELL_UPPERLIMIT_STEPSIZE;
    //if(CSimulator::GetInstance()->GetSimulationStepNumber() < 0)//MODELSTARTTIME + 10)

#ifdef DISABLE_PERSISTENCE_HACK
    integrationtimeofcrm = m_fIntegrationTime;
#else
    if(CSimulator::GetInstance()->GetSimulationStepNumber() < MODELSTARTTIME + 10)
    { integrationtimeofcrm = m_fStartExpIntegrationTime; }
    else {
        integrationtimeofcrm = GetIntegrationTime_StepFunction();}
#endif


    while(integration_t < integrationtimeofcrm)
    {
        /*if(this->robotAgent->GetIdentification() == 8 && CSimulator::GetInstance()->GetSimulationStepNumber() == 5973)
        {
            {
                it_tcells = listTcells.begin();
                std::cout << std::endl << std::endl << " special last " << " (*it_tcells).fE " << (*it_tcells).fE << " (*it_tcells).fR " << (*it_tcells).fR;
            }
            printf("\n\n The integration time is NOW %f\n",integration_t);
            printf("\nConvergence error = %f (perc:%f)", m_dconvergence_error, m_dpercconvergence_error);
            unsigned PrntRobotId = robotAgent->GetIdentification();
            robotAgent->PrintFeatureVectorDistribution(PrntRobotId);
            PrintAPCList(PrntRobotId); PrintTcellList(PrntRobotId);
            PrintConjugatestoAPCList(PrntRobotId, CONJ);
            PrintConjugatestoTcellList(PrntRobotId, CONJ);
        }*/


        // Compute number of conjugates for T cells listTcells members fE and fR. Stores conjugates in listApcs member listConjugatesonAPC having member conjugate fConjugates
        if(FDMODELTYPE == CRM) //!TODO to avoid this check all the time, we could preprocess the code and define out the unused conjugate functions.
        {
            ConjugatesQSS(integration_t == 0.0, K0);
            // Compute derivative for T cells listTcells members fE and fR, and conjugates listApcs member listConjugatesonAPC having member conjugate fConjugates. // Stores derivative in fDeltaE_k0, fDeltaR_k0
            Derivative(K0);
        }
        else
        {
            if(listTcells.size()==0)
                printf("\n Integration time %f", integration_t);

            ConjugatesQSS_ExcessTcells(b_tcelldeath, K0);
            b_tcelldeath = false;
            Derivative_ExcessTcells(K0);
        }


        for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        {
            (*it_tcells).fE_Eu = (*it_tcells).fE + step_h * (*it_tcells).fDeltaE_k0;
            (*it_tcells).fR_Eu = (*it_tcells).fR + step_h * (*it_tcells).fDeltaR_k0;

            if((*it_tcells).fE_Eu < 0.0)
                (*it_tcells).fE_Eu = 0.0;

            if((*it_tcells).fR_Eu < 0.0)
                (*it_tcells).fR_Eu = 0.0;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(4);
#endif
        }

        /*if (integration_t > 2306903.0  && integration_t <= 2453140.0)
        {
            it_tcells = listTcells.begin();
            std::cout << std::endl << std::endl << " special " << " (*it_tcells).fE_Eu " << (*it_tcells).fE_Eu << " (*it_tcells).fR_Eu " << (*it_tcells).fR_Eu << " step_size " << step_h;
        }*/



        if(FDMODELTYPE == CRM)
        {
            // Compute number of conjugates for T cells listTcells members fE_Eu and fR_Eu. Stores conjugates in listApcs member listConjugatesonAPC having member conjugate fConjugates
            ConjugatesQSS(false, K1);
            // Compute derivative for T cells listTcells members fE_Eu and fR_Eu, and conjugates listApcs member listConjugatesonAPC having member conjugate fConjugates. Stores derivative in listTcells  members fDeltaE_k1, fDeltaR_k1
            Derivative(K1);
        }
        else
        {
            ConjugatesQSS_ExcessTcells(false, K1);
            Derivative_ExcessTcells(K1);
        }

        double absDiffHuenEuler = -1.0;
        for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        {
            (*it_tcells).fE_Hu = (*it_tcells).fE + 0.5 * step_h * ((*it_tcells).fDeltaE_k0 +
                                                                   (*it_tcells).fDeltaE_k1);
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(4);
#endif

            if((*it_tcells).fE_Hu < 0.0)
                (*it_tcells).fE_Hu = 0.0;
            else
            {
                double tmp_absdiff = fabs((*it_tcells).fE_Hu - (*it_tcells).fE_Eu);
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(2);
#endif
                if(tmp_absdiff > absDiffHuenEuler)
                    absDiffHuenEuler = tmp_absdiff;
            }

            (*it_tcells).fR_Hu = (*it_tcells).fR + 0.5 * step_h * ((*it_tcells).fDeltaR_k0 +
                                                                   (*it_tcells).fDeltaR_k1);
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(4);
#endif

            if((*it_tcells).fR_Hu < 0.0)
                (*it_tcells).fR_Hu = 0.0;
            else
            {
                double tmp_absdiff = fabs((*it_tcells).fR_Hu - (*it_tcells).fR_Eu);
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(2);
#endif
                if(tmp_absdiff > absDiffHuenEuler)
                    absDiffHuenEuler = tmp_absdiff;
            }
        }

        /*if (integration_t > 2306903.0  && integration_t <= 2453140.0)
        {
            it_tcells = listTcells.begin();
            std::cout << std::endl << std::endl << " special " << " (*it_tcells).fE_Hu " << (*it_tcells).fE_Hu << " (*it_tcells).fR_Hu " << (*it_tcells).fR_Hu << " step_size " << step_h << " absDiffHuenEuler " << absDiffHuenEuler;
        }*/

        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && step_h == m_fTCELL_LOWERLIMIT_STEPSIZE)
            {
                printf("\n The T-cell population solution is stalled");

                if(m_fERRORALLOWED_TCELL_STEPSIZE <= 1.0e-10)
                    break;
                else
                {
                    m_fERRORALLOWED_TCELL_STEPSIZE = m_fERRORALLOWED_TCELL_STEPSIZE/10.0;
                    m_fTCELL_CONVERGENCE = m_fTCELL_CONVERGENCE / 10.0;
                    b_prevdiff0occurance = false;

#ifdef FLOATINGPOINTOPERATIONS
                    robotAgent->IncNumberFloatingPtOperations(2);
#endif
                    continue;
                }
            }
            step_h = step_h / 2.0;

            if(step_h < m_fTCELL_LOWERLIMIT_STEPSIZE)
                step_h = m_fTCELL_LOWERLIMIT_STEPSIZE;

            printf("\n New stepsize %f - integration time %e",step_h,integration_t);
            b_prevdiff0occurance = true;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
            continue;
        }

        b_prevdiff0occurance = false;

        assert(absDiffHuenEuler >= 0.0);
        step_h *= sqrt(ERRORALLOWED_TCELL_STEPSIZE/absDiffHuenEuler);
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(3);
#endif

        if(step_h > m_fTCELL_UPPERLIMIT_STEPSIZE)
            step_h = m_fTCELL_UPPERLIMIT_STEPSIZE;
        else if(step_h < m_fTCELL_LOWERLIMIT_STEPSIZE)
            step_h = m_fTCELL_LOWERLIMIT_STEPSIZE;

        /*if (integration_t > 2306903.0  && integration_t <= 2453140.0)
        {
            it_tcells = listTcells.begin();
            std::cout << std::endl << std::endl << " special " << " (*it_tcells).fE_prev " << (*it_tcells).fE << " (*it_tcells).fDeltaE_k0 " << (*it_tcells).fDeltaE_k0 << " step_size " << step_h;
            std::cout << std::endl << std::endl << " special " << " (*it_tcells).fR_prev " << (*it_tcells).fR << " (*it_tcells).fDeltaR_k0 " << (*it_tcells).fDeltaR_k0 << " step_size " << step_h;
        }*/


        convergence_errormax = -1.0;
        it_tcells = listTcells.begin();
        while(it_tcells != listTcells.end()) // REMEMBER A FOR LOOP HERE WILL CAUSE RUNTIME ERRORS IF THE LAST T-CELL CLONAL POPULATION IS DELETED.
        {
            (*it_tcells).fE_prev = (*it_tcells).fE; (*it_tcells).fR_prev = (*it_tcells).fR;

            double tmp_incr = step_h * (*it_tcells).fDeltaE_k0;
            (*it_tcells).fE += tmp_incr;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(2);
#endif

            if((*it_tcells).fE < 0.0)
                (*it_tcells).fE = 0.0;
            else
            {
                double tmp_absincr = fabs(tmp_incr);
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(1);
#endif
                if(tmp_absincr > convergence_errormax)
                {
                    convergence_errormax      = tmp_absincr;
                    perc_convergence_errormax = (convergence_errormax / (*it_tcells).fE_prev) * 100.0;
#ifdef FLOATINGPOINTOPERATIONS
                    robotAgent->IncNumberFloatingPtOperations(2);
#endif
                }
            }

            tmp_incr = step_h * (*it_tcells).fDeltaR_k0;
            (*it_tcells).fR += tmp_incr;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(2);
#endif

            if((*it_tcells).fR < 0.0)
                (*it_tcells).fR = 0.0;
            else
            {
                double tmp_absincr = fabs(tmp_incr);
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(1);
#endif
                if(tmp_absincr > convergence_errormax)
                {
                    convergence_errormax      = tmp_absincr;
                    perc_convergence_errormax = (convergence_errormax / (*it_tcells).fR_prev) * 100.0;
#ifdef FLOATINGPOINTOPERATIONS
                    robotAgent->IncNumberFloatingPtOperations(2);
#endif
                }
            }

            /*if (integration_t > 2306903.0  && integration_t <= 2453140.0)
            {
                list<structTcell>::iterator it_tcells1 = listTcells.begin();
                std::cout << std::endl << std::endl << " special in loop" << " (*it_tcells1).fE " << (*it_tcells1).fE << " (*it_tcells1).fR " << (*it_tcells1).fR;
            }*/

            if(((*it_tcells).fE + (*it_tcells).fR) <= CELLLOWERBOUND)
            {
                if(FDMODELTYPE == CRM_TCELLSINEXCESS)
                    b_tcelldeath = true;
                MarkConjugatesOfDeadTcell(&it_tcells); //deleted later on when computing free stites and cells
                (*it_tcells).listPtrstoConjugatesofTcell.clear();

                /*if(this->robotAgent->GetIdentification() == 8 && CSimulator::GetInstance()->GetSimulationStepNumber() == 5973)
                {
                    //how is fv62 deleted before fv52. are they not in ascending order? ISSUE WAS WE WERE ERASING THE LAST ELEMENT OF THE LIST, ITERATING IN A FOR LOOP.
                    std::cout << " (*it_tcells).uFV is being deleted " << (*it_tcells).uFV << std::endl;
                    std::cout << " (*it_tcells).fE is being deleted " << (*it_tcells).fE << std::endl;
                    std::cout << " (*it_tcells).fR is being deleted " << (*it_tcells).fR << std::endl << std::endl << std::endl;
                }*/

                it_tcells = listTcells.erase(it_tcells);
                /*std::cout  << std::endl << " (*it_tcells).uFV is now pointed to" << (*it_tcells).uFV << std::endl;
                list<structTcell>::iterator it_tcells1 = listTcells.end();
                std::cout  << std::endl << " (listTcells.end()).uFV is now last pointed to" << (*it_tcells1).uFV << std::endl;*/

                /*if (integration_t > 2306903.0  && integration_t <= 2453140.0)
                {
                    list<structTcell>::iterator it_tcells1 = listTcells.begin();
                    std::cout << std::endl << std::endl << " special in death loop" << " (*it_tcells1).fE " << (*it_tcells1).fE << " (*it_tcells1).fR " << (*it_tcells1).fR;
                }*/
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(1);
#endif
                continue;
            }

            ++it_tcells;
        }

        /*if (integration_t > 2306903.0  && integration_t <= 2453140.0)
        {
            it_tcells = listTcells.begin();
            std::cout << std::endl << std::endl << " special " << " (*it_tcells).fE " << (*it_tcells).fE << " (*it_tcells).fR " << (*it_tcells).fR;
        }*/

        if(listTcells.size() == 0)
        {
            convergence_errormax = -1.0;
            integration_t = 0.0; step_h = 1.0;
            b_prevdiff0occurance = false; b_tcelldeath = false;

            // initialize T-cells and conjugates
            listTcells = listTcells_cpy;
            UpdateConjugatesToAPCList(); //O(m-apc *(~n-conj + n-tcell))
            UpdateConjugatesToTcellList();

            m_fTCELL_UPPERLIMIT_STEPSIZE = m_fTCELL_UPPERLIMIT_STEPSIZE/10.0;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
            continue;
        }

        /*if (integration_t > 2306903.0  && integration_t <= 2453140.0)
        {
            it_tcells = listTcells.begin();
            std::cout << std::endl << std::endl << " special " << " (*it_tcells).fE " << (*it_tcells).fE << " (*it_tcells).fR " << (*it_tcells).fR;
        }*/


        m_dconvergence_error     = convergence_errormax;
        m_dpercconvergence_error = perc_convergence_errormax;

        if(m_dpercconvergence_error <= 0.001)
            break;

        integration_t += step_h;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(1);
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::UpdateConjugatesToAPCList()
{
    // called when APC sub-populations added or removed
    // or when Tcell clonal types added or removed

    // the function can be greatly optimized if we can get access to the conjugate std::list on the apc from a pointer to an individual element (stored) on the tcell (n^² to n)
    // in that case updates to the list on t-cell death could be faster.
    // but when many new t-cells and apcs are added/removed at the start of a simulation step, this optimization would be less beneficial.

    list<structAPC>::iterator it_apcs;
    unsigned u_NumFpOperations;

    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        u_NumFpOperations = (*it_apcs).UpdateConjugateList(&listTcells, m_fcross_affinity);
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(u_NumFpOperations);
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::UpdateConjugatesToTcellList()
{
    // called when APC sub-populations added or removed
    // and when Tcell clonal types added .not needed to be called when a clonaltype dies since then pointers to conjugates will also be destroyed. in that case only UpdateConjugatetoAPCList() needs to be called.

    //!TODO the function is now actually recreating the list. if it only updated the list, we would save on memory reallocations
    list<structTcell>::iterator it_tcells; list<structAPC>::iterator it_apcs;
    list<structConj>::iterator it_conjs;

    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        (*it_tcells).listPtrstoConjugatesofTcell.clear();

    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        for(it_conjs = (*it_apcs).listConjugatesonAPC.begin();
            it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
        {
            structTcell* structPtrTcell = (*it_conjs).ptrTcell;
            structPtrTcell->listPtrstoConjugatesofTcell.push_back(&(*it_conjs));
        }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::MarkConjugatesOfDeadTcell(list<structTcell>::iterator* ptrit_tcells)
{
    list<structConj*>::iterator it_conjptr;
    for(it_conjptr = (*ptrit_tcells)->listPtrstoConjugatesofTcell.begin();
        it_conjptr != (*ptrit_tcells)->listPtrstoConjugatesofTcell.end(); ++it_conjptr)
    {
        (*it_conjptr)->deadconjugate = true;
        (*it_conjptr)->ptrTcell = nullptr; ////NEW
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::ConjugatesQSS(bool bResetConjugates, TcellIntegrationPhase K)
{
    list<structAPC>::iterator it_apcs; list<structConj>::iterator it_conjs;

    // check if t-cell pop is lower than conjuagtes
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        double fConjugatesOnAPC = 0.0;
        it_conjs = (*it_apcs).listConjugatesonAPC.begin();
        while(it_conjs != (*it_apcs).listConjugatesonAPC.end())
        {
            if((*it_conjs).deadconjugate) {
                it_conjs = (*it_apcs).listConjugatesonAPC.erase(it_conjs); continue;
            }

            fConjugatesOnAPC += (*it_conjs).GetConjugate(CONJ); ++it_conjs;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
        }

        if(fConjugatesOnAPC - (*it_apcs).fTotalSites  > CONJUGATION_OVERFLOW_LIMIT) {
            bResetConjugates = true; //printf("\nfConjugatesOnAPC %f - (*it_apcs).fTotalSites %f\n",fConjugatesOnAPC,(*it_apcs).fTotalSites);
            break; }
    }


    if(bResetConjugates) // conjugates are reset to 0 only when APC sub-population has changed, or if t-cell pop is lower than conjuagtes, if so reset
        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
                (*it_conjs).fConjugates = 0.0;


    conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;
    double error = 1.0;
    double conjintegration_t = 0.0;
    unsigned n_iteration = 0;
    bool b_prevdiff0occurance = false;
    while(error > CONJ_CONVERGENCE)
    {
        if(conjintegration_t > FAILSAFE_CONJ_INTEGRATION_TIME)
        {
            printf("\nATTENTION. The numerical integration of conjugates has undergone %d iterations. And yet the error is %f (CONJ_CONVERGENCE=%f). Breaking off now\n",n_iteration,error,CONJ_CONVERGENCE);
            break;
        }

        n_iteration++;

        FreeTcellsAndAvailableAPCSites(K, CONJ);

        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        {
            (*it_apcs).fTotalConjugates = 0.0;
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
            {
                (*it_conjs).fDelta_k0 = kon * (*it_conjs).affinity *
                        ((*it_conjs).ptrTcell)->fFreeTcells *
                        (*it_apcs).fAvailableSites -
                        koff * (*it_conjs).fConjugates;

                (*it_conjs).fConjugates_k0 = (*it_conjs).fConjugates +
                        conjstep_h * (*it_conjs).fDelta_k0;

                if((*it_conjs).fConjugates_k0 < 0.0)
                    (*it_conjs).fConjugates_k0 = 0.0;

                (*it_apcs).fTotalConjugates += (*it_conjs).fConjugates_k0;
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(8);
#endif
            }
        }

        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        ScaleDownConjugates(CONJ_K0);

        //update the number of free tcells and available binding sites - with euler approx of conjugates
        FreeTcellsAndAvailableAPCSites(K, CONJ_K0);

        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        {
            (*it_apcs).fTotalConjugates = 0.0;
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
            {
                (*it_conjs).fDelta_k1 = kon * (*it_conjs).affinity *
                        ((*it_conjs).ptrTcell)->fFreeTcells *
                        (*it_apcs).fAvailableSites -
                        koff * (*it_conjs).fConjugates_k0;

                (*it_conjs).fConjugates_k1 = (*it_conjs).fConjugates +
                        0.5 * conjstep_h * ((*it_conjs).fDelta_k0 +
                                            (*it_conjs).fDelta_k1);

                if((*it_conjs).fConjugates_k1 < 0.0)
                    (*it_conjs).fConjugates_k1 = 0.0;

                (*it_apcs).fTotalConjugates += (*it_conjs).fConjugates_k1;
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(10);
#endif
            }
        }

        ScaleDownConjugates(CONJ_K1);

        double absDiffHuenEuler = -1.0, tmp_absDiffHuenEuler;
        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
            {
                tmp_absDiffHuenEuler = fabs((*it_conjs).fConjugates_k1 -
                                            (*it_conjs).fConjugates_k0);

                if(tmp_absDiffHuenEuler > absDiffHuenEuler)
                    absDiffHuenEuler = tmp_absDiffHuenEuler;

#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(2);
#endif
            }

        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && conjstep_h == CONJ_LOWERLIMIT_STEPSIZE)
            {
                if(n_iteration == 2) // the numerical integration of conjugates has started out with a solution that has already converged.
                    break;
                else         {
                    printf("\n The Conjugation solution is stalled");
                    PrintConjugatestoAPCList(this->robotAgent->GetIdentification(), CONJ);
                    PrintConjugatestoAPCList(this->robotAgent->GetIdentification(), CONJ_K0);
                    PrintConjugatestoAPCList(this->robotAgent->GetIdentification(), CONJ_K1);
                    exit(-1);}
            }

            conjstep_h = conjstep_h / 2.0;

            if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE)
                conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;

            //printf("\n new conjugation step %f, n_iteration %u",conjstep_h,n_iteration);
            b_prevdiff0occurance = true;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
            continue;
        }

        b_prevdiff0occurance = false;

        assert(absDiffHuenEuler >= 0.0);

        if(conjintegration_t > REDUCESTEPSIZE_CONJ_INTEGRATION_TIME &&
                ERRORALLOWED_CONJ_STEPSIZE >= 1.0e-3) {
            /*The system is most likely stiff and oscillating around the "true" value, as the slope approaches 0*/
            /*we reduce the step size to reduce the difference between the oscillating values*/

            /*If the error allowed was lower, the corresponding step sizes would already be lower,
            and so would be the differences between oscillating values*/
            conjstep_h /= 2.0;

            if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE)
                conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
        }
        else {
            conjstep_h *= sqrt(ERRORALLOWED_CONJ_STEPSIZE/absDiffHuenEuler);
            if(conjstep_h > CONJ_UPPERLIMIT_STEPSIZE)
                conjstep_h = CONJ_UPPERLIMIT_STEPSIZE;
            else if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE)
                conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(3);
#endif
        }

        double error_max = -1.0;
        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        {
            (*it_apcs).fTotalConjugates = 0.0;
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
            {
                double tmp_incr = conjstep_h * (*it_conjs).fDelta_k0;
                (*it_conjs).fConjugates += tmp_incr ;
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(2);
#endif

                if((*it_conjs).fConjugates < 0.0)
                    (*it_conjs).fConjugates = 0.0;
                else {
                    double tmp_absincr = fabs(tmp_incr);
#ifdef FLOATINGPOINTOPERATIONS
                    robotAgent->IncNumberFloatingPtOperations(1);
#endif
                    if(tmp_absincr > error_max)
                        error_max = tmp_absincr;
                }

                (*it_apcs).fTotalConjugates += (*it_conjs).fConjugates;
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(1);
#endif
            }
        }

        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        ScaleDownConjugates(CONJ);

        conjintegration_t = conjintegration_t + conjstep_h;
        error = error_max;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(1);
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::ConjugatesQSS_ExcessTcells(bool bClearDeadConjugates, TcellIntegrationPhase TK)
{
    list<structAPC>::iterator  it_apcs; list<structConj>::iterator it_conj;

    //    if(listTcells.size() == 0)
    //    {
    //        printf("\n Robot id: %d",this->robotAgent->GetIdentification());
    //        printf("\n Simulation step: %d",CSimulator::GetInstance()->GetSimulationStepNumber());
    //    }

    if(bClearDeadConjugates)
        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        {
            it_conj = (*it_apcs).listConjugatesonAPC.begin();

            /*if((*it_conj).affinity == 0.0)
             {
                 printf("\n affinity is 0.0");
                 unsigned apcsize, tcellsize, conjtcellsize ;
                 apcsize   = listAPCs.size();
                 tcellsize = listTcells.size();
                 conjtcellsize = ((*it_apcs).listConjugatesonAPC).size();
             }*/

            while(it_conj != (*it_apcs).listConjugatesonAPC.end())
                if((*it_conj).deadconjugate)
                    it_conj = (*it_apcs).listConjugatesonAPC.erase(it_conj);
                else
                    ++it_conj;
        }

    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        (*it_apcs).f_tcellsweightedaffinity_tmp = 0.0;
        (*it_apcs).f_ecellsweightedaffinity_tmp = 0.0; (*it_apcs).f_rcellsweightedaffinity_tmp = 0.0;
        it_conj = (*it_apcs).listConjugatesonAPC.begin();


        if((*it_apcs).listConjugatesonAPC.size() == 0)
        {
            printf("\n conjtcellsize = listConjugatesonAPC.size();");
            unsigned apcsize, tcellsize, conjtcellsize ;
            apcsize   = listAPCs.size();
            tcellsize = listTcells.size();
            conjtcellsize = (*it_apcs).listConjugatesonAPC.size();

            std::cout << " apcsize, tcellsize, conjtcellsize (*it_apcs).fTotalSites  (*it_apcs).fAPC  (*it_apcs).uFV:   " << apcsize << "    " << tcellsize << "    " <<  conjtcellsize << "    " <<  (*it_apcs).fTotalSites << "    " <<    (*it_apcs).fAPC << "    " <<  (*it_apcs).uFV << std::endl << std::endl;
            PrintCRMDetails(this->robotAgent->GetIdentification());
        }

        while(it_conj != (*it_apcs).listConjugatesonAPC.end())
        {
            //assert((*it_conj).affinity == NegExpDistAffinity((*it_conj).ptrTcell->uFV, (*it_apcs).uFV, m_fcross_affinity));

            (*it_apcs).f_tcellsweightedaffinity_tmp +=
                    (*it_conj).affinity * ((*it_conj).ptrTcell->GetE(TK) +
                                           (*it_conj).ptrTcell->GetR(TK));

            (*it_apcs).f_ecellsweightedaffinity_tmp +=
                    (*it_conj).affinity * (*it_conj).ptrTcell->GetE(TK);

            (*it_apcs).f_rcellsweightedaffinity_tmp +=
                    (*it_conj).affinity * (*it_conj).ptrTcell->GetR(TK);

            ++it_conj;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(7);
#endif
        }

        if((*it_apcs).f_tcellsweightedaffinity_tmp == 0.0)
        {
            printf("\n (*it_apcs).f_tcellsweightedaffinity_tmp == 0.0;");
            unsigned apcsize, tcellsize, conjtcellsize ;
            apcsize   = listAPCs.size();
            tcellsize = listTcells.size();
            conjtcellsize = ((*it_apcs).listConjugatesonAPC).size();

            std::cout << " apcsize, tcellsize, conjtcellsize (*it_apcs).fTotalSites  (*it_apcs).fAPC  (*it_apcs).uFV:   " << apcsize << "    " << tcellsize << "    " <<  conjtcellsize << "    " <<  (*it_apcs).fTotalSites << "    " <<    (*it_apcs).fAPC << "    " <<  (*it_apcs).uFV << std::endl << std::endl;
            PrintCRMDetails(this->robotAgent->GetIdentification());
        }


        (*it_apcs).fTotalConjugates = ((*it_apcs).fTotalSites *
                                       (*it_apcs).f_tcellsweightedaffinity_tmp) /
                                       ((*it_apcs).f_tcellsweightedaffinity_tmp + 1.0);
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(3);
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::Derivative(TcellIntegrationPhase TK)
{
    list<structTcell>::iterator it_tcells;    list<structAPC>::iterator   it_apcs;
    list<structConj>::iterator  it_conj;      list<structConj*>::iterator it_conjptr;

    // Dividing the conjugates into Effector and Regulator type
    double tmp_totalcells;
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        tmp_totalcells = (*it_tcells).GetE(TK) + (*it_tcells).GetR(TK);
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(1);
#endif

        if(tmp_totalcells == 0.0) //t-cells at 0 (euler or huen - not final rk2 value)
        {
            for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
                it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)
            {
                (*it_conjptr)->fEffectorConjugates  = 0.0; (*it_conjptr)->fRegulatorConjugates = 0.0;
            }
        }
        else
        {
            for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
                it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)
            {
                (*it_conjptr)->fEffectorConjugates  = ((*it_tcells).GetE(TK) / tmp_totalcells) *
                        (*it_conjptr)->fConjugates;
                (*it_conjptr)->fRegulatorConjugates = ((*it_tcells).GetR(TK) / tmp_totalcells) *
                        (*it_conjptr)->fConjugates;
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(4);
#endif
            }
        }
    }

    // Computing the total number of effector and regulator conjugates per APC
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        (*it_apcs).fEffectorConjugatesPerAPC  = 0.0; (*it_apcs).fRegulatorConjugatesPerAPC = 0.0;
        for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
            it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)
        {
            (*it_apcs).fEffectorConjugatesPerAPC  += (*it_conj).fEffectorConjugates;
            (*it_apcs).fRegulatorConjugatesPerAPC += (*it_conj).fRegulatorConjugates;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(2);
#endif
        }

        assert(((*it_apcs).fEffectorConjugatesPerAPC + (*it_apcs).fRegulatorConjugatesPerAPC -
                (*it_apcs).fTotalSites) <= CONJUGATION_OVERFLOW_LIMIT);
    }

    ComputeNewDerivative(TK);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::Derivative_ExcessTcells(TcellIntegrationPhase TK)
{
    list<structAPC>::iterator  it_apcs; list<structConj>::iterator it_conj;
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        double tmp_ratio = (*it_apcs).fTotalConjugates / (*it_apcs).f_tcellsweightedaffinity_tmp;
        (*it_apcs).fEffectorConjugatesPerAPC  = tmp_ratio * (*it_apcs).f_ecellsweightedaffinity_tmp;
        (*it_apcs).fRegulatorConjugatesPerAPC = tmp_ratio * (*it_apcs).f_rcellsweightedaffinity_tmp;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(3);
#endif

        if(!(((*it_apcs).fEffectorConjugatesPerAPC +
              (*it_apcs).fRegulatorConjugatesPerAPC -
              (*it_apcs).fTotalSites) <= CONJUGATION_OVERFLOW_LIMIT))
        {
            std::cout << std::endl << std::endl << " tmp_ratio " << tmp_ratio << " (*it_apcs).fTotalConjugates " << (*it_apcs).fTotalConjugates << " (*it_apcs).f_tcellsweightedaffinity_tmp " << (*it_apcs).f_tcellsweightedaffinity_tmp << std::endl<< std::endl<< std::endl;

            std::cout << std::endl << " m_fTCELL_UPPERLIMIT_STEPSIZE  " << m_fTCELL_UPPERLIMIT_STEPSIZE  << " m_fTCELL_LOWERLIMIT_STEPSIZE " << m_fTCELL_LOWERLIMIT_STEPSIZE << " m_fERRORALLOWED_TCELL_STEPSIZE " << m_fERRORALLOWED_TCELL_STEPSIZE << " m_fTCELL_CONVERGENCE " << m_fTCELL_CONVERGENCE << std::endl<< std::endl<< std::endl;
        }
        assert((*it_apcs).fEffectorConjugatesPerAPC +
               (*it_apcs).fRegulatorConjugatesPerAPC -
               (*it_apcs).fTotalSites <= CONJUGATION_OVERFLOW_LIMIT);



        it_conj = (*it_apcs).listConjugatesonAPC.begin();
        while(it_conj != (*it_apcs).listConjugatesonAPC.end())
        {
            double totaltcellcount = (*it_conj).ptrTcell->GetE(TK) + (*it_conj).ptrTcell->GetR(TK);
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif

            if(totaltcellcount <= CELLLOWERBOUND)
            {
                (*it_conj).fEffectorConjugates  = 0.0; (*it_conj).fRegulatorConjugates = 0.0;
                (*it_conj).fConjugates          = 0.0;
            }
            else
            {
                (*it_conj).fConjugates = tmp_ratio * (*it_conj).affinity * totaltcellcount;

                (*it_conj).fEffectorConjugates  = ((*it_conj).ptrTcell->GetE(TK)/totaltcellcount) *
                        (*it_conj).fConjugates;
                (*it_conj).fRegulatorConjugates = ((*it_conj).ptrTcell->GetR(TK)/totaltcellcount) *
                        (*it_conj).fConjugates;
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(6);
#endif
            }
            ++it_conj;
        }
    }

    ComputeNewDerivative(TK);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::ComputeNewDerivative(TcellIntegrationPhase TK)
{
    // We now compute the derivative of each T-cell clonaltype, based on their old values, proliferation and cell death, and thymic generation of new cells
    list<structTcell>::iterator it_tcells;    list<structConj*>::iterator it_conjptr;

    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        double effector_incr = 0.0, regulator_incr = 0.0;
        for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
            it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)
        {
            structAPC* tmp_apcptr = (*it_conjptr)->ptrAPC;
            double tmp_apcconc  = tmp_apcptr->fAPC;

            double tmp_exp1 = 9.0 * tmp_apcconc * tmp_apcconc;
            double tmp_exp2 = (tmp_apcptr->fRegulatorConjugatesPerAPC - 3.0 * tmp_apcconc) *
                    (tmp_apcptr->fRegulatorConjugatesPerAPC - 3.0 * tmp_apcconc);
            double Pe = tmp_exp2 / tmp_exp1;

            effector_incr  += kpe * Pe * (*it_conjptr)->fEffectorConjugates;

            double Pr = (6.0 * tmp_apcconc - tmp_apcptr->fEffectorConjugatesPerAPC) *
                    tmp_apcptr->fEffectorConjugatesPerAPC / tmp_exp1;
            regulator_incr += kpr * Pr * (*it_conjptr)->fRegulatorConjugates;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(18);
#endif
        }

#ifndef SELECTIVE_TCELL_INFLUX_RATE
        effector_incr  += se_rate - kde * (*it_tcells).GetE(TK);
        regulator_incr += sr_rate - kdr * (*it_tcells).GetR(TK);
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(6);
#endif
#else

        if(GetAPC((*it_tcells).uFV) == 0) //no apcs
        {
            effector_incr  += -kde * (*it_tcells).GetE(TK);
            regulator_incr += -kdr * (*it_tcells).GetR(TK);
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(4);
#endif
        }
        else
        {
            effector_incr  += se_rate - kde * (*it_tcells).GetE(TK);
            regulator_incr += sr_rate - kdr * (*it_tcells).GetR(TK);
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(6);
#endif
        }
#endif

        (*it_tcells).SetDelta(TK, effector_incr, regulator_incr);

    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::FreeTcellsAndAvailableAPCSites(TcellIntegrationPhase TK, ConjugationIntegrationPhase CONJK)
{
    list<structAPC>::iterator  it_apcs;     list<structTcell>::iterator it_tcells;
    list<structConj>::iterator it_conj;     list<structConj*>::iterator it_conjptr;
    double tmp_freesites;

    // available binding sites
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        double fConjugatesOnAPC = 0.0;
        it_conj = (*it_apcs).listConjugatesonAPC.begin();
        while(it_conj != (*it_apcs).listConjugatesonAPC.end())
        {
            if((*it_conj).deadconjugate) {
                it_conj = (*it_apcs).listConjugatesonAPC.erase(it_conj); continue;
            }

            fConjugatesOnAPC += (*it_conj).GetConjugate(CONJK); ++it_conj;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
        }

        tmp_freesites  = -fConjugatesOnAPC + (*it_apcs).fTotalSites;
        assert(-tmp_freesites <= CONJUGATION_OVERFLOW_LIMIT);
        (*it_apcs).fAvailableSites = tmp_freesites;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(1);
#endif
    }

    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        double fConjugatesOfTcell = 0.0;
        for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
            it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)
        {
            fConjugatesOfTcell += (*it_conjptr)->GetConjugate(CONJK);
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
        }
        (*it_tcells).fFreeTcells = (*it_tcells).GetE(TK) + (*it_tcells).GetR(TK) -
                fConjugatesOfTcell;

        //we scale down the conjugates factored based on their initial values if the error is not too high. the conjugation error is ERRORALLOWED_CONJ_STEPSIZE which is orders of magniture less than the CONJUGATION_OVERFLOW_LIMIT
        if(-((*it_tcells).fFreeTcells) > CONJUGATION_OVERFLOW_LIMIT)
        {
            for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
                it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)
            {
                (*it_conjptr)->SetConjugate(CONJK,
                                            (*it_conjptr)->GetConjugate(CONJK)/fConjugatesOfTcell *
                                            ((*it_tcells).GetE(TK) + (*it_tcells).GetR(TK)));
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(3);
#endif
            }
        }
        else
            assert(-((*it_tcells).fFreeTcells) <= ERRORALLOWED_CONJ_STEPSIZE);


#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(2);
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintCRMDetails(unsigned id)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    robotAgent->PrintFeatureVectorDistribution(id);

    PrintAPCList(id);
    PrintTcellList(id);
    PrintConjugatestoAPCList(id, CONJ);
    PrintConjugatestoTcellList(id, CONJ);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintAPCList(unsigned int id)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structAPC>::iterator it_apcs;
    printf("\n==========R%d APCs list================\n", robotAgent->GetIdentification());
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        printf("APC[%d]=%f  ", (*it_apcs).uFV, (*it_apcs).fAPC);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintTcellResponseToAPCList(unsigned int id)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structAPC>::iterator it_apcs;
    printf("\n==========R%d APCs list================\n", robotAgent->GetIdentification());
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        printf("APC[%d]=%f [wtsumE]:%f [wtsumR]:%f ", (*it_apcs).uFV, (*it_apcs).fAPC,
               (*it_apcs).fE_weightedbyaffinity, (*it_apcs).fR_weightedbyaffinity);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintTcellList(unsigned int id)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structTcell>::iterator it_tcells;
    printf("\n===========R%d T cell list==============\n", robotAgent->GetIdentification());
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        printf("E[%d]=%e,R[%d]=%e (A=%f) [History=%lu]  ", (*it_tcells).uFV, (*it_tcells).fE,
               (*it_tcells).uFV, (*it_tcells).fR, GetAPC((*it_tcells).uFV), (*it_tcells).uHistory);

    //!TODO: Solve the bug causing the below stmt in the printf function to print nonsensical A densities
    //!TODO:  ((*it_tcells).ptrAPCWithAffinity1) == NULL ?
    //            GetAPC((*it_tcells).uFV) : ((*it_tcells).ptrAPCWithAffinity1)->fAPC)
    //    printf("E[%d]=%e,R[%d]=%e (A=%f)   ", (*it_tcells).uFV, (*it_tcells).fE,
    //            (*it_tcells).uFV, (*it_tcells).fR,
    //            ((*it_tcells).ptrAPCWithAffinity1) == NULL ?
    //            GetAPC((*it_tcells).uFV) : ((*it_tcells).ptrAPCWithAffinity1)->fAPC);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintConjugatestoAPCList(unsigned int id, ConjugationIntegrationPhase CONJK)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structAPC>::iterator it_apcs; list<structConj>::iterator it_conj;
    printf("\n===========R%d Conjugates_%d to APCs=======\n", robotAgent->GetIdentification(), CONJK);
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        printf("\nAPC:%d\t",(*it_apcs).uFV);
        for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
            it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)
        {
            if((*it_conj).deadconjugate)
                continue;
            printf("Tcell:%d %e  ",((*it_conj).ptrTcell)->uFV, (*it_conj).GetConjugate(CONJK));
        }

    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintConjugatestoTcellList(unsigned int id, ConjugationIntegrationPhase CONJK)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structTcell>::iterator it_tcells; list<structConj*>::iterator it_conjptr;
    printf("\n===========R%d Conjugates_%d to Tcells=======\n", robotAgent->GetIdentification(), CONJK);
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        printf("\nTcell:%d\t",(*it_tcells).uFV);
        for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
            it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)
        {
            if((*it_conjptr)->deadconjugate)
                continue;
            printf("APC:%d %e  ",((*it_conjptr)->ptrAPC)->uFV, (*it_conjptr)->GetConjugate(CONJK));
        }
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::UpdateState()
{
    unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();

    double tmp_E, tmp_R, tmp_affinity;
    list<structAPC>::iterator it_apcs = listAPCs.begin();
    list<structTcell>::iterator it_tcells;

    list<structFVsSensed>* fvsensed = robotAgent->GetFeatureVectorsSensed();
    list<structFVsSensed>::iterator it_fvsensed = fvsensed->begin();

    while(it_apcs != listAPCs.end())
    {
        assert((*it_fvsensed).uFV == (*it_apcs).uFV);

        tmp_E = 0.0; tmp_R = 0.0;
        for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        {
            tmp_affinity = NegExpDistAffinity((*it_tcells).uFV, (*it_apcs).uFV, m_fcross_affinity);
            tmp_E += tmp_affinity * (*it_tcells).fE;
            tmp_R += tmp_affinity * (*it_tcells).fR;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(4+3); //3 operations in NegExpDistAffinity
#endif
        }

        (*it_apcs).fE_weightedbyaffinity = tmp_E;
        (*it_apcs).fR_weightedbyaffinity = tmp_R;

        if ((tmp_E + tmp_R) <= CELLLOWERBOUND || fabs(tmp_E - tmp_R) <= CELLLOWERBOUND)
        {
            //Dont know - no T-cells to make decision or E approx. equal to R
            robotAgent->SetMostWantedList(&it_fvsensed, 0);
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(3);
#endif
        }

        else if (tmp_E > tmp_R) // (tmp_E/tmp_R > 1.0)
            //else if (tmp_E/tmp_R > 0.9)// > 0.95 // (tmp_E/tmp_R > 1.0)
            // Attack
        {
            robotAgent->SetMostWantedList(&it_fvsensed, 1);
#ifndef CRM_ENABLE_SENSORY_HISTORY
            if (m_uHistoryTcells > 0)
            {
                double suspicioncounter = 0.0, totalcounter = 0.0;
                double tolerantcounter = 0.0, attackcounter = 0.0, dontknowcounter = 0.0;

                list< list<structTcell> >::iterator pop_index;
                int tmp_index = 0; bool fv_present = false;
                for(pop_index = listlistTcells.begin(); pop_index != listlistTcells.end(); ++pop_index)
                {
                    tmp_index++;
                    double tmp_E1, tmp_R1, tmp_affinity1;
                    tmp_E1 = 0.0; tmp_R1 = 0.0;
                    list<structTcell>::iterator it_tcells1;
                    for(it_tcells1 =  (*pop_index).begin();
                        it_tcells1 != (*pop_index).end(); ++it_tcells1)
                    {
                        tmp_affinity1 = NegExpDistAffinity((*it_tcells1).uFV, (*it_apcs).uFV, m_fcross_affinity);
                        tmp_E1 += tmp_affinity1 * (*it_tcells1).fE;
                        tmp_R1 += tmp_affinity1 * (*it_tcells1).fR;

                        unsigned int hammingdistance  = CRMinRobotAgentOptimised::GetNumberOfSetBits((*it_apcs).uFV ^ (*it_tcells1).uFV);
                        if ((double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES < 2.0/6.0)
                        //if((*it_apcs).uFV == (*it_tcells1).uFV)
                            fv_present = true;

#ifdef FLOATINGPOINTOPERATIONS
                        robotAgent->IncNumberFloatingPtOperations(4+3); //3 operations in NegExpDistAffinity
#endif
                    }

                    if(fv_present == false)
                        //suspicioncounter +=1.0;
                        suspicioncounter += m_fNewFVSuspicionIncr; // 1.0/10.0; //effect of taking different sub-unit values?
                    else if (tmp_E1 > tmp_R1)
                        suspicioncounter +=1.0;

                    //                    if (this->robotAgent->GetIdentification() == 7 && CSimulator::GetInstance()->GetSimulationStepNumber() == 7650)
                    //                    {
                    //                        // std::cerr << tmp_index << ": FV " << (*it_apcs).uFV <<  " suspicioncounter " << suspicioncounter << std::endl;
                    //                        double tmp_E1, tmp_R1, tmp_affinity1;
                    //                        tmp_E1 = 0.0; tmp_R1 = 0.0;
                    //                        for(it_tcells1 =  (*pop_index).begin();
                    //                            it_tcells1 != (*pop_index).end(); ++it_tcells1)
                    //                        {
                    //                            tmp_affinity1 = NegExpDistAffinity((*it_tcells1).uFV, (*it_apcs).uFV, m_fcross_affinity);
                    //                            //std::cerr << "E[" << (*it_tcells1).uFV << "] = " << (*it_tcells1).fE << "R[" << (*it_tcells1).uFV << "] = " << (*it_tcells1).fR << "H[" << (*it_tcells1).uFV << "] = " <<  (*it_tcells1).uHistory  <<  "; ";

                    //                            tmp_E1 += tmp_affinity1 * (*it_tcells1).fE;
                    //                            tmp_R1 += tmp_affinity1 * (*it_tcells1).fR;
                    //                        }
                    //                        //std::cerr << "Total E = " << tmp_E1 << "Total R = " << tmp_R1;
                    //                        //std::cerr << std::endl  << std::endl;
                    //                    }
                }


                //                if (this->robotAgent->GetIdentification() == 7 && CSimulator::GetInstance()->GetSimulationStepNumber() == 7650)
                //                {
                //                    std::cerr << " FV " << (*it_apcs).uFV <<  " suspicioncounter " << suspicioncounter << std::endl;

                //                    //exit(-1);
                //                }

                //                Exp set up 2
                //Step 7650
                //                ====R7 Feature Vector Distribution=====
                //                FV:12, Robots:1.000000, Suspicion:0.986000 FV:54, Robots:2.000000, Suspicion:0.000000 FV:60, Robots:1.000000, Suspicion:0.000000 FV:62, Robots:6.000000, Suspicion:0.000000

                //                FOR FV:12 how many dont knows, how many attack, and how many tolerate? dont knows may have to contribute to raise suspicion

                suspicioncounter = suspicioncounter / listlistTcells.size();

                //if(this->robotAgent->GetIdentification() == 9)
                   //std::cerr << "\n\nsuspicioncounter  =  " << suspicioncounter;

                //suspicioncounter = suspicioncounter / totalcounter;
                robotAgent->SetSuspicion(&it_fvsensed, suspicioncounter); // set suspicion
                if (suspicioncounter < m_fSuspicionThreshold)
                    robotAgent->SetMostWantedList(&it_fvsensed, 4); // deemed suspicious - but not abnormal; else deemed abnormal.




                // using the master t-cell list
                /*double tmp_E1, tmp_R1, tmp_affinity1;
                tmp_E1 = 0.0; tmp_R1 = 0.0;
                list<structTcell>::iterator it_mastertcellslist;
                for(it_mastertcellslist =  masterlistTcells.begin();
                    it_mastertcellslist != masterlistTcells.end(); ++it_mastertcellslist)
                {
                    tmp_affinity1 = NegExpDistAffinity((*it_mastertcellslist).uFV, (*it_apcs).uFV, m_fcross_affinity);
                    tmp_E1 += tmp_affinity1 * (*it_mastertcellslist).fE;
                    tmp_R1 += tmp_affinity1 * (*it_mastertcellslist).fR;
#ifdef FLOATINGPOINTOPERATIONS
                    robotAgent->IncNumberFloatingPtOperations(4+3); //3 operations in NegExpDistAffinity
#endif
                }

                if ((tmp_E1 + tmp_R1) <= CELLLOWERBOUND || fabs(tmp_E1 - tmp_R1) <= CELLLOWERBOUND)
                { }
                else if (tmp_E1 > tmp_R1)
                    robotAgent->SetMostWantedList(&it_fvsensed, 1); // deemed abnormal
                else
                    robotAgent->SetMostWantedList(&it_fvsensed, 4); // suspicion

                robotAgent->SetSuspicion(&it_fvsensed, tmp_E1-tmp_R1); */
            }
#endif
        }
        else
        {
            // Tolerate
            robotAgent->SetMostWantedList(&it_fvsensed, 2);
        }
        ++it_apcs; ++it_fvsensed;
    }

#ifndef CRM_ENABLE_SENSORY_HISTORY
    if (m_uHistoryTcells > 0)
    {
        if(CurrentStepNumber < MODELSTARTTIME + m_uHistoryTcells + 1)
        {
            listlistTcells.push_back(listTcells);
            //MergeIntoMasterTcellList(listTcells, +1);
        }
        else
        {
            listlistTcells.push_back(listTcells);
            //MergeIntoMasterTcellList(listTcells, +1);

            //MergeIntoMasterTcellList(listlistTcells.front(), -1);
            listlistTcells.pop_front();
        }
    }
#endif
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::UpdateAPCList()
{
    list<structFVsSensed>* fvsensed = robotAgent->GetFeatureVectorsSensed();
    list<structFVsSensed>::iterator it_fvsensed;
    list<structAPC>::iterator it_apcs;

    it_fvsensed = fvsensed->begin(); it_apcs = listAPCs.begin();
    while(it_apcs != listAPCs.end() && it_fvsensed != fvsensed->end())
    {
        if((*it_fvsensed).uFV == (*it_apcs).uFV)
        {
            if(m_fApcscalingtype == 0)
                (*it_apcs).fAPC = (*it_fvsensed).fRobots * m_fFVtoApcscaling;
            else // (*it_fvsensed).fRobots >= 0
                (*it_apcs).fAPC = m_fFVtoApcscaling_expbase * exp(((*it_fvsensed).fRobots - 1) *
                                                                  m_fFVtoApcscaling_exprate);

            (*it_apcs).fTotalSites = (*it_apcs).fAPC * (double)sites;

            (*it_apcs).fEffectorConjugatesPerAPC = 0.0; (*it_apcs).fRegulatorConjugatesPerAPC = 0.0;
            (*it_apcs).fE_weightedbyaffinity = 0.0;     (*it_apcs).fR_weightedbyaffinity = 0.0;
            ++it_apcs; ++it_fvsensed;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(2 + m_fApcscalingtype*2);
#endif
            continue;
        }

        if((*it_apcs).uFV > (*it_fvsensed).uFV)
        {
            if(m_fApcscalingtype == 0)
                listAPCs.insert(it_apcs, structAPC((*it_fvsensed).uFV,
                                                   (*it_fvsensed).fRobots * m_fFVtoApcscaling,
                                                   (double)sites));
            else // (*it_fvsensed).fRobots >= 1
                listAPCs.insert(it_apcs, structAPC((*it_fvsensed).uFV,
                                                   m_fFVtoApcscaling_expbase * exp(((*it_fvsensed).fRobots - 1)*
                                                                                   m_fFVtoApcscaling_exprate),
                                                   (double)sites));
            ++it_fvsensed;

#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(2 + m_fApcscalingtype*2); // also a multiplication of apcs*sites in constructor of structAPC
#endif
            continue;
        }
        (*it_apcs).listConjugatesonAPC.clear();
        it_apcs = listAPCs.erase(it_apcs);
    }

    while(it_apcs != listAPCs.end()) {
        (*it_apcs).listConjugatesonAPC.clear();
        it_apcs = listAPCs.erase(it_apcs);}

    while(it_fvsensed != fvsensed->end()) {
        if(m_fApcscalingtype == 0)
            listAPCs.push_back(structAPC((*it_fvsensed).uFV,
                                         (*it_fvsensed).fRobots * m_fFVtoApcscaling, (double)sites));
        else // (*it_fvsensed).fRobots >= 1
            listAPCs.push_back(structAPC((*it_fvsensed).uFV,
                                         m_fFVtoApcscaling_expbase * exp(((*it_fvsensed).fRobots - 1)*
                                                                         m_fFVtoApcscaling_exprate),
                                         (double)sites));
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(2 + m_fApcscalingtype*2);  // also a multiplication of apcs*sites in constructor of structAPC
#endif
        ++it_fvsensed; }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::UpdateTcellList(unsigned hammingdistance)
{
    //hammingdistance = 0;
    //argument hammingdistance: At the start of the simulation, injects new t-cells of receptors within hammingdistance of present apcs
    // during each of the following simulation time-steps hammingdistance is passed as 0

    list<structAPC>::iterator   it_apcs;
    list<structTcell>::iterator it_tcells;

    if(CSimulator::GetInstance()->GetSimulationStepNumber() == (MODELSTARTTIME+1))
    {
        it_tcells = listTcells.begin();

        if(hammingdistance == CFeatureVector::NUMBER_OF_FEATURES)
        {
            it_apcs = listAPCs.begin();
            for(unsigned int index_tcells = 0; index_tcells < m_unNumberOfReceptors; ++index_tcells)
            {
                if((*it_apcs).uFV == index_tcells) {
                    listTcells.push_back(structTcell(index_tcells, seedE, seedR, 0, &(*it_apcs)));
                    ++it_apcs;}
                else
                    listTcells.push_back(structTcell(index_tcells, seedE, seedR, 0, NULL));
            }
            return;
        }

        if(hammingdistance == 0)
        {
            for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
                listTcells.push_back(structTcell((*it_apcs).uFV, seedE, seedR, 0, &(*it_apcs)));

            return;
        }
        printf("Other hamming dist to be coded."); exit(-1);
    }

    it_apcs = listAPCs.begin(); it_tcells = listTcells.begin();
    while(it_tcells != listTcells.end() && it_apcs != listAPCs.end())
    {
        if((*it_apcs).uFV == (*it_tcells).uFV)
        {
#ifndef SELECTIVE_TCELL_INFLUX_DENSITY
            (*it_tcells).fE += se; (*it_tcells).fR += sr;
#endif
            if((*it_tcells).ptrAPCWithAffinity1 == NULL)
                (*it_tcells).ptrAPCWithAffinity1 = &(*it_apcs);

            (*it_tcells).uHistory++;

            ++it_tcells; ++it_apcs;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(2);
#endif
            continue;
        }

        if((*it_tcells).uFV > (*it_apcs).uFV)
        {
            listTcells.insert(it_tcells, structTcell((*it_apcs).uFV, se, sr, 0, &(*it_apcs)));
            ++it_apcs;
            continue;
        }

        //check if tcell clonaltype is dead - an infusion of an already low number of t cells after calling the Diffusion function
        double tmp_totalcellcount = (*it_tcells).fE + (*it_tcells).fR;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(1);
#endif
        if(tmp_totalcellcount <= CELLLOWERBOUND) {
            (*it_tcells).listPtrstoConjugatesofTcell.clear();
            it_tcells = listTcells.erase(it_tcells);
            continue;
        }

        (*it_tcells).uHistory++;

        (*it_tcells).ptrAPCWithAffinity1 = NULL; //tcell with no apc having the same fv
        ++it_tcells;
    }

    while(it_tcells != listTcells.end())
    {
        double tmp_totalcellcount = (*it_tcells).fE + (*it_tcells).fR;
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(1);
#endif
        if(tmp_totalcellcount <= CELLLOWERBOUND) {
            (*it_tcells).listPtrstoConjugatesofTcell.clear(); it_tcells = listTcells.erase(it_tcells);
            continue;
        }
        ++it_tcells;
    }

    while(it_apcs != listAPCs.end()) {
        listTcells.push_back(structTcell((*it_apcs).uFV, se, sr, 0, &(*it_apcs)));
        ++it_apcs; }

    listTcells_cpy.clear();
    listTcells_cpy = listTcells;

    /*if(this->robotAgent->GetIdentification() == 8 && CSimulator::GetInstance()->GetSimulationStepNumber() == 5973)
    {
        assert(listTcells.size() == 26);
        it_tcells = listTcells.begin();
        (*it_tcells).fE=1.013573e+01; (*it_tcells).fR=1.239414e+01;++it_tcells;
        (*it_tcells).fE=1.004220e+01; (*it_tcells).fR=1.072882e+01;++it_tcells;
        (*it_tcells).fE=1.019084e+01; (*it_tcells).fR=1.199276e+01;++it_tcells;
        (*it_tcells).fE=1.021920e+01; (*it_tcells).fR=1.024296e+01;++it_tcells;
        (*it_tcells).fE=1.042126e+01; (*it_tcells).fR=1.017626e+01;++it_tcells;
        (*it_tcells).fE=1.008367e+01; (*it_tcells).fR=1.032832e+01;++it_tcells;
        (*it_tcells).fE=1.043156e+01; (*it_tcells).fR=1.063774e+01;++it_tcells;
        (*it_tcells).fE=1.078326e+01; (*it_tcells).fR=1.032383e+01;++it_tcells;
        (*it_tcells).fE=1.009964e-01; (*it_tcells).fR=2.587830e-03;++it_tcells;
        (*it_tcells).fE=2.128364e-02; (*it_tcells).fR=3.396210e-04;++it_tcells;
        (*it_tcells).fE=3.128210e-01; (*it_tcells).fR=3.589119e-03;++it_tcells;
        (*it_tcells).fE=1.035613e+01; (*it_tcells).fR=1.083624e+01;++it_tcells;
        (*it_tcells).fE=4.432948e-02; (*it_tcells).fR=4.569674e-02;++it_tcells;
        (*it_tcells).fE=1.435718e-01; (*it_tcells).fR=3.467479e-01;++it_tcells;
        (*it_tcells).fE=1.816949e-01; (*it_tcells).fR=2.945318e-02;++it_tcells;
        (*it_tcells).fE=3.128492e-01; (*it_tcells).fR=1.021297e-02;++it_tcells;
        (*it_tcells).fE=1.258025e+00; (*it_tcells).fR=9.787533e-03;++it_tcells;
        (*it_tcells).fE=1.608477e+00; (*it_tcells).fR=1.172626e-02;++it_tcells;
        (*it_tcells).fE=1.254773e+00; (*it_tcells).fR=5.136244e-07;++it_tcells;
        (*it_tcells).fE=7.813555e-02; (*it_tcells).fR=3.647662e-08;++it_tcells;
        (*it_tcells).fE=1.315272e-02; (*it_tcells).fR=1.277780e-04;++it_tcells;
        (*it_tcells).fE=6.932363e-02; (*it_tcells).fR=1.844398e-01;++it_tcells;
        (*it_tcells).fE=4.342204e-01; (*it_tcells).fR=3.109407e-03;++it_tcells;
        (*it_tcells).fE=3.195924e-01; (*it_tcells).fR=2.990794e-03;++it_tcells;
        (*it_tcells).fE=5.653300e-02; (*it_tcells).fR=9.636729e-02;++it_tcells;
        (*it_tcells).fE=2.442573e-04; (*it_tcells).fR=1.188711e-02;
        return;
    }*/
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::MergeIntoMasterTcellList(list<structTcell>& listTcells, int mergeflag)
{
    list<structTcell>::iterator it_tcells = listTcells.begin();
    list<structTcell>::iterator it_tcellsmaster = masterlistTcells.begin();

    while(it_tcells != listTcells.end() && it_tcellsmaster != masterlistTcells.end())
    {
        if((*it_tcells).uFV == (*it_tcellsmaster).uFV)
        {
            if(mergeflag == +1)
            {
                (*it_tcellsmaster).fE += (*it_tcells).fE ;
                (*it_tcellsmaster).fR += (*it_tcells).fR ;
            }
            else
            {
                (*it_tcellsmaster).fE = std::min(0.0, (*it_tcellsmaster).fE - (*it_tcells).fE);
                (*it_tcellsmaster).fR = std::min(0.0, (*it_tcellsmaster).fR - (*it_tcells).fR);
            }

            ++it_tcells; ++it_tcellsmaster;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(4);
#endif
            continue;
        }

        if((*it_tcells).uFV < (*it_tcellsmaster).uFV)
        {
            if(mergeflag == +1)
            {
                masterlistTcells.insert(it_tcellsmaster, structTcell((*it_tcells).uFV,
                                                                     (*it_tcells).fE,
                                                                     (*it_tcells).fR,
                                                                     (*it_tcells).uHistory,
                                                                     NULL));
                ++it_tcells;
            }
            else
            {
                ++it_tcells;
            }
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(2);
#endif
        }
        else
        {
            ++it_tcellsmaster;
        }
    }

    while(it_tcells != listTcells.end())
    {
        if (mergeflag == +1) {
            masterlistTcells.push_back(structTcell((*it_tcells).uFV,
                                                   (*it_tcells).fE,
                                                   (*it_tcells).fR,
                                                   (*it_tcells).uHistory,
                                                   NULL));
            ++it_tcells;}
        else
        {
            std::cerr << "Error in merge list" << std::endl;
            assert(mergeflag == +1);
        }
#ifdef FLOATINGPOINTOPERATIONS
        robotAgent->IncNumberFloatingPtOperations(1);
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgentOptimised::NegExpDistAffinity(unsigned int v1, unsigned int v2, double k)
{
    /* k is proportional to the level of cross affinity*/
    /* k=0.01 affinity of 1 when HD is 0, else 0  */
    /* k=inf  affinity of 1 for all HD */

    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int hammingdistance  = CRMinRobotAgentOptimised::GetNumberOfSetBits(unXoredString);

    //return 1.0 * exp(-(1.0/k) * (double)hammingdistance);

    // Should we normalize the hammingdistance when input to the exp function, or as above?

#ifdef CRM_ENABLE_SENSORY_HISTORY
    if ((double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES < 2.0/6.0)
        return 1.0 * exp(-(1.0/k) * (double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES);
    else  // Affinities less than 0.108 have no effect on T-cell population cross-interactions. We do this to prevent intermediary regulatory T-cells (with FV between abnormal and normal FVs) to result in tolerance of abnromal FVs. This can occur even when the APC sub-populations are normalized. Without CRM_ENABLE_SENSORY_HISTORY such intermediary T-cell populations would disappear quickly and not linger in the FV history
        return 0.0;
#else
    return 1.0 * exp(-(1.0/k) * (double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES);
#endif
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgentOptimised::GetCurrE(unsigned int thtype)
{
    list<structTcell>::iterator it_tcells;
    for(it_tcells = listTcells.begin(); it_tcells!= listTcells.end(); ++it_tcells)
        if((*it_tcells).uFV == thtype)
            return (*it_tcells).fE;

    return 0.0;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgentOptimised::GetCurrR(unsigned int thtype)
{
    list<structTcell>::iterator it_tcells;
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        if((*it_tcells).uFV == thtype)
            return (*it_tcells).fR;

    return 0.0;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgentOptimised::GetAPC(unsigned int apctype)
{
    list<structAPC>::iterator it_apcs;
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        if((*it_apcs).uFV == apctype)
            return (*it_apcs).fAPC;

    return 0.0;
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::SetCurrE(unsigned int thtype, double f_currE)
{
    list<structTcell>::iterator it_tcells;
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        if((*it_tcells).uFV == thtype)
        { (*it_tcells).fE = f_currE; return; }

    printf("\n Effector clonaltype %d not found in list - code can be modified to insert in list",thtype);
    exit(-1);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::SetCurrR(unsigned int thtype, double f_currR)
{
    list<structTcell>::iterator it_tcells;
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        if((*it_tcells).uFV == thtype)
        { (*it_tcells).fR = f_currR; return; }

    printf("\n Regulator clonaltype %d not found in list - code can be modified to insert in list",thtype);
    exit(-1);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::ScaleDownConjugates(ConjugationIntegrationPhase CONJK)
{
    list<structAPC>::iterator   it_apcs;
    list<structConj>::iterator  it_conj;

    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        if((*it_apcs).fTotalConjugates > (*it_apcs).fTotalSites)
        {
            double scaledownfactor = (*it_apcs).fTotalConjugates / (*it_apcs).fTotalSites;
#ifdef FLOATINGPOINTOPERATIONS
            robotAgent->IncNumberFloatingPtOperations(1);
#endif
            for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
                it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)
            {
                (*it_conj).SetConjugate(CONJK, (*it_conj).GetConjugate(CONJK) / scaledownfactor);
#ifdef FLOATINGPOINTOPERATIONS
                robotAgent->IncNumberFloatingPtOperations(1);
#endif
            }
        }
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRMinRobotAgentOptimised::GetNumberOfSetBits(unsigned int x)
{
    // from http://stackoverflow.com/questions/1639723/ruby-count-the-number-of-1s-in-a-binary-number
    unsigned int m1 = 0x55555555;
    unsigned int m2 = 0x33333333;
    unsigned int m4 = 0x0f0f0f0f;
    x -= (x >> 1) & m1;
    x = (x & m2) + ((x >> 2) & m2);
    x = (x + (x >> 4)) & m4;
    x += x >> 8;
    return (x + (x >> 16)) & 0x3f;
}

/******************************************************************************/
/******************************************************************************/
