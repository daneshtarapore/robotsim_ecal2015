#include "crminrobotagent_optimised.h"

/******************************************************************************/
/******************************************************************************/

#define TCELL_UPPERLIMIT_STEPSIZE 500000.0 //todo: could be set as a propotion of the INTEGRATION_TIME
#define TCELL_LOWERLIMIT_STEPSIZE 1.0e-6

#define CONJ_UPPERLIMIT_STEPSIZE 10.0
#define CONJ_LOWERLIMIT_STEPSIZE 1.0e-6

#define ERRORALLOWED_TCELL_STEPSIZE 1.0e-2 //todo: set as percentage instead of absolute value
#define ERRORALLOWED_CONJ_STEPSIZE  1.0e-3 //todo: set as percentage instead of absolute value; else will introduce problems when m_fFVtoApcscaling is reduced, and dealing with density of conjugates in order of 1e-6


#define INTEGRATION_TIME  5.0e+7 // was 1.5e+7 on elephant01a  earlier 1.0e+7
#define FAILSAFE_CONJ_INTEGRATION_TIME  5.0e+5 // a failsafe to prevent endless integrations of a stiff system.
//TODO: Could instead use the differences in the error values (between time-steps), being same over a period of time as a requirement to reduce time-step
#define REDUCESTEPSIZE_CONJ_INTEGRATION_TIME 1.0e+5 // lowers the step size when the conjugation integration has passed this limit, and the error allowed is high (>1e-3).



#define TCELL_CONVERGENCE  1.0e-2 //todo: set as percentage instead of absolute value. Already using the percentage values to break out of integration loop
#define CONJ_CONVERGENCE   1.0e-3 //todo: set as percentage instead of absolute value


/******************************************************************************/
/******************************************************************************/

CRMinRobotAgentOptimised::CRMinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent,
                                                   CArguments* m_crmArguments)
{
    robotAgent = ptr_robotAgent;

    m_fWeight         = 1.0;
    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_crmArguments->GetArgumentAsIntOr("numberoffeatures", 4);
    currE = m_crmArguments->GetArgumentAsDoubleOr("currE", 10.0);  // : Density of effector cells
    currR = m_crmArguments->GetArgumentAsDoubleOr("currR", 10.0);  // : Density of regulatory cells
    kon   = m_crmArguments->GetArgumentAsDoubleOr("kon", .1);   // : Conjugation rate
    koff  = m_crmArguments->GetArgumentAsDoubleOr("koff", .1);  // : Dissociation rate
    kpe   = m_crmArguments->GetArgumentAsDoubleOr("kpe", 1e-3);   // : Proliferation rate for effector cells
    kde   = m_crmArguments->GetArgumentAsDoubleOr("kde", 1e-6);   // : Death rate for effector cells
    kpr   = m_crmArguments->GetArgumentAsDoubleOr("kpr", 0.5e-3);//0.6e-2   // : Proliferation rate for regulatory cells
    kdr   = m_crmArguments->GetArgumentAsDoubleOr("kdr", 1e-6);   // : Death rate for regulatory cells

    m_fTryExchangeProbability = m_crmArguments->GetArgumentAsDoubleOr("exchangeprob", 0.0);
    // is now set based on characteristics of robot
    //m_fExchangeRange          = m_crmArguments->GetArgumentAsDoubleOr("exchangerange", 2.0);

    se                        = m_crmArguments->GetArgumentAsDoubleOr("sourcerateE", 0.0); //1.1e-3  // Source rate of E cell generation
    sr                        = m_crmArguments->GetArgumentAsDoubleOr("sourcerateR", 0.0); //0.6e-3 // Source rate of R cell generation

    m_fcross_affinity         = m_crmArguments->GetArgumentAsDoubleOr("cross-affinity", 0.4);

    m_fFVtoApcscaling         = m_crmArguments->GetArgumentAsDoubleOr("fvapcscaling", 1.0e-3);


    if(FDMODELTYPE == CRM_TCELLSINEXCESS)
        assert(kon == koff);

    if (m_crmArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#             Number of features in a single FV [%d]\n"
               "currE=#.#                     Density of effector cells [%f]\n"
               "currR=#.#                     Density of regulatory cells [%f]\n"
               "kon=#.#                       Conjugation rate [%f]\n"
               "koff=#.#                      Dissociation rate [%f]\n"
               "kpe=#.#                       Proliferation rate for effector cells [%f]\n"
               "kde=#.#                       Death rate for effector cells [%f]\n"
               "kpr=#.#                       Proliferation rate for regulatory cells [%f]\n"
               "kdr=#.#                       Death rate for regulatory cells [%f]\n"
               "exchangeprob=#.#              Probability of trying to exchange cells with other robots [%f]\n"
               "Source_E=#.#                  Source rate of E cell generation [%f]\n"
               "Source_R=#.#                  Source rate of R cell generation [%f]\n"
               "cross-affinity=#.#            Level of cross-affinity (>0)     [%2.5f]\n"
               "fvapcscaling=#.#              Scaling parameter of [FV] to [APC] [%e]\n",
               CFeatureVector::NUMBER_OF_FEATURES,
               currE,
               currR,
               kon,
               koff,
               kpe,
               kde,
               kpr,
               kdr,
               m_fTryExchangeProbability,
               se,
               sr,
               m_fcross_affinity,
               m_fFVtoApcscaling);
        bHelpDisplayed = true;
    }


    m_unNumberOfReceptors = 1 << (CFeatureVector::NUMBER_OF_FEATURES);




    //    m_pfEffectors       = new double[m_unNumberOfReceptors];
    //    m_pfRegulators      = new double[m_unNumberOfReceptors];

    //    m_pfEffectors_prev  = new double[m_unNumberOfReceptors];
    //    m_pfRegulators_prev = new double[m_unNumberOfReceptors];

    //    // predicted number of cells at time t+step with Euler method
    //    m_pfEffectors_Eu    = new double[m_unNumberOfReceptors];
    //    m_pfRegulators_Eu   = new double[m_unNumberOfReceptors];
    //    // predicted number of cells at time t+step with Huen method
    //    m_pfEffectors_Hu    = new double[m_unNumberOfReceptors];
    //    m_pfRegulators_Hu   = new double[m_unNumberOfReceptors];

    //    // the slopes at time = t and time = t+step
    //    m_pfDeltaEffectors_k0  = new double[m_unNumberOfReceptors];
    //    m_pfDeltaRegulators_k0 = new double[m_unNumberOfReceptors];
    //    m_pfDeltaEffectors_k1  = new double[m_unNumberOfReceptors];
    //    m_pfDeltaRegulators_k1 = new double[m_unNumberOfReceptors];

    //    // the total number of effectors and regulators weighted by affinity
    //    // used to make decision on FVs
    //    m_pfSumEffectorsWeightedbyAffinity  = new double[m_unNumberOfReceptors];
    //    m_pfSumRegulatorsWeightedbyAffinity = new double[m_unNumberOfReceptors];


    //    m_pfAPCs		      = new double[m_unNumberOfReceptors]; // In this implementation, each type of APC presents one FV.


    //    m_pfEffectorConjugatesPerAPC  = new double[m_unNumberOfReceptors];
    //    m_pfRegulatorConjugatesPerAPC = new double[m_unNumberOfReceptors];

    //    m_pfConjugates = new double* [m_unNumberOfReceptors];
    //    m_pfConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfConjugates[i] = m_pfConjugates[i-1] + m_unNumberOfReceptors;
    //    }

    //    m_pfConjugates_tmp = new double* [m_unNumberOfReceptors];
    //    m_pfConjugates_tmp[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfConjugates_tmp[i] = m_pfConjugates_tmp[i-1] + m_unNumberOfReceptors;
    //    }

    //    m_pfConjugates_Eu = new double* [m_unNumberOfReceptors];
    //    m_pfConjugates_Eu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfConjugates_Eu[i] = m_pfConjugates_Eu[i-1] + m_unNumberOfReceptors;
    //    }

    //    m_pfAffinities = new double* [m_unNumberOfReceptors];
    //    m_pfAffinities[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfAffinities[i] = m_pfAffinities[i-1] + m_unNumberOfReceptors;
    //    }


    //    m_pfEffectorConjugates = new double* [m_unNumberOfReceptors];
    //    m_pfEffectorConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfEffectorConjugates[i] = m_pfEffectorConjugates[i-1] + m_unNumberOfReceptors;
    //    }


    //    m_pfRegulatorConjugates = new double* [m_unNumberOfReceptors];
    //    m_pfRegulatorConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfRegulatorConjugates[i] = m_pfRegulatorConjugates[i-1] + m_unNumberOfReceptors;
    //    }



    //    m_pfDeltaConjugates_k0    = new double* [m_unNumberOfReceptors];
    //    m_pfDeltaConjugates_k0[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfDeltaConjugates_k0[i] = m_pfDeltaConjugates_k0[i-1] + m_unNumberOfReceptors;
    //    }

    //    m_pfDeltaConjugates_k1 = new double* [m_unNumberOfReceptors];
    //    m_pfDeltaConjugates_k1[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfDeltaConjugates_k1[i] = m_pfDeltaConjugates_k1[i-1] + m_unNumberOfReceptors;
    //    }

    //    m_pfConj_tmp_Eu = new double* [m_unNumberOfReceptors];
    //    m_pfConj_tmp_Eu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfConj_tmp_Eu[i] = m_pfConj_tmp_Eu[i-1] + m_unNumberOfReceptors;
    //    }

    //    m_pfConj_tmp_Hu = new double* [m_unNumberOfReceptors];
    //    m_pfConj_tmp_Hu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfConj_tmp_Hu[i] = m_pfConj_tmp_Hu[i-1] + m_unNumberOfReceptors;
    //    }




    //    for (unsigned int i = 0; i < m_unNumberOfReceptors; i++)
    //    {
    //#ifdef TCELLCLONEEXCHANGEANALYSIS
    //        if(this->robotAgent->GetIdentification() <= 9) {
    //        m_pfEffectors[i]       = currE;
    //        m_pfRegulators[i]      = currR;}
    //        else {
    //            m_pfEffectors[i]       = currE*10.0;
    //            m_pfRegulators[i]      = 0.0;}
    //#else
    //        m_pfEffectors[i]       = currE;
    //        m_pfRegulators[i]      = currR;
    //#endif

    //        m_pfAPCs[i]            = 0.0;
    //        m_pfEffectorConjugatesPerAPC[i]  = 0.0;
    //        m_pfRegulatorConjugatesPerAPC[i] = 0.0;

    //        m_pfSumEffectorsWeightedbyAffinity[i]  = 0.0;
    //        m_pfSumRegulatorsWeightedbyAffinity[i] = 0.0;

    //        for (unsigned int j = 0; j < m_unNumberOfReceptors; j++)
    //        {
    //            m_pfAffinities[i][j]              = NegExpDistAffinity(i,j,m_fcross_affinity);
    //            //printf("Af(%d,%d)=%f\n",i,j,m_pfAffinities[i][j]);

    //            m_pfConjugates[i][j]              = 0.0;
    //            m_pfConjugates_tmp[i][j]          = 0.0;
    //            m_pfConjugates_Eu[i][j]           = 0.0;

    //            m_pfEffectorConjugates[i][j]      = 0.0;
    //            m_pfRegulatorConjugates[i][j]     = 0.0;
    //        }
    //    }


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
    if(this->robotAgent->GetIdentification() == 19)
        robotAgent->PrintFeatureVectorDistribution(19);
    // Convert the feature vectors of robot agents in the vicinity to APCs for the CRM to work with

    if(this->robotAgent->GetIdentification() == 19)
        PrintAPCList(19);
    UpdateAPCList(); //O(m-fv + n-apc)
    if(this->robotAgent->GetIdentification() == 19)
        PrintAPCList(19);

#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to regenerate the same sequence of random numbers generated with the normal working of the CRM, so that the same agent behaviors may be obtained when CRM is disabled.
    if(this->robotAgent->GetIdentification() == 19)
        PrintTcellList(19);

    SourceTcells(CFeatureVector::NUMBER_OF_FEATURES); //O(m-apc + n-tcell)

    if(this->robotAgent->GetIdentification() == 19)
        PrintTcellList(19);

    // allocate memory for the new soon to be conjugates (from t-cells, apcs added), and remove non-existing conjugates (from t-cells, apcs dead)

    if(this->robotAgent->GetIdentification() == 19)
        PrintConjugatestoAPCList(19);
    UpdateConjugatesToAPCList(); //O(m-apc *(~n-conj + n-tcell))

    if(this->robotAgent->GetIdentification() == 19)
        PrintConjugatestoAPCList(19);

    if(this->robotAgent->GetIdentification() == 19)
        PrintConjugatestoTcellList(19);
    UpdateConjugatesToTcellList(); // update (actually recreating) list of pointers to conjugates, allocated to the apcs. //O(m-apc * ~n-conj)
    if(this->robotAgent->GetIdentification() == 19)
        PrintConjugatestoTcellList(19);


    TcellNumericalIntegration_RK2();

    if (this->robotAgent->GetIdentification() == 19) {
        printf("\n postprocessing\n"); PrintCRMDetails(19); }
#endif


    DiffuseTcells();


    if(this->robotAgent->GetIdentification()==1 || this->robotAgent->GetIdentification()==15)
    {
        printf("\nAllTcellClonesTime: %d, RobotId: %d, ", CSimulator::GetInstance()->GetSimulationStepNumber(), this->robotAgent->GetIdentification());

        list<structTcell>::iterator it_tcells;
        for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        {
            //todo: GetAPC is a linear operation, making this print stmt O(n^2). We should store a ptr to the apc with hghest affinity
//            printf("Clone: %d, A: %f, E: %f, R: %f ",(*it_tcells).uFV, GetAPC((*it_tcells).uFV), (*it_tcells).fE, (*it_tcells).fR);
            printf("Clone: %d, A: %f, E: %f, R: %f ",(*it_tcells).uFV,
                   ((*it_tcells).ptrAPCWithAffinity1)==NULL?
                   GetAPC((*it_tcells).uFV):((*it_tcells).ptrAPCWithAffinity1)->fAPC,
                   (*it_tcells).fE, (*it_tcells).fR);
        }
        printf("\n");}

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
        m_fWeight += (*it_tcells).fE + (*it_tcells).fR;

    m_fWeight = m_fWeight * m_fWeight;
    robotAgent->SetWeight(m_fWeight);


    // select the robot from one of the 10 nearest neighbours - but in these expts. comm does not seem to be needed (comment: we dont know this for sure)
    CRobotAgentOptimised* pcRemoteRobotAgent =
            robotAgent->GetRandomRobotWithWeights(robotAgent->GetSelectedNumNearestNbrs());

    if(pcRemoteRobotAgent == NULL) {
        printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
               CSimulator::GetInstance()->GetSimulationStepNumber(),
               robotAgent->GetIdentification(), -1);
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
            continue;
        }

        if((*it_tcells).uFV > (*it_remotetcells).uFV)
        {
            listTcells.insert(it_tcells, structTcell((*it_remotetcells).uFV,
                                                     (*it_remotetcells).fE * m_fTryExchangeProbability,
                                                     (*it_remotetcells).fR * m_fTryExchangeProbability,
                                                     NULL));
            (*it_remotetcells).fE -= (*it_remotetcells).fE * m_fTryExchangeProbability;
            (*it_remotetcells).fR -= (*it_remotetcells).fR * m_fTryExchangeProbability;
            ++it_remotetcells;
        }
        else
        {
            listRemoteTcells->insert(it_remotetcells, structTcell((*it_tcells).uFV,
                                                      (*it_tcells).fE * m_fTryExchangeProbability,
                                                      (*it_tcells).fR * m_fTryExchangeProbability,
                                                      NULL));
            (*it_tcells).fE -= (*it_tcells).fE * m_fTryExchangeProbability;
            (*it_tcells).fR -= (*it_tcells).fR * m_fTryExchangeProbability;
            ++it_tcells;
        }
   }

    if(it_tcells != listTcells.end())
    {
        while(it_tcells != listTcells.end()) {
            listRemoteTcells->push_back(structTcell((*it_tcells).uFV,
                                                   (*it_tcells).fE * m_fTryExchangeProbability,
                                                   (*it_tcells).fR * m_fTryExchangeProbability,
                                                   NULL));
            (*it_tcells).fE -= (*it_tcells).fE * m_fTryExchangeProbability;
            (*it_tcells).fR -= (*it_tcells).fR * m_fTryExchangeProbability;
            ++it_tcells; }
        it_remotetcells = listRemoteTcells->end();
    }

    while(it_remotetcells != listRemoteTcells->end()) {
        listTcells.push_back(structTcell((*it_remotetcells).uFV,
                                         (*it_remotetcells).fE * m_fTryExchangeProbability,
                                         (*it_remotetcells).fR * m_fTryExchangeProbability,
                                         NULL));
        (*it_remotetcells).fE -= (*it_remotetcells).fE * m_fTryExchangeProbability;
        (*it_remotetcells).fR -= (*it_remotetcells).fR * m_fTryExchangeProbability;
        ++it_remotetcells; }


    printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
               CSimulator::GetInstance()->GetSimulationStepNumber(),
               robotAgent->GetIdentification(), pcRemoteRobotAgent->GetIdentification());
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
    list<structTcell>::iterator it_tcells;;

    while(integration_t < INTEGRATION_TIME)
    {
        if(b_tcelldeath)
        {
            //todo - create a list of dead tcells. call a modified version of UpdateConjugatesToAPCList to only delete conjugates of those tcells.
            UpdateConjugatesToAPCList(); //expensive function
            b_tcelldeath = false;
        }

        // Compute number of conjugates for T cells listTcells members fE and fR. Stores conjugates in listApcs member listConjugatesonAPC having member conjugate fConjugates
        if(FDMODELTYPE == CRM)
        {
            ConjugatesQSS(integration_t == 0.0, K0);
            // Compute derivative for T cells listTcells members fE and fR, and conjugates listApcs member listConjugatesonAPC having member conjugate fConjugates. // Stores derivative in fDeltaE_k0, fDeltaR_k0
            Derivative(K0);
        }
        else
        {
            ConjugatesQSS_ExcessTcells(K0);
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
        }
        //        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        //        {
        //            m_pfEffectors_Eu[thtype]  = m_pfEffectors[thtype]  +
        //                                        step_h * m_pfDeltaEffectors_k0[thtype];
        //            m_pfRegulators_Eu[thtype] = m_pfRegulators[thtype] +
        //                                        step_h * m_pfDeltaRegulators_k0[thtype];
        //            if(m_pfEffectors_Eu[thtype] < 0.0) {
        //                m_pfEffectors_Eu[thtype] = 0.0;}
        //            if(m_pfRegulators_Eu[thtype] < 0.0) {
        //                m_pfRegulators_Eu[thtype] = 0.0;}
        //        }


        if(FDMODELTYPE == CRM)
        {
            // Compute number of conjugates for T cells listTcells members fE_Eu and fR_Eu. Stores conjugates in listApcs member listConjugatesonAPC having member conjugate fConjugates
            ConjugatesQSS(false, K1);
            // Compute derivative for T cells listTcells members fE_Eu and fR_Eu, and conjugates listApcs member listConjugatesonAPC having member conjugate fConjugates. Stores derivative in listTcells  members fDeltaE_k1, fDeltaR_k1
            Derivative(K1);
        }
        else
        {
            ConjugatesQSS_ExcessTcells(K1);
            Derivative_ExcessTcells(K1);
        }

        double absDiffHuenEuler = -1.0;
        for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        {
            (*it_tcells).fE_Hu = (*it_tcells).fE + 0.5 * step_h * ((*it_tcells).fDeltaE_k0 +
                                                                   (*it_tcells).fDeltaE_k1);
            if((*it_tcells).fE_Hu < 0.0)
                (*it_tcells).fE_Hu = 0.0;
            else
                if(fabs((*it_tcells).fE_Hu - (*it_tcells).fE_Eu) > absDiffHuenEuler)
                    absDiffHuenEuler = fabs((*it_tcells).fE_Hu - (*it_tcells).fE_Eu);

            (*it_tcells).fR_Hu = (*it_tcells).fR + 0.5 * step_h * ((*it_tcells).fDeltaR_k0 +
                                                                   (*it_tcells).fDeltaR_k1);
            if((*it_tcells).fR_Hu < 0.0)
                (*it_tcells).fR_Hu = 0.0;
            else
                if(fabs((*it_tcells).fR_Hu - (*it_tcells).fR_Eu) > absDiffHuenEuler)
                    absDiffHuenEuler = fabs((*it_tcells).fR_Hu - (*it_tcells).fR_Eu);
        }
        //        double absDiffHuenEuler = -1.0;
        //        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        //        {
        //            m_pfEffectors_Hu[thtype]  = m_pfEffectors[thtype]  +
        //                                        0.5 * step_h *
        //                                        (m_pfDeltaEffectors_k0[thtype] + m_pfDeltaEffectors_k1[thtype]);
        //            if(m_pfEffectors_Hu[thtype] < 0.0) {
        //                m_pfEffectors_Hu[thtype] = 0.0;}
        //            if(fabs(m_pfEffectors_Hu[thtype] - m_pfEffectors_Eu[thtype]) > absDiffHuenEuler)
        //            {
        //                absDiffHuenEuler = fabs(m_pfEffectors_Hu[thtype] - m_pfEffectors_Eu[thtype]);
        //            }
        //            m_pfRegulators_Hu[thtype] = m_pfRegulators[thtype] +
        //                                        0.5 * step_h *
        //                                        (m_pfDeltaRegulators_k0[thtype] +
        //                                         m_pfDeltaRegulators_k1[thtype]);
        //            if(m_pfRegulators_Hu[thtype] < 0.0) {
        //                m_pfRegulators_Hu[thtype] = 0.0;}
        //            if(fabs(m_pfRegulators_Hu[thtype] - m_pfRegulators_Eu[thtype]) > absDiffHuenEuler)
        //            {
        //                absDiffHuenEuler = fabs(m_pfRegulators_Hu[thtype] - m_pfRegulators_Eu[thtype]);
        //            }
        //        }


        //        if(!(absDiffHuenEuler > 0.0))
        //        {
        //            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        //            {
        //                printf("\n E_Hu %f, E_Eu %f        R_Hu %f, R_Eu %f",
        //                       m_pfEffectors_Hu[thtype],m_pfEffectors_Eu[thtype],
        //                       m_pfRegulators_Hu[thtype],m_pfRegulators_Eu[thtype]);
        //            }

        //            printf("\n Stepsize %e",step_h);
        //        }

        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && step_h == TCELL_LOWERLIMIT_STEPSIZE)
            {
                printf("\n The T-cell population solution is stalled");
                exit(-1);
            }
            step_h = step_h / 2.0;

            if(step_h < TCELL_LOWERLIMIT_STEPSIZE) {
                step_h = TCELL_LOWERLIMIT_STEPSIZE;}

            printf("\n New stepsize %f - integration time %e",step_h,integration_t);

            b_prevdiff0occurance = true;

            continue;
        }

        b_prevdiff0occurance = false;

        assert(absDiffHuenEuler >= 0.0);
        step_h *= sqrt(ERRORALLOWED_TCELL_STEPSIZE/absDiffHuenEuler);
        if(step_h > TCELL_UPPERLIMIT_STEPSIZE) {
            step_h = TCELL_UPPERLIMIT_STEPSIZE;}
        else if(step_h < TCELL_LOWERLIMIT_STEPSIZE) {
            step_h = TCELL_LOWERLIMIT_STEPSIZE;}


        convergence_errormax = -1.0;
        for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        {
            (*it_tcells).fE_prev = (*it_tcells).fE; (*it_tcells).fR_prev = (*it_tcells).fR;

            (*it_tcells).fE += step_h * (*it_tcells).fDeltaE_k0;
            if((*it_tcells).fE < 0.0)
                (*it_tcells).fE = 0.0;
            else
                if(fabs(step_h * (*it_tcells).fDeltaE_k0) > convergence_errormax)   {
                    convergence_errormax      = fabs(step_h * (*it_tcells).fDeltaE_k0);
                    perc_convergence_errormax = (convergence_errormax / (*it_tcells).fE_prev) * 100.0;}

            (*it_tcells).fR += step_h * (*it_tcells).fDeltaR_k0;
            if((*it_tcells).fR < 0.0)
                (*it_tcells).fR = 0.0;
            else
                if(fabs(step_h * (*it_tcells).fDeltaR_k0) > convergence_errormax)   {
                    convergence_errormax      = fabs(step_h * (*it_tcells).fDeltaR_k0);
                    perc_convergence_errormax = (convergence_errormax / (*it_tcells).fR_prev) * 100.0;}


            if(((*it_tcells).fE + (*it_tcells).fR) <= CELLLOWERBOUND)
            {
                b_tcelldeath = true;
                it_tcells = listTcells.erase(it_tcells);
            }
        }

        //        convergence_errormax = -1.0;
        //        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        //        {
        //            m_pfEffectors_prev[thtype]  = m_pfEffectors[thtype];
        //            m_pfRegulators_prev[thtype] = m_pfRegulators[thtype];
        //            m_pfEffectors[thtype]  = m_pfEffectors[thtype]  + step_h * m_pfDeltaEffectors_k0[thtype];
        //            m_pfRegulators[thtype] = m_pfRegulators[thtype] + step_h * m_pfDeltaRegulators_k0[thtype];
        //            if(m_pfEffectors[thtype] < 0.0){
        //                m_pfEffectors[thtype] = 0.0;}
        //            else
        //            {
        //                if(fabs(step_h * m_pfDeltaEffectors_k0[thtype]) > convergence_errormax)
        //                {
        //                    convergence_errormax      = fabs(step_h * m_pfDeltaEffectors_k0[thtype]);
        //                    perc_convergence_errormax = (convergence_errormax / m_pfEffectors_prev[thtype]) *
        //                                                100.0;
        //                }
        //            }
        //            if(m_pfRegulators[thtype] < 0.0){
        //                m_pfRegulators[thtype] = 0.0;}
        //            else
        //            {
        //                if(fabs(step_h * m_pfDeltaRegulators_k0[thtype]) > convergence_errormax)
        //                {
        //                    convergence_errormax      = fabs(step_h * m_pfDeltaRegulators_k0[thtype]);
        //                    perc_convergence_errormax = (convergence_errormax / m_pfRegulators_prev[thtype]) *
        //                                                100.0;
        //                }
        //            }
        //        }

        m_dconvergence_error     = convergence_errormax;
        m_dpercconvergence_error = perc_convergence_errormax;

        if(m_dpercconvergence_error <= 0.001)
            break;

        integration_t += step_h;
    }

    if(b_tcelldeath) // for any remaining tcell dead
        //todo - create a list of dead tcells. call a modified version of UpdateConjugatesToAPCList to only delete conjugates of those tcells.
        UpdateConjugatesToAPCList();
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

    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        (*it_apcs).UpdateConjugateList(&listTcells, m_fcross_affinity);
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

    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); it_tcells++)
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

void CRMinRobotAgentOptimised::ConjugatesQSS(bool bResetConjugates, TcellIntegrationPhase K)
{
    list<structAPC>::iterator it_apcs; list<structConj>::iterator it_conjs;

    // since we are now deleting pathogens and the rate of deletion mayb be faster than the conjugate unbindingrate, we always start with 0 conjugates to compute QSS values
    // consequently set free tcells and available binding sites accordingly
//    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
//    {
//        (*it_tcells).fFreeTcells = (*it_tcells).GetE(K) + (*it_tcells).GetR(K);
//    }
    if(bResetConjugates) // conjugates are reset to 0 only when APC sub-population has changed.
        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        {
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
            {
                (*it_conjs).fConjugates = 0.0;
            }
        }

    
    conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;
    double error = 1.0;
    double conjintegration_t = 0.0;
    unsigned n_iteration = 0;
    bool b_prevdiff0occurance=false;

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
            }
        }


//        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//        {
//            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    if(m_pfAPCs[apctype] > 0.0)
//                    {
//                        m_pfDeltaConjugates_k0[thtype][apctype] =
//                                ((kon * m_pfAffinities[thtype][apctype] *
//                                  FreeThCells(E, R, C, thtype) *
//                                  AvailableBindingSites(C, apctype)) -
//                                 koff*C[thtype][apctype]);
//                    }
//                    else
//                        m_pfDeltaConjugates_k0[thtype][apctype] = 0.0;
//                }
//            }
//            else
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    m_pfDeltaConjugates_k0[thtype][apctype] = 0.0;
//                }
//            }
//        }

        //!todo this can be merged with the nested loop above, since delta computed on fConjugates is not being changed
//        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
//            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
//                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
//            {

//                (*it_conjs).fConjugates_k0 = (*it_conjs).fConjugates +
//                                              conjstep_h * (*it_conjs).fDelta_k0;

//                if((*it_conjs).fConjugates_k0 < 0.0)
//                    (*it_conjs).fConjugates_k0 = 0.0;
//            }


//        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//        {
//            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    if(m_pfAPCs[apctype] > 0.0)
//                    {
//                        m_pfConj_tmp_Eu[thtype][apctype] = C[thtype][apctype] + conjstep_h *
//                                                           m_pfDeltaConjugates_k0[thtype][apctype];

//                        if(m_pfConj_tmp_Eu[thtype][apctype] < 0.0)
//                            m_pfConj_tmp_Eu[thtype][apctype] = 0.0;
//                    }
//                    else
//                        m_pfConj_tmp_Eu[thtype][apctype] = 0.0;
//                }
//            }
//            else
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    m_pfConj_tmp_Eu[thtype][apctype] = 0.0;
//                }
//            }
//        }


        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        ScaleDownConjugates(CONJ_K0);
        //ScaleDownConjugates(m_pfConj_tmp_Eu);

        //update the number of free tcells and available binding sites - with euler approx of conjugates
        FreeTcellsAndAvailableAPCSites(K, CONJ_K0);


        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)

                (*it_conjs).fDelta_k1 = kon * (*it_conjs).affinity *
                                        ((*it_conjs).ptrTcell)->fFreeTcells *
                                        (*it_apcs).fAvailableSites -
                                        koff * (*it_conjs).fConjugates_k0;

//        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//        {
//            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    if(m_pfAPCs[apctype] > 0.0)
//                    {
//                        m_pfDeltaConjugates_k1[thtype][apctype] =
//                                ((kon * m_pfAffinities[thtype][apctype] *
//                                  FreeThCells(E, R, m_pfConj_tmp_Eu, thtype) *
//                                  AvailableBindingSites(m_pfConj_tmp_Eu, apctype)) -
//                                 koff*m_pfConj_tmp_Eu[thtype][apctype]);
//                    }
//                    else
//                        m_pfDeltaConjugates_k1[thtype][apctype] = 0.0;
//                }
//            }
//            else
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    m_pfDeltaConjugates_k1[thtype][apctype] = 0.0;
//                }
//            }
//        }

        double absDiffHuenEuler = -1.0, tmp_absDiffHuenEuler;
        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
            {

                (*it_conjs).fConjugates_k1 = (*it_conjs).fConjugates +
                                              0.5 * (conjstep_h * (*it_conjs).fDelta_k0 +
                                                     conjstep_h * (*it_conjs).fDelta_k1);

                if((*it_conjs).fConjugates_k1 < 0.0)
                    (*it_conjs).fConjugates_k1 = 0.0;
                else
                {
                    tmp_absDiffHuenEuler = fabs((*it_conjs).fConjugates_k1 -
                                                (*it_conjs).fConjugates_k0);
                    if(tmp_absDiffHuenEuler > absDiffHuenEuler)
                        absDiffHuenEuler = tmp_absDiffHuenEuler;
                }
            }


//        double absDiffHuenEuler = -1.0;
//        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//        {
//            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    if(m_pfAPCs[apctype] > 0.0)
//                    {
//                        m_pfConj_tmp_Hu[thtype][apctype] = C[thtype][apctype] + 0.5 * conjstep_h * (m_pfDeltaConjugates_k0[thtype][apctype] + m_pfDeltaConjugates_k1[thtype][apctype]);

//                        if(m_pfConj_tmp_Hu[thtype][apctype] < 0.0)
//                            m_pfConj_tmp_Hu[thtype][apctype] = 0.0;


//                        if(fabs(m_pfConj_tmp_Hu[thtype][apctype] - m_pfConj_tmp_Eu[thtype][apctype]) > absDiffHuenEuler)
//                        {
//                            absDiffHuenEuler = fabs(m_pfConj_tmp_Hu[thtype][apctype] - m_pfConj_tmp_Eu[thtype][apctype]);
//                        }
//                    }
//                    else
//                        m_pfConj_tmp_Hu[thtype][apctype] = 0.0;
//                }
//            }
//            else
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    m_pfConj_tmp_Hu[thtype][apctype] = 0.0;
//                }
//            }
//        }


        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        //t!todo: not needed as the huen values are not used anymore
        //ScaleDownConjugates(CONJ_K1);
        //ScaleDownConjugates(m_pfConj_tmp_Hu);

        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && conjstep_h == CONJ_LOWERLIMIT_STEPSIZE)
            {
                printf("\n The Conjugation solution is stalled");
                exit(-1);
            }

            conjstep_h = conjstep_h / 2.0;

            if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
                conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}

            printf("\n new conjugation step %f, n_iteration %u",conjstep_h,n_iteration);

            b_prevdiff0occurance = true;

            continue;
        }

        b_prevdiff0occurance = false;

        assert(absDiffHuenEuler >= 0.0);


        if(conjintegration_t > REDUCESTEPSIZE_CONJ_INTEGRATION_TIME &&
           ERRORALLOWED_CONJ_STEPSIZE >= 1.0e-3)
        {
            /*The system is most likely stiff and oscillating around the "true" value, as the slope approaches 0*/
            /*we reduce the step size to reduce the difference between the oscillating values*/

            /*If the error allowed was lower, the corresponding step sizes would already be lower,
            and so would be the differences between oscillating values*/
            conjstep_h /= 2.0;
            if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
                conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}
        }
        else
        {
            conjstep_h *= sqrt(ERRORALLOWED_CONJ_STEPSIZE/absDiffHuenEuler);
            if(conjstep_h > CONJ_UPPERLIMIT_STEPSIZE) {
                conjstep_h = CONJ_UPPERLIMIT_STEPSIZE;}
            else if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
                conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}
        }


        double error_max = -1.0;
        for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
        {
            (*it_apcs).fTotalConjugates = 0.0;
            for(it_conjs =  (*it_apcs).listConjugatesonAPC.begin();
                it_conjs != (*it_apcs).listConjugatesonAPC.end(); ++it_conjs)
            {

                (*it_conjs).fConjugates = (*it_conjs).fConjugates +
                                           conjstep_h * (*it_conjs).fDelta_k0;

                if((*it_conjs).fConjugates < 0.0)
                    (*it_conjs).fConjugates = 0.0;
                else
                    if(fabs(conjstep_h * (*it_conjs).fDelta_k0) > error_max)
                        error_max = fabs(conjstep_h * (*it_conjs).fDelta_k0);

                (*it_apcs).fTotalConjugates += (*it_conjs).fConjugates;
            }
        }


//        double error_max = -1.0;
//        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//        {
//            if(E[thtype] + R[thtype] <= CELLLOWERBOUND)
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    C[thtype][apctype] = 0.0;
//                }
//            }
//            else
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    if(m_pfAPCs[apctype] > 0.0)
//                    {
//                        C[thtype][apctype] = C[thtype][apctype] + conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype];
//                        if(C[thtype][apctype] < 0.0)
//                        {
//                            C[thtype][apctype] = 0.0;
//                        }
//                        else
//                        {
//                            if(fabs(conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype]) > error_max)
//                            {
//                                error_max = fabs(conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype]);
//                            }
//                        }
//                    }
//                    else
//                        C[thtype][apctype] = 0.0;
//                }
//            }
//        }

        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        ScaleDownConjugates(CONJ);


        conjintegration_t = conjintegration_t + conjstep_h;
        error = error_max;
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::ConjugatesQSS_ExcessTcells(TcellIntegrationPhase TK)
{
//    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//    {
//        if(m_pfAPCs[apctype])
//        {
//            double tcellsweightedaffinity = 0.0;
//            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//                tcellsweightedaffinity += m_pfAffinities[thtype][apctype] * (E[thtype] + R[thtype]);

//            // storing all the conjuagtes for APC of subpopulation apctype at the position of the first T-cell clonaltype. will factorize it into effector and regulatory conjuagtes at the Derivative function
//            C[0][apctype] = (m_pfAPCs[apctype]*((double)sites)*tcellsweightedaffinity) /
//                            (tcellsweightedaffinity + 1.0);
//        }
//        else
//            C[0][apctype] = 0.0;

//    }
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
        for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
            it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)
        {
            (*it_conjptr)->fEffectorConjugates  = ((*it_tcells).GetE(TK) / tmp_totalcells) *
                                                 (*it_conjptr)->fConjugates;
            (*it_conjptr)->fRegulatorConjugates = ((*it_tcells).GetR(TK) / tmp_totalcells) *
                                                 (*it_conjptr)->fConjugates;
        }
    }

//        // Dividing the conjugates into Effector and Regulator type
//        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//        {
//            if((E[thtype] + R[thtype]) <= CELLLOWERBOUND)
//            {
//                //TODO: check I think we are doing this initialization to 0, twice. done before in computation of conjugates (ConjugateQSS(...))
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    m_pfEffectorConjugates[thtype][apctype]  =  0.0;
//                    m_pfRegulatorConjugates[thtype][apctype] =  0.0;
//                }
//            }
//            else
//            {
//                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//                {
//                    m_pfEffectorConjugates[thtype][apctype] =
//                            (E[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];

//                    m_pfRegulatorConjugates[thtype][apctype] =
//                            (R[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];
//                }
//            }
//        }

    // Computing the total number of effector and regulator conjugates per APC
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        (*it_apcs).fEffectorConjugatesPerAPC  = 0.0;
        (*it_apcs).fRegulatorConjugatesPerAPC = 0.0;
        for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
            it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)
        {
            (*it_apcs).fEffectorConjugatesPerAPC  += (*it_conj).fEffectorConjugates;
            (*it_apcs).fRegulatorConjugatesPerAPC += (*it_conj).fRegulatorConjugates;
        }

        assert(((*it_apcs).fEffectorConjugatesPerAPC + (*it_apcs).fRegulatorConjugatesPerAPC -
               (*it_apcs).fTotalSites) <= CONJUGATION_OVERFLOW_LIMIT);
    }

//        // Computing the total number of effector and regulator conjugates per APC
//        for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
//        {
//            m_pfEffectorConjugatesPerAPC[apctype]  = 0.0;
//            m_pfRegulatorConjugatesPerAPC[apctype] = 0.0;

//            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//            {
//                m_pfEffectorConjugatesPerAPC[apctype]  += m_pfEffectorConjugates[thtype][apctype];
//                m_pfRegulatorConjugatesPerAPC[apctype] += m_pfRegulatorConjugates[thtype][apctype];
//            }

//            assert((m_pfEffectorConjugatesPerAPC[apctype]+m_pfRegulatorConjugatesPerAPC[apctype]) -
//                   m_pfAPCs[apctype]*((double)sites) <= CONJUGATION_OVERFLOW_LIMIT);
//        }

    ComputeNewDerivative(TK);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::Derivative_ExcessTcells(TcellIntegrationPhase TK)
{
//    for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
//    {
//        if(m_pfAPCs[apctype] == 0.0)
//        {
//            m_pfEffectorConjugatesPerAPC[apctype] = 0.0;
//            m_pfRegulatorConjugatesPerAPC[apctype] = 0.0;

//            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//            {
//                m_pfEffectorConjugates[thtype][apctype]  = 0.0;
//                m_pfRegulatorConjugates[thtype][apctype] = 0.0;
//            }
//        }
//        else
//        {
//            double tcellsweightedaffinity = 0.0, ecellsweightedaffinity = 0.0,
//            rcellsweightedaffinity = 0.0;

//            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//            {
//                tcellsweightedaffinity += m_pfAffinities[thtype][apctype] * (E[thtype] + R[thtype]);
//                ecellsweightedaffinity += m_pfAffinities[thtype][apctype] * E[thtype];
//                rcellsweightedaffinity += m_pfAffinities[thtype][apctype] * R[thtype];
//            }

//            // using the conjuagtes for APC of subpopulation apctype that was stored at the position of the first T-cell clonaltype. will factorize it into effector and regulatory conjuagtes at the Derivative function
//            m_pfEffectorConjugatesPerAPC[apctype]  = C[0][apctype] *
//                                                     ecellsweightedaffinity/tcellsweightedaffinity;
//            m_pfRegulatorConjugatesPerAPC[apctype] = C[0][apctype] *
//                                                     rcellsweightedaffinity/tcellsweightedaffinity;

//            assert((m_pfEffectorConjugatesPerAPC[apctype]+m_pfRegulatorConjugatesPerAPC[apctype]) -
//                   m_pfAPCs[apctype]*((double)sites) <= CONJUGATION_OVERFLOW_LIMIT);


//            double totalconjugatesonapc = C[0][apctype];
//            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//            {
//                if((E[thtype] + R[thtype]) <= CELLLOWERBOUND)
//                {
//                    m_pfEffectorConjugates[thtype][apctype]  =  0.0;
//                    m_pfRegulatorConjugates[thtype][apctype] =  0.0;
//                }
//                else
//                {
//                    C[thtype][apctype] = totalconjugatesonapc *
//                                         m_pfAffinities[thtype][apctype] * (E[thtype] + R[thtype]) /
//                                         tcellsweightedaffinity;


//                    m_pfEffectorConjugates[thtype][apctype] =
//                            (E[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];

//                    m_pfRegulatorConjugates[thtype][apctype] =
//                            (R[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];
//                }
//            }
//        }
//    }

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

        }

        effector_incr  += se - kde * (*it_tcells).GetE(TK);
        regulator_incr += sr - kdr * (*it_tcells).GetR(TK);

        (*it_tcells).SetDelta(TK, effector_incr, regulator_incr);
    }

//    // We now compute the derivative of each T-cell clonaltype, based on their old values, proliferation and cell death, and thymic generation of new cells
//    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//    {
//        double effector_incr = 0.0, regulator_incr = 0.0;

//        for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
//        {
//            if(m_pfAPCs[apctype])
//            {
//                double exp_tmp1 = (9.0*m_pfAPCs[apctype]*m_pfAPCs[apctype]);
//                double exp_tmp2 = (m_pfRegulatorConjugatesPerAPC[apctype] - 3.0*m_pfAPCs[apctype])*
//                                  (m_pfRegulatorConjugatesPerAPC[apctype] - 3.0*m_pfAPCs[apctype]);

//                double Pe = exp_tmp2 / exp_tmp1;
//                effector_incr  += kpe*Pe*m_pfEffectorConjugates[thtype][apctype];

//                // 		effector_incr  += kpe*Hyp(0, m_pfRegulatorConjugatesPerAPC[apctype],
//                // 				      	          m_pfAPCs[apctype]*(double)sites, (double)sites) *
//                // 			      ((m_pfAPCs[apctype]*(double)sites)/((m_pfAPCs[apctype]*(double)sites) - m_pfRegulatorConjugatesPerAPC[apctype])) *
//                // 			      m_pfEffectorConjugates[thtype][apctype];

//                double Pr = (6.0*m_pfAPCs[apctype] -m_pfEffectorConjugatesPerAPC[apctype])*
//                            m_pfEffectorConjugatesPerAPC[apctype] / exp_tmp1;
//                regulator_incr += kpr*Pr*m_pfRegulatorConjugates[thtype][apctype];

//                // 		regulator_incr += kpr*((double)(sites-1) / (double)sites) *
//                // 			      (m_pfEffectorConjugatesPerAPC[apctype]/m_pfAPCs[apctype]) *
//                // 			      m_pfRegulatorConjugates[thtype][apctype];
//            }
//        }

//        effector_incr  += se - kde * E[thtype];
//        regulator_incr += sr - kdr * R[thtype];

//        deltaE[thtype] = effector_incr;
//        deltaR[thtype] = regulator_incr;
//    }
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
        for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
            it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)

            fConjugatesOnAPC += (*it_conj).GetConjugate(CONJK);

        tmp_freesites  = -fConjugatesOnAPC + (*it_apcs).fTotalSites;
        assert(-tmp_freesites <= CONJUGATION_OVERFLOW_LIMIT);

        (*it_apcs).fAvailableSites = tmp_freesites;
    }

    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        double fConjugatesOfTcell = 0.0;
        for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
            it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)

            fConjugatesOfTcell += (*it_conjptr)->GetConjugate(CONJK);


        (*it_tcells).fFreeTcells = (*it_tcells).GetE(TK) + (*it_tcells).GetR(TK) -
                                   fConjugatesOfTcell;

        //assert(-((*it_tcells).fFreeTcells) <= CONJUGATION_OVERFLOW_LIMIT);
        if(!(-((*it_tcells).fFreeTcells) <= CONJUGATION_OVERFLOW_LIMIT)) {
            PrintCRMDetails(robotAgent->GetIdentification()); exit(-1);  }
    }
}

/******************************************************************************/
/******************************************************************************/

//double CRMinRobotAgentOptimised::FreeThCells(double* E, double* R, double** C, unsigned int thtype)
//{
//    double conjugatedcells = 0.0;
//    for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
//    {
//        conjugatedcells += C[thtype][apctype];
//    }

//    return E[thtype] + R[thtype] - conjugatedcells;
//}

/******************************************************************************/
/******************************************************************************/

//double CRMinRobotAgentOptimised::AvailableBindingSites(double** C, unsigned int apctype)
//{
//    double conjugatedcells = 0.0;
//    for(unsigned thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
//    {
//        conjugatedcells += C[thtype][apctype];
//    }

//    assert(conjugatedcells - (m_pfAPCs[apctype]*((double)sites)) <= CONJUGATION_OVERFLOW_LIMIT);

//    return m_pfAPCs[apctype]*sites - conjugatedcells;
//}
/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintCRMDetails(unsigned id)
{
    //unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    if(!(robotAgent->GetIdentification() == id))
        return;

    robotAgent->PrintFeatureVectorDistribution(id);

    PrintAPCList(id);
    PrintTcellList(id);
    PrintConjugatestoAPCList(id);
    PrintConjugatestoTcellList(id);


//    // Print NonZero APCs
//    printf("\n");
//    list<structAPC>::iterator it_apcs;
//    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
//        printf("APC[%d]=%f  ", (*it_apcs).uFV, (*it_apcs).fAPC);

////    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
////    {
////        if(m_pfAPCs[apctype]>0.0)
////            printf("APC[%d]=%f  ",apctype,m_pfAPCs[apctype]);
////    }

//    // Print table of conjugates
//    printf("\n=================================\n");
//    list<structConj>::iterator it_conj;
//    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
//    {
//        printf("\nAPC:%d\t",(*it_apcs).uFV);
//        for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
//            it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)

//            printf("%e  ",(*it_conj).fConjugates);
//    }

////    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
////    {
////        printf("\nAPC:%d\t",apctype);
////        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
////        {
////            printf("%e  ",m_pfConjugates[thtype][apctype]);
////        }
////    }


//    // Print Effector and regulatory clonaltypes
//    printf("\n=================================\n");
//    list<structTcell>::iterator it_tcells;
//    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
//        printf("E[%d]=%e,R[%d]=%e  ", (*it_tcells).uFV, (*it_tcells).fE,
//                                      (*it_tcells).uFV, (*it_tcells).fR);
////    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
////    {
////        printf("E[%d]=%e,R[%d]=%e  ",thtype,m_pfEffectors[thtype],thtype,m_pfRegulators[thtype]);
////    }
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

void CRMinRobotAgentOptimised::PrintTcellList(unsigned int id)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structTcell>::iterator it_tcells;
    printf("\n===========R%d T cell list==============\n", robotAgent->GetIdentification());
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        printf("E[%d]=%e,R[%d]=%e (A=%f)   ", (*it_tcells).uFV, (*it_tcells).fE,
                                      (*it_tcells).uFV, (*it_tcells).fR, GetAPC((*it_tcells).uFV));
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintConjugatestoAPCList(unsigned int id)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structAPC>::iterator it_apcs; list<structConj>::iterator it_conj;
    printf("\n===========R%d Conjugates to APCs=======\n", robotAgent->GetIdentification());
    for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
    {
        printf("\nAPC:%d\t",(*it_apcs).uFV);
        for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
            it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)

            printf("Tcell:%d %e  ",((*it_conj).ptrTcell)->uFV, (*it_conj).fConjugates);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::PrintConjugatestoTcellList(unsigned int id)
{
    if(!(robotAgent->GetIdentification() == id))
        return;

    list<structTcell>::iterator it_tcells; list<structConj*>::iterator it_conjptr;
    printf("\n===========R%d Conjugates to Tcells====\n", robotAgent->GetIdentification());
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
    {
        printf("\nTcell:%d\t",(*it_tcells).uFV);
        for(it_conjptr = (*it_tcells).listPtrstoConjugatesofTcell.begin();
            it_conjptr != (*it_tcells).listPtrstoConjugatesofTcell.end(); ++it_conjptr)

            printf("APC:%d %e  ",((*it_conjptr)->ptrAPC)->uFV, (*it_conjptr)->fConjugates);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::UpdateState()
{
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
        }

        (*it_apcs).fE_weightedbyaffinity = tmp_E;
        (*it_apcs).fR_weightedbyaffinity = tmp_R;

        if ((tmp_E + tmp_R) <= CELLLOWERBOUND || fabs(tmp_E - tmp_R) <= CELLLOWERBOUND)
            //Dont know - no T-cells to make decision or E approx. equal to R
            robotAgent->SetMostWantedList(&it_fvsensed, 0);

        else if (tmp_E > tmp_R)
            // Attack
            robotAgent->SetMostWantedList(&it_fvsensed, 1);
        else
            // Tolerate
            robotAgent->SetMostWantedList(&it_fvsensed, 2);

         ++it_apcs; ++it_fvsensed;
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::UpdateAPCList()
{
    list<structFVsSensed>* fvsensed = robotAgent->GetFeatureVectorsSensed();
    list<structFVsSensed>::iterator it_fvsensed;
    list<structAPC>::iterator it_apcs;

//    if(CSimulator::GetInstance()->GetSimulationStepNumber() == (MODELSTARTTIME+1))
//    {
//        it_apcs = listAPCs.begin();
//        for(it_fvsensed = fvsensed->begin(); it_fvsensed != fvsensed->end(); ++it_fvsensed)
//        {
//            listAPCs.insert(it_apcs, structAPC((*it_fvsensed).uFV,
//                                               (*it_fvsensed).fRobots * m_fFVtoApcscaling,
//                                               (double)sites));
//            ++it_apcs;
//        }
//        return;
//    }


    it_fvsensed = fvsensed->begin(); it_apcs = listAPCs.begin();
    while(it_apcs != listAPCs.end() && it_fvsensed != fvsensed->end())
    {
        if((*it_fvsensed).uFV == (*it_apcs).uFV)
        {
            (*it_apcs).fAPC = (*it_fvsensed).fRobots * m_fFVtoApcscaling;
            (*it_apcs).fTotalSites = (*it_apcs).fAPC * (double)sites;
            (*it_apcs).fEffectorConjugatesPerAPC = 0.0; (*it_apcs).fRegulatorConjugatesPerAPC = 0.0;
            (*it_apcs).fE_weightedbyaffinity = 0.0; (*it_apcs).fR_weightedbyaffinity = 0.0;
            ++it_apcs; ++it_fvsensed;
            continue;
        }

        if((*it_apcs).uFV > (*it_fvsensed).uFV)
        {
            listAPCs.insert(it_apcs, structAPC((*it_fvsensed).uFV,
                                               (*it_fvsensed).fRobots * m_fFVtoApcscaling,
                                               (double)sites));
            ++it_fvsensed;
            continue;
        }

        (*it_apcs).listConjugatesonAPC.clear();
        it_apcs = listAPCs.erase(it_apcs);
    }

    while(it_apcs != listAPCs.end()) {
        (*it_apcs).listConjugatesonAPC.clear();
        it_apcs = listAPCs.erase(it_apcs);}

    while(it_fvsensed != fvsensed->end()) {
        listAPCs.push_back(structAPC((*it_fvsensed).uFV,
                                     (*it_fvsensed).fRobots * m_fFVtoApcscaling, (double)sites));
        ++it_fvsensed; }




    //    // Ask the robot you belong to for the number of feature vectors of different types
    //    // returns in m_pfFeaturesSensed
    //    float* m_pfFeaturesSensed  = robotAgent->GetFeatureVectorsSensed();

    //    for (int i = 0; i < m_unNumberOfReceptors; i++)
    //    {
    //        m_pfAPCs[i] = (double) m_pfFeaturesSensed[i];///(M_PI * robotAgent->GetFVSenseRange()  * robotAgent->GetFVSenseRange());
    //        m_pfAPCs[i] = m_fFVtoApcscaling * m_pfAPCs[i];
    //    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::SourceTcells(unsigned hammingdistance)
{
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
                    listTcells.push_back(structTcell(index_tcells, currE, currR, &(*it_apcs)));
                    ++it_apcs;}
                else
                    listTcells.push_back(structTcell(index_tcells, currE, currR, NULL));
            }

            return;
        }

        if(hammingdistance == 0)
        {
            for(it_apcs = listAPCs.begin(); it_apcs != listAPCs.end(); ++it_apcs)
                listTcells.push_back(structTcell((*it_apcs).uFV, currE, currR, &(*it_apcs)));

            return;
        }
        printf("Other hamming dist to be coded."); exit(-1);
    }

    it_apcs = listAPCs.begin(); it_tcells = listTcells.begin();
    while(it_tcells != listTcells.end() && it_apcs != listAPCs.end())
    {
        if((*it_apcs).uFV == (*it_tcells).uFV)
        {
            (*it_tcells).fE += currE; (*it_tcells).fR += currR;
            if((*it_tcells).ptrAPCWithAffinity1 == NULL)
                (*it_tcells).ptrAPCWithAffinity1 = &(*it_apcs);
            ++it_tcells; ++it_apcs;
            continue;
        }

        if((*it_tcells).uFV > (*it_apcs).uFV)
        {
            listTcells.insert(it_tcells, structTcell((*it_apcs).uFV, currE, currR, &(*it_apcs)));
            ++it_apcs;
            continue;
        }

        (*it_tcells).ptrAPCWithAffinity1 = NULL; //tcell with no apc having the same fv
        ++it_tcells;
    }

    while(it_apcs != listAPCs.end()) {
        listTcells.push_back(structTcell((*it_apcs).uFV, currE, currR, &(*it_apcs)));
        ++it_apcs; }
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

    return 1.0 * exp(-(1.0/k) * (double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES);
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
        if((*it_tcells).uFV == thtype) {
        (*it_tcells).fE = f_currE; return; }

    printf("\n Effector clonaltype %d not found in list - code can be modified to insert in list",thtype);
    exit(-1);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgentOptimised::SetCurrR(unsigned int thtype, double f_currR)
{
    list<structTcell>::iterator it_tcells;
    for(it_tcells = listTcells.begin(); it_tcells != listTcells.end(); ++it_tcells)
        if((*it_tcells).uFV == thtype) {
        (*it_tcells).fR = f_currR; return; }

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
    {
//        double fConjugatesOnAPC = 0.0;
//        for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
//        it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)
//            fConjugatesOnAPC += (*it_conj).GetConjugate(CONJK);

        if((*it_apcs).fTotalConjugates > (*it_apcs).fTotalSites)
        {
            double scaledownfactor = (*it_apcs).fTotalConjugates / (*it_apcs).fTotalSites;
            for(it_conj = (*it_apcs).listConjugatesonAPC.begin();
                it_conj != (*it_apcs).listConjugatesonAPC.end(); ++it_conj)
                (*it_conj).SetConjugate(CONJK, (*it_conj).GetConjugate(CONJK) / scaledownfactor);
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
