#include "crminrobotagent.h"

/******************************************************************************/
/******************************************************************************/

#define TCELL_UPPERLIMIT_STEPSIZE 500000
#define TCELL_LOWERLIMIT_STEPSIZE 1.0e-6

#define CONJ_UPPERLIMIT_STEPSIZE 10
#define CONJ_LOWERLIMIT_STEPSIZE 1.0e-6

#define ERRORALLOWED_TCELL_STEPSIZE 1.0e-1
#define ERRORALLOWED_CONJ_STEPSIZE  1.0e-3

#define INTEGRATION_TIME  1e+6

#define TCELL_CONVERGENCE  0.1
#define CONJ_CONVERGENCE   0.001


/******************************************************************************/
/******************************************************************************/

CRMinRobotAgent::CRMinRobotAgent(CRobotAgent* ptr_robotAgent, CArguments* m_crmArguments)
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

    m_pfEffectors       = new double[m_unNumberOfReceptors];
    m_pfRegulators      = new double[m_unNumberOfReceptors];

    m_pfEffectors_prev  = new double[m_unNumberOfReceptors];
    m_pfRegulators_prev = new double[m_unNumberOfReceptors];

    // predicted number of cells at time t+step with Euler method
    m_pfEffectors_Eu    = new double[m_unNumberOfReceptors];
    m_pfRegulators_Eu   = new double[m_unNumberOfReceptors];
    // predicted number of cells at time t+step with Huen method
    m_pfEffectors_Hu    = new double[m_unNumberOfReceptors];
    m_pfRegulators_Hu   = new double[m_unNumberOfReceptors];

    // the slopes at time = t and time = t+step
    m_pfDeltaEffectors_k0  = new double[m_unNumberOfReceptors];
    m_pfDeltaRegulators_k0 = new double[m_unNumberOfReceptors];
    m_pfDeltaEffectors_k1  = new double[m_unNumberOfReceptors];
    m_pfDeltaRegulators_k1 = new double[m_unNumberOfReceptors];


    //Allocated and Deleted in RobotAgent class
    //m_punFeaturesSensed = new unsigned int[m_unNumberOfReceptors];
    m_pfAPCs		      = new double[m_unNumberOfReceptors]; // In this implementation, each type of APC presents one FV.


    m_pfEffectorConjugatesPerAPC  = new double[m_unNumberOfReceptors];
    m_pfRegulatorConjugatesPerAPC = new double[m_unNumberOfReceptors];

    m_pfConjugates = new double* [m_unNumberOfReceptors];
    m_pfConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConjugates[i] = m_pfConjugates[i-1] + m_unNumberOfReceptors;
    }

    m_pfConjugates_tmp = new double* [m_unNumberOfReceptors];
    m_pfConjugates_tmp[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConjugates_tmp[i] = m_pfConjugates_tmp[i-1] + m_unNumberOfReceptors;
    }

    m_pfConjugates_Eu = new double* [m_unNumberOfReceptors];
    m_pfConjugates_Eu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConjugates_Eu[i] = m_pfConjugates_Eu[i-1] + m_unNumberOfReceptors;
    }

    m_pfAffinities = new double* [m_unNumberOfReceptors];
    m_pfAffinities[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfAffinities[i] = m_pfAffinities[i-1] + m_unNumberOfReceptors;
    }


    m_pfEffectorConjugates = new double* [m_unNumberOfReceptors];
    m_pfEffectorConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfEffectorConjugates[i] = m_pfEffectorConjugates[i-1] + m_unNumberOfReceptors;
    }


    m_pfRegulatorConjugates = new double* [m_unNumberOfReceptors];
    m_pfRegulatorConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfRegulatorConjugates[i] = m_pfRegulatorConjugates[i-1] + m_unNumberOfReceptors;
    }



    m_pfDeltaConjugates_k0 = new double* [m_unNumberOfReceptors];
    m_pfDeltaConjugates_k0[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfDeltaConjugates_k0[i] = m_pfDeltaConjugates_k0[i-1] + m_unNumberOfReceptors;
    }

    m_pfDeltaConjugates_k1 = new double* [m_unNumberOfReceptors];
    m_pfDeltaConjugates_k1[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfDeltaConjugates_k1[i] = m_pfDeltaConjugates_k1[i-1] + m_unNumberOfReceptors;
    }

    m_pfConj_tmp_Eu = new double* [m_unNumberOfReceptors];
    m_pfConj_tmp_Eu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConj_tmp_Eu[i] = m_pfConj_tmp_Eu[i-1] + m_unNumberOfReceptors;
    }

    m_pfConj_tmp_Hu = new double* [m_unNumberOfReceptors];
    m_pfConj_tmp_Hu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConj_tmp_Hu[i] = m_pfConj_tmp_Hu[i-1] + m_unNumberOfReceptors;
    }




    for (unsigned int i = 0; i < m_unNumberOfReceptors; i++)
    {
        m_pfEffectors[i]       = currE;
        m_pfRegulators[i]      = currR;

        m_pfAPCs[i]            = 0.0;
        m_pfEffectorConjugatesPerAPC[i]  = 0.0;
        m_pfRegulatorConjugatesPerAPC[i] = 0.0;

        for (unsigned int j = 0; j < m_unNumberOfReceptors; j++)
        {
            m_pfAffinities[i][j]              = NegExpDistAffinity(i,j,m_fcross_affinity);
            //printf("Af(%d,%d)=%f\n",i,j,m_pfAffinities[i][j]);

            m_pfConjugates[i][j]              = 0.0;
            m_pfConjugates_tmp[i][j]          = 0.0;
            m_pfConjugates_Eu[i][j]           = 0.0;

            m_pfEffectorConjugates[i][j]      = 0.0;
            m_pfRegulatorConjugates[i][j]     = 0.0;
        }
    }

    sites = 3U;
    step_h = 1.0;

    assert(sites == 3U);
    assert(m_fcross_affinity > 0.0);
}

/******************************************************************************/
/******************************************************************************/

CRMinRobotAgent::~CRMinRobotAgent()
{
    delete m_pfEffectors;
    delete m_pfRegulators;
    delete m_pfEffectors_prev;
    delete m_pfRegulators_prev;

    delete m_pfAPCs;
    delete m_pfEffectorConjugatesPerAPC;
    delete m_pfRegulatorConjugatesPerAPC;

    delete m_pfEffectors_Eu;
    delete m_pfRegulators_Eu;
    delete m_pfEffectors_Hu;
    delete m_pfRegulators_Hu;

    delete m_pfDeltaEffectors_k0;
    delete m_pfDeltaRegulators_k0;
    delete m_pfDeltaEffectors_k1;
    delete m_pfDeltaRegulators_k1;


    delete [] m_pfDeltaConjugates_k0[0];
    delete [] m_pfDeltaConjugates_k0;

    delete [] m_pfDeltaConjugates_k1[0];
    delete [] m_pfDeltaConjugates_k1;

    delete [] m_pfConj_tmp_Eu[0];
    delete [] m_pfConj_tmp_Eu;

    delete [] m_pfConj_tmp_Hu[0];
    delete [] m_pfConj_tmp_Hu;


    delete [] m_pfConjugates[0];
    delete [] m_pfConjugates;

    delete [] m_pfConjugates_tmp[0];
    delete [] m_pfConjugates_tmp;

    delete [] m_pfConjugates_Eu[0];
    delete [] m_pfConjugates_Eu;

    delete [] m_pfEffectorConjugates[0];
    delete [] m_pfEffectorConjugates;

    delete [] m_pfRegulatorConjugates[0];
    delete [] m_pfRegulatorConjugates;

    delete [] m_pfAffinities[0];
    delete [] m_pfAffinities;
}


/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::SimulationStepUpdatePosition()
{
    // Convert the number of feature vectors from robot agents in the vicinity to APCs for the CRM
    Sense();

    int selectedclonaltype = Random::nextInt(m_unNumberOfReceptors);
    m_pfEffectors[selectedclonaltype]  += 10.0;
    m_pfRegulators[selectedclonaltype] += 10.0;


    // --- Numerical integration to compute m_pfEffectors[] and m_pfRegulators[] to reflect m_pfAPCs[]
    //PrintCRMDetails();
    m_bConvergenceFlag = false;
    m_dconvergence_error = 10.0;
    double convergence_errormax = -1.0;
    double integration_t = 0.0;
    double step_h = 1.0;
    bool b_prevdiff0occurance = false;
    while(integration_t < INTEGRATION_TIME)
    {
        //PrintCRMDetails();

        // Compute number of conjugates for T cells m_pfEffectors[..] + m_pfRegulators[..];
        // Stored in m_pfConjugates[i][j], the conjugates of Ti to APCj
        ConjugatesQSS(m_pfEffectors, m_pfRegulators, m_pfConjugates);
        Derivative(m_pfEffectors, m_pfRegulators, m_pfConjugates, m_pfDeltaEffectors_k0, m_pfDeltaRegulators_k0);

        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            m_pfEffectors_Eu[thtype]  = m_pfEffectors[thtype]  +
                                        step_h * m_pfDeltaEffectors_k0[thtype];
            m_pfRegulators_Eu[thtype] = m_pfRegulators[thtype] +
                                        step_h * m_pfDeltaRegulators_k0[thtype];

            if(m_pfEffectors_Eu[thtype] < 0.0) {
                m_pfEffectors_Eu[thtype] = 0.0;}

            if(m_pfRegulators_Eu[thtype] < 0.0) {
                m_pfRegulators_Eu[thtype] = 0.0;}
        }

        ConjugatesQSS(m_pfEffectors_Eu, m_pfRegulators_Eu, m_pfConjugates_Eu);
        Derivative(m_pfEffectors_Eu, m_pfRegulators_Eu, m_pfConjugates_Eu, m_pfDeltaEffectors_k1, m_pfDeltaRegulators_k1);

        double absDiffHuenEuler = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            m_pfEffectors_Hu[thtype]  = m_pfEffectors[thtype]  +
                                        0.5 * step_h *
                                        (m_pfDeltaEffectors_k0[thtype] + m_pfDeltaEffectors_k1[thtype]);

            if(m_pfEffectors_Hu[thtype] < 0.0) {
                m_pfEffectors_Hu[thtype] = 0.0;}


            if(fabs(m_pfEffectors_Hu[thtype] - m_pfEffectors_Eu[thtype]) > absDiffHuenEuler)
            {
                absDiffHuenEuler = fabs(m_pfEffectors_Hu[thtype] - m_pfEffectors_Eu[thtype]);
            }


            m_pfRegulators_Hu[thtype] = m_pfRegulators[thtype] +
                                        0.5 * step_h *
                                        (m_pfDeltaRegulators_k0[thtype] +
                                         m_pfDeltaRegulators_k1[thtype]);
            if(m_pfRegulators_Hu[thtype] < 0.0) {
                m_pfRegulators_Hu[thtype] = 0.0;}


            if(fabs(m_pfRegulators_Hu[thtype] - m_pfRegulators_Eu[thtype]) > absDiffHuenEuler)
            {
                absDiffHuenEuler = fabs(m_pfRegulators_Hu[thtype] - m_pfRegulators_Eu[thtype]);
            }
        }


        if(!(absDiffHuenEuler > 0.0))
        {
            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
            {
                printf("\n E_Hu %f, E_Eu %f        R_Hu %f, R_Eu %f",
                       m_pfEffectors_Hu[thtype],m_pfEffectors_Eu[thtype],
                       m_pfRegulators_Hu[thtype],m_pfRegulators_Eu[thtype]);
            }

            printf("\n Stepsize %e",step_h);
        }

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



        //printf("\nintegration_t = %f,absDiffHuenEuler = %e,new step_h = %f",integration_t,absDiffHuenEuler,step_h);

        convergence_errormax = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            m_pfEffectors_prev[thtype]  = m_pfEffectors[thtype];
            m_pfRegulators_prev[thtype] = m_pfRegulators[thtype];

            m_pfEffectors[thtype]  = m_pfEffectors[thtype]  + step_h * m_pfDeltaEffectors_k0[thtype];
            m_pfRegulators[thtype] = m_pfRegulators[thtype] + step_h * m_pfDeltaRegulators_k0[thtype];


            if(m_pfEffectors[thtype] < 0.0){
                m_pfEffectors[thtype] = 0.0;}
            else
            {
                if(fabs(step_h * m_pfDeltaEffectors_k0[thtype]) > convergence_errormax)
                {
                    convergence_errormax = fabs(step_h * m_pfDeltaEffectors_k0[thtype]);
                }
            }


            if(m_pfRegulators[thtype] < 0.0){
                m_pfRegulators[thtype] = 0.0;}
            else
            {
                if(fabs(step_h * m_pfDeltaRegulators_k0[thtype]) > convergence_errormax)
                {
                    convergence_errormax = fabs(step_h * m_pfDeltaRegulators_k0[thtype]);
                }
            }
        }


        m_dconvergence_error = convergence_errormax;

   /*printf("\n agent %d: m_dconvergence_error=%e,step_h=%f,slope=%e,integration time %f",
          robotAgent->GetIdentification(),
          m_dconvergence_error,step_h,m_dconvergence_error/step_h,integration_t);*/
        
   if((m_dconvergence_error - TCELL_CONVERGENCE) <= 0.00001)
        {
            m_bConvergenceFlag = true;
            break;
        }
        integration_t += step_h;

    }

    //PrintCRMDetails(70);
//    if(robotAgent->GetIdentification() == 70)
//    {
//        printf("\n agent 70: m_dconvergence_error=%e,step_h=%f,slope=%e,integration time %f",
//                                 m_dconvergence_error,step_h,m_dconvergence_error/step_h,integration_t);
//    }


    m_fWeight = 0.0;
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        m_fWeight += m_pfEffectors[thtype] + m_pfRegulators[thtype];
    }
    m_fWeight = m_fWeight * m_fWeight;
    robotAgent->SetWeight(m_fWeight);



    //We set the communication range to be twice that of the FV sensory range
    //CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights(2.0*robotAgent->GetFVSenseRange());

    // could we also select the robot from one of the 10 nearest neighbours - but in these expts. comm does not seem to be needed
    CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights(robotAgent->GetSelectedNumNearestNbrs());

    if (pcRemoteRobotAgent != NULL)
    {
        CRMinRobotAgent* crminRemoteRobotAgent = pcRemoteRobotAgent->GetCRMinRobotAgent();

        if (crminRemoteRobotAgent)
        {
            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
            {
                double currEtoSend = m_pfEffectors[thtype]  * m_fTryExchangeProbability;
                double currRtoSend = m_pfRegulators[thtype] * m_fTryExchangeProbability;

                double remoteCurrE = crminRemoteRobotAgent->GetCurrE(thtype);
                double remoteCurrR = crminRemoteRobotAgent->GetCurrR(thtype);

                double currEtoReceive = remoteCurrE * m_fTryExchangeProbability;
                double currRtoReceive = remoteCurrR * m_fTryExchangeProbability;

                crminRemoteRobotAgent->SetCurrR(thtype, remoteCurrR + currRtoSend - currRtoReceive);
                crminRemoteRobotAgent->SetCurrE(thtype, remoteCurrE + currEtoSend - currEtoReceive);

                m_pfRegulators[thtype] += currRtoReceive - currRtoSend;
                m_pfEffectors[thtype]  += currEtoReceive - currEtoSend;

            }
        }
    }

    UpdateState();

}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::ConjugatesQSS(double *E, double *R, double **C)
{
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
        {
            // since we are now deleting pathogens and the rate of deletion mayb be faster than the unbindingrate, we always start with 0 conjugates to compute QSS values
            C[thtype][apctype] = 0.0;
        }
    }

    conjstep_h = 0.001;
    double error = 1.0;
    unsigned long n_iteration = 0;
    bool b_prevdiff0occurance=false;
    //while(error > CONJ_CONVERGENCE)
    while((error - CONJ_CONVERGENCE) > 0.00001)
    {
        n_iteration++;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfDeltaConjugates_k0[thtype][apctype] =
                            ((kon * m_pfAffinities[thtype][apctype] *
                              FreeThCells(E, R, C, thtype) *
                              AvailableBindingSites(C, apctype)) -
                             koff*C[thtype][apctype]);
                }
            }
        }

        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfConj_tmp_Eu[thtype][apctype] = C[thtype][apctype] + conjstep_h *
                                                       m_pfDeltaConjugates_k0[thtype][apctype];

                    if(m_pfConj_tmp_Eu[thtype][apctype] < 0.0)
                        m_pfConj_tmp_Eu[thtype][apctype] = 0.0;
                }
            }
        }

        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfDeltaConjugates_k1[thtype][apctype] =
                            ((kon * m_pfAffinities[thtype][apctype] *
                              FreeThCells(E, R, m_pfConj_tmp_Eu, thtype) *
                              AvailableBindingSites(m_pfConj_tmp_Eu, apctype)) -
                             koff*m_pfConj_tmp_Eu[thtype][apctype]);
                }
            }
        }

        double absDiffHuenEuler = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfConj_tmp_Hu[thtype][apctype] = C[thtype][apctype] + 0.5 * conjstep_h * (m_pfDeltaConjugates_k0[thtype][apctype] + m_pfDeltaConjugates_k1[thtype][apctype]);

                    if(m_pfConj_tmp_Hu[thtype][apctype] < 0.0)
                        m_pfConj_tmp_Hu[thtype][apctype] = 0.0;


                    if(fabs(m_pfConj_tmp_Hu[thtype][apctype] - m_pfConj_tmp_Eu[thtype][apctype]) > absDiffHuenEuler)
                    {
                        absDiffHuenEuler = fabs(m_pfConj_tmp_Hu[thtype][apctype] - m_pfConj_tmp_Eu[thtype][apctype]);
                    }
                }
            }
        }



//        printf("\n\n\n\n\n");
//        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//        {
//            if(m_pfAPCs[apctype]>0.0)
//                printf("APC[%d]=%f  ",apctype,m_pfAPCs[apctype]);
//        }

//        // Print Effector and regulatory clonaltypes
//        printf("\n=================================\n");
//        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//        {
//            printf("E[%d]=%f,R[%d]=%f  ",thtype,E[thtype],thtype,R[thtype]);
//        }


//        // Print table of conjugates
//        printf("\n==============ConjEu===================\n");
//        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//        {
//            printf("\nCj:%d\t",apctype);
//            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
//            {
//                printf("%e  ",m_pfConj_tmp_Eu[thtype][apctype]);
//            }
//        }




        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && conjstep_h == CONJ_LOWERLIMIT_STEPSIZE)
            {
                printf("\n The T-cell population solution is stalled");
                exit(-1);
            }

            conjstep_h = conjstep_h / 2.0;

            if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
                        conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}

            printf("\n new conjugation step %f, n_iteration %lu",conjstep_h,n_iteration);

            b_prevdiff0occurance = true;

            continue;
        }

        b_prevdiff0occurance = false;

        assert(absDiffHuenEuler >= 0.0);
        conjstep_h *= sqrt(ERRORALLOWED_CONJ_STEPSIZE/absDiffHuenEuler);
        if(conjstep_h > CONJ_UPPERLIMIT_STEPSIZE) {
            conjstep_h = CONJ_UPPERLIMIT_STEPSIZE;}
        else if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
            conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}


        double error_max = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(E[thtype] + R[thtype] <= CELLLOWERBOUND)
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    C[thtype][apctype] = 0.0;
                }
            }
            else
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    C[thtype][apctype] = C[thtype][apctype] + conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype];
                    if(C[thtype][apctype] < 0.0)
                    {
                        C[thtype][apctype] = 0.0;
                    }
                    else
                    {
                        if(fabs(conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype]) > error_max)
                        {
                            error_max = fabs(conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype]);
                        }
                    }
                }
            }
        }
        error = error_max;


        //printf("\nConj.step=%f,error=%e",conjstep_h,error);
    }



    //Iterative procedure to compute the number of conjugates for current number of T cells
    //TODO eulers or repair the fast iterative process - need to clarify points with Jorge, use Eulers method for now
    //   double error = 1.0;
    //   while(error >= 0.01)
    //   {
    //       for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    //       {
    // 	  for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    // 	  {
    // 	      m_pfConjugates_tmp[thtype][apctype] =
    // 		  C[thtype][apctype] +
    // 		  ((kon * m_pfAffinities[thtype][apctype] *
    // 		  FreeThCells(E, R, C, thtype) * AvailableBindingSites(C, apctype)) -
    // 		  koff*C[thtype][apctype]) * conjstep_h;
    // 	  }
    //       }
    //
    //       double error_max = -1.0;
    //       for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    //       {
    // 	  for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    // 	  {
    // 	      if (fabs(C[thtype][apctype] - m_pfConjugates_tmp[thtype][apctype]) > error_max)
    // 	      {
    // 		  error_max = fabs(C[thtype][apctype] - m_pfConjugates_tmp[thtype][apctype]);
    // 	      }
    //
    // 	      C[thtype][apctype] = m_pfConjugates_tmp[thtype][apctype];
    // 	  }
    //       }
    //
    //       error = error_max;
    //TODO Add a fail safe counter to exit from this while loop
    //   }
}

void CRMinRobotAgent::Derivative(double *E, double *R, double **C, double *deltaE, double *deltaR)
{
    // Dividing the conjugates into Effector and Regulator type
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        if((E[thtype] + R[thtype]) <= CELLLOWERBOUND)
        {
            for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
            {
                m_pfEffectorConjugates[thtype][apctype]  =  0.0;
                m_pfRegulatorConjugates[thtype][apctype] =  0.0;
            }
        }
        else
        {
            for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
            {
                m_pfEffectorConjugates[thtype][apctype] =
                            (E[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];

                m_pfRegulatorConjugates[thtype][apctype] =
                            (R[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];
            }
        }
    }

    // Computing the total number of effector and regulator conjugates per APC
    for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
    {
        m_pfEffectorConjugatesPerAPC[apctype]  = 0.0;
        m_pfRegulatorConjugatesPerAPC[apctype] = 0.0;

        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            m_pfEffectorConjugatesPerAPC[apctype]  += m_pfEffectorConjugates[thtype][apctype];
            m_pfRegulatorConjugatesPerAPC[apctype] += m_pfRegulatorConjugates[thtype][apctype];
        }
    }


    // We now compute the new values of m_pfEffectors[..] and m_pfRegulators[..], based on their old values, proliferation and cell death, and thymic generation of new cells
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        double effector_incr = 0.0, regulator_incr = 0.0;

        for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
        {
            if(m_pfAPCs[apctype])
            {
                double exp_tmp1 = (9.0*m_pfAPCs[apctype]*m_pfAPCs[apctype]);
                double exp_tmp2 = (m_pfRegulatorConjugatesPerAPC[apctype] - 3.0*m_pfAPCs[apctype])*
                                  (m_pfRegulatorConjugatesPerAPC[apctype] - 3.0*m_pfAPCs[apctype]);

                double Pe = exp_tmp2 / exp_tmp1;
                effector_incr  += kpe*Pe*m_pfEffectorConjugates[thtype][apctype];

                // 		effector_incr  += kpe*Hyp(0, m_pfRegulatorConjugatesPerAPC[apctype],
                // 				      	          m_pfAPCs[apctype]*(double)sites, (double)sites) *
                // 			      ((m_pfAPCs[apctype]*(double)sites)/((m_pfAPCs[apctype]*(double)sites) - m_pfRegulatorConjugatesPerAPC[apctype])) *
                // 			      m_pfEffectorConjugates[thtype][apctype];

                double Pr = (6.0*m_pfAPCs[apctype] -m_pfEffectorConjugatesPerAPC[apctype])*
                            m_pfEffectorConjugatesPerAPC[apctype] / exp_tmp1;
                regulator_incr += kpr*Pr*m_pfRegulatorConjugates[thtype][apctype];

                // 		regulator_incr += kpr*((double)(sites-1) / (double)sites) *
                // 			      (m_pfEffectorConjugatesPerAPC[apctype]/m_pfAPCs[apctype]) *
                // 			      m_pfRegulatorConjugates[thtype][apctype];
            }
        }

        effector_incr  += se - kde * E[thtype];
        regulator_incr += sr - kdr * R[thtype];

        deltaE[thtype] = effector_incr;
        deltaR[thtype] = regulator_incr;
    }
}


double CRMinRobotAgent::FreeThCells(double* E, double* R, double** C, unsigned int thtype)
{
    double conjugatedcells = 0.0;
    for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
    {
//        if(C[thtype][apctype] < 0.0)
//        {
//            printf("\nC[%d][%d]=%f",thtype,apctype,C[thtype][apctype]);
//            printf("\nRobot id:%d, Step:%d",robotAgent->GetIdentification(),CSimulator::GetInstance()->GetSimulationStepNumber());
//            exit(-1);
//        }
//        if(E[thtype] < 0.0 || R[thtype] < 0.0)
//        {
//            printf("\nE[%d]=%e;R[%d]=%e",thtype,E[thtype],thtype,R[thtype]);
//            printf("\nRobot id:%d, Step:%d",robotAgent->GetIdentification(),CSimulator::GetInstance()->GetSimulationStepNumber());
//            exit(-1);
//        }

        conjugatedcells += C[thtype][apctype];
    }

    return E[thtype] + R[thtype] - conjugatedcells;
}
/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::AvailableBindingSites(double** C, unsigned int apctype)
{
    double conjugatedcells = 0.0;
    for(unsigned thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
    {
        conjugatedcells += C[thtype][apctype];
    }

    return m_pfAPCs[apctype]*sites - conjugatedcells;
}
/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::Factorial(double val)
{
    assert(val >= 0);

    // tgamma applies the gamma function to x. The gamma function is defined as
    // gamma (x) = integral from 0 to inf of t^(x-1) e^-t dt
    // http://www.delorie.com/gnu/docs/glibc/libc_393.html

    return tgamma(val+1.0);
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::Combination(double N, double R)
{
    if((N-R) < 0)
        return 0.0;

    return (Factorial(N) / (Factorial(N-R) * Factorial(R)));
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::Hyp(double N, double No, double M, double L)
{
    if(M < 3) // If the number of APCs*sites is < 3, return 0 - prevents 0 in denominator
    {
        return 0.0;
    }

    return (Combination(No, N) * Combination(M - No, L - N))/Combination(M, L);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::PrintCRMDetails(unsigned id)
{
    //unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    if(!(robotAgent->GetIdentification() == id))
        return;

    // Print NonZero APCs
    printf("\n");
    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    {
        if(m_pfAPCs[apctype]>0.0)
            printf("APC[%d]=%f  ",apctype,m_pfAPCs[apctype]);
    }

    // Print table of conjugates
    printf("\n=================================\n");
    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    {
        printf("\nAPC:%d\t",apctype);
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            printf("%e  ",m_pfConjugates[thtype][apctype]);
        }
    }


    // Print Effector and regulatory clonaltypes
    printf("\n=================================\n");
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        printf("E[%d]=%e,R[%d]=%e  ",thtype,m_pfEffectors[thtype],thtype,m_pfRegulators[thtype]);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::UpdateState()
{
    double E, R;

    //if(m_bConvergenceFlag)
        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
        {
            E = 0.0; R = 0.0;
            for(unsigned thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
            {
                // Brute force approach to cell generation
                //cells which are not reacting to APCs would have stable states at se/kde and sr/kdr.
                //we ignore these clonaltypes in the tolerance decision
                /*if ((fabs(m_pfEffectors[thtype] - se/kde)  < 0.00001  &&
                     fabs(m_pfRegulators[thtype] - sr/kdr) < 0.00001) ||
                    (m_pfAPCs[apctype] == 0.0))
                {
                    continue;
                }*/

                E += m_pfAffinities[thtype][apctype] * m_pfEffectors[thtype];
                R += m_pfAffinities[thtype][apctype] * m_pfRegulators[thtype];
            }

            if (((E + R) <= CELLLOWERBOUND) || fabs(E - R) <= CELLLOWERBOUND)
            {
                // Dont know - no T-cells to make decision or E approx. equal to R
                robotAgent->SetMostWantedList(apctype, 0);
            }
            else if (E > R)
            {
                // Attack
                robotAgent->SetMostWantedList(apctype, 1);
            }
            else if(R > E)
            {
                // Tolerate
                robotAgent->SetMostWantedList(apctype, 2);
            }
        }
//    else
//        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//        {
//            E = 0.0; R = 0.0;
//            for(unsigned thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
//            {
//                E += m_pfAffinities[thtype][apctype] *
//                     (m_pfEffectors[thtype] - m_pfEffectors_prev[thtype]);
//                R += m_pfAffinities[thtype][apctype] *
//                     (m_pfRegulators[thtype] - m_pfRegulators_prev[thtype]);
//            }

//            if (E > R)
//            {
//                m_pbAttack[apctype] = 1;
//                robotAgent->SetMostWantedList(apctype, true);
//            }
//            else if (R > E)
//            {
//                m_pbAttack[apctype] = 2;
//                robotAgent->SetMostWantedList(apctype, false);
//            }

//            //(E==R) case ingnored does not give any information. If we dont know, tolerate!
//        }

}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::Sense()
{
    // Ask the robot you belong to for the number of feature vectors of different types
    // returns in m_punFeaturesSensed
    unsigned int* m_punFeaturesSensed  = robotAgent->GetFeaturesSensed();

    for (int i = 0; i < m_unNumberOfReceptors; i++)
    {
        m_pfAPCs[i] = m_punFeaturesSensed[i];///(M_PI * robotAgent->GetFVSenseRange()  * robotAgent->GetFVSenseRange());
        m_pfAPCs[i] = m_fFVtoApcscaling * m_pfAPCs[i];
    }
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::NormalizedAffinity(unsigned int v1, unsigned int v2)
{
    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int unMatching  = GetNumberOfSetBits(unXoredString);

    //TODO: Have to change affinity computation
    return (double) (CFeatureVector::NUMBER_OF_FEATURES - unMatching) / (double) CFeatureVector::NUMBER_OF_FEATURES;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::NegExpDistAffinity(unsigned int v1, unsigned int v2, double k)
{
    /* k is proportional to the level of cross affinity*/
    /* k=0.01 affinity of 1 when HD is 0, else 0  */
    /* k=inf  affinity of 1 for all HD */

    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int hammingdistance  = GetNumberOfSetBits(unXoredString);

    //return 1.0 * exp(-(1.0/k) * (double)hammingdistance);
    // Should we normalize the hammingdistance when input to the exp function, or as above?

    return 1.0 * exp(-(1.0/k) * (double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES);
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetCurrE(unsigned thtype)
{
    return m_pfEffectors[thtype];
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetCurrR(unsigned thtype)
{
    return m_pfRegulators[thtype];
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetAPC(unsigned apctype)
{
    return m_pfAPCs[apctype];
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::SetCurrE(unsigned thtype, double f_currE)
{
    m_pfEffectors[thtype] = f_currE;
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::SetCurrR(unsigned thtype, double f_currR)
{
    m_pfRegulators[thtype] = f_currR;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetWeight()
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetFVtoApcScaling()
{
    return m_fFVtoApcscaling;
}

/******************************************************************************/
/******************************************************************************/

bool CRMinRobotAgent::GetConvergenceFlag()
{
    return m_bConvergenceFlag;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetConvergenceError()
{
    return m_dconvergence_error;
}

/******************************************************************************/
/******************************************************************************/

unsigned int GetNumberOfSetBits(unsigned int x)
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
