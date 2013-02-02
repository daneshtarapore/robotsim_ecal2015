#ifndef CRMINROBOTAGENTOPTIMISED_H
#define CRMINROBOTAGENTOPTIMISED_H

/******************************************************************************/
/******************************************************************************/
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <list>
#include "arguments.h"
#include "featurevector.h"
#include "random.h"
#include "robotagent_optimised.h"
#include "celldatacontainers.h"

/******************************************************************************/
/******************************************************************************/

#define CELLLOWERBOUND 1e-3 //todo: set as percentage instead of absolute value
// note: could result in euler-huen diff at 0, for high error thresholds. In that case, lower this value

#define CONJUGATION_OVERFLOW_LIMIT 1.0e-15  //todo: set as percentage instead of absolute value

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

class CRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

struct structTcell; struct structAPC;
enum ConjugationIntegrationPhase : unsigned {CONJ_K0, CONJ_K1, CONJ};
enum TcellIntegrationPhase : unsigned {K0, K1};

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgentOptimised
{
public:
    CRMinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent, CArguments* m_crmArguments);

    virtual ~CRMinRobotAgentOptimised();

    virtual double GetCurrE(unsigned int thtype);
    virtual double GetCurrR(unsigned int thtype);

    virtual void SetCurrE(unsigned int thtype, double f_currE);
    virtual void SetCurrR(unsigned int thtype, double f_currR);

    virtual double GetAPC(unsigned int apctype);

//    virtual inline double FreeThCells(double* E, double* R, double** C, unsigned int thtype);
//    virtual inline double AvailableBindingSites(double** C, unsigned int apctype);

    virtual void FreeTcellsAndAvailableAPCSites(TcellIntegrationPhase TK, ConjugationIntegrationPhase CONJK);

    virtual void ConjugatesQSS(bool bResetConjugates, TcellIntegrationPhase TK); //double* E, double* R, double** C);
    virtual void Derivative(TcellIntegrationPhase TK); //double* E, double* R, double** C, double* deltaE, double* deltaR);

    virtual void ConjugatesQSS_ExcessTcells(TcellIntegrationPhase TK); //double* E, double* R, double** C);
    virtual void Derivative_ExcessTcells(TcellIntegrationPhase TK); //double* E, double* R, double** C, double* deltaE, double* deltaR);
    virtual void ComputeNewDerivative(TcellIntegrationPhase TK);

    virtual inline double GetFVtoApcScaling() {return m_fFVtoApcscaling;}

    virtual inline double GetConvergenceError() {return m_dconvergence_error;}
    virtual inline double GetConvergenceError_Perc() {return m_dpercconvergence_error;}

    virtual void PrintCRMDetails(unsigned int id);
    virtual void PrintAPCList(unsigned int id);
    virtual void PrintTcellList(unsigned int id);
    virtual void PrintConjugatestoAPCList(unsigned int id);
    virtual void PrintConjugatestoTcellList(unsigned int id);

    virtual void TcellNumericalIntegration_RK2();
    virtual void SimulationStepUpdatePosition();
    virtual void DiffuseTcells();

    void ScaleDownConjugates(ConjugationIntegrationPhase CONJK);

//    double* m_pfSumEffectorsWeightedbyAffinity;
//    double* m_pfSumRegulatorsWeightedbyAffinity;

    inline list<structAPC>*    GetListAPCs() {return &listAPCs;}
    inline list<structTcell>*  GetListTcells() {return &listTcells;}

    static double NegExpDistAffinity(unsigned int v1, unsigned int v2, double k);
    static unsigned int GetNumberOfSetBits(unsigned int x);

protected:

    CRobotAgentOptimised* robotAgent;

    virtual void UpdateState();

    virtual void UpdateAPCList(); //Sense()
    virtual void SourceTcells(unsigned int hammingdistance); //unsigned hammingdistance
    virtual void UpdateConjugatesToAPCList();
    virtual void UpdateConjugatesToTcellList();
    //virtual void RemoveDeadTcellConjugatesFromAPCList();

    virtual inline double GetWeight() {return m_fWeight;}


    double step_h; double conjstep_h; // internal step count of the CRM instance
    double currE; // : Density of effector cells at the start
    double currR; // : Density of regulatory cells at the start
    double kon;   // : Conjugation rate
    double koff;  // : Dissociation rate
    double kpe;   // : Proliferation rate for effector cells
    double kde;   // : Death rate for effector cells
    double kpr;   // : Proliferation rate for regulatory cells
    double kdr;   //  Death rate for regulatory cells
    double se;    // Rate of generation of new effector cells
    double sr;    // Rate of generation of new regulatory cells
    unsigned int sites; // Number of binding sites on each APC

    // For communication of cells between robots
    double m_fTryExchangeProbability; // Probability of trying to exchange cells with other robots
    //double m_fExchangeRange;


//    double*        m_pfEffectors;
//    double*        m_pfRegulators;
//    double*        m_pfEffectors_prev;
//    double*        m_pfRegulators_prev;

//    double*        m_pfAPCs;


//    // predicted number of cells at time t+step with Euler method
//    double* m_pfEffectors_Eu;
//    double* m_pfRegulators_Eu;
//    // predicted number of cells at time t+step with Huen method
//    double* m_pfEffectors_Hu;
//    double* m_pfRegulators_Hu;
//    // the slopes at time = t and time = t+step
//    double* m_pfDeltaEffectors_k0;
//    double* m_pfDeltaRegulators_k0;
//    double* m_pfDeltaEffectors_k1;
//    double* m_pfDeltaRegulators_k1;

//    // for the computation of conjugates in QSS
//    double** m_pfDeltaConjugates_k0;
//    double** m_pfDeltaConjugates_k1;
//    double** m_pfConj_tmp_Eu;
//    double** m_pfConj_tmp_Hu;

    list<structTcell> listTcells;
    list<structAPC>   listAPCs;
    virtual inline void IncIt(list<structTcell>::iterator *it_tcell, list<structTcell>* list)
    { (*it_tcell) == list->end() ? (*it_tcell):++(*it_tcell); }
    virtual inline void IncIt(list<structAPC>::iterator *it_apc, list<structAPC>* list)
    { (*it_apc)   == list->end() ? (*it_apc):++(*it_apc); }

    unsigned int   m_unNumberOfReceptors;

//    double**        m_pfConjugates;
//    double**        m_pfConjugates_tmp;
//    double**        m_pfConjugates_Eu; //  the number of conjugates for cells at time t+step, predicted with Eulers method

//    double**        m_pfEffectorConjugates;
//    double**        m_pfRegulatorConjugates;
//    double*         m_pfEffectorConjugatesPerAPC;
//    double*         m_pfRegulatorConjugatesPerAPC;

//    double**        m_pfAffinities;

    double          m_fcross_affinity; /* the level of cross affinity*/


    double          m_fWeight;
    double          m_fFVtoApcscaling;

    bool            m_bConvergenceFlag;
    double          m_dconvergence_error;
    double          m_dpercconvergence_error;

};


/******************************************************************************/
/******************************************************************************/

#endif // CRMINROBOTAGENTOPTIMISED_H
