#ifndef AGENT_H_
#define AGENT_H_

/******************************************************************************/
/******************************************************************************/

class CAgent;

#include "common.h"

/******************************************************************************/
/******************************************************************************/

typedef list<CAgent*>               TAgentList;
typedef list<CAgent*>::iterator     TAgentListIterator;
typedef list<TAgentList*>           TAgentListList;
typedef list<TAgentList*>::iterator TAgentListListIterator;

typedef vector<CAgent*>             TAgentVector;
typedef vector<CAgent*>::iterator   TAgentVectorIterator;


/******************************************************************************/
/******************************************************************************/

typedef struct {
    float fRed;
    float fGreen;
    float fBlue;
} TColor3f;
       


enum EControllerType 
{
    RANDOMWALK,
    REGULARBOUNCE,
    RANDOMBOUNCE
};


enum EAgentType 
{
    ANY,
    ROBOT,
    LIGHT
};

/******************************************************************************/
/******************************************************************************/

class CAgent : public CSimObject
{
public: 
    CAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments);
    virtual ~CAgent();
    
    // Get the current position of the agent:
    virtual const TVector2d* GetPosition() const;

    // Set the current position of the agent:
    virtual void SetPosition(TVector2d* pt_new_position);

    // Get the current velocity of the agent:
    virtual const TVector2d* GetVelocity() const;

    // Get the current velocity of the agent:
    virtual void SetVelocity(TVector2d* pt_velocity_position);
       
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStep(unsigned int n_step_number);    
    virtual void SimulationStepUpdatePosition();

    virtual void   SetMaximumSpeed(double f_max_speed);
    virtual double GetMaximumSpeed();

    static unsigned int g_unGlobalNumberOfAgentsCreated;
    
    virtual unsigned int GetColor();
    virtual void         SetColor(unsigned int un_index);

    virtual double       GetSize(); 
       
    virtual EAgentType   GetType() = 0;
//    virtual bool         AcceptConnections() = 0;
    
    virtual unsigned int GetIdentification();

    bool    m_bTempWithInRange;
        
    virtual TVector2d    GetCenterOfMassOfSurroundingAgents(double f_range, EAgentType e_type);
    virtual TVector2d    GetAverageVelocityOfSurroundingAgents(double f_range, EAgentType e_type);
    virtual void         MarkAgentsWithinRange(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual CAgent*      GetRandomAgentWithinRange(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual unsigned int CountAgentsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual unsigned int CountAgents(double f_range, EAgentType e_type);
 
    virtual void SetRandomVelocity();    
    virtual void MoveTowards(TVector2d t_position, double f_max_speed);

protected:
    TVector2d    m_tPosition;
    TVector2d    m_tVelocity;

    double       m_fMaximumSpeed;
    double       m_fMaximumPhysicalRange_Recruitment;

    EControllerType    m_eControllerType;
    
    bool               m_bInteractable;

    unsigned int       m_unIdentification;
    unsigned int       m_unColor;

    CArguments*        m_pcArguments;
};

/******************************************************************************/
/******************************************************************************/

#endif
