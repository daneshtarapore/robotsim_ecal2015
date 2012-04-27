#ifndef AGENT_H_
#define AGENT_H_

/******************************************************************************/
/******************************************************************************/

class CAgent;

#include "common.h"

/******************************************************************************/
/******************************************************************************/

typedef list<CAgent*>           TAgentList;
typedef list<CAgent*>::iterator TAgentListIterator;
typedef list<TAgentList*>           TAgentListList;
typedef list<TAgentList*>::iterator TAgentListListIterator;

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
    THAGENT,
    APCAGENT,
    PATHOGENAGENT
};

/******************************************************************************/
/******************************************************************************/

class CAgent;

class CAgent : public CSimObject
{
public: 
    CAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments);
    virtual ~CAgent();
    
    // Get the current position of the agent:
    virtual const TPosition* GetPosition() const;

    // Set the current position of the agent:
    virtual void SetPosition(TPosition* pt_new_position);
    
    
    // Set whether other agents may be able to interact with this
    // agent or not
    virtual void SetInteractable(bool on_off);
    virtual bool IsInteractable();

    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStep(unsigned int n_step_number);    
    virtual void SimulationStepUpdatePosition();

    virtual void SetMaximumSpeed(double f_max_speed);
    virtual void SetChangeDirectionProbability(double f_prob);

    static unsigned int g_unGlobalNumberOfAgentsCreated;
    
    virtual unsigned int GetColor();
    virtual void         SetColor(unsigned int un_index);

    virtual double       GetSize(); 
       
    virtual EAgentType   GetType() = 0;
    virtual bool         AcceptConnections() = 0;
    
    virtual unsigned int GetIdentification();

    bool    m_bTempWithInRange;
    
protected:

    virtual CAgent* TryToConnectToRandomAgent(EAgentType type);
    
    virtual CAgent*      GetRandomAgentWithinRange(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual unsigned int CountAgentsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual unsigned int CountAgentsWithinPhysicalRange(EAgentType e_type);
 
    virtual void SetRandomVelocity();    
    virtual void MoveAgentTowards(TPosition t_position, double f_max_speed);

protected:
    TPosition    m_tPosition;
    TPosition    m_tVelocity;

    double       m_fMaximumSpeed;
    double       m_fMaximumPhysicalRange;
    double       m_fMaximumPhysicalRange_Recruitment;

    unsigned int m_unMaximumNumberOfPhysicalConnections;

    EControllerType    m_eControllerType;
    
    double             m_fChangeDirectionProbability;
    bool               m_bInteractable;

    unsigned int       m_unIdentification;
    unsigned int       m_unColor;

    CArguments*        m_pcArguments;
};

/******************************************************************************/
/******************************************************************************/

#endif
