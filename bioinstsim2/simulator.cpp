#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

CSimulator* CSimulator::m_pcSimulator = NULL;

/******************************************************************************/
/******************************************************************************/

CSimulator::CSimulator(unsigned int un_number_of_cycles) : 
    CSimObject("CSimulator"), 
    m_unNumberOfCycles(un_number_of_cycles),
    m_bEndSimulation(false)    
{
    m_pcSimulator = this;
    m_unCurrentSimulationStep = 0;
}    


/******************************************************************************/
/******************************************************************************/

CSimulator::~CSimulator()
{
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::Run()
{
    unsigned int unCurrentStep = 0;
    m_bEndSimulation = false;    

    while (unCurrentStep < m_unNumberOfCycles && !m_bEndSimulation)
    {
        SimulationStep(unCurrentStep);
        unCurrentStep++;
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SetArena(CArena* pc_arena)
{
    m_pcArena = pc_arena;
    AddChild(m_pcArena);
}

/******************************************************************************/
/******************************************************************************/

CSimulator* CSimulator::GetInstance()
{
    return m_pcSimulator;
}

/******************************************************************************/
/******************************************************************************/

CArena* CSimulator::GetArena()
{
    return m_pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::EndSimulation()
{
    m_bEndSimulation = true;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::AddAgent(CAgent* pc_new_agent)
{
    m_tAllAgents.push_back(pc_new_agent);
    AddChild(pc_new_agent);
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::RemoveAgent(CAgent* pc_agent)
{
    TAgentListIterator i = m_tAllAgents.begin();
    while (i != m_tAllAgents.end() && (*i) != pc_agent)
        i++;

    if (i == m_tAllAgents.end())
    {        
        ERROR1("Trying to remove non-existing agent: %s!", pc_agent->GetName());
    }
    else
        m_tAllAgents.erase(i);

    RemoveChild(pc_agent);

}

/******************************************************************************/
/******************************************************************************/

TAgentList* CSimulator::GetAllAgents()
{
    return &m_tAllAgents;
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::AddAgentToDeleteList(CAgent* pc_agent) 
{
    m_tDeleteList.push_front(pc_agent);
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SimulationStep(unsigned int un_step_number)
{
    m_unCurrentSimulationStep = un_step_number;
    CSimObject::SimulationStep(un_step_number);
    
    
    m_tDeleteList.sort();
    m_tDeleteList.unique();
    
    for (TAgentListIterator i = m_tDeleteList.begin(); i != m_tDeleteList.end(); i++)
    {
        RemoveAgent(*i);        
        if ((*i)->IsInteractable())
        {
            m_pcArena->RemoveAgent(*i);
        }
        delete (*i);
    }
    m_tDeleteList.clear();
}

/******************************************************************************/
/******************************************************************************/

void CSimulator::SetExperiment(CExperiment* pc_experiment) 
{
    m_pcExperiment = pc_experiment;
    AddChild(pc_experiment);
}

/******************************************************************************/
/******************************************************************************/


CExperiment* CSimulator::GetExperiment() 
{
    return m_pcExperiment;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CSimulator::GetSimulationStepNumber() const
{
    return m_unCurrentSimulationStep;
}

/******************************************************************************/
/******************************************************************************/
