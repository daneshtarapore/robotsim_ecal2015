#include "behavior.h"

class CBehavior 
{
public:
    CBehavior(CAgent* pc_agent);
    virtual ~CBehavior();

    virtual bool TakeControl() = NULL;
    virtual void Suppress();
    virtual void Action();
    virtual void SimulationStep();

protected:
    CAgent*   m_pcAgent;
};
