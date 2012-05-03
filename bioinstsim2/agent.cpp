#include "agent.h"
#include "simulator.h"
#include "random.h"
#include <math.h>

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::g_unGlobalNumberOfAgentsCreated = 0;

/******************************************************************************/
/******************************************************************************/


CAgent::CAgent(const char* pch_name, unsigned un_identification, CArguments* pc_arguments) :
    CSimObject(pch_name)
{    
    m_pcArguments = pc_arguments;


    const char* pchControllerType = pc_arguments->GetArgumentAsStringOr("controller", "RANDOMWALK");

    if (strcmp(pchControllerType, "RANDOMWALK") == 0) 
    {
        m_eControllerType = RANDOMWALK;
    }
    else if (strcmp(pchControllerType, "RANDOMBOUNCE") == 0)
    {
        m_eControllerType = RANDOMBOUNCE;
    }
    else if (strcmp(pchControllerType, "REGULARBOUNCE") == 0)
    {
        m_eControllerType = REGULARBOUNCE;
    } 
     
    m_fMaximumSpeed                         = pc_arguments->GetArgumentAsDoubleOr("maxspeed",             0.01);

    static bool bHelpDisplayed = false;

    if (pc_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed) 
    {
        printf("Agent help:\n"
               "  controller=[RANDOMWALK,RANDOMBOUNCE,REGULARBOUNCE]\n"
               "  maxspeed=#.#             Max speed of the agents per time-step [%f]\n"
               "  recruitment_range=#.#    Max physical distance for recruitment only [%f]\n",
               m_fMaximumSpeed,
               m_fMaximumPhysicalRange_Recruitment);
        bHelpDisplayed = true;
    }

    g_unGlobalNumberOfAgentsCreated++;

    m_tPosition.m_fX = 0;
    m_tPosition.m_fY = 0;
    
    m_unIdentification = un_identification;
    m_bInteractable    = false;
    m_unColor          = 0;

    SetRandomVelocity();
}

/******************************************************************************/
/******************************************************************************/

CAgent::~CAgent() 
{
}

/******************************************************************************/
/******************************************************************************/

const TPosition* CAgent::GetPosition() const
{
    return (const TPosition*) &m_tPosition;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetPosition(TPosition* pt_new_position)
{
    m_tPosition = (*pt_new_position);
}

/******************************************************************************/
/******************************************************************************/

const TPosition* CAgent::GetVelocity() const
{
    return (const TPosition*) &m_tVelocity;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetVelocity(TPosition* pt_new_velocity)
{
    m_tVelocity = (*pt_new_velocity);
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SimulationStep(unsigned int un_step_number)
{
    SimulationStepUpdatePosition();
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SimulationStepUpdatePosition()
{
    bool bWallHit           = false;
    bool bVerticalWallHit   = false;
    bool bHorizontalWallHit = false;

    double fSpeedFactor = 1;
    
    // PRINTPOS("Position: ", m_tPosition);
    // PRINTPOS("Velocity: ", m_tVelocity);

    TPosition tNewPosition = { m_tPosition.m_fX + m_tVelocity.m_fX * fSpeedFactor, 
                               m_tPosition.m_fY + m_tVelocity.m_fY * fSpeedFactor };

    if (CSimulator::GetInstance()->GetArena()->IsObstacle(&tNewPosition) && !CSimulator::GetInstance()->GetArena()->g_bIsBoundless)
    {
        bWallHit = true;
      
        TPosition tHorizontalTestPosition = { m_tPosition.m_fX + m_tVelocity.m_fX, m_tPosition.m_fY };
        if (CSimulator::GetInstance()->GetArena()->IsObstacle(&tHorizontalTestPosition))
        {
            bVerticalWallHit = true;
        } 


        TPosition tVerticalTestPosition = { m_tPosition.m_fX, m_tPosition.m_fY + m_tVelocity.m_fY };
        if (CSimulator::GetInstance()->GetArena()->IsObstacle(&tVerticalTestPosition))
        {
            bHorizontalWallHit = true;
        } 
    } else {
        CSimulator::GetInstance()->GetArena()->MoveAgent(this, &tNewPosition);
    }

    // if (m_eControllerType == RANDOMWALK)
    // {        
    //     if (bWallHit || Random::nextDouble() < m_fChangeDirectionProbability)
    //     {
    //         SetRandomVelocity();
    //     } 
    // } 
    // else if (m_eControllerType == RANDOMBOUNCE)
    // {
    //     if (bWallHit)
    //     {
    //         SetRandomVelocity();            
    //     } 
    // } 
    // else if (m_eControllerType == REGULARBOUNCE)
    // {
    //     if (bVerticalWallHit)
    //     {
    //         m_tVelocity.m_fX = -m_tVelocity.m_fX;
    //     }        
        
    //     if (bHorizontalWallHit)
    //     {
    //         m_tVelocity.m_fY = -m_tVelocity.m_fY;
    //     }        
    // }   
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetMaximumSpeed(double f_max_speed)
{
    m_fMaximumSpeed = f_max_speed;
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetMaximumSpeed()
{
    return m_fMaximumSpeed;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::CountAgentsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type)
{
    unsigned int unReturn = 0;
    TAgentListListIterator i;

    double fSquareRange = f_range * f_range;

    for (i = ptlist_agent_list_list->begin(); i != ptlist_agent_list_list->end(); i++)
    {
        TAgentListIterator j;
        for (j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->GetType() == e_type || e_type == ANY)
            {
                (*j)->m_bTempWithInRange = (GetSquaredDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition()) <= fSquareRange);
                if ((*j)->m_bTempWithInRange)
                    unReturn++;
            } else {
                (*j)->m_bTempWithInRange = false;
            }

        }
    }

    return unReturn;
}

/******************************************************************************/
/******************************************************************************/

// CAgent* CAgent::TryToConnectToRandomAgent(EAgentType e_type)
// {
//     CAgent* pcReturn = NULL;
    
//     TAgentListList tAgentListList; 
//     CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), m_fMaximumPhysicalRange);
//     if (tAgentListList.size() == 0)
//     {
//         ERROR2("This should never happen - the agent list-list is empty - maybe the position of the agent is wrong (%f,%f)", 
//                m_tPosition.m_fX, 
//                m_tPosition.m_fY);
//     }

//     CAgent* pcAgent = GetRandomAgentWithinRange(&tAgentListList, m_fMaximumPhysicalRange, e_type);

//     if (pcAgent != NULL && pcAgent != this)
//     {
//         if (pcAgent->AcceptConnections())
//         {
//             pcReturn = pcAgent;
//         }
//     }

//     return pcReturn;
// }

// /******************************************************************************/
// /******************************************************************************/

CAgent* CAgent::GetRandomAgentWithinRange(TAgentListList* pt_agents_list_list, double f_range, EAgentType e_type)
{
    unsigned int unNumberOfAgents = CountAgentsInAgentListList(pt_agents_list_list, f_range, e_type);
    if (unNumberOfAgents == 0)
    {
        return NULL;
    } 
    unsigned int unSelectedAgent  = Random::nextInt(0, unNumberOfAgents);

    TAgentList* ptAgentList  = NULL;
    TAgentListListIterator i = pt_agents_list_list->begin();
    CAgent* pcAgentSelected  = NULL;

    do
    {        
        while ((*i)->size() == 0) 
        {
            i++;
        }
        TAgentListIterator j = (*i)->begin();

        while (j != (*i)->end() && unSelectedAgent > 0) 
        {
            if ((*j)->m_bTempWithInRange) 
                unSelectedAgent--;
            if (unSelectedAgent > 0)
                j++;
        }

        if (unSelectedAgent > 0)
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
    } while (pcAgentSelected == NULL && i != pt_agents_list_list->end());
    
    if (i == pt_agents_list_list->end())
    {
        ERROR("The random generator seems to be wrong");
    } 
        
    return pcAgentSelected;
} 

/******************************************************************************/
/******************************************************************************/

void CAgent::SetRandomVelocity()
{
    double fSpeed = m_fMaximumSpeed;    
    double fAngle = Random::nextDouble() * 2 * 3.141592;

    m_tVelocity.m_fX = cos(fAngle) * fSpeed;
    m_tVelocity.m_fY = sin(fAngle) * fSpeed;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::GetColor() 
{
    return m_unColor;
}

/******************************************************************************/
/******************************************************************************/

double   CAgent::GetSize() 
{
    return 2.0;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::GetIdentification() 
{
    return m_unIdentification;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::MoveTowards(TPosition t_position, double f_max_speed)
{    
    CArena* pcArena = CSimulator::GetInstance()->GetArena();
   

    if (CArena::g_bIsBoundless)
    { 
        double fArenaWidth;
        double fArenaHeight;
        pcArena->GetSize(&fArenaWidth, &fArenaHeight);

        if (fabs(t_position.m_fX - m_tPosition.m_fX) > fArenaWidth / 2) 
        {
            if (t_position.m_fX < m_tPosition.m_fX)
            {
                t_position.m_fX += fArenaWidth;
            } else {
                t_position.m_fX -= fArenaWidth;
            }
        }

        if (fabs(t_position.m_fY - m_tPosition.m_fY) > fArenaHeight / 2) 
        {
            if (t_position.m_fY < m_tPosition.m_fY)
            {
                t_position.m_fY += fArenaHeight;
            } else {
                t_position.m_fY -= fArenaHeight;
            }
        }
    }

    m_tVelocity.m_fX = (t_position.m_fX - m_tPosition.m_fX);
    m_tVelocity.m_fY = (t_position.m_fY - m_tPosition.m_fY);

    double fSpeed = sqrt(m_tVelocity.m_fX * m_tVelocity.m_fX + m_tVelocity.m_fY * m_tVelocity.m_fY);

    if (fSpeed > f_max_speed) 
    {
        double fModifier = fSpeed / f_max_speed;
        m_tVelocity.m_fX /= fModifier;
        m_tVelocity.m_fY /= fModifier;
    }
                
    TPosition tNewPosition = { m_tPosition.m_fX + m_tVelocity.m_fX, 
                                   m_tPosition.m_fY + m_tVelocity.m_fY };
    
    if (!CSimulator::GetInstance()->GetArena()->IsObstacle(&tNewPosition))
        CSimulator::GetInstance()->GetArena()->MoveAgent(this, &tNewPosition);
    
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetColor(unsigned int un_color) 
{
    m_unColor = un_color;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::MarkAgentsWithinRange(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type)
{
    CountAgentsInAgentListList(ptlist_agent_list_list, f_range, e_type);    
}

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::CountAgents(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    return CountAgentsInAgentListList(tAgentListList, f_range, e_type);    
}

/******************************************************************************/
/******************************************************************************/


TPosition CAgent::GetCenterOfMassOfSurroundingAgents(double f_range, EAgentType e_type)
{   
    double fArenaWidth;
    double fArenaHeight;
    CArena* pcArena = CSimulator::GetInstance()->GetArena();
    pcArena->GetSize(&fArenaWidth, &fArenaHeight);

    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TPosition tCenter = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {        
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++) 
        {
            if ((*j)->m_bTempWithInRange) 
            {
                TPosition posAgent = { (*j)->GetPosition()->m_fX, (*j)->GetPosition()->m_fY} ;

                if (fabs(posAgent.m_fX - m_tPosition.m_fX) > fArenaWidth / 2) 
                {
                    if (posAgent.m_fX < m_tPosition.m_fX)
                    {
                        posAgent.m_fX += fArenaWidth;
                    } else {
                        posAgent.m_fX -= fArenaWidth;
                    }
                }

                if (fabs(posAgent.m_fY - m_tPosition.m_fY) > fArenaHeight / 2) 
                {
                    if (posAgent.m_fY < m_tPosition.m_fY)
                    {
                        posAgent.m_fY += fArenaHeight;
                    } else {
                        posAgent.m_fY -= fArenaHeight;
                    }
                }

                tCenter.m_fX += posAgent.m_fX - m_tPosition.m_fX;
                tCenter.m_fY += posAgent.m_fY - m_tPosition.m_fY;
                unCount++;
            }
        }
    }

    if (unCount > 0) 
    {
        tCenter.m_fX /= (double) unCount;
        tCenter.m_fY /= (double) unCount;
        
        tCenter.m_fX -= m_tPosition.m_fX;
        tCenter.m_fY -= m_tPosition.m_fY;
    }

    return tCenter;

}

/******************************************************************************/
/******************************************************************************/

TPosition CAgent::GetAverageVelocityOfSurroundingAgents(double f_range, EAgentType e_type)
{   
    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TPosition tVelocity = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {        
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++) 
        {
            if ((*j)->m_bTempWithInRange) 
            {
                tVelocity.m_fX += (*j)->GetVelocity()->m_fX;
                tVelocity.m_fY += (*j)->GetVelocity()->m_fX;
                unCount++;
            }
        }
    }

    if (unCount > 0) 
    {
        tVelocity.m_fX /= unCount;
        tVelocity.m_fY /= unCount;;
    }

    return tVelocity;

}

/******************************************************************************/
/******************************************************************************/
