#include "agent.h"
#include "simulator.h"
#include "random.h"
#include <math.h>

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::g_unGlobalNumberOfAgentsCreated = 0;

// E-puck radius * 10:
double CAgent::RADIUS = 0.375;


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

    m_tPosition.x = 0;
    m_tPosition.y = 0;
    
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

const TVector2d* CAgent::GetPosition() const
{
    return (const TVector2d*) &m_tPosition;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetPosition(TVector2d* pt_new_position)
{
    m_tPosition = (*pt_new_position);
}

/******************************************************************************/
/******************************************************************************/

const TVector2d* CAgent::GetVelocity() const
{
    return (const TVector2d*) &m_tVelocity;
}

/******************************************************************************/
/******************************************************************************/

const TVector2d* CAgent::GetAcceleration() const
{
    return (const TVector2d*) &m_tAcceleration;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetVelocity(TVector2d* pt_new_velocity)
{
    m_tVelocity = (*pt_new_velocity);
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetAngularVelocity()
{
    return m_tAngularVelocity;
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetAngularAcceleration()
{
    return m_tAngularAcceleration;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SimulationStep(unsigned int un_step_number)
{
    TVector2d tTemp = m_tVelocity;
    double tTempAngVelocity = m_tAngularVelocity;

    SimulationStepUpdatePosition();
    m_tAcceleration = m_tVelocity;
    m_tAcceleration.x -= tTemp.x;
    m_tAcceleration.y -= tTemp.y;


    if (Vec2dLength(tTemp) > EPSILON && Vec2dLength(m_tVelocity) > EPSILON)
        m_tAngularVelocity = Vec2dAngle(m_tVelocity,tTemp);
    else
        m_tAngularVelocity = 0.0;


    m_tAngularAcceleration = m_tAngularVelocity - tTempAngVelocity;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SimulationStepUpdatePosition()
{
    TVector2d tNewPosition = { m_tPosition.x + m_tVelocity.x, 
                               m_tPosition.y + m_tVelocity.y  };

    CSimulator::GetInstance()->GetArena()->MoveAgent(this, &tNewPosition);     

    CAgent* pcCollidingAgent = GetClosestAgent(RADIUS * 2.0, ROBOT);

    if (pcCollidingAgent) 
    {        
        TVector2d vecCollidingAgentPos = *(pcCollidingAgent->GetPosition());
        TVector2d vecTemp = vecCollidingAgentPos;

        vecTemp.x = vecTemp.x - GetPosition()->x;
        vecTemp.y = vecTemp.y - GetPosition()->y;
        
        Vec2dNormalize(vecTemp);

        vecTemp.x = (-vecTemp.x * RADIUS * 2.0 + vecCollidingAgentPos.x); 
        vecTemp.y = (-vecTemp.y * RADIUS * 2.0 + vecCollidingAgentPos.y); 
        
        CSimulator::GetInstance()->GetArena()->MoveAgent(this, &vecTemp);

        //m_tVelocity.x = 0;
        //m_tVelocity.y = 0;
    }
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetMaximumSpeed(double f_max_speed)
{
    m_fMaximumSpeed = f_max_speed;
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetMaximumSpeed() const
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
            if (((*j)->GetType() == e_type || e_type == ANY) && (*j) != this)
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
    double fAngle = Random::nextDouble() * 2.0 * 3.141592;

    m_tVelocity.x = cos(fAngle) * fSpeed;
    m_tVelocity.y = sin(fAngle) * fSpeed;
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

void CAgent::MoveTowards(TVector2d t_position, double f_max_speed)
{    
    CArena* pcArena = CSimulator::GetInstance()->GetArena();
    double fArenaWidth;
    double fArenaHeight;
    pcArena->GetSize(&fArenaWidth, &fArenaHeight);
        
    if (CArena::g_bIsBoundless)
    { 

        if (fabs(t_position.x - m_tPosition.x) > fArenaWidth / 2) 
        {
            if (t_position.x < m_tPosition.x)
            {
                t_position.x += fArenaWidth;
            } else {
                t_position.x -= fArenaWidth;
            }
        }

        if (fabs(t_position.y - m_tPosition.y) > fArenaHeight / 2) 
        {
            if (t_position.y < m_tPosition.y)
            {
                t_position.y += fArenaHeight;
            } else {
                t_position.y -= fArenaHeight;
            }
        }
    }

    m_tVelocity.x = (t_position.x - m_tPosition.x);
    m_tVelocity.y = (t_position.y - m_tPosition.y);

    double fSpeed = sqrt(m_tVelocity.x * m_tVelocity.x + m_tVelocity.y * m_tVelocity.y);

    if (fSpeed > f_max_speed) 
    {
        double fModifier = fSpeed / f_max_speed;
        m_tVelocity.x /= fModifier;
        m_tVelocity.y /= fModifier;
    }
                
    TVector2d tNewPosition = { m_tPosition.x + m_tVelocity.x, 
                                   m_tPosition.y + m_tVelocity.y };
    
    if (tNewPosition.x >= fArenaWidth / 2.0)
        tNewPosition.x -= fArenaWidth;

    if (tNewPosition.x <= -fArenaWidth / 2.0)
        tNewPosition.x += fArenaWidth;

    if (tNewPosition.y >= fArenaHeight / 2.0)
        tNewPosition.y -= fArenaHeight;

    if (tNewPosition.x <= -fArenaHeight / 2.0)
        tNewPosition.x += fArenaHeight;

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
    return CountAgentsInAgentListList(&tAgentListList, f_range, e_type);    
}

/******************************************************************************/
/******************************************************************************/

CAgent* CAgent::GetClosestAgent(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    double fShortestDistanceSquared = f_range * f_range;
    CAgent* pcAgent  = NULL;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {        
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++) 
        {
            if ((*j) != this) 
            {
                double fDistanceSqaured = GetSquaredDistanceBetweenPositions((*j)->GetPosition(), GetPosition());
                if (fDistanceSqaured < fShortestDistanceSquared) 
                {
                    fShortestDistanceSquared = fDistanceSqaured;
                    pcAgent = (*j);
//                    printf("Closest agent found --- dist: %f, range: %f \n", sqrt(fDistanceSqaured), f_range);
                }
            }
        }
    }

    return pcAgent;  
}

/******************************************************************************/
/******************************************************************************/


TVector2d CAgent::GetCenterOfMassOfSurroundingAgents(double f_range, EAgentType e_type)
{   
    double fArenaWidth;
    double fArenaHeight;
    CArena* pcArena = CSimulator::GetInstance()->GetArena();
    pcArena->GetSize(&fArenaWidth, &fArenaHeight);

    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TVector2d tCenter = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {        
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++) 
        {
            if ((*j)->m_bTempWithInRange) 
            {
                TVector2d posAgent = { (*j)->GetPosition()->x, (*j)->GetPosition()->y} ;

                if (fabs(posAgent.x - m_tPosition.x) > fArenaWidth / 2) 
                {
                    if (posAgent.x < m_tPosition.x)
                    {
                        posAgent.x += fArenaWidth;
                    } else {
                        posAgent.x -= fArenaWidth;
                    }
                }

                if (fabs(posAgent.y - m_tPosition.y) > fArenaHeight / 2) 
                {
                    if (posAgent.y < m_tPosition.y)
                    {
                        posAgent.y += fArenaHeight;
                    } else {
                        posAgent.y -= fArenaHeight;
                    }
                }

                tCenter.x += posAgent.x - m_tPosition.x;
                tCenter.y += posAgent.y - m_tPosition.y;
                unCount++;
            }
        }
    }

    if (unCount > 0) 
    {
        tCenter.x /= (double) unCount;
        tCenter.y /= (double) unCount;
        
        tCenter.x += m_tPosition.x;
        tCenter.y += m_tPosition.y;
    }

    return tCenter;

}

/******************************************************************************/
/******************************************************************************/

TVector2d CAgent::GetAverageVelocityOfSurroundingAgents(double f_range, EAgentType e_type)
{   
    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TVector2d tVelocity = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {        
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++) 
        {
            if ((*j)->m_bTempWithInRange) 
            {
                tVelocity.x += (*j)->GetVelocity()->x;
                tVelocity.y += (*j)->GetVelocity()->y;
                unCount++;
            }
        }
    }

    if (unCount > 0) 
    {
        tVelocity.x /= (double) unCount;
        tVelocity.y /= (double) unCount;
    }

    return tVelocity;

}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetAverageDistanceToSurroundingAgents(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    double distance = 0;

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {        
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++) 
        {
            if ((*j)->m_bTempWithInRange) 
            {
                distance += GetDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition()); 
                unCount++;
            }
        }
    }

    if (unCount > 0) 
    {
        distance /= (double) unCount;
    } else {
        distance = f_range;
    }

    return distance;
}

/******************************************************************************/
/******************************************************************************/

TVector2d CAgent::GetAverageAccelerationOfSurroundingAgents(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList; 
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TVector2d tAcceleration = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {        
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++) 
        {
            if ((*j)->m_bTempWithInRange) 
            {
                tAcceleration.x += (*j)->GetAcceleration()->x;
                tAcceleration.y += (*j)->GetAcceleration()->y;
                unCount++;
            }
        }
    }

    if (unCount > 0) 
    {
        tAcceleration.x /= (double) unCount;
        tAcceleration.y /= (double) unCount;
    }

    return tAcceleration;
}

/******************************************************************************/
/******************************************************************************/

