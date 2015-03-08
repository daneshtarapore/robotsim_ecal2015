#include "rectangulararena.h"
#include <math.h>

/******************************************************************************/
/******************************************************************************/

CRectangularArena::CRectangularArena(
    const char* pch_name, 
    double f_size_x,       
    double f_size_y, 
    unsigned int un_res_x, 
    unsigned int un_res_y) 
    : 
    CArena(pch_name, f_size_x, f_size_y, un_res_x, un_res_y)
{
}        

/******************************************************************************/
/******************************************************************************/

CRectangularArena::~CRectangularArena()
{
}

/******************************************************************************/
/******************************************************************************/

bool CRectangularArena::IsObstacle(TVector2d* t_position)
{
    if (fabs(t_position->x) < m_fSizeX / 2.0 && fabs(t_position->y) < m_fSizeY / 2.0)
        return false;
    else
        return true;
}


/******************************************************************************/
/******************************************************************************/

void CRectangularArena::MoveAgent(CAgent* pc_agent, TVector2d* pt_new_position)
{
    if (!IsObstacle(pt_new_position)) {

        unsigned int unOldArrayPosition = XYToArrayPosition(pc_agent->GetPosition());
        unsigned int unNewArrayPosition = XYToArrayPosition(pt_new_position); 

        pc_agent->SetPosition(pt_new_position);

        if (unOldArrayPosition != unNewArrayPosition)
        {
            RemoveAgent(pc_agent, unOldArrayPosition);
            AddAgent(pc_agent, unNewArrayPosition);
        }
    }
}

/******************************************************************************/
/******************************************************************************/

