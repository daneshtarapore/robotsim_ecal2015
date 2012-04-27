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

bool CRectangularArena::IsObstacle(TPosition* t_position)
{
    if (fabs(t_position->m_fX) < m_fSizeX / 2 && fabs(t_position->m_fY) < m_fSizeY / 2)
        return false;
    else
        return true;
}


/******************************************************************************/
/******************************************************************************/

