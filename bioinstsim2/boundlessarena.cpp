#include "boundlessarena.h"
#include <math.h>


/******************************************************************************/
/******************************************************************************/

CBoundlessArena::CBoundlessArena(const char* pch_name, 
                                 double f_size_x,       
                                 double f_size_y, 
                                 unsigned int un_res_x, 
                                 unsigned int un_res_y) : CArena(pch_name, f_size_x, f_size_y, un_res_x, un_res_y)
{
    g_bIsBoundless = true;
}

/******************************************************************************/
/******************************************************************************/



void CBoundlessArena::MoveAgent(CAgent* pc_agent, TPosition* pt_new_position)
{
    if (pt_new_position->m_fX > m_fSizeX / 2)
    {
        pt_new_position->m_fX -= m_fSizeX;
    } 
    else if (pt_new_position->m_fX < -m_fSizeX / 2)
    {
        pt_new_position->m_fX += m_fSizeX;
    }

    if (pt_new_position->m_fY > m_fSizeX / 2)
    {
        pt_new_position->m_fY -= m_fSizeY;
    } 
    else if (pt_new_position->m_fY < -m_fSizeY / 2)
    {
        pt_new_position->m_fY += m_fSizeY;
    }

    CArena::MoveAgent(pc_agent, pt_new_position);
}

/******************************************************************************/
/******************************************************************************/

bool CBoundlessArena::IsObstacle(TPosition* t_position)
{
    if (t_position->m_fX > m_fSizeX / 2.0 || 
        t_position->m_fX < -m_fSizeX / 2.0 ||
        t_position->m_fY > m_fSizeY / 2.0 || 
        t_position->m_fY < -m_fSizeY / 2.0)
        return true;
    else
        return false;
}

/******************************************************************************/
/******************************************************************************/

void CBoundlessArena::GetAgentsCloseTo(TAgentListList* pt_output_list, 
                              const TPosition* pt_position,
                              double f_radius)
{
    pt_output_list->clear();

    TPosition tTranslatedPosition;

    double fCellSizeX = m_fSizeX / (double) m_unResX; 
    double fCellSizeY = m_fSizeY / (double) m_unResY; 

    f_radius += max(fCellSizeX, fCellSizeY);

    // We translate all coordinates to the first quadrant:
    tTranslatedPosition.m_fX = pt_position->m_fX + (m_fSizeX / 2);
    tTranslatedPosition.m_fY = pt_position->m_fY + (m_fSizeY / 2);

    double fStartCellX = (tTranslatedPosition.m_fX - f_radius) / fCellSizeX;
    double fStartCellY = (tTranslatedPosition.m_fY - f_radius) / fCellSizeY;

    // Go to the center of the start square:
    double fStartX = (fStartCellX + (double) 0.49) * (double) fCellSizeX;
    double fStartY = (fStartCellY + (double) 0.49) * (double) fCellSizeY;

    double fEndX   = (tTranslatedPosition.m_fX + f_radius) + fCellSizeX / 2;
    double fEndY   = (tTranslatedPosition.m_fY + f_radius) + fCellSizeY / 2;

    double fCellY = fStartCellY;
    for (double fY = fStartY; fY < fEndY; fY += fCellSizeY, fCellY += 1)
    {
        int nCellY = (int) floor(fCellY);
        if (nCellY < 0)         nCellY += m_unResY;
        if (nCellY >= m_unResY) nCellY -= m_unResY;
        
        double fCellX = fStartCellX;

        for (double fX = fStartX; fX < (tTranslatedPosition.m_fX + f_radius) + fCellSizeX / 2 ; fX += fCellSizeX, fCellX += 1)
        {                        
            double fRelativeX = fabs(fX - tTranslatedPosition.m_fX);
            if (fRelativeX > m_fSizeX / 2) fRelativeX -= m_fSizeX / 2;
                
            double fRelativeY = fabs(fY - tTranslatedPosition.m_fY);
            if (fRelativeY > m_fSizeY / 2) fRelativeY -= m_fSizeY / 2;


            if (sqrt(fRelativeX * fRelativeX + fRelativeY * fRelativeY) <= f_radius)
            {
                int nCellX = (int) floor(fCellX);
                if (nCellX < 0)         nCellX += m_unResX;
                if (nCellX >= m_unResX) nCellX -= m_unResX;
                
                unsigned int unArrayPosition = nCellX + nCellY * m_unResX;
//                    printf("Array position: %d\n", unArrayPosition);
                pt_output_list->push_back(&m_plistAgents[unArrayPosition]);
            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/
