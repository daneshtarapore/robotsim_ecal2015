#include "common.h"
#include <math.h>
#include "arena.h"
#include "simulator.h"

double GetDistanceBetweenPositions(const TPosition* pt_pos1, const TPosition* pt_pos2)
{
    if (CArena::g_bIsBoundless)
    {
        CArena* pcArena = CSimulator::GetInstance()->GetArena();
        double fSizeX;
        double fSizeY;
        pcArena->GetSize(&fSizeX, &fSizeY);

        double fX = fabs(pt_pos1->m_fX - pt_pos2->m_fX);
        if (fX > fSizeX / 2)
        {
            fX = fSizeX - fX;
        }
        double fY = fabs(pt_pos1->m_fY - pt_pos2->m_fY);
        if (fY > fSizeY / 2)
        {
            fY = fSizeY - fY;
        }

        return sqrt(fX * fX + fY * fY);
    }
    {
        double fX = pt_pos1->m_fX - pt_pos2->m_fX;
        double fY = pt_pos1->m_fY - pt_pos2->m_fY;

        return sqrt(fX * fX + fY * fY);
    }
}


double GetSquaredDistanceBetweenPositions(const TPosition* pt_pos1, const TPosition* pt_pos2)
{
    if (CArena::g_bIsBoundless)
    {
        CArena* pcArena = CSimulator::GetInstance()->GetArena();
        double fSizeX;
        double fSizeY;
        pcArena->GetSize(&fSizeX, &fSizeY);

        double fX = fabs(pt_pos1->m_fX - pt_pos2->m_fX);
        if (fX > fSizeX / 2)
        {
            fX = fSizeX - fX;
        }
        double fY = fabs(pt_pos1->m_fY - pt_pos2->m_fY);
        if (fY > fSizeY / 2)
        {
            fY = fSizeY - fY;
        }

        return fX * fX + fY * fY;
    }
    {
        double fX = pt_pos1->m_fX - pt_pos2->m_fX;
        double fY = pt_pos1->m_fY - pt_pos2->m_fY;

        return fX * fX + fY * fY;
    }
}
