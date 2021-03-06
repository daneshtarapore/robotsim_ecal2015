#include "render.h"

/******************************************************************************/
/******************************************************************************/

CRender::CRender(const char* pch_name) : CSimObject(pch_name), m_fFrameRate(1)
{
}

/******************************************************************************/
/******************************************************************************/

CRender::~CRender()
{
}

/******************************************************************************/
/******************************************************************************/

void CRender::SetFrameRate(double f_frame_rate)
{
    m_fFrameRate = f_frame_rate;
}

/******************************************************************************/
/******************************************************************************/

double CRender::GetFrameRate()
{
    return m_fFrameRate;
}

/******************************************************************************/
/******************************************************************************/
