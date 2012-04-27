#include "simobject.h"

/******************************************************************************/
/******************************************************************************/

CSimObject::CSimObject(const char* pch_name) 
{
    if (pch_name != NULL)
    {
        m_pchName = (char*) malloc(strlen(pch_name) + 1);
        strcpy(m_pchName, pch_name);
    } else {
        m_pchName = NULL;
    }
}

/******************************************************************************/
/******************************************************************************/

CSimObject::~CSimObject() 
{
    // Delete children:
    TSimObjectsListIterator i = m_listSimObjectChildren.begin();
    for (i = m_listSimObjectChildren.begin(); i != m_listSimObjectChildren.end(); i++)
    {        
        delete (*i);
    }


    if (m_pchName) 
        free(m_pchName);
}

/******************************************************************************/
/******************************************************************************/

const char* CSimObject::GetName() const
{
    return m_pchName;
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::Draw(CRender* pc_render)
{
    TSimObjectsListIterator i = m_listSimObjectChildren.begin();

    for (i = m_listSimObjectChildren.begin(); i != m_listSimObjectChildren.end(); i++)
    {
        (*i)->Draw(pc_render);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::SimulationStep(unsigned int un_step_number)
{

    TSimObjectsListIterator i = m_listSimObjectChildren.begin();

    for (i = m_listSimObjectChildren.begin(); i != m_listSimObjectChildren.end(); i++)
    {
        (*i)->SimulationStep(un_step_number);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::Keypressed(int keycode)
{
    TSimObjectsListIterator i = m_listSimObjectChildren.begin();

    for (i = m_listSimObjectChildren.begin(); i != m_listSimObjectChildren.end(); i++)
    {
        (*i)->Keypressed(keycode);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::AddChild(CSimObject* pc_child)
{
    m_listSimObjectChildren.push_back(pc_child);
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::RemoveChild(CSimObject* pc_child)
{
    TSimObjectsListIterator i = m_listSimObjectChildren.begin();

    while (i != m_listSimObjectChildren.end() && (*i) != pc_child)
        i++;

    if (i == m_listSimObjectChildren.end())
    {
        ERROR2("%s tried to remove a non-existing child %s", GetName(), pc_child->GetName());
    } else {
        m_listSimObjectChildren.erase(i);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::PrintfChildren(unsigned indent)
{
    for (int j = 0; j < indent; j++)
        printf(" ");

    if (m_pchName)
        printf("%s\n", GetName());
    else
        printf("NULL\n", GetName());
        
    TSimObjectsListIterator i = m_listSimObjectChildren.begin();
    for (i = m_listSimObjectChildren.begin(); i != m_listSimObjectChildren.end(); i++)
    {
        (*i)->PrintfChildren(indent + 2);
    }
}

/******************************************************************************/
/******************************************************************************/

TSimObjectsList* CSimObject::GetChildren()
{
    return &m_listSimObjectChildren;
}

/******************************************************************************/
/******************************************************************************/
