#ifndef BOUNDLESSARENA_H
#define BOUNDLESSARENA_H_

/******************************************************************************/
/******************************************************************************/

#include "arena.h"

/******************************************************************************/
/******************************************************************************/

class CBoundlessArena : public CArena
{
public:
    CBoundlessArena(const char* pch_name, 
                    double f_size_x,       
                    double f_size_y, 
                    unsigned int un_res_x, 
                    unsigned int un_res_y);

    virtual void MoveAgent(CAgent* pc_agent, TPosition* pt_new_position);

    virtual bool IsObstacle(TPosition* t_position);

    virtual void GetAgentsCloseTo(TAgentListList* pt_output_list, 
                                  const TPosition* pt_position,
                                  double f_radius);
};

/******************************************************************************/
/******************************************************************************/

#endif
    

