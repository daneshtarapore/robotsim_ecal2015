#include "thresholdinrobotagent_optimised.h"

/******************************************************************************/
/******************************************************************************/

ThresholdinRobotAgentOptimised::ThresholdinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent,
                                                   CArguments* m_crmArguments)
{
    robotAgent = ptr_robotAgent;

    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_crmArguments->GetArgumentAsIntOr("numberoffeatures", 6);

    m_uThreshold   = m_crmArguments->GetArgumentAsIntOr("th", 1);

    m_uTolerableHD = m_crmArguments->GetArgumentAsIntOr("tol", 0); // Tolerable hamming distance THD (in bits). FVs THD apart are not classified separately

    if (m_crmArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#            Number of features in a single FV [%d]\n"
               "th=#                          Abnormal agent detection threshold [%d]\n"
               "tolhd=#                       Tolerable hamming distance [%d]\n" ,
               CFeatureVector::NUMBER_OF_FEATURES,
               m_uThreshold,
               m_uTolerableHD);
        bHelpDisplayed = true;
    }
}

/******************************************************************************/
/******************************************************************************/

ThresholdinRobotAgentOptimised::~ThresholdinRobotAgentOptimised()
{
}

/******************************************************************************/
/******************************************************************************/

void ThresholdinRobotAgentOptimised::SimulationStepUpdatePosition()
{    
    Random::nextDouble(); /* We want the same sequence of random numbers generated as with the CRM */

    UpdateState();
}

/******************************************************************************/
/******************************************************************************/

void ThresholdinRobotAgentOptimised::UpdateState()
{
    list<structFVsSensed>* fvsensed = robotAgent->GetFeatureVectorsSensed();
    list<structFVsSensed>::iterator it_fvsensed = fvsensed->begin();

    while(it_fvsensed != fvsensed->end())
    {
        double fRobotsFV = (*it_fvsensed).fRobots;

        list<structFVsSensed>::iterator itnested_fvsensed = fvsensed->begin();
        while(itnested_fvsensed != fvsensed->end())
        {
            if((*itnested_fvsensed).uFV != (*it_fvsensed).uFV)
            {
                /* XOr operation between the 2 feature vectors */
                unsigned int unXoredString = ((*it_fvsensed).uFV ^ (*itnested_fvsensed).uFV);
                /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
                unsigned int hammingdistance  = CRMinRobotAgentOptimised::GetNumberOfSetBits(unXoredString);

                if(hammingdistance <= m_uTolerableHD)
                {
                    fRobotsFV += (*itnested_fvsensed).fRobots;
                }
            }
            ++itnested_fvsensed;
        }


        //if((*it_fvsensed).fRobots <= (double)m_uThreshold)
        if(fRobotsFV <= (double)m_uThreshold)
            robotAgent->SetMostWantedList(&it_fvsensed, 1); //Attack
        else
            robotAgent->SetMostWantedList(&it_fvsensed, 2); //Tolerate

         ++it_fvsensed;
    }
}

/******************************************************************************/
/******************************************************************************/
