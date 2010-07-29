#ifndef HAND_MODEL_H_
#define HAND_MODEL_H_

#include <string>

namespace opticaldataglove{

class HandModel
{

public :

    double THJ3_to_THJ2;
    double THJ2_to_THJ1;
    
    double FFJ3_to_FFJ2;
    double FFJ2_to_FFJ1;
    
    double MFJ3_to_MFJ2;
    double MFJ2_to_MFJ1;
    
    double RFJ3_to_RFJ2;
    double RFJ2_to_RFJ1;

    double LFJ3_to_LFJ2;
    double LFJ2_to_LFJ1;
    
    HandModel() :
        THJ3_to_THJ2(1.0),THJ2_to_THJ1(2.0),FFJ3_to_FFJ2(1.0),FFJ2_to_FFJ1(2.0),
        MFJ3_to_MFJ2(1.0),MFJ2_to_MFJ1(2.0),RFJ3_to_RFJ2(1.0),RFJ2_to_RFJ1(2.0),
        LFJ3_to_LFJ2(1.0),LFJ2_to_LFJ1(2.0)
    {
    }

    HandModel(HandModel& hm) :
        THJ3_to_THJ2(hm.THJ3_to_THJ2),THJ2_to_THJ1(hm.THJ2_to_THJ1),FFJ3_to_FFJ2(hm.FFJ3_to_FFJ2),FFJ2_to_FFJ1(hm.FFJ2_to_FFJ1),
        MFJ3_to_MFJ2(hm.MFJ3_to_MFJ2),MFJ2_to_MFJ1(hm.MFJ2_to_MFJ1),RFJ3_to_RFJ2(hm.RFJ3_to_RFJ2),RFJ2_to_RFJ1(hm.RFJ2_to_RFJ1),
        LFJ3_to_LFJ2(hm.LFJ3_to_LFJ2),LFJ2_to_LFJ1(hm.LFJ2_to_LFJ1)
    {
    };

    HandModel(const HandModel& hm) :
        THJ3_to_THJ2(hm.THJ3_to_THJ2),THJ2_to_THJ1(hm.THJ2_to_THJ1),FFJ3_to_FFJ2(hm.FFJ3_to_FFJ2),FFJ2_to_FFJ1(hm.FFJ2_to_FFJ1),
        MFJ3_to_MFJ2(hm.MFJ3_to_MFJ2),MFJ2_to_MFJ1(hm.MFJ2_to_MFJ1),RFJ3_to_RFJ2(hm.RFJ3_to_RFJ2),RFJ2_to_RFJ1(hm.RFJ2_to_RFJ1),
        LFJ3_to_LFJ2(hm.LFJ3_to_LFJ2),LFJ2_to_LFJ1(hm.LFJ2_to_LFJ1)
    {
}

#endif //HAND_MODEL_H_
