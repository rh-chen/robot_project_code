
#ifndef __FEATURSTRACKINGSTREAMEXT_H__
#define __FEATURSTRACKINGSTREAMEXT_H__

#include "FeaturesTrackingFrame.h"
//#include "BaseFrameExt.h"

namespace InuDev
{

    class CFeaturesTrackingFrameExt : public CFeaturesTrackingFrame
    {
    public:
        INUSTREAMS_API CFeaturesTrackingFrameExt (uint32_t bufferSize) : CFeaturesTrackingFrame(bufferSize) {}
        INUSTREAMS_API CFeaturesTrackingFrameExt () : CFeaturesTrackingFrame() {}

        /// \brief    Copy constructor - deep Copy.
        INUSTREAMS_API CFeaturesTrackingFrameExt(const CFeaturesTrackingFrameExt& input) : CFeaturesTrackingFrame(input) {}

        //assignment operator
        INUSTREAMS_API CFeaturesTrackingFrameExt& operator=(const CFeaturesTrackingFrameExt& input) {
        	CFeaturesTrackingFrame::operator=(input);
            return *this;
        }

        //move assignment operator
        INUSTREAMS_API CFeaturesTrackingFrameExt& operator=(CFeaturesTrackingFrameExt&& input) {
        	CFeaturesTrackingFrame::operator=(std::move(input));
            return *this;
        }

        /// \brief    Move constructor. 
        INUSTREAMS_API CFeaturesTrackingFrameExt(CFeaturesTrackingFrame&& input) : CFeaturesTrackingFrame(input) {}
        virtual ~CFeaturesTrackingFrameExt() {}
    };
}

#endif // __FEATURSTRACKINGSTREAMEXT_H__

