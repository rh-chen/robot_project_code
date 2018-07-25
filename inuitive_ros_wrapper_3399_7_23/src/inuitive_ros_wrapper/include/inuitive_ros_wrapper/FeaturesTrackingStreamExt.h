#ifndef __FEATURE_TRACKINGSTREAMEXT_H__  
#define __FEATURE_TRACKINGSTREAMEXT_H__

#include "FeaturesTrackingStream.h"
#include "FeaturesTrackingFrameExt.h"

namespace InuDev
{

	class CFeaturesTrackingStreamExt : public CFeaturesTrackingStream
	{

	public:

		typedef std::function<void(std::shared_ptr<CFeaturesTrackingStreamExt>, const CFeaturesTrackingFrameExt&, CInuError)> CallbackFunctionExt;

		CFeaturesTrackingStreamExt() { }
		
		virtual ~CFeaturesTrackingStreamExt() {}

		using CFeaturesTrackingStream::GetFrame;
		virtual CInuError GetFrame(CFeaturesTrackingFrameExt& oStream, unsigned int iTimeout = FPS_BASED_TIMEOUT, bool iCopyLocal = true) = 0;

		using CFeaturesTrackingStream::Register;
		virtual CInuError Register(CallbackFunctionExt iCallback, unsigned int iTimeout = FPS_BASED_TIMEOUT, bool iCopyLocal = true) = 0;

	protected:


	private:

	};
}

#endif // __FEATURE_TRACKINGSTREAMEXT_H__

