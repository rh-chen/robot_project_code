
#ifndef __DEPTHSTREAMEXT_H__
#define __DEPTHSTREAMEXT_H__

#include "DepthStream.h"

namespace InuDev
{
    struct CSensorParamsExt;

    struct CDepthFilterParams 
    {
        enum InputTypes
        {
            eDepth = 0,
            eDisparityNoStretch = 1,
            eDisparityStretch = 2,
            eBasicOptions = 2,
            eLeftY = 3,
            eRightY = 4,
            eOption6 = 6,
            eOption7 = 7,
            eOption8 = 8,
            eOption9 = 9
        };

        enum ColorTypes
        {
            eColor = 0,
            eGray = 1
        };


        CDepthFilterParams() : filterType(eDepth), colorType(eColor){}
        InputTypes filterType;
        ColorTypes colorType;

    };

    ////////////////////////////////////////////////////////////////////////
    /// \brief DepthStream extension class 
    ///
    ///
    /// Role: Adds extended features to the DepthStream interface
    ///       Intended for internal use
    ///
    ///
    /// Responsibilities: 
    ///      1. Add extended API interface
    ///
    /// Comments: 
    ////////////////////////////////////////////////////////////////////////
    class CDepthStreamExt : public CDepthStream
    {

    public:

        //typedef std::function<void(std::shared_ptr<CDepthStreamExt> , const CImageFrame&,  CInuError)> CallbackFunctionExt;

        CDepthStreamExt() {}
        virtual ~CDepthStreamExt() {}

        //Extended methods

        virtual CInuError         SetDisplayParams(const CDepthFilterParams& params) = 0;
        virtual CInuError         GetDisplayParams(CDepthFilterParams& params) = 0;
    protected:


    private:

    };
}

#endif // __DEPTHSTREAMEXT_H__

