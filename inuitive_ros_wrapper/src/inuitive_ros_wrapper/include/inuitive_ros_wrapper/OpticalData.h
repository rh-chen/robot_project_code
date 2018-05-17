#ifndef __OPTICALDATA__
#define __OPTICALDATA__

#include "InuDefs.h"
namespace InuDev
{
    /// \brief   Optical Data 
    class COpticalData
    {
    public:

		COpticalData() : Version(5), Baseline(float(UNDEFINED_BASELINE)) {}

        // Structure version (for backward compatibility)
        uint32_t Version;

        /// \brief StructuralParameters:

        /// \brief Translation of the real left camera relates to the unit reference system (right camera reference system) <X,Y,Z> (Extrinsic parameter)
        CPoint3D TranslationRealLeft;

        /// \brief Translation of the real right camera relates to the unit reference system <X, Y, Z>(Extrinsic parameter)
        CPoint3D TranslationRealRight;

        /// \brief Translation of the Webcam camera relates to the unit reference system <X,Y,Z> (Extrinsic parameter)
        CPoint3D TranslationRealWebcam;

        /// \brief Rotation (encoded to the 3 Rodrigues rotation  parameters) of the real left camera relates to the unit reference system (Extrinsic parameter)
        CPoint3D RotationRealLeft;

        /// \brief Rotation (encoded to the 3 Rodrigues rotation  parameters) of the real right camera relates to the unit reference system (Extrinsic parameter)
        CPoint3D RotationRealRight;

        /// \brief Rotation (encoded to the 3 Rodrigues rotation  parameters) of the real webcam camera relates to the unit reference system (Extrinsic parameter)
        CPoint3D RotationRealWebcam;

        /// \brief Translation of the rectified left camera relates to the unit reference system <X,Y,Z> (Extrinsic parameter)
        CPoint3D TranslationRectifiedLeft;

        /// \brief Translation of the rectified right camera relates to the unit reference system <X,Y,Z> (Extrinsic parameter)
        CPoint3D TranslationRectifiedRight;

        /// \brief Rotation (encoded to the 3 Rodrigues rotation  parameters) of the rectified left camera relates to the unit reference system (Extrinsic parameter)
        CPoint3D RotationRectifiedLeft;

        /// \brief Rotation (encoded to the 3 Rodrigues rotation  parameters) of the rectified right camera relates to unit reference system (Extrinsic parameter)
        CPoint3D RotationRectifiedRight;


        /// \brief OpticalParameters :

        /// \brief Image resolution according to Caltech(Bypass resolution)
        CPoint2D SensorDimensions;

        /// \brief Focal Length <X,Y> of the real left camera (Intrinsic parameter)
        CPoint2D FocalLengthRealLeft;

        /// \brief Focal Length <X,Y> of the real right camera (Intrinsic parameter)
        CPoint2D FocalLengthRealRight;

        /// \brief Focal Length <X,Y> of the real webcam camera (Intrinsic parameter)
        CPoint2D FocalLengthRealWebcam;

        /// \brief Optical center <X,Y> of the real left camera (Intrinsic parameter)
        CPoint2D OpticalCenterRealLeft;

        /// \brief Optical center <X,Y> of the real right camera (Intrinsic parameter)
        CPoint2D OpticalCenterRealRight;

        /// \brief Optical center <X,Y> of the Webcam sensor  (Intrinsic parameter)
        CPoint2D OpticalCenterRealWebcam;

        /// \brief Real left camera Distortion constants <K1,K2,K3,K4,K5> (Intrinsic parameter)
        float LensDistortionsRealLeft[5];

        /// \brief Real right camera Distortion constants <K1,K2,K3,K4,K5> (Intrinsic parameter)
        float LensDistortionsRealRight[5];

        /// \brief Webcam Distortion constants <K1,K2,K3,K4,K5> (Intrinsic parameter)
        float LensDistortionsRealWebcam[5];


        /// \brief RectifiedParameters :

        /// \brief Sensor size (resolution) <X,Y> according to run_dsr (not necessarily the final output resolution)
        CPoint2D DimensionsBase;

        /// \brief Focal Length of the left sensor (Intrinsic parameter)
        CPoint2D FocalLengthBaseLeft;

        /// \brief Focal Length of the Right sensor (Intrinsic parameter)
        CPoint2D FocalLengthBaseRight;

        /// \brief Optical center <X,Y> of the left sensor (Intrinsic parameter)
        CPoint2D OpticalCenterBaseLeft;

        /// \brief Optical center <X,Y> of the right sensor (Intrinsic parameter)
        CPoint2D OpticalCenterBaseRight;

        /// \brief Internal use for fine tuning
        CPoint2D TranslateUV;

		/// \brief General Parameters :
		CPoint2D Squint;

		/// \brief	Physical distance between stereo cameras (left and right)
		float Baseline;

		/// \brief	Baseline is not defined in old versions of COpticalData
		static const int UNDEFINED_BASELINE = -1;

        /// \brief    true if and only if Webcam parameters are valid
        bool  WebcamDataValid; //  from version 3

        // AccelerometerMisalignment
        float AccelerometerMisalignment[9];
        float AccelerometerScale[9];
        float AccelerometerBias[3];
        float AccelerometerNoise[3];
        float AccelerometerTemperatureSlope[3];
        float AccelerometerTemperatureReference;

        float GyroscopeMisalignment[9];
        float GyroscopeScale[9];
        float GyroscopeBias[3];
        float GyroscopeNoise[3];
        float GyroscopeTemperatureSlope[3];
        float GyroscopeTemperatureReference;
        float TimeAlignment;
        float TCam2IMU[16];

        bool  IMUParametersValid; //  from version 3

    };
}

#endif
