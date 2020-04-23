
#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>

#include <unistd.h>

#include "cairo_objects.hh"
#include "display.hh"
#include "matrices.h"
#include <openvr.h>

using namespace std;
using namespace std::chrono;

#define SCREEN_RES_X 2880
#define SCREEN_RES_Y 1600

void writeTextRaster(Raster420 & yuv_raster, float pos_x, float pos_y) {
    Cairo cairo { SCREEN_RES_X, SCREEN_RES_Y };
    Pango pango { cairo };
    cairo_new_path( cairo );

    Pango::Font myfont { "Times New Roman, 80" };
    Pango::Text mystring { cairo, pango, myfont, "Eye" };
    mystring.draw_centered_at( cairo, pos_x * SCREEN_RES_X/2, pos_y * SCREEN_RES_Y/2);
    cairo_set_source_rgba( cairo, 1, 1, 1, 1 );
    cairo_fill( cairo );
    cairo.flush();

    unsigned int stride = cairo.stride();
    for ( unsigned int y = 0; y < SCREEN_RES_Y; y++ ) {
      for ( unsigned int x = 0; x < SCREEN_RES_X; x++ ) {
        yuv_raster.Y.at( x, y ) = cairo.pixels()[y * stride + 1 + ( x * 4 )];
      }
    }
  
}

//-----------------------------------------------------------------------------
// Purpose:
//------------------------------------------------------------------------------
class CMainApplication
{
public:
  CMainApplication( int argc, char* argv[] );
  virtual ~CMainApplication();

  bool BInit();

  void Shutdown();

  void RunMainLoop();

  Matrix4 GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye );
  Matrix4 GetHMDMatrixPoseEye( vr::Hmd_Eye nEye );
  Matrix4 GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );
  void UpdateHMDMatrixPose(vr::HmdVector3_t& position);

  void printDevicePositionalData( const char* deviceName,
                                  const char devClass,
                                  vr::HmdMatrix34_t posMatrix,
                                  vr::HmdVector3_t position,
                                  vr::HmdQuaternion_t quaternion );
  vr::HmdVector3_t GetPosition( vr::HmdMatrix34_t matrix );
  vr::HmdQuaternion_t GetRotation( vr::HmdMatrix34_t matrix );

  Matrix4 ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t& matPose );

private:
  vr::IVRSystem* m_pHMD;
  std::string m_strDriver;
  std::string m_strDisplay;
  vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
  Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];

  struct ControllerInfo_t
  {
    vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
    vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
    Matrix4 m_rmat4Pose;
    std::string m_sRenderModelName;
    bool m_bShowController;
  };

  enum EHand
  {
    Left = 0,
    Right = 1,
  };
  ControllerInfo_t m_rHand[2];

private: // OpenGL bookkeeping
  int m_iTrackedControllerCount;
  int m_iTrackedControllerCount_Last;
  int m_iValidPoseCount;
  int m_iValidPoseCount_Last;
  bool m_bShowCubes;
  Vector2 m_vAnalogValue;

  std::string m_strPoseClasses;                        // what classes we saw poses for this frame
  char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount]; // for each device, a character representing its class

  int m_iSceneVolumeWidth;
  int m_iSceneVolumeHeight;
  int m_iSceneVolumeDepth;
  float m_fScaleSpacing;
  float m_fScale;

  int m_iSceneVolumeInit; // if you want something other than the default 20x20x20

  float m_fNearClip;
  float m_fFarClip;

  Matrix4 m_mat4HMDPose;
  Matrix4 m_mat4eyePosLeft;
  Matrix4 m_mat4eyePosRight;

  Matrix4 m_mat4ProjectionCenter;
  Matrix4 m_mat4ProjectionLeft;
  Matrix4 m_mat4ProjectionRight;
};

//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a rising edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionRisingEdge( vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr )
{
  vr::InputDigitalActionData_t actionData;
  vr::VRInput()->GetDigitalActionData( action, &actionData, sizeof( actionData ), vr::k_ulInvalidInputValueHandle );
  if ( pDevicePath ) {
    *pDevicePath = vr::k_ulInvalidInputValueHandle;
    if ( actionData.bActive ) {
      vr::InputOriginInfo_t originInfo;
      if ( vr::VRInputError_None ==
           vr::VRInput()->GetOriginTrackedDeviceInfo( actionData.activeOrigin, &originInfo, sizeof( originInfo ) ) ) {
        *pDevicePath = originInfo.devicePath;
      }
    }
  }
  return actionData.bActive && actionData.bChanged && actionData.bState;
}

//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a falling edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionFallingEdge( vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr )
{
  vr::InputDigitalActionData_t actionData;
  vr::VRInput()->GetDigitalActionData( action, &actionData, sizeof( actionData ), vr::k_ulInvalidInputValueHandle );
  if ( pDevicePath ) {
    *pDevicePath = vr::k_ulInvalidInputValueHandle;
    if ( actionData.bActive ) {
      vr::InputOriginInfo_t originInfo;
      if ( vr::VRInputError_None ==
           vr::VRInput()->GetOriginTrackedDeviceInfo( actionData.activeOrigin, &originInfo, sizeof( originInfo ) ) ) {
        *pDevicePath = originInfo.devicePath;
      }
    }
  }
  return actionData.bActive && actionData.bChanged && !actionData.bState;
}

//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and its state is true
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionState( vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr )
{
  vr::InputDigitalActionData_t actionData;
  vr::VRInput()->GetDigitalActionData( action, &actionData, sizeof( actionData ), vr::k_ulInvalidInputValueHandle );
  if ( pDevicePath ) {
    *pDevicePath = vr::k_ulInvalidInputValueHandle;
    if ( actionData.bActive ) {
      vr::InputOriginInfo_t originInfo;
      if ( vr::VRInputError_None ==
           vr::VRInput()->GetOriginTrackedDeviceInfo( actionData.activeOrigin, &originInfo, sizeof( originInfo ) ) ) {
        *pDevicePath = originInfo.devicePath;
      }
    }
  }
  return actionData.bActive && actionData.bState;
}

//-----------------------------------------------------------------------------
// Purpose: Constructor
//-----------------------------------------------------------------------------
CMainApplication::CMainApplication( int argc, char* argv[] )
  : m_pHMD( NULL )
  , m_iValidPoseCount( 0 )
  , m_iValidPoseCount_Last( -1 )
  , m_iSceneVolumeInit( 20 )
  , m_strPoseClasses( "" ) {};

//-----------------------------------------------------------------------------
// Purpose: Destructor
//-----------------------------------------------------------------------------
CMainApplication::~CMainApplication()
{
  // work is done in Shutdown
  // dprintf( "Shutdown" );
}

//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString( vr::TrackedDeviceIndex_t unDevice,
                                    vr::TrackedDeviceProperty prop,
                                    vr::TrackedPropertyError* peError = NULL )
{
  uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
  if ( unRequiredBufferLen == 0 )
    return "";

  char* pchBuffer = new char[unRequiredBufferLen];
  unRequiredBufferLen =
    vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
  std::string sResult = pchBuffer;
  delete[] pchBuffer;
  return sResult;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::BInit()
{

  // Loading the SteamVR Runtime
  vr::EVRInitError eError = vr::VRInitError_None;
  m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

  if ( eError != vr::VRInitError_None ) {
    m_pHMD = NULL;
    char buf[1024];
    // TODO: this is only since C11
    // sprintf_s( buf, sizeof( buf ), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError
    // ) );
    // SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
    return false;
  }

  m_strDriver = "No Driver";
  m_strDisplay = "No Display";

  m_strDriver = GetTrackedDeviceString( vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String );
  m_strDisplay = GetTrackedDeviceString( vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String );

  return true;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::Shutdown()
{
  if ( m_pHMD ) {
    vr::VR_Shutdown();
    m_pHMD = NULL;
  }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::RunMainLoop()
{
  bool bQuit = false;
  VideoDisplay display { SCREEN_RES_X, SCREEN_RES_Y, false }; // fullscreen window @ 1920x1080 luma resolution
  Raster420 yuv_raster { SCREEN_RES_X, SCREEN_RES_Y };
  vr::HmdVector3_t head_position_start;
  UpdateHMDMatrixPose(head_position_start);

  vr::HmdVector3_t head_position = head_position_start;

  while ( !bQuit ) {

    UpdateHMDMatrixPose(head_position);

    writeTextRaster(yuv_raster, 0.5 - ( head_position.v[0] - head_position_start.v[0] ), 0.5 + head_position.v[1] - head_position_start.v[1]);
    Texture420 texture { yuv_raster };
    display.draw( texture );

  }
}

//-----------------------------------------------------------------------------
// Purpose: Gets a Matrix Projection Eye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye )
{
  if ( !m_pHMD )
    return Matrix4();

  vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix( nEye, m_fNearClip, m_fFarClip );

  return Matrix4( mat.m[0][0],
                  mat.m[1][0],
                  mat.m[2][0],
                  mat.m[3][0],
                  mat.m[0][1],
                  mat.m[1][1],
                  mat.m[2][1],
                  mat.m[3][1],
                  mat.m[0][2],
                  mat.m[1][2],
                  mat.m[2][2],
                  mat.m[3][2],
                  mat.m[0][3],
                  mat.m[1][3],
                  mat.m[2][3],
                  mat.m[3][3] );
}

//-----------------------------------------------------------------------------
// Purpose: Gets an HMDMatrixPoseEye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetHMDMatrixPoseEye( vr::Hmd_Eye nEye )
{
  if ( !m_pHMD )
    return Matrix4();

  vr::HmdMatrix34_t matEyeRight = m_pHMD->GetEyeToHeadTransform( nEye );
  Matrix4 matrixObj( matEyeRight.m[0][0],
                     matEyeRight.m[1][0],
                     matEyeRight.m[2][0],
                     0.0,
                     matEyeRight.m[0][1],
                     matEyeRight.m[1][1],
                     matEyeRight.m[2][1],
                     0.0,
                     matEyeRight.m[0][2],
                     matEyeRight.m[1][2],
                     matEyeRight.m[2][2],
                     0.0,
                     matEyeRight.m[0][3],
                     matEyeRight.m[1][3],
                     matEyeRight.m[2][3],
                     1.0f );

  return matrixObj.invert();
}

//-----------------------------------------------------------------------------
// Purpose: Gets a Current View Projection Matrix with respect to nEye,
//          which may be an Eye_Left or an Eye_Right.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye )
{
  Matrix4 matMVP;
  if ( nEye == vr::Eye_Left ) {
    matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
  } else if ( nEye == vr::Eye_Right ) {
    matMVP = m_mat4ProjectionRight * m_mat4eyePosRight * m_mat4HMDPose;
  }

  return matMVP;
}

vr::HmdQuaternion_t CMainApplication::GetRotation( vr::HmdMatrix34_t matrix )
{
  vr::HmdQuaternion_t q;

  q.w = sqrt( fmax( 0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = sqrt( fmax( 0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.y = sqrt( fmax( 0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.z = sqrt( fmax( 0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = copysign( q.x, matrix.m[2][1] - matrix.m[1][2] );
  q.y = copysign( q.y, matrix.m[0][2] - matrix.m[2][0] );
  q.z = copysign( q.z, matrix.m[1][0] - matrix.m[0][1] );
  return q;
}

vr::HmdVector3_t CMainApplication::GetPosition( vr::HmdMatrix34_t matrix )
{
  vr::HmdVector3_t vector;

  vector.v[0] = matrix.m[0][3];
  vector.v[1] = matrix.m[1][3];
  vector.v[2] = matrix.m[2][3];

  return vector;
}

void CMainApplication::printDevicePositionalData( const char* deviceName,
                                                  const char devClass,
                                                  vr::HmdMatrix34_t posMatrix,
                                                  vr::HmdVector3_t position,
                                                  vr::HmdQuaternion_t quaternion )
{
  // Print position and quaternion (rotation).
  fprintf( stderr,
           "%s (%c), x = %.5f, y = %.5f, z = %.5f, qw = %.5f, qx = %.5f, qy = %.5f, qz = %.5f\n",
           deviceName,
           devClass,
           position.v[0],
           position.v[1],
           position.v[2],
           quaternion.w,
           quaternion.x,
           quaternion.y,
           quaternion.z );

  // Uncomment this if you want to print entire transform matrix that contains both position and rotation matrix.
  // dprintf("\n%lld,%s,%.5f,%.5f,%.5f,x: %.5f,%.5f,%.5f,%.5f,y: %.5f,%.5f,%.5f,%.5f,z: %.5f,qw: %.5f,qx: %.5f,qy:
  // %.5f,qz: %.5f",
  //    qpc.QuadPart, whichHand.c_str(),
  //    posMatrix.m[0][0], posMatrix.m[0][1], posMatrix.m[0][2], posMatrix.m[0][3],
  //    posMatrix.m[1][0], posMatrix.m[1][1], posMatrix.m[1][2], posMatrix.m[1][3],
  //    posMatrix.m[2][0], posMatrix.m[2][1], posMatrix.m[2][2], posMatrix.m[2][3],
  //    quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::UpdateHMDMatrixPose(vr::HmdVector3_t& position_out)
{
  if ( !m_pHMD )
    return;

  vr::VRCompositor()->WaitGetPoses( m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

  m_iValidPoseCount = 0;
  m_strPoseClasses = "";
  for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice ) {
    if ( m_rTrackedDevicePose[nDevice].bPoseIsValid ) {
      m_iValidPoseCount++;
      m_rmat4DevicePose[nDevice] =
        ConvertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
      vr::HmdVector3_t position = GetPosition( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
      vr::HmdQuaternion_t quaternion = GetRotation( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
      if ( m_rDevClassChar[nDevice] == 0 ) {
        switch ( m_pHMD->GetTrackedDeviceClass( nDevice ) ) {
          case vr::TrackedDeviceClass_Controller:
            m_rDevClassChar[nDevice] = 'C';
            break;
          case vr::TrackedDeviceClass_HMD:
            m_rDevClassChar[nDevice] = 'H';
            break;
          case vr::TrackedDeviceClass_Invalid:
            m_rDevClassChar[nDevice] = 'I';
            break;
          case vr::TrackedDeviceClass_GenericTracker:
            m_rDevClassChar[nDevice] = 'G';
            break;
          case vr::TrackedDeviceClass_TrackingReference:
            m_rDevClassChar[nDevice] = 'T';
            break;
          default:
            m_rDevClassChar[nDevice] = '?';
            break;
        }
      }
      if ( m_pHMD->GetTrackedDeviceClass( nDevice ) == vr::TrackedDeviceClass_HMD ) {
        printDevicePositionalData( "HMD",
                                   m_rDevClassChar[nDevice],
                                   m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking,
                                   position,
                                   quaternion );
        position_out = position;                           
      }

      m_strPoseClasses += m_rDevClassChar[nDevice];
    }
  }

  if ( m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid ) {
    m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
    m_mat4HMDPose.invert();
  }
}

//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t& matPose )
{
  Matrix4 matrixObj( matPose.m[0][0],
                     matPose.m[1][0],
                     matPose.m[2][0],
                     0.0,
                     matPose.m[0][1],
                     matPose.m[1][1],
                     matPose.m[2][1],
                     0.0,
                     matPose.m[0][2],
                     matPose.m[1][2],
                     matPose.m[2][2],
                     0.0,
                     matPose.m[0][3],
                     matPose.m[1][3],
                     matPose.m[2][3],
                     1.0f );
  return matrixObj;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
  CMainApplication* pMainApplication = new CMainApplication( argc, argv );

  if ( !pMainApplication->BInit() ) {
    pMainApplication->Shutdown();
    return 1;
  }

  pMainApplication->RunMainLoop();

  pMainApplication->Shutdown();

  return 0;
}
