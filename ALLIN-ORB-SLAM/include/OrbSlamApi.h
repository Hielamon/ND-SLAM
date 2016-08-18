#ifndef ORBSLAMAPI_H_
#define ORBSLAMAPI_H_

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef ORB_SLAM2_SHARED_LIBS
#ifdef ORB_SLAM2_EXPORTS
#define ORB_SLAM2_API __declspec(dllexport)
#else
#define ORB_SLAM2_API __declspec(dllimport)
#endif
#else
#define ORB_SLAM2_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define ORB_SLAM2_API
#endif

#endif // ORBSLAMAPI_H_