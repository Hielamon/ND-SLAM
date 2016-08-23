#define MONOCULAR_TEST

#ifdef MONOCULAR_TEST



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  


#include<System.h>

using namespace std;
using namespace cv;

double counter()
{
#if defined WIN32 || defined WIN64 || defined _WIN64 || defined WINCE
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return (double)counter.QuadPart;
#elif defined __linux || defined __linux__
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (double)tp.tv_sec * 1000000000 + tp.tv_nsec;
#else
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
	return (double)tv.tv_sec * 1000000 + tv.tv_usec;
#endif
}

double frequency()
{
#if defined WIN32 || defined WIN64 || defined _WIN64 || defined WINCE
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return (double)freq.QuadPart;
#elif defined __linux || defined __linux__
	return 1e9;
#else
	return 1e6;
#endif
}

int main(int argc, char **argv)
{

	// Retrieve paths to images
	string vocpath = "ORBvoc.bin";
	string setpath = "KITTI.yaml";

	ORB_SLAM2::System SLAM(vocpath, setpath, ORB_SLAM2::System::MONOCULAR, true);


	vector<float> vTimesTrack;

	cv::VideoCapture capture("3.mp4");

	if (!capture.isOpened())
	{
		cout << "fail to open!" << endl;
		return -1;
	}
	bool bQuit = false;
	int flag, skipTime = 33;
	int nImages = 0;

	cv::Mat frame;
	Size dsize = Size(640, 480);
	//Mat xfrm = Mat(dsize, CV_8UC3);

	while (!bQuit)
	{
		if (!capture.read(frame))
		{
			cout << "Cannot read the frame from video file" << endl;
			break;
		}

		//flip(frame, frame, 0);
		//resize(frame, frame, dsize);

		double tframe = counter() / frequency();

		double t1 = counter() / frequency();

		// Pass the image to the SLAM system
		SLAM.TrackMonocular(frame, tframe);

		double t2 = counter() / frequency();

		double ttrack = (t2 - t1)*1e3;

		vTimesTrack.push_back(ttrack);

		nImages++;
		// Wait to load the next frame

		if (ttrack < 34.5)
			Sleep(34.5 - ttrack);
		else
			Sleep(1);

		//flag = cvWaitKey(skipTime);
		//switch (flag)
		//{
		//case 'q':
		//case 'Q':
		//case 27:	bQuit = true;				break;	//Esc
		//default:	break;
		//}
	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	std::sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	std::cout << "-------" << endl << endl;
	std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	std::cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;
}

#endif // MONOCULAR_TEST
