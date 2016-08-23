//#define STEREO_TEST

#ifdef STEREO_TEST
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  


#include<System.h>

#define EuRoC
//#define USE_CAMERA

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

bool ReadNextFrames(cv::VideoCapture &cap0, cv::VideoCapture &cap1, cv::Mat &frame0,cv::Mat &frame1)
{

	if (!cap0.grab() || !cap1.grab())
	{
		std::cerr << "Failed to grab the camera" << std::endl;
		return false;
	}

	if (!cap0.retrieve(frame0) || !cap1.retrieve(frame1))
	{
		std::cerr << "Failed to retrieve the camera" << std::endl;
		return false;
	}
	return true;
}

int main(int argc, char **argv)
{

	// Retrieve paths to images
	string vocpath = "ORBvoc.bin";
	string setpath = "EuRoC.yaml";

	ORB_SLAM2::System SLAM(vocpath, setpath, ORB_SLAM2::System::STEREO, true);

	vector<float> vTimesTrack;

#ifdef EuRoC

	std::string binocular0 = "C:/Users/Hielamon/Desktop/ORB-SLAM/V1_02_medium/mav0/cam0/data/";
	std::string binocular1 = "C:/Users/Hielamon/Desktop/ORB-SLAM/V1_02_medium/mav0/cam1/data/";
	std::string namedata = "C:/Users/Hielamon/Desktop/ORB-SLAM/V1_02_medium/mav0/cam0/data.csv";

	std::vector<std::string> name_vec;
	std::vector<long long> timestamp;
	std::fstream fs(namedata, std::ios::in);
	if (!fs.is_open())
	{
		std::cout << "Failed to open file" << namedata << " for read" << std::endl;
		exit(-1);
	}

	std::string first_line;
	std::getline(fs, first_line);

	while (!fs.eof())
	{
		long long time;
		std::string imgname;
		fs >> time;
		fs >> imgname;
		imgname = imgname.substr(1, imgname.size());
		timestamp.push_back(time);
		name_vec.push_back(imgname);
	}

#else

#ifdef USE_CAMERA

	cv::VideoCapture capture1(1);
	cv::VideoCapture capture2(2);

#else

	/*std::string binocular0 = "../../BinocularCamera/binocular_0/%08d.jpg";
	std::string binocular1 = "../../BinocularCamera/binocular_1/%08d.jpg";*/

	/*std::string binocular0 = "C:/Funny-Working/Code/SLAM/BinocularsCalibration/JPTest/Binocular/binocular_0/%08d.jpg";
	std::string binocular1 = "C:/Funny-Working/Code/SLAM/BinocularsCalibration/JPTest/Binocular/binocular_1/%08d.jpg";*/

	std::string binocular0 = "C:/Funny-Working/Code/SLAM/BinocularsCalibration/JPTest/Binocular/binocular_0.avi";
	std::string binocular1 = "C:/Funny-Working/Code/SLAM/BinocularsCalibration/JPTest/Binocular/binocular_1.avi";

	cv::VideoCapture capture1(binocular0);
	cv::VideoCapture capture2(binocular1);

#endif // USE_CAMERA

	

	

	if (!capture1.isOpened() || !capture2.isOpened())
	{
		cout << "fail to open!" << endl;
		return -1;
	}

#endif // EuRoC

	
	bool bQuit = false;
	int flag, skipTime = 33;
	int nImages = 0;

	cv::Mat frameL, frameR;
	Size dsize = Size(640, 480);
	//Mat xfrm = Mat(dsize, CV_8UC3);

	while (!bQuit)
	{
#ifdef EuRoC
		frameL = cv::imread(binocular0 + name_vec[nImages]);
		frameR = cv::imread(binocular1 + name_vec[nImages]);

		if (frameL.empty() || frameR.empty())break;

#else
		if (!ReadNextFrames(capture1, capture2, frameL, frameR))break;
#endif // EuRoC

		//flip(frame, frame, 0);
		//resize(frame, frame, dsize);

		double tframe = counter() / frequency();

		double t1 = counter() / frequency();

		// Pass the image to the SLAM system
		SLAM.TrackStereo(frameL, frameR, tframe);

		double t2 = counter() / frequency();

		double ttrack = (t2 - t1)*1e3;

		vTimesTrack.push_back(ttrack);

		nImages++;
		// Wait to load the next frame

		if (ttrack < 34.5)
			Sleep(34.5 - ttrack);
		else
			Sleep(1);
	}

	system("PAUSE");

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

#endif // STEREO_TEST
