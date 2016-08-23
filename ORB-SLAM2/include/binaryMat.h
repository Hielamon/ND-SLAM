#pragma once
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <fstream>


void saveMat(cv::Mat &src, std::fstream &fs);

void loadMat(cv::Mat &src, std::fstream &fs);