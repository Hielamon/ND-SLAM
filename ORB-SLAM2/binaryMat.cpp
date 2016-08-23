#include <binaryMat.h>

static int rct_size = 3 * sizeof(int);

void saveMat(cv::Mat &src, std::fstream &fs)
{
	int type = src.type();
	int rows = src.rows, cols = src.cols;
	int rct[3] = { rows, cols, type };
	fs.write((char*)rct, rct_size);

	fs.write((char*)src.data, src.step[0] * rows);
}

void loadMat(cv::Mat &src, std::fstream &fs)
{
	int rct[3];
	fs.read((char*)rct, rct_size);
	src.create(rct[0], rct[1], rct[2]);
	fs.read((char*)src.data, src.step[0] * rct[0]);
}