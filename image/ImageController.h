#ifndef __IMAGE_CONTROLLER_H__
#define __IMAGE_CONTROLLER_H__

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <boost/asio.hpp>
#include <time.h>
#include <png.h>
#include <zbar.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace zbar;
using namespace cv;
using namespace std;
#define FILE_PATH_MAX_LEN	1024

/* the meaning of return value */
#define OK		0
#define IMAGE_ERR	-1
#define PATH_ERR	-2

class ImageController
{
public:
	ImageController(const string &);
	~ImageController();

public:
	int run();
	std::string DecodeQRImage(Mat img);
	std::string GetQRInBinImg(Mat binImg);
	std::string GetQR(Mat img);
private:
	string l_szFileName;
};
#endif
