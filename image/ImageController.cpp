#include "ImageController.h"


ImageController::ImageController(const string & name)//验证路径的正确性
{	
if ( access( name.c_str(), 0 ) == -1 ) {
	cout<<"["<<__FILE__<<"]"<<"["<<__FUNCTION__<<"]"<<"["<<__LINE__<<"]"<<"Path is not exist."<<endl; 
	exit(PATH_ERR);
}
else cout<<"Path is exist."<<endl;
l_szFileName=name;
}

ImageController::~ImageController()
{
}
int ImageController::run(){
	Mat imageSource=imread(l_szFileName,0);
 	std::string result=GetQRInBinImg(imageSource);
	return 0;
}
std::string ImageController::DecodeQRImage(Mat img)
{
    std::string result;
    ImageScanner scanner;
    const void *raw = (&img)->data;
    // configure the reader
    scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
    // wrap image data
    Image image(img.cols, img.rows, "Y800", raw, img.cols * img.rows);
    // scan the image for barcodes
    scanner.scan(image);
    // extract results
    result = image.symbol_begin()->get_data();
    image.set_data(NULL, 0);//使图像不被清理
    printf("the qr code content:%s\n",result.c_str());
    return result;
}
//对二值图像进行识别，如果失败则开运算进行二次识别	膨胀腐蚀
std::string ImageController::GetQRInBinImg(Mat binImg)
{
    std::string result = DecodeQRImage(binImg);
    if(result.empty())
    {	
	printf("[%s][%s][%d] 扫描器第一次未发现符号或解析发生错误，准备开运算进行二次识别\n", __FILE__, __FUNCTION__, __LINE__);
        Mat openImg;
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));//返回指定形状和尺寸的结构元素
        morphologyEx(binImg, openImg, MORPH_OPEN, element);//	形态学运算的类型  形态学运算的内核
        result = DecodeQRImage(openImg);
	if(result.empty()) {
	printf("[%s][%s][%d] 扫描器第开运算依然无法识别，请检查图片源\n", __FILE__, __FUNCTION__, __LINE__);
	exit(IMAGE_ERR);
	}
    }
    return result;
}

//main function	阈值化操作  目前对于二维码不处于中间位置的图片处理不了  所以暂时不用
std::string ImageController::GetQR(Mat img)
{
    Mat binImg;
    //在otsu二值结果的基础上，不断增加阈值，用于识别模糊图像
    int thre = threshold(img, binImg, 0, 255, cv::THRESH_OTSU);
    std::string result;
    while(result.empty() && thre<255)
    {
        threshold(img, binImg, thre, 255, cv::THRESH_BINARY);
        result = GetQRInBinImg(binImg);
        thre += 1;//阈值步长设为20，步长越大，识别率越低，速度越快
    }
    return result;
}


