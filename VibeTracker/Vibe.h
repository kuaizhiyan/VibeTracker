#pragma once
#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

#define NUM_SAMPLES 20		//每个像素点的样本个数
#define MIN_MATCHES 2		//#min指数
#define RADIUS 20		//Sqthere半径
#define SUBSAMPLE_FACTOR 16	//子采样概率，决定背景更新的概率  


class ViBe_BGS
{
public:
	ViBe_BGS(void); //构造函数  
	~ViBe_BGS(void);//析构函数，对开辟的内存做必要的清理工作  

	void init(const Mat _image);   //初始化
	void processFirstFrame(const Mat _image); //利用第一帧进行建模   
	void testAndUpdate(const Mat _image);  //判断前景与背景，并进行背景跟新  
	Mat getMask(void) { return m_mask; };//得到前景  


	
private:
	Mat m_samples[NUM_SAMPLES];  //每一帧图像的每一个像素的样本集  
	Mat m_foregroundMatchCount; //统计像素被判断为前景的次数，便于更新  
	Mat m_mask;					//前景提取后的掩膜，灰度图 0 为背景，255 为前景
};