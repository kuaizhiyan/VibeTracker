#include"Vibe.h"
#include"Tracker.h"
#include"ViBeT.h"
#include<iostream>
#include<cstdio>
using namespace std;

int FRAME_MAX_COL;			// 
int FRAME_MAX_ROW;

int main() {

		
	Mat frame, gray, mask;//一帧图像，灰度化，前景
	VideoCapture capture;
	//capture.open("D:\\MOT_LOCAL\\dataset\\kftest.mp4");//输入端口
	//capture.open("D:\\MOT_LOCAL\\Radar\\visiontraffic--good.avi");//输入端口
	//capture.open("D:\\MOT_LOCAL\\Radar\\test2.mp4");// MOT1 输入端口
	capture.open("D:\\MOT_LOCAL\\Radar\\car.avi");//输入端口

	if (!capture.isOpened())
	{

		cout << " ====== No camera or video input! =======\n" << endl;
		return -1;
	}

	ViBe_BGS Vibe_Bgs;			//定义一个背景差分对象  
	int count = 0;				//帧计数器，统计为第几帧   

	float start, time;

	vector<bbox> dets;			// 保存 Vibe 检测到 bboxes   "检测框"

	ViBeT mot_tracker;		// 实例化 VibeT 对象
	int flag = 0;			// 测试单目标用的标记

	int total = 0;			// 统计处理的帧数


	// 循环处理
	while (1)
	{
		start = static_cast<float>(getTickCount());
		count++;
		capture >> frame;

		mot_tracker.FRAME_MAX_COL = frame.cols;
		mot_tracker.FRAME_MAX_ROW = frame.rows;

		if (frame.empty())//直到视频最后一帧退出循环
			break;
		cvtColor(frame, gray, COLOR_RGB2GRAY);//转化为灰度图像   

		namedWindow("masked", 0);
		//resizeWindow("masked", frame.cols, frame.rows);
		namedWindow("origin", 0);
		//resizeWindow("origin", frame.cols, frame.rows);


		/////////////                   STEP 1 : 使用 Vibe 获得当前帧所有 检测框            //////////////////////////
		dets.clear();

		if (count == 1)//若为第一帧  
		{
			Vibe_Bgs.init(gray);// 初始化
			Vibe_Bgs.processFirstFrame(gray);//背景模型初始化  ,利用第一帧进行建模  
			cout << " Training ViBe complete!" << endl;
		}
		else
		{
			Vibe_Bgs.testAndUpdate(gray);	//判断前景与背景，并进行背景更新  

			mask = Vibe_Bgs.getMask();		//得到前景 

			Mat kernel = cv::getStructuringElement(MORPH_RECT, Size(7, 7));
			morphologyEx(mask, mask, MORPH_CLOSE, kernel);		// 先膨胀后腐蚀，打通连通域,默认 [3*3]
			morphologyEx(mask, mask, MORPH_CLOSE, kernel);		// 先膨胀后腐蚀，打通连通域,默认 [3*3]
			morphologyEx(mask, mask, MORPH_CLOSE, kernel);		// 先膨胀后腐蚀，打通连通域,默认 [3*3]
			morphologyEx(mask, mask, MORPH_CLOSE, kernel);		// 先膨胀后腐蚀，打通连通域,默认 [3*3]
			morphologyEx(mask, mask, MORPH_CLOSE, kernel);		// 先膨胀后腐蚀，打通连通域,默认 [3*3]
			morphologyEx(mask, mask, MORPH_CLOSE, kernel);		// 先膨胀后腐蚀，打通连通域,默认 [3*3]


			dets = getBboxs(mask,1200);
			//cout << mask << endl;
			imshow("masked", mask);
		}

		//bbox statePredict(0.0,0.0,0.0,0.0);


		/////////////                   STEP 2 : 调用 VibeT方法 做一次更新          //////////////////////////
		vector<int> trks = mot_tracker.update(dets);		// 返回可以显示的 预测框 在mot_trakcer 列表中的下标索引



		/////////////                   STEP 3 : 可视化显示出检测框和预测框          //////////////////////////
		vector<bbox> trks_draw;
		for (int i = 0; i < trks.size(); ++i) {
			trks_draw.push_back(mot_tracker.trackers[trks[i]].get_state());
		}
		if(!dets.empty()) drawBbox(frame, dets, true);
		if(!trks_draw.empty())	drawBbox(frame, trks_draw, false);
		imshow("origin", frame);

		

		time = ((float)getTickCount() - start) / getTickFrequency() * 1000;
		//cout << "Time of processing one frame: " << time << " ms " << endl;

		if (waitKey(10) == 'q')
			break;
		//1.这waitKey(10)为等待10ms;
		//2.加上while循环，即为无限等待
	}

	cv::destroyAllWindows();
	capture.release();
	
	
		


	return 0;
}