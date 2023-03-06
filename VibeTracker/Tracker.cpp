#pragma once
#include"Tracker.h"
#include<iostream>
#include<opencv2/video/tracking.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<queue>
#include<cstring>
#include <dlib/optimization/max_cost_assignment.h>

#define INF 0x3fffffff
using namespace std; 


short visit[4000][700];				// 用于 bfs 中访问控制，为避免重复申请内存声明为全局变量


cv::Mat bbox::getMat()
{
	return (cv::Mat_<float>(4, 1) << x1, y1, x2, y2);
}
cv::Mat zbox::getMat()
{
	return (cv::Mat_<float>(4, 1) << x, y, s, r);
}

/* bbox / zbox / cv::Mat 之间的转换 */
zbox convert_bbox_to_zbox(const bbox& box) {
	int w = box.x2 - box.x1;
	int h = box.y2 - box.y1;
	int x = box.x1 + w / 2.0;
	int y = box.y1 + h / 2.0;
	float s = w * h;
	float r = w / (float)h;
	return zbox(x, y, s, r);
}

bbox convert_z_to_bbox(const zbox& box) {
	float w = sqrt(box.s * box.r);		// sqrt(w*h * w/h)
	float h = box.s / w;				// w*h / w
	return bbox(box.x - w / 2.0, box.y - h / 2.0, box.x + w / 2.0, box.y + h / 2.0);
}

zbox convert_mat_to_zbox(const cv::Mat& mat) {
	// 将 (4,1) 的列向量转为 zbox
	return zbox(*mat.ptr<float>(0), *mat.ptr<float>(1), *mat.ptr<float>(2), *mat.ptr<float>(3));
}

bbox convert_mat_to_bbox(const cv::Mat& mat)
{
	return convert_z_to_bbox(convert_mat_to_zbox(mat));
}


cv::Mat convert_bbox_to_zmat(const bbox& box) {
	/* 将 bbox 转换为 zbox 的 mat 形式 */
	return (convert_bbox_to_zbox(box)).getMat();
}


/* 工具函数从 Mask 提取出所有的 bbox */

PIV bfs(int input_x, int input_y, const cv::Mat& mask) {
	/* 从一个点出发在 mask 中 BFS 遍历连通域，记录下角点 */

	// 函数外控制mask 值 为 255 的像素点才可进入本函数，因此最小的情况为 1 个像素点
	// 最好加一个非法判断，避免 非255 传入的情况
	if (mask.at<uchar>(input_x, input_y) != 255) {
		cout << "Not a valid point !" << endl;
		return make_pair(make_pair(-1, -1), make_pair(-1, -1));
	}

	int x1 = 0x3fffffff, y1 = 0x3fffffff;	// 左上点
	int x2 = -1, y2 = -1;	// 右下点
	//short visit[4000][700];				// 已在文件头声明为全局变量

	int dir[4][2] = { {0,-1},{1,0},{0,1},{0,-1} };



	// 创建队列
	queue<PII> qu;
	qu.push(make_pair(input_x, input_y));

	while (!qu.empty()) {
		// 获取队首元素
		int topx = (qu.front()).first;
		int topy = (qu.front()).second;

		// 判断更新角点
		if (topx < x1) x1 = topx;
		if (topy < y1) y1 = topy;
		if (topx > x2) x2 = topx;
		if (topy > y2) y2 = topy;


		// 循环入队
		for (int i = 0; i < 4; ++i) {
			int tmpx = topx + dir[i][0];
			int tmpy = topy + dir[i][1];

			if (tmpx >= 0 && tmpx < mask.rows && tmpy >= 0 && tmpy < mask.cols && !visit[tmpx][tmpy] && mask.at<uchar>(tmpx, tmpy) == 255) {
				qu.push(make_pair(tmpx, tmpy));
				visit[tmpx][tmpy] = 1;
			}
		};

		qu.pop();
	}


	return  make_pair(make_pair(x1, y1), make_pair(x2, y2));

};



vector<bbox> getBboxs(const cv::Mat& mat, int area) {

	vector<bbox> bboxs;
	memset(visit, 0, sizeof(visit));			// 重置 visit

	for (int i = 0; i < mat.rows; ++i)
		for (int j = 0; j < mat.cols; ++j) {
			if (mat.at<uchar>(i, j) == 255 && !visit[i][j]) {
				PIV res = bfs(i, j, mat);
				if ((res.second.first - res.first.first) * (res.second.second - res.first.second) >= area)
					bboxs.push_back(bbox(res.first.first, res.first.second, res.second.first, res.second.second));
				//printf("LU:(%d,%d)\nRD:(%d,%d)\n\n", res.first.first, res.first.second, res.second.first, res.second.second);
			}
		}
	return bboxs;
}

float iou(const bbox& box1, const bbox& box2)
{
	float xx1 = max(box1.x1, box2.x1);
	float yy1 = max(box1.y1, box2.y1);
	float xx2 = min(box1.x2, box2.x2);
	float yy2 = min(box1.y2, box2.y2);
	float w = max((float)0, xx2 - xx1);
	float h = max((float)0, yy2 - yy1);
	float wh = w * h;						//求交集面积
	//cout << "wh: " << wh << endl;
	float o = (float)wh / ((box1.x2 - box1.x1) * (box1.y2 - box1.y1) + (box2.x2 - box2.x1) * (box2.y2 - box2.y1) - wh);

	if (o <= 1e-9) return 0.0;
	return o;
}




int Tracker::cnt = 0;		//静态成员初始化

Tracker::Tracker() {

};
//
//Tracker::~Tracker(void) {
//
//};

Tracker::Tracker(bbox box) {
	/*
		使用 bbox 创建一个卡尔曼跟踪器
	*/


	const int stateNum = 7;		// 状态量的维度为 7 [u,v,s,r,u',v',s']
	const int measureNum = 4;	// 观测量的维度为 4 [u,v,s,r]
	this->KF = cv::KalmanFilter(stateNum, measureNum, 0);

	/*  跟踪器参数初始化 */
	float F[7][7] = {
		1,0,0,0,1,0,0,
		0,1,0,0,0,1,0,
		0,0,1,0,0,0,1,
		0,0,0,1,0,0,0,
		0,0,0,0,1,0,0,
		0,0,0,0,0,1,0,
		0,0,0,0,0,0,1
	};
	// 状态转移矩阵 F
	this->KF.transitionMatrix = (cv::Mat_<float>(7, 7) << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1);
	//cout << "F matrix in creater:\n" << KF.transitionMatrix << endl;
	cv::setIdentity(KF.measurementMatrix);			// 测量矩阵 H ,设置为对角阵，默认为 1
	//cout << "H matrix in creater:\n" << KF.measurementMatrix << endl;

	// 以下参数可进一步调整
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));		// 系统噪声方差矩阵 Q
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));	// 测量噪声方差矩阵 R

	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));			// 后验错误估计协方差矩阵 P
	for (int i = 4; i < 7; ++i)	KF.errorCovPost.at<float>(i, i) = 1000.0;	// 将初始误差调大

	/*  输入原始状态量 */
	//KF.statePost = convert_bbox_to_zbox(box).getMat();
	cv::Mat tmp = convert_bbox_to_zbox(box).getMat();
	for (int i = 0; i < 4; ++i)
		KF.statePost.at<float>(i, 0) = tmp.at<float>(i, 0);


	/* 辅助参数初始化 */
	this->time_since_update = 0;
	this->id = Tracker::cnt + 1;
	Tracker::cnt++;
	this->hits = this->hit_streak = this->age = 0;

}

bbox Tracker::predict() {
	// 避免面积预测值小于 0
	if (this->KF.statePost.at<float>(6, 0) + this->KF.statePost.at<float>(2, 0) <= 0.0)
		this->KF.statePost.at<float>(6, 0) *= 0.0;

	//cout << "KF.transitionMatrix\n" << KF.transitionMatrix << KF.transitionMatrix.type() << endl;
	//cout << "KF.statePost \n" << KF.statePost << KF.statePost.type() << endl;
	//cout << "KF.statePre\n" << KF.statePre << KF.statePre.type() << endl;
	//cout << "KF.errorCovPost\n" << KF.errorCovPost << KF.errorCovPost.type() << endl;
	//cout << "KF.processNoiseCov\n" << KF.processNoiseCov << KF.processNoiseCov.type() << endl;

	cv::Mat statePre = this->KF.predict();		// 获得预测结果 

	if (this->time_since_update > 0)	this->hit_streak = 0;		// ???
	this->time_since_update++;

	this->history.push_back(convert_z_to_bbox(convert_mat_to_zbox(statePre)));

	return history[history.size() - 1];

}

void Tracker::update(bbox box) {
	// 调用更新函数，意味着匹配成功
	this->time_since_update = 0;	// 重置
	this->history.clear();

	this->hits++;
	this->hit_streak++;

	this->KF.correct(convert_bbox_to_zmat(box));	// update
}

bbox Tracker::get_state() {
	zbox z = zbox(*this->KF.statePost.ptr<float>(0), *this->KF.statePost.ptr<float>(1), *this->KF.statePost.ptr<float>(2), *this->KF.statePost.ptr<float>(3));
	return convert_z_to_bbox(z);
}


vector<vector<double> >  iou_batch(const vector<bbox>& detections, const vector<bbox>& trackers) {
	/*
	返回 detections 和 trackers  bboxs 两两 iou 计算结果；
	同时取最大维度构造 nr == nc 的数组，不足补零（为了使用 max_cost_assignment 函数）

	input:
		detections: 检测框bbox vector
		trackers: 预测框 bbox vector
	output:
		iou 计算结果矩阵，二维 vector

	*/


	int num_det = detections.size();
	int num_tra = trackers.size();
	int max_num = max(num_det, num_tra);

	// 当有其一为空时，无法计算 iou 匹配的矩阵，直接返回空
	if (!num_det || !num_tra) {
		vector<vector<double> >  vec;
		return vec;
	}


	vector<vector<double> > vec(max_num, vector<double>(max_num));			// 创建空表
	for (int i = 0; i < max_num; ++i)
		for (int j = 0; j < max_num; ++j) {
			if (i >= num_det) vec[i][j] = 0;
			else if (j >= num_tra) vec[i][j] = 0;
			else {
				vec[i][j] = iou(detections[i], trackers[j]);		// 计算两两之间的 iou 值
			}

		}

	return vec;
}

void associate_detections_to_trackers(const vector<bbox>& detections, const vector<bbox>& trackers, vector<pair<int, int> >& _matches, vector<int>& _unmatched_detections, vector<int>& _unmatched_trackers, float iou_threshold = 0.3) {

	/*
	将检测框 bbox 与 预测框进行关联匹配

	input:
		detections: 检测框 bbox 数组
		trackers:	跟踪框 bbox 数组
	output:
		通过引用的方式返回
		_matches: 匹配成功的 detection/tracker 编号对
		_unmatched_deteciotns 匹配失败的 detection 编号
		_unmatched_trackers 匹配失败的 tracker 编号

	*/

	vector<pair<int, int> > vec;

	// 待返回的三个参数
	vector<pair<int, int> > matched;			// max_sum_assignment 匹配的 detection/tracker 编号对 
	vector<pair<int, int> > matches;			// 检验过的 detection/tracker 编号对
	vector<int> unmatched_detections;			// 未匹配成功的 detection 编号
	vector<int> unmatched_trackers;				// 未匹配成功的 tracker 编号

	/////////////////////////////////////////////////////
	if (trackers.empty()) {
		// 没有预测框时，直接匹配失败
		// matches 为空
		// unmatched_detections 为全部
		for (int i = 0; i < detections.size(); ++i)	unmatched_detections.push_back(i);
		// unmatched_trackers 为空

		_matches = matches;
		_unmatched_detections = unmatched_detections;
		_unmatched_trackers = unmatched_trackers;

		return;

	}

	// 计算获得 iou 矩阵 （补为方形）
	vector<vector<double> > iou_matrix = iou_batch(detections, trackers);
	// 根据 阈值 筛选
	int max_num = iou_matrix.size();
	vector<int> vec2;


	/////////////                STEP 1 : 计算 iou cost matrix               ///////////////

	// 存在匹配成功的情况	
	if (max_num > 0) {

		// 根据阈值初步筛选
		/*for (int i = 0; i < max_num; ++i)
			for (int j = 0; j < max_num; ++j)
				if (iou_matrix[i][j] < iou_threshold) iou_matrix[i][j] = 0;*/

				// 使用匈牙利算法计算 cost 矩阵（dlib 中计算的是最大代价，python 中 linear_sum_assignment 是最小）
		dlib::matrix<long> cost(max_num, max_num);

		// dlib 的 max_cost_assignment 只能接收整型，因此将 iou * 1000 再取整
		for (int i = 0; i < max_num; ++i)
			for (int j = 0; j < max_num; ++j) {
				cost(i, j) = int(iou_matrix[i][j] * 1000);
			}

		// 按照 0,1,2.. 返回对应的下标
		std::vector<long> assignment = max_cost_assignment(cost);





		/////////////                STEP 2 : matched  (初步匹配结果)             /////////////
		// 将 detection 的编号加入，构成 (N,2) 的数组
		for (int i = 0; i < assignment.size(); ++i)	matched.push_back(make_pair(i, assignment[i]));




		/////////////                STEP 3 : unmatched_detections               /////////////
	/*
		新增目标即为匹配失败的 检测框 (unmatched_detections)
		判断方法：匹配到 非法 tracker 的 detection ; iou 小于阈值的 detection
	*/
		for (int i = 0; i < detections.size(); ++i)
			if (matched[i].second >= trackers.size() || iou_matrix[matched[i].first][matched[i].second] < iou_threshold)
				unmatched_detections.push_back(i);






		/////////////                STEP 4 : unmatched_trackers               /////////////
		/*
			匹配失败的跟踪框 unmatched_trackers
			判断方法：匹配到非法 detection 的 tracker;iou 小于阈值的 tracker

		*/
		for (int i = 0; i < matched.size(); ++i) {
			if (matched[i].second < trackers.size()) {
				if (matched[i].first >= detections.size() || iou_matrix[matched[i].first][matched[i].second] < iou_threshold)
					unmatched_trackers.push_back(matched[i].second);
			}
		}



		/////////////                STEP 5 : matches               /////////////
		/*
			合法的匹配对 matches

		*/
		for (int i = 0; i < matched.size(); ++i) {
			if (matched[i].first < detections.size() && matched[i].second < trackers.size() && iou_matrix[matched[i].first][matched[i].second] >= iou_threshold)
				matches.push_back(matched[i]);
		}




	}
	else {
		// 没有匹配成功，则 matched 直接为空



		// detections 全为 unmatched_detections 
		for (int i = 0; i < detections.size(); ++i)	unmatched_detections.push_back(i);

		// trackers 全为 unmatched_trackers
		for (int i = 0; i < trackers.size(); ++i)	unmatched_trackers.push_back(i);

	}


	_matches = matches;
	_unmatched_detections = unmatched_detections;
	_unmatched_trackers = unmatched_trackers;

}

// 显示
//int fontface = FONT_HERSHEY_COMPLEX_SMALL;
//const char* GTname = "GT";
//const char* Predictname = "Predict";
//for (int i = 0; i < vec.size(); ++i) {
//	cv::rectangle(frame, cv::Point(vec[i].y1, vec[i].x1), cv::Point(vec[i].y2, vec[i].x2), Scalar(255, 0, 0), 1, 1, 0);
//	cv::putText(frame, GTname, cv::Point(vec[i].y1, vec[i].x1 - 10), fontface, 0.5, cv::Scalar(255, 0, 0));
//}
//
//if (statePredict.x1 != 0.0) {
//	cv::rectangle(frame, cv::Point(statePredict.y1, statePredict.x1), cv::Point(statePredict.y2, statePredict.x2), Scalar(0, 255, 0), 1, 1, 0);
//	cv::rectangle(mask, cv::Point(statePredict.y1, statePredict.x1), cv::Point(statePredict.y2, statePredict.x2), Scalar(0, 255, 0), 1, 1, 0);
//	cv::putText(frame, Predictname, cv::Point(statePredict.y1, statePredict.x1 - 10), fontface, 0.5, cv::Scalar(0, 255, 0));
//	cv::putText(mask, Predictname, cv::Point(statePredict.y1, statePredict.x1 - 10), fontface, 0.5, cv::Scalar(0, 255, 0));
//}

void drawBbox(cv::Mat& frame, const vector<bbox>& bboxes, bool isDet) {
	/*
		将 bboxes 画在 frame 上
		input:
			frame:当前帧，待作画的frame
			bboxes:待作画的 检测框/预测框
			isDet:检测框标志，用于区分检测框/预测框颜色

	*/

	for (int i = 0; i < bboxes.size(); ++i) {
		int fontface = cv::FONT_HERSHEY_COMPLEX_SMALL;
		if (isDet) {
			const char* GTname = "GT";
			cv::rectangle(frame, cv::Point(bboxes[i].y1, bboxes[i].x1), cv::Point(bboxes[i].y2, bboxes[i].x2), cv::Scalar(255, 0, 0), 1, 1, 0);
			cv::putText(frame, GTname, cv::Point(bboxes[i].y1, bboxes[i].x1 - 10), fontface, 0.5, cv::Scalar(255, 0, 0));
		}
		else { 
			const char* GTname = "Predict";
			cv::rectangle(frame, cv::Point(bboxes[i].y1, bboxes[i].x1), cv::Point(bboxes[i].y2, bboxes[i].x2), cv::Scalar(0, 255 , 0), 1, 1, 0);
			cv::putText(frame, GTname, cv::Point(bboxes[i].y1, bboxes[i].x1 - 10), fontface, 0.5, cv::Scalar( 0,255, 0));
		}

	}


}