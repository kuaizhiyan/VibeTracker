#pragma once
#include<opencv2\video\tracking.hpp>

#include<cmath>
#include<vector>
using namespace std;
typedef  pair<int, int> PII;
typedef	 pair<PII, PII> PIV;



struct bbox
{
	bbox(float _x1, float _y1, float _x2, float _y2) {
		x1 = _x1; y1 = _y1; x2 = _x2; y2 = _y2;
	}
	float x1, y1;		// box 左上角坐标, 为了精度统一设为 float，显示注意转为int
	float x2, y2;		// box 右下角坐标

	cv::Mat getMat();
};

struct zbox
{	
	zbox(float _x, float _y, float _s, float _r) {
		x = _x; y = _y; s = _s; r = _r;
	}
	float x, y;		// box 中心点坐标 
	float s;			// box 面积			w*h
	float r;			// box 的横纵比		w/h

	cv::Mat getMat();
};




class Tracker {
	Tracker();				// 默认构造函数
	
	//~Tracker(void);		// 析构函数
	
public:
	Tracker(bbox box);		// 使用 bbox 创建

	static int cnt;		// 静态变量，统计存在的跟踪器个数；类内定义，类外初始化 
		
	cv::KalmanFilter KF;	// Kalman Filter

	/* 辅助跟踪器管理的参数 */
	bool isInited = false;	// 是否被初始化
	int id;
	int age = 0;			// 寿命
	int hits = 0;			// 命中次数
	int hit_streak = 0;
	int time_since_update;	//
	vector<bbox> history; //存放历史预测
	
	
	void update(bbox box);	// 更新
	bbox predict();			// 预测
	bbox get_state();		// 获取当前状态量
};

zbox convert_bbox_to_zbox(const bbox& box);
bbox convert_z_to_bbox(const zbox& box);
zbox convert_mat_to_zbox(const cv::Mat& mat);
bbox convert_mat_to_bbox(const cv::Mat& mat);
cv::Mat convert_bbox_to_zmat(const bbox& box);
PIV bfs(int x1, int y1, const cv::Mat& mask);			// 返回一个连通域的左上右下角点
vector<bbox> getBboxs(const cv::Mat& mat,int area);		// 从 掩膜 mask 中获取出所有bbox
float iou(const bbox& box1,const bbox& box2);			// 计算两 box iou
void associate_detections_to_trackers(const vector<bbox>& detections, const vector<bbox>& trackers, vector<pair<int, int> >& _matches, vector<int>& _unmatched_detections, vector<int>& _unmatched_trackers, float iou_threshold);	// 将检测框和预测框进行匹配
void drawBbox(cv::Mat& frame, const vector<bbox>& bboxes, bool isDet);		// 在 frame 上绘制 box  和 文字提示