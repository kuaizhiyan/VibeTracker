#pragma once
#include<vector>
#include"Tracker.h"
using namespace std;

class ViBeT {
	/*
		利用 ViBe 实现跟踪的类(类比 Sort 类)
	*/

	

public:
	ViBeT();													// 默认构造函数
	ViBeT(int _max_age, int _min_hits, float _iou_threshold);	// 自定义构造函数

	int max_age;				// tracker 的最大寿命
	int min_hits;				// 最小匹配次数
	float iou_threshold;		// 匹配阈值
	int frame_count;

	int FRAME_MAX_COL;			// 记录下 frame 的尺寸
	int FRAME_MAX_ROW;

	vector<Tracker> trackers;	// 存放现有 的跟踪器

	vector<int> update( vector<bbox>& dets);		// 利用现有检测框做一次更新，返回可以显示的 tracker 索引
	bool isLeagal(const bbox& box);					// 判断 box 是否合法
};
