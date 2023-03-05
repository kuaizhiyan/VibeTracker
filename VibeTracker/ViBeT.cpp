#pragma once
#include"ViBeT.h"
#include<algorithm>

ViBeT::ViBeT()
{
	this->max_age = 5;
	this->min_hits = 3;
	this->iou_threshold = 0.3;
	this->frame_count = 0;
}

ViBeT::ViBeT(int _max_age=5, int _min_hits=2, float _iou_threshold=0.3) {
	this->max_age = _max_age;
	this->min_hits = _min_hits;
	this->iou_threshold = _iou_threshold;
	this->frame_count = 0;
}

// 判断 box 是否合法的判断函数
bool ViBeT::isLeagal(const bbox& box) {
	if (box.x1 < 0 || box.y1 < 0 || box.x2 < 0 || box.y2 < 0)	return false;
	if (box.x1 >= box.x2 || box.y1 >= box.y2)	return false;
	
	if (box.x1 >= this->FRAME_MAX_ROW || box.y1 >= this->FRAME_MAX_COL)	return false; 
	if (box.x2 > this->FRAME_MAX_ROW || box.y2 > this->FRAME_MAX_COL)	return false;

}

vector<int> ViBeT::update(vector<bbox>& dets) {
/*
	ViBeT 用新来的一帧中检测出的 检测框 做一次更新，包括：
		1.检测框预测框匹配
		2.创建新的跟踪器
		3.删除旧的跟踪器
		4.更新匹配成功的跟踪器
	input: 
		dets: frame 中检测出的 检测框 列表
	output:
		trks:满足条件，可以绘画的 预测框 列表



*/
	
	this->frame_count++;		// 处理帧数+1
	vector<int> to_del;			// 待删除的 tracker 编号列表
	vector<int> ret;			// 可以显示的 tracker 编号列表

	vector<bbox> trks;			// 存放预测的 跟踪框
	
	///////    STEP 1 : 现有的跟踪器（列表） 全部做一次预测
	for (int i = 0; i < this->trackers.size(); ++i) {
		bbox trk = this->trackers[i].predict(); 
		trks.push_back(trk);
		if (!isLeagal(trk)) {
			// 若预测结果不合法，则直接加入待删列表
			to_del.push_back(i);
		}
	}

	///////    STEP 2 : 删除 to_del 中对应的跟踪器 ???? 应当放到最后删除，否则 trks 数量会大于 trackers 数量导致越界
	// 安全删除：保证从 vector 后往前删
	/*sort(to_del.begin(), to_del.end());
	for (int i = to_del.size() - 1; i >= 0; --i) {
		this->trackers.erase(this->trackers.begin() + to_del[i]);

	}*/

	///////    STEP 3 : 检测框和预测框匹配
	vector<PII> matched;
	vector<int> unmatched_dets;
	vector<int> unmatched_trks;
	associate_detections_to_trackers(dets, trks, matched, unmatched_dets, unmatched_trks,this->iou_threshold);

	///////    STEP 4 : 更新匹配成功的跟踪器
	for (int i = 0; i < matched.size(); ++i) {
		int trk_id = matched[i].second;
		int det_id = matched[i].first;
		this->trackers[trk_id].update(dets[det_id]);
	}

	///////    STEP 5 : 为匹配失败的检测框创建新的跟踪器
	for (int i = 0; i < unmatched_dets.size(); ++i) {
		this->trackers.push_back(Tracker(dets[i]));
	}

	///////    STEP 6 : 筛选出可以显示的 预测框
	// 删除超过最大年限的跟踪器
	//vector<int> to_del2;
	for (int i = 0; i < this->trackers.size(); ++i) {
		if (this->trackers[i].time_since_update > this->max_age)
			to_del.push_back(i);
	}
	sort(to_del.begin(), to_del.end());
	for (int i = to_del.size() - 1; i >= 0; --i) {
		this->trackers.erase(this->trackers.begin() + to_del[i]);
	}

	// 寿命满足 且 命中次数满足的，加入 ret
	for (int i = this->trackers.size() - 1; i >= 0; --i) {
		if (this->trackers[i].time_since_update < 1 && this->trackers[i].hit_streak >= this->min_hits)
			ret.push_back(i);
	}

	return ret;

}