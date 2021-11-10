#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <boost/format.hpp>
#include <keyframe.hpp>
#include <registration.hpp>

#include <g2o/types/slam3d/vertex_se3.h>


// 一个成功回环的数据结构
struct Loop 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  Loop(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Matrix4f& relpose)
    : key1(key1),
      key2(key2),
      relative_pose(relpose)
  {}

public:
  // 回环的一对关键帧 
  KeyFrame::Ptr key1;   
  KeyFrame::Ptr key2;
  // 相对的位姿约束  
  Eigen::Matrix4f relative_pose;
};

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector 
{
public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief constructor
   * @param pnh
   */
  LoopDetector(ros::NodeHandle& pnh) {
    distance_thresh = pnh.param<double>("distance_thresh", 12.0);
    accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 50);
    candidates_distance = pnh.param<double>("candidates_distance", 70);      // 最多运动30M  最末段10M的闭环范围  再考虑阈值 30 所以候选距离是70 
    distance_from_last_detect_thresh = pnh.param<double>("min_edge_interval", 60);  // 初始的检测 id距离阈值   
    
    fitness_score_max_range = pnh.param<double>("fitness_score_max_range", std::numeric_limits<double>::max());
    fitness_score_thresh = pnh.param<double>("Loop_score_thresh", 0.5);
    // 设定回环的匹配方法 
    int ndt_resolution = pnh.param<int>("Loop_ndt_resolution", 4.0);
    registration = Set_NDTOMP_param(pnh, ndt_resolution);
    last_detect_index = 0;
  }

  /**
   * @brief detect loops and add them to the pose graph    首先为本次 new_keyframes 寻找闭环候选帧     然后在new_keyframes和闭环候选帧中找出一对距离最近的   计算匹配得分  低于阈值则闭环成功
   * @param keyframes       keyframes
   * @param new_keyframes   newly registered keyframes
   */
  Loop::Ptr detect(const std::vector<KeyFrame::Ptr>& keyframes, const std::vector<KeyFrame::Ptr>& new_keyframes) 
  {
    // too close to the last registered loop edge 与上一次闭环的关键帧应该足够的远    这个决定闭环检测的频率 
    if(new_keyframes[0]->id - last_detect_index < distance_from_last_detect_thresh) 
    {
      return nullptr;
    }
    last_detect_index = new_keyframes[0]->id;
    // 首先为本次loop detect 寻找候选关键帧   用new_keyframes 中最早的帧去寻找keyframes中 new_keyframes 中所有帧的候选帧
    auto candidates = find_candidates(keyframes, new_keyframes[0]);
    std::cout<<"loop candidates num: "<<candidates.size()<<std::endl;
    if(candidates.empty()) 
    { // 没有候选帧   要么当前帧与其他帧隔太远了  candidates_distance = 70  至少 30 之内没别的帧 
      distance_from_last_detect_thresh = 15;   
      return nullptr;      // 没有候选帧直接退出
    }
    
    // 遍历全部新关键帧  找到与候选帧中距离最近的一对
    double min_dist = distance_thresh;  
    std::pair<KeyFrame::Ptr, int> matchs;
    for(const auto& new_keyframe : new_keyframes) {
      // 遍历候选帧   找一个距离最小的  
      for(const int& index:candidates)
      {
        // 获得关键帧的位移  
        const auto& pos1 = keyframes[index]->Pose.translation();
        const auto& pos2 = new_keyframe->Pose.translation();
        // 直线距离 
        double dist = (pos1.head<2>() - pos2.head<2>()).norm();
        if(dist<min_dist)
        {
          min_dist = dist;
          matchs.first = new_keyframe;
          matchs.second = index;
        }
      }       
    }
    // 说明没有检测到距离小于distance_thresh的
    if(min_dist == distance_thresh)    
    { 
      // 附近有很多帧  但是距离都不近   则回环检测的频率提高 
      distance_from_last_detect_thresh = 5;
      return nullptr;
    }
    std::cout<<"find low dist KFS, id ("<< matchs.first->id<<", "<<matchs.second<<")"<<" dist: "<<min_dist<<std::endl;

    // 构造局部地图进行匹配    取 10帧  
    int idx = matchs.second>=5? matchs.second-5:0;
    pcl::PointCloud<PointT>::Ptr Match_map(new pcl::PointCloud<PointT>());      // 构造匹配的Map  
    pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());    
    Eigen::Isometry3d relpose = Eigen::Isometry3d::Identity();
    for(int i=0; i<10; i++)
    {   
        // Tcm*Tmi = Tci   
        relpose = keyframes[matchs.second]->Pose.matrix().inverse()*keyframes[idx]->Pose.matrix();       
        // 转到局部Map坐标系
        pcl::transformPointCloud( *(keyframes[idx]->cloud), *transformed, relpose.cast<float>());
        *Match_map += *transformed;
        idx++;
        // ROS_INFO_STREAM("LOOP Match_map size: "<<Match_map->size());
    }
    // 闭环的相对位姿       setInputSource ->  setInputTarget   c->map 
    Eigen::Matrix4f relative_pose = (keyframes[matchs.second]->Pose.matrix().inverse() * matchs.first->Pose.matrix()).cast<float>();
    // 进行匹配    
    if(!matching(Match_map, matchs.first, relative_pose))   {
      distance_from_last_detect_thresh = 5;
      return nullptr;
    }

    distance_from_last_detect_thresh = 30;    // 回环成功   则休眠50 帧  
    return std::make_shared<Loop>(matchs.first, keyframes[matchs.second], relative_pose);
  }

  double get_distance_thresh() const {
    return distance_thresh;
  }

private:

  /**
   * @brief 在对new_keyframes每一帧寻找keyframes中的闭环时， 首先利用new_keyframes最早的一帧序找keyframes中可能的闭环候选帧
   * @param keyframes      candidate keyframes of loop start
   * @param new_keyframe   new_keyframes中最早的一帧关键帧    
   * @return loop candidates
   */
  std::vector<int> find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const {

    // 选择候选帧
    std::vector<int> candidates;
    // 遍历历史全部关键帧   首先 累计距离要足够远     即 id 差距至少   accum_distance_thresh - 10     然后位移足够近  
    for(int i = 0; i<keyframes.size(); i++) {
      // traveled distance between keyframes is too small  如果距离近了直接退出     
      if(new_keyframe->id - keyframes[i]->id < accum_distance_thresh  ) {     // 扩大10  因为 new_keyframes中最多就10帧    增加检测范围 
        break;
      }
      // 获得关键帧的位移  
      const auto& pos1 = keyframes[i]->Pose.translation();
      const auto& pos2 = new_keyframe->Pose.translation();
      // 直线距离 
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();

      // 如果当前距离小于 之前 候选距离     70  
      if(dist<=candidates_distance)
         candidates.push_back(i);         // 记录候选帧      
      
    }
    return candidates;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched well, the loop is added to the pose graph
   * @param candidate_keyframes  candidate keyframes of loop start
   * @param new_keyframe         loop end keyframe
   */
  bool matching(const pcl::PointCloud<PointT>::Ptr& map, const KeyFrame::Ptr& new_keyframe, Eigen::Matrix4f& relative_pose) {
    // 新关键帧作为目标     好像Target必须要设置为Map  如果是
    registration->setInputTarget(map);

    auto t1 = ros::Time::now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());

    registration->setInputSource(new_keyframe->cloud);

    relative_pose(2, 3) = 0.0;    // z轴为0  可能是因为z轴容易飘     回环的之间一般z轴距离小  
    registration->align(*aligned, relative_pose);
    // 获取匹配得分 
    double score = registration->getFitnessScore(fitness_score_max_range);

    auto t2 = ros::Time::now();

    std::cout << " loop match score: " << boost::format("%.3f") % score << "    time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
    // 条件  ： 1、收敛   2、小于得分阈值  
    if(!registration->hasConverged() || score > fitness_score_thresh) {
      std::cout << "loop match failed" << std::endl;
      return false;
    }
    std::cout << "loop find!!" << std::endl;
    relative_pose = registration->getFinalTransformation();
    // 返回这次回环的结果  
    return true;
  }

private:
  double distance_thresh;                 // estimated distance between keyframes consisting a loop must be less than this distance
  double candidates_distance;             // 在所有历史关键帧挑选本次闭环候选帧的阈值   会比较大一点
  int accum_distance_thresh;              // traveled distance between ...
  double distance_from_last_detect_thresh;  // a new loop edge must far from the last one at least this distance
  double fitness_score_max_range;         // maximum allowable distance between corresponding points
  double fitness_score_thresh;            // threshold for scan matching
  int last_detect_index;

  // pcl::Registration<PointT, PointT>::Ptr registration;
  // 匹配算法
  boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> registration;      // 常用多线程NDT 
}; // class LoopDetector

#endif // LOOP_DETECTOR_HPP
