
#include <information_matrix_calculator.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

 
InformationMatrixCalculator::InformationMatrixCalculator(ros::NodeHandle& nh) {
  use_const_inf_matrix = nh.param<double>("use_const_inf_matrix", false);
  const_stddev_x = nh.param<double>("const_stddev_x", 0.5);
  const_stddev_q = nh.param<double>("const_stddev_q", 0.1);

  var_gain_a = nh.param<double>("var_gain_a", 20.0);
  min_stddev_x = nh.param<double>("min_stddev_x", 0.1);
  max_stddev_x = nh.param<double>("max_stddev_x", 5.0);
  min_stddev_q = nh.param<double>("min_stddev_q", 0.05);
  max_stddev_q = nh.param<double>("max_stddev_q", 0.2);
  fitness_score_thresh = nh.param<double>("fitness_score_thresh", 0.5);      // 匹配得分的阈值
  max_range = 0.5;
}

InformationMatrixCalculator::~InformationMatrixCalculator() 
{
}

// 计算信息矩阵   用于激光里程计的边
// relpose = Tc1<-c2  
Eigen::MatrixXd InformationMatrixCalculator::calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const {
  if(use_const_inf_matrix) {
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= const_stddev_x;
    inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
    return inf;
  }  
  // 计算匹配的得分    匹配程度越高  得分越低  这个得分反映点的平均距离   一般<0.1m     将cloud2 转到 cloud1      relpose * cloud2 = cloud1     
  double fitness_score = calc_fitness_score(cloud1, cloud2, relpose, max_range);   
  std::cout<<"fitness_score: "<<fitness_score<<std::endl;
  double min_var_x = std::pow(min_stddev_x, 2);   // min_stddev_x 0.1  0.01
  double max_var_x = std::pow(max_stddev_x, 2);   // 5   25
  double min_var_q = std::pow(min_stddev_q, 2);   // min_stddev_q 0.05 
  double max_var_q = std::pow(max_stddev_q, 2);   // 0.2
  // 计算权重
  // 点云相似度越高  越接近 min_var_x  min_var_q
  float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);      // XYZ的权重   最小 min_var_x = 0.01   最大25  
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);      // 旋转的权重   最小 min_var_q = 0.05^2   最大  0.2^2
  // std::cout<<"w_x: "<<w_x<<" w_q: "<<w_q<<std::endl;
  // 设置信息矩阵
  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  // 点云相似度越高     w_x   w_q  越小    则相应的信息矩阵相应部分就越大   
  inf.topLeftCorner(3, 3).array() /= w_x;                        // 1 / w_x
  inf.bottomRightCorner(3, 3).array() /= w_q;     

  return inf;
}

// 计算匹配得分    使用kdtree  
// 这个得分反映点云之间的平均距离   越接近则得分越低
double InformationMatrixCalculator::calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range) {
  pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
  tree_->setInputCloud(cloud1);
  double fitness_score = 0.0;
  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointT> input_transformed;
  // cloud2 通过  relpose 转到  input_transformed  
  pcl::transformPointCloud (*cloud2, input_transformed, relpose.cast<float>());

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset   对cloud2每一个点到cloud1中搜索最近的点
  int nr = 0;
  double ratio=0;      // 记录内点数占总数的占比   
  
  for (size_t i = 0; i < input_transformed.points.size(); ++i)
  {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
    // Deal with occlusions (incomplete targets)    如果最近的点小于阈值
    if (nn_dists[0] <= max_range)     // max_range 为形式参数  
    {
      // Add to the fitness score
      fitness_score += nn_dists[0];   // 只考虑距离合适的点   这样可以避免动态障碍的影响 
      nr++;
    }
  }
  // 计算好点的占比   
  ratio = (double) nr / input_transformed.points.size();
  // std::cout<<"------------------ good% "<<ratio<<std::endl;
  // 如果动态障碍物多的话  那么这个ratio可能会下降   如果小于阈值 说明匹配可能有误 不应该信任       
  if (ratio > 0.8)    
    return (fitness_score / nr);          // 距离合适的点的平均距离   
  else
    return (std::numeric_limits<double>::max());
}



