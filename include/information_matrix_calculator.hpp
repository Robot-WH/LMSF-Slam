#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class InformationMatrixCalculator {
public:
  using PointT = pcl::PointXYZI;

  InformationMatrixCalculator(){}
  InformationMatrixCalculator(ros::NodeHandle& nh);
  ~InformationMatrixCalculator();

  template<typename ParamServer>
  void load(ParamServer& params) {
    use_const_inf_matrix = params.template param<bool>("use_const_inf_matrix", false);
    const_stddev_x = params.template param<double>("const_stddev_x", 0.5);
    const_stddev_q = params.template param<double>("const_stddev_q", 0.1);

    var_gain_a = params.template param<double>("var_gain_a", 20.0);
    min_stddev_x = params.template param<double>("min_stddev_x", 0.1);
    max_stddev_x = params.template param<double>("max_stddev_x", 5.0);
    min_stddev_q = params.template param<double>("min_stddev_q", 0.05);
    max_stddev_q = params.template param<double>("max_stddev_q", 0.2);
    fitness_score_thresh = params.template param<double>("fitness_score_thresh", 2.5);
    
  }

  static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range = std::numeric_limits<double>::max());

  Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const;
private:

  //   (1 - e^-a*x) / (1.0 - e^-a*max_x)
  double weight(double a, double fitness_score_thresh, double min_y, double max_y, double fitness_score) const {
    // fitness_score 越大   y就越大         fitness_score <=  fitness_score_thresh   一般 fitness_score_thresh = 0.5  
    double y = (1.0 - std::exp(-a * fitness_score)) / (1.0 - std::exp(-a * fitness_score_thresh));
    return min_y + (max_y - min_y) * y;      //  fitness_score 越小  越接近min_y   当  fitness_score = 0 则  = min_y,    若fitness_score 越大    越接近max_y
  }

private:
  bool use_const_inf_matrix;
  double const_stddev_x;
  double const_stddev_q;
  double max_range;
  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;

};


#endif // INFORMATION_MATRIX_CALCULATOR_HPP
