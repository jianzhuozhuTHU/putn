#ifndef GPR_H
#define GPR_H


#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

//#ifdef USE_DOUBLE_PRECISION
//typedef double REALTYPE;
//#else
//typedef float REALTYPE;
//#endif



template<typename REALTYPE>
class GaussianProcessRegression{

  typedef Eigen::Matrix<REALTYPE,Eigen::Dynamic,Eigen::Dynamic> MatrixXr;
  typedef Eigen::Matrix<REALTYPE,Eigen::Dynamic,1> VectorXr;

  MatrixXr input_data_;
  MatrixXr output_data_;
  MatrixXr KXX;  //covariance between the training data
  MatrixXr KXX_;
  VectorXr KXx;  //covariance between the training data and test data
  //MatrixXr KxX;

  int n_data_;
  bool b_need_prepare_;

  double l_scale_;
  double sigma_f_;
  double sigma_n_;

  VectorXr dist;

  VectorXr regressors;

  //  std::vector<Eigen::FullPivLU<MatrixXr> > decompositions_;
  MatrixXr alpha_;
  
public:
  GaussianProcessRegression(int inputDim, int outputDim);

  void SetHyperParams(double l, double f, double n){l_scale_ = l; sigma_f_ = f; sigma_n_ = n;};
  void GetHyperParams(double & l, double & f, double & n){l = l_scale_; f = sigma_f_; n = sigma_n_;
                                                         std::cout<<l<<" "<<f<<" "<<n;};

  // add data one by one
  void AddTrainingData(const VectorXr& newInput, const VectorXr& newOutput);
  // batch add data
  void AddTrainingDataBatch(const MatrixXr& newInput,const MatrixXr& newOutput);

  REALTYPE SQEcovFuncD(VectorXr x1,VectorXr x2);
  void Debug();

  MatrixXr SQEcovFunc(MatrixXr x1);
  VectorXr SQEcovFunc(MatrixXr x1, VectorXr x2);
  // these are fast methods 
  void PrepareRegression(bool force_prepare = false);
  VectorXr DoRegression(const VectorXr & inp,bool prepare = false);
  VectorXr DoRegressioncov(const VectorXr& inp,bool prepare = false);


  // these are the old implementations that are slow, inaccurate and easy to understand
  void PrepareRegressionOld(bool force_prepare = false);
  VectorXr DoRegressionOld(const VectorXr & inp,bool prepare = false);

  int get_n_data(){return n_data_;};
  const MatrixXr& get_input_data()  {return input_data_;};
  const MatrixXr& get_output_data()  {return output_data_;};

  void ClearTrainingData();



};

#include "gaussian_process_regression.hxx"
#endif //GPR_H
