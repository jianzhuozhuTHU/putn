//#ifndef GAUSSIAN_PROCESS_REGRESSION_HXX
//#define GAUSSIAN_PROCESS_REGRESSION_HXX


#include "gaussian_process_regression/gaussian_process_regression.h"

template<typename R>
GaussianProcessRegression<R>::GaussianProcessRegression(int inputDim,int outputDim)
{
  input_data_.resize(inputDim,0);
  output_data_.resize(outputDim,0);
  n_data_ = 0;
}



template<typename R>
void GaussianProcessRegression<R>::AddTrainingData(const VectorXr& newInput,const VectorXr& newOutput)
{
  n_data_++;
  if(n_data_>=input_data_.cols()){
    input_data_.conservativeResize(input_data_.rows(),n_data_);
    output_data_.conservativeResize(output_data_.rows(),n_data_);
  }
  input_data_.col(n_data_-1) = newInput;
  output_data_.col(n_data_-1) = newOutput;
  b_need_prepare_ = true;
}

// void show_dim(Eigen::MatrixXf a){
//   std::cout<<a.rows()<<" "<<a.cols()<<std::endl;
// }

template<typename R>
void GaussianProcessRegression<R>::AddTrainingDataBatch(const MatrixXr& newInput, const MatrixXr& newOutput)
{
  // sanity check of provided data
  assert(newInput.cols() == newOutput.cols());
  // if this is the first data, just add it..
  if(n_data_ == 0){
    input_data_ = newInput;
    output_data_ = newOutput;
    n_data_ = input_data_.cols();
  }
  // if we already have data, first check dimensionaly match
  else{
    assert(input_data_.rows() == newInput.rows());
    assert(output_data_.rows() == newOutput.rows());
    size_t n_data_old = n_data_;
    n_data_ += newInput.cols();
    // resize the matrices
    if(n_data_ > input_data_.cols()){
      input_data_.conservativeResize(input_data_.rows(),n_data_);
      output_data_.conservativeResize(output_data_.rows(),n_data_);
    }
    // insert the new data using block operations
    input_data_.block(0,n_data_old,newInput.rows(),newInput.cols()) = newInput;
    output_data_.block(0,n_data_old,newOutput.rows(),newOutput.cols()) = newOutput;
  }
  // in any case after adding a batch of data we need to recompute decomposition (in lieu of matrix inversion)
  b_need_prepare_ = true;
}

//gaussian kernel fuction for compute covariance
template<typename R>
R GaussianProcessRegression<R>::SQEcovFuncD(VectorXr x1, VectorXr x2)
{
  dist = x1-x2;
  double d = dist.dot(dist);
  d = sigma_f_*sigma_f_*exp(-1/l_scale_/l_scale_/2*d);
  return d;
}

template<typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::SQEcovFunc(MatrixXr x1, VectorXr x2){
  int nCol = x1.cols();
  VectorXr KXx(nCol);
  for(int i=0;i<nCol;i++){
    KXx(i)=SQEcovFuncD(x1.col(i),x2);
  }
  return KXx;
}


template <typename R>
void GaussianProcessRegression<R>::PrepareRegression(bool force_prepare){
  if(!b_need_prepare_ & !force_prepare)
    return;

  KXX = SQEcovFunc(input_data_);
  KXX_ = KXX;
  // add measurement noise
  // we set noise to zero
  for(int i=0;i<KXX.cols();i++)
    KXX_(i,i) += sigma_n_*sigma_n_;
  alpha_.resize(output_data_.rows(),output_data_.cols());
  // pretty slow decomposition to compute
  //Eigen::FullPivLU<MatrixXr> decomposition(KXX_);
  // this is much much faster:
  Eigen::LDLT<MatrixXr> decomposition(KXX_);
  for (size_t i=0; i < output_data_.rows(); ++i)
    {
      alpha_.row(i) = (decomposition.solve(output_data_.row(i).transpose())).transpose();
    }
  b_need_prepare_ = false;
}

// This is a slow and and deprecated version that is easier to understand. 
template<typename R>
void GaussianProcessRegression<R>::PrepareRegressionOld(bool force_prepare)
{
  if(!b_need_prepare_ & !force_prepare)
    return;

  KXX = SQEcovFunc(input_data_);
  KXX_ = KXX;
  // add measurement noise
  for(int i=0;i<KXX.cols();i++)
    KXX_(i,i) += sigma_n_*sigma_n_;

  // this is a time theif:
  KXX_ = KXX_.inverse();
  b_need_prepare_ = false;
}

// This is the right way to do it but this code should be refactored and tweaked so that the decompositon is not recomputed unless new training data has arrived. 
template <typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::DoRegression(const VectorXr& inp,bool prepare){
  // if(prepare || b_need_prepare_){
  //   PrepareRegression();
  // }
  // can return immediately if no training data has been added..
  VectorXr outp(output_data_.rows());
  outp.setZero();
  if(n_data_==0)
    return outp;
  
  PrepareRegression(prepare);
  //ok

  outp.setZero();
  KXx = SQEcovFunc(input_data_,inp);
  //std::cout<<"KXx  "<<KXx<<std::endl;
  for (size_t i=0; i < output_data_.rows(); ++i)
    outp(i) = KXx.dot(alpha_.row(i));
  
  MatrixXr outp1 =   SQEcovFunc(inp,inp) -  KXx*KXX* (KXx.transpose());
  //R output2  =  SQEcovFunc(inp,inp) -  KXx*KXX* KXx.transpose();

  return outp;
}

template <typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::DoRegressioncov(const VectorXr& inp,bool prepare){
  // if(prepare || b_need_prepare_){
  //   PrepareRegression();
  // }
  // can return immediately if no training data has been added..
  VectorXr outp(output_data_.rows());
  outp.setZero();
  if(n_data_==0)
    return outp;
  
  PrepareRegression(prepare);
  //ok

  outp.setZero();
  KXx = SQEcovFunc(input_data_,inp);
  //std::cout<<"KXx  "<<KXx<<std::endl;
  for (size_t i=0; i < output_data_.rows(); ++i)
    outp(i) = KXx.dot(alpha_.row(i));
  
  MatrixXr outp1 =   SQEcovFunc(inp,inp) -  KXx*KXX* (KXx.transpose());
  //R output2  =  SQEcovFunc(inp,inp) -  KXx*KXX* KXx.transpose();

  return outp1;
}



template <typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::DoRegressionOld(const VectorXr& inp,bool prepare){
  if(prepare || b_need_prepare_){
    PrepareRegressionOld();
  }
  VectorXr outp(output_data_.rows());
  outp.setZero();
  KXx = SQEcovFunc(input_data_,inp);
  //KxX = SQEcovFunc(input_data_,inp).transpose();
  VectorXr tmp(input_data_.cols());
  // this line is the slow one, hard to speed up further?
  tmp = KXX_*KXx;
  // the rest is noise in comparison with the above line.
  for(int i=0;i<output_data_.rows();i++){
    outp(i)=tmp.dot(output_data_.row(i));
  }
  return outp;
}

template<typename R>
void GaussianProcessRegression<R>::ClearTrainingData()
{
  input_data_.resize(input_data_.rows(),0);
  output_data_.resize(output_data_.rows(),0);
  b_need_prepare_ = true;
  n_data_ = 0;
}
//return the covariance fuction Kff(the covariance between the training data)
template<typename R>
typename GaussianProcessRegression<R>::MatrixXr GaussianProcessRegression<R>::SQEcovFunc(MatrixXr x1){
  int nCol = x1.cols();
  MatrixXr retMat(nCol,nCol);
  for(int i=0;i<nCol;i++){
    for(int j=i;j<nCol;j++){
      retMat(i,j)=SQEcovFuncD(x1.col(i),x1.col(j));
      retMat(j,i)=retMat(i,j);
    }
  }
  return retMat;
}

template<typename R>
void GaussianProcessRegression<R>::Debug()
{
  std::cout<<"input data \n"<<input_data_<<std::endl;
  std::cout<<"output data \n"<<output_data_<<std::endl;
}

//#endif /* GAUSSIAN_PROCESS_REGRESSION_HXX */
