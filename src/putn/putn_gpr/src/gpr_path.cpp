#include "gaussian_process_regression/gaussian_process_regression.h"
#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace Eigen;

double length_scale ;
double sigma_f ;
double sigma_n ;
const size_t input_dim(2), output_dim(1);
typedef Eigen::Matrix<float,3,1> input_type;
typedef Eigen::Matrix<float,1,1> output_type;
std::vector<input_type> train_inputs, test_inputs;
std::vector<output_type> train_outputs, test_outputs;
ros::Publisher _surf_predict_pub;

string filepath;
    
template<typename input_type, typename output_type>
void load_data(const char *fname, std::vector<input_type> &inputs, std::vector<output_type> &outputs, int input_dim, int output_dim) {
  std::cout<<"entry this branch........"<<std::endl;
  input_type inp,tinp;
  output_type outp,toutp;
  std::ifstream myfile(fname);
  if (!myfile.is_open())
  {
      std::cout << "Fail to open the file" << std::endl;
      return;
  }
  std::string line;
  while(getline(myfile,line)){
      std::cout<<line<<" ";
    std::istringstream line_stream(line);
    for(size_t k = 0; k < input_dim; k++)
      line_stream>>inp(k);
    for(size_t k = 0; k < output_dim; k++)
      line_stream>>outp(k);
    inputs.push_back(inp);
    outputs.push_back(outp);
  }
  std::cout<<"finish loading..."<<std::endl;
}
template<typename R>
void set_hyperparameters_from_file(const char *fname, GaussianProcessRegression<R> & gpr) {
  std::ifstream myfile;
  myfile.open(fname);
  if (!myfile.is_open())
  {
      std::cout << "Fail to open the file" << std::endl;
      return;
  }
  R l, f, n;
  myfile>>l>>f>>n;
  myfile.close();
  gpr.SetHyperParams(l,f,n);
}

void ClearVectorIn( vector< input_type >& vt ) 
{
  vector< input_type >  veTemp; 
  veTemp.swap( vt );
}
void ClearVectorOut( vector< output_type >& vt ) 
{
  vector< output_type > veTemp; 
  veTemp.swap( vt );
}

void treecb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  double dur;
  clock_t start,end;start = clock();
  ROS_INFO("[node] receive the tree");
  if(msg->data.size() == 0) return;

  int num = (int)(msg->data.size()/4);
	for (int i=0; i<num; i++) 
  {
    input_type tmp_in;
    tmp_in << msg->data[4*i],msg->data[4*i+1],msg->data[4*i+2];
    train_inputs.push_back(tmp_in);
    output_type tmp_out;
    tmp_out << msg->data[4*i+3] ;
    train_outputs.push_back(tmp_out);
	}

  //GPr
  GaussianProcessRegression<float> myGPR(input_dim, output_dim);
  set_hyperparameters_from_file(filepath.c_str(),myGPR);

  for(size_t k=0; k<train_inputs.size(); k++){
      myGPR.AddTrainingData(train_inputs[k], train_outputs[k]);
   }
  
  double threshold = 0.1;


  if(test_inputs.size()==0)
  {
    ClearVectorIn(train_inputs);
    ClearVectorIn(test_inputs);
    ClearVectorOut(train_outputs);
    ClearVectorOut(test_outputs);
  }


   std_msgs::Float32MultiArray out_ym;
   std_msgs::Float32MultiArray out_ys;

  for(size_t k=0; k<test_inputs.size(); k++)
  {
    auto outp = myGPR.DoRegression(test_inputs[k]);
    output_type tmp_out;
    tmp_out <<outp;
    test_outputs.push_back(tmp_out);
    //covariance
    auto outp_cov = myGPR.DoRegressioncov(test_inputs[k]);
    out_ym.data.push_back(test_inputs[k](0,0));
    out_ym.data.push_back(test_inputs[k](1,0));
    out_ym.data.push_back(test_inputs[k](2,0));
    out_ym.data.push_back(tmp_out(0,0));
    out_ym.data.push_back(outp_cov(0,0));

  }
  _surf_predict_pub.publish(out_ym);
  std_msgs::Float32MultiArray tmp_out;
  ClearVectorIn(train_inputs);
  ClearVectorIn(test_inputs);
  ClearVectorOut(train_outputs);
  ClearVectorOut(test_outputs);
  end = clock();
  dur = (double)(end - start);
  cout<<"Time consume ："<<dur/1000<<" ms"<<endl;
}

void pathcb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO("[node] receive the path");
  if(msg->data.size() == 0) return;

  int num = (int)(msg->data.size()/3) ;
	for (int i=0; i<num; i++) 
  {
    input_type tmp_in;
    tmp_in <<msg->data[3*i],msg->data[3*i+1],msg->data[3*i+2];
    test_inputs.push_back(tmp_in);
	}
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "GPR");
    ros::NodeHandle ph("~");
    ros::Subscriber _tree_sub,_path_sub;
    
    _tree_sub = ph.subscribe( "/global_planning_node/tree_tra",  1,treecb  );
    _path_sub = ph.subscribe( "/global_planning_node/global_path",  1,pathcb  );
    _surf_predict_pub = ph.advertise< std_msgs::Float32MultiArray>("/surf_predict_pub",1);

    ph.param<string>("file/cfg_path",filepath,"");   
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
};
