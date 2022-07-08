#include "PUTN_planner.h"
#include <std_msgs/Float32MultiArray.h>
#include <random>

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace ros;
using namespace PUTN;
using namespace PUTN::visualization;
using namespace PUTN::planner;

PFRRTStar::PFRRTStar(){}
PFRRTStar::PFRRTStar(const double &height,World* world):h_surf_(height),world_(world){}
PFRRTStar::~PFRRTStar()
{
    clean_vector(tree_);
    delete node_target_;
    node_target_=NULL;
}

void PFRRTStar::initWithGoal(const Vector3d &start_pos,const Vector3d &end_pos)
{
    Vector2d last_end_pos_2D=end_pos_2D_;
    PlanningState last_planning_state=planning_state_;
    curr_iter_=0;
    curr_time_=0.0;
    end_pos_2D_=project2plane(end_pos);

    Node* node_origin=fitPlane(start_pos);

    if(node_origin==NULL)
    {
        planning_state_=Invalid;
        return;
    }
    
    Node* node_target=fitPlane(end_pos);

    planning_state_=(node_target==NULL)?Roll:Global;

    close_check_record_.clear();

    bool inherit_flag=false;

    switch(planning_state_)
    {
        case Global:
        {
            switch(last_planning_state)
            {
                case Global:
                    if(last_end_pos_2D==end_pos_2D_ && inheritPath(node_origin,Path::Global))
                    {
                        inherit_flag=true;
                        delete node_target;node_target=NULL;
                    }
                    else{delete node_target_;node_target_=node_target;}
                    break;
                case WithoutGoal:
                    delete node_target_;node_target_=node_target;
                    inherit_flag=inheritTree(node_origin);
                    break;
                default:
                    delete node_target_;node_target_=node_target;
                    break;
            }
        }
        break;
        case Roll:
        {
            switch(last_planning_state)
            {
                case Roll:
                    inherit_flag=(last_end_pos_2D==end_pos_2D_ && inheritPath(node_origin,Path::Sub));
                    break;
                case WithoutGoal:
                    inherit_flag=inheritTree(node_origin);
                    break;
                default:
                    break;
            }
        }
        break;
        default:
        break;
    }

    if(!inherit_flag)
    {
        path_=Path();
        clean_vector(tree_);
        node_origin_=node_origin;
        tree_.push_back(node_origin_);
    }

    if(planning_state_==Roll && last_end_pos_2D!=end_pos_2D_) sub_goal_threshold_=1.0f;

    vector<Node*> origin_and_goal{node_origin_};
    if(planning_state_==Global) origin_and_goal.push_back(node_target_);
    visOriginAndGoal(origin_and_goal,goal_vis_pub_);
}

void PFRRTStar::initWithoutGoal(const Vector3d &start_pos)
{
    curr_iter_=0;
    curr_time_=0.0;

    close_check_record_.clear();
    path_=Path();

    PlanningState last_planning_state=planning_state_;
  
    Node* node_origin=fitPlane(start_pos);

    if(node_origin==NULL)
    {
        planning_state_=Invalid;
        return;
    }

    planning_state_=WithoutGoal;

    delete node_target_;
    node_target_=NULL;

    if(last_planning_state!=WithoutGoal || !inheritTree(node_origin))
    {
        clean_vector(tree_);
        node_origin_=node_origin;
        tree_.push_back(node_origin_);
    }
}

void PFRRTStar::updateNode(Node* node_input)
{
    if(node_input->parent_!=NULL)//Skip the root node
        node_input->cost_=node_input->parent_->cost_+calCostBetweenTwoNode(node_input,node_input->parent_);

    closeCheck(node_input);

    //Update by recursion
    for(auto &node:node_input->children_) updateNode(node);
}

void PFRRTStar::addInvalidNodes(Node* &node_input,const bool &ifdelete,vector<Node*> &invalid_nodes)
{
    if(node_input==NULL) return;
    bool delete_flag=false;
    if(ifdelete || node_input->plane_==NULL) delete_flag=true;
    else
    {
        if(node_input->parent_!=NULL)
            delete_flag=!world_->collisionFree(node_input,node_input->parent_);
        else//If the root node is input 
            delete_flag=!world_->isFree(node_input->position_);
    }
    if(delete_flag) invalid_nodes.push_back(node_input);
    for(auto &node:node_input->children_) addInvalidNodes(node,delete_flag,invalid_nodes);
}

void PFRRTStar::trimTree()
{
    vector<Node*> invalid_nodes;
    addInvalidNodes(node_origin_,false,invalid_nodes);
    for(auto &node:invalid_nodes)
    {
        if(node->parent_!=NULL) deleteChildren(node->parent_,node);
        for(vector<Node*>::iterator it=tree_.begin();it!=tree_.end();++it)
        {
            if(*it==node)
            {
                tree_.erase(it);
                break;
            }
        }
    }
    clean_vector(invalid_nodes);
}

bool PFRRTStar::inheritTree(Node* new_root)
{ 
    bool result=false;

    //considering that the update of grid map may affect the result of plane-fitting
    for(auto&node:tree_) fitPlane(node);
        
    trimTree();

    float min_dis = INF;
    Node* node_insert = NULL;
  
    for(const auto&node:tree_) 
    { 
        float tmp_dis=EuclideanDistance(node,new_root);
        if(tmp_dis < min_dis && world_->collisionFree(node,new_root)) 
        {
            min_dis = tmp_dis;
            node_insert = node;
        }
    }

    if(node_insert!=NULL)
    {
        result=true;
        Node* node=node_insert;
        vector<Node*> node_record;
        while(node!=NULL)
        {
            node_record.push_back(node);
            node=node->parent_;
        }
        for(size_t i=node_record.size()-1;i>0;i--)
        {
            deleteChildren(node_record[i],node_record[i-1]);
            node_record[i]->parent_=node_record[i-1];
            node_record[i-1]->children_.push_back(node_record[i]);
        }
        new_root->children_.push_back(node_insert);
        node_insert->parent_=new_root;
        tree_.push_back(new_root);
        updateNode(new_root);
        node_origin_=new_root;

        vector<pair<Node*,float>> neighbor_record;
        findNearNeighbors(node_origin_,neighbor_record);
        reWire(node_origin_,neighbor_record);
    }
    return result;
}

bool PFRRTStar::inheritPath(Node* new_root,Path::Type type)
{
    bool result=false;
    if(path_.type_==type)
    {
        //copy the path
        vector<Node*> tmp_nodes;
        for(size_t i = 0;i < path_.nodes_.size();i++)
        {
            Node* node_now=path_.nodes_[i];
            tmp_nodes.push_back(fitPlane(node_now->plane_->init_coord));
            if(tmp_nodes[i]==NULL || (tmp_nodes.size()>1 && !world_->collisionFree(tmp_nodes[i],tmp_nodes[i-1])))
                return false;
            //if the distance between the current node and the new root is less
            //than the threshold and there is no obstacle between them.
            if(EuclideanDistance(tmp_nodes[i],new_root) < inherit_threshold_ && world_->collisionFree(tmp_nodes[i],new_root))
            {
                result=true;
                break;
            }
        }
        if(result)
        {
            tmp_nodes.push_back(new_root);
            size_t start_index=(type==Path::Global?1:0);
            for(size_t i=start_index;i<tmp_nodes.size()-1;i++)
            {
                tmp_nodes[i]->parent_=tmp_nodes[i+1];
                tmp_nodes[i+1]->children_.push_back(tmp_nodes[i]);
            }
            path_=Path();
            clean_vector(tree_);
            tree_.assign(tmp_nodes.begin()+start_index,tmp_nodes.end());          
            node_origin_=new_root;
            updateNode(node_origin_);
            generatePath();
        }
    }
    return result;
}

float PFRRTStar::getRandomNum()
{
    random_device rand_rd;
    mt19937 rand_gen(rand_rd());
    uniform_real_distribution<> rand_unif(0, 1.0);
    return rand_unif(rand_gen);
}

Vector2d PFRRTStar::getRandom2DPoint()
{
    Vector3d lb = world_->getLowerBound();
    Vector3d ub = world_->getUpperBound();

    Vector2d rand_point=Vector2d( (ub(0)-lb(0))*getRandomNum()+lb(0),(ub(1)-lb(1))*getRandomNum()+lb(1));
    return rand_point;           
}

Vector3d PFRRTStar::sampleInEllipsoid()
{
    float cmin=EuclideanDistance(node_target_,node_origin_);
    Vector3d a_1=(node_target_->position_-node_origin_->position_)/cmin;  
    RowVector3d id_t(1,0,0);
    Matrix3d M=a_1*id_t;
    JacobiSVD<MatrixXd> SVD(M,ComputeFullU|ComputeFullV);
    Matrix3d U=SVD.matrixU();   
    Matrix3d V=SVD.matrixV();
    Matrix3d A=Matrix3d::Zero();
    A(0,0)=A(1,1)=1,A(2,2)=U.determinant()*V.determinant();
    Matrix3d C=U*A*V;

    float cbest=path_.dis_+1.0f;

    Matrix3d L=Matrix3d::Zero();
    L(0,0)=cbest*0.5,L(1,1)=L(2,2)=sqrt(powf(cbest, 2)-powf(cmin,2))*0.5;

    float theta1=acos(2*getRandomNum()-1);
    float theta2=2*PI*getRandomNum();
    float radius=powf(getRandomNum(),1.0/3);

    Vector3d random_ball;
    random_ball << radius*sin(theta1)*cos(theta2),
                   radius*sin(theta1)*sin(theta2),
                   radius*cos(theta1);

    Vector3d random_ellipsoid=C*L*random_ball;

    Vector3d center= (node_origin_->position_+node_target_->position_)*0.5;
    Vector3d point=random_ellipsoid+center;
    return point;
}

Vector2d PFRRTStar::sampleInSector()
{
    float sample_sector_lb_=2.0f;

    vector<float> theta_record;

    Vector2d start_2D=project2plane(node_origin_->position_);
    for(size_t i=0;i<path_.nodes_.size()-1;i++)
    {
        Vector2d pt_2D=project2plane(path_.nodes_[i]->position_);
        Vector2d diff=pt_2D-start_2D;
        if(diff.norm() < sample_sector_lb_) continue;
        float theta=atan2f(diff(1),diff(0));
        theta_record.push_back(theta);       
    }

    if(theta_record.empty()) return getRandom2DPoint(); 

    default_random_engine engine;

    uniform_int_distribution<unsigned> rand_int(0,theta_record.size()-1);

    float theta_rate = getRandomNum();

    float theta=theta_record[rand_int(engine)]+(theta_rate-0.5)*20.0*PI/180.0;

    Vector2d sub_goal_2D=project2plane(path_.nodes_.front()->position_);

    float sample_sector_ub=(EuclideanDistance(start_2D,sub_goal_2D)+EuclideanDistance(start_2D,end_pos_2D_))*0.5+1.0f;

    float rand_num=getRandomNum();

    float R=sqrt(rand_num)*(sample_sector_ub-sample_sector_lb_)+sample_sector_lb_;

    Vector2d rand_point(R*cos(theta) , R*sin(theta));
    rand_point+=project2plane(node_origin_->position_);

    return rand_point;
}

Vector2d PFRRTStar::sample()
{
    Vector2d point_sample;
    switch(planning_state_)
    {
        case Global:
        {
            if(!path_.nodes_.empty()) point_sample=project2plane(sampleInEllipsoid());
            else
                point_sample=(getRandomNum() < goal_biased_ )?project2plane(node_target_->position_):getRandom2DPoint();
        }
        break;
        case Roll:
            point_sample=path_.nodes_.empty()?getRandom2DPoint():sampleInSector();
        break;
        case WithoutGoal:
            point_sample=getRandom2DPoint();
        break;
        default:
        break;
    }
    return point_sample;
}

Node* PFRRTStar::findNearest(const Vector2d &point)
{
    float min_dis = INF;
    Node* node_closest = NULL;
  
    for(const auto&node:tree_) 
    { 
        //Here use Manhattan distance instead of Euclidean distance to improve the calculate speed.
        float tmp_dis=fabs(point(0)-node->position_(0))+fabs(point(1)-node->position_(1));
        if(tmp_dis < min_dis) 
        {
            min_dis = tmp_dis;
            node_closest = node;
        }
    }
    return node_closest;
}

Vector2d PFRRTStar::steer(const Vector2d &point_rand_projection, const Vector2d &point_nearest_projection)
{
    Vector2d point_new_projection;
    Vector2d steer_p = point_rand_projection - point_nearest_projection;
    float steer_norm=steer_p.norm();
    if (steer_norm > step_size_) //check if the EuclideanDistance between two nodes is larger than the maximum travel step size
        point_new_projection = point_nearest_projection +  steer_p * step_size_/steer_norm;
    else 
        point_new_projection=point_rand_projection;
    return point_new_projection;
}

Node* PFRRTStar::fitPlane(const Vector2d &p_original)
{
    Node* node = NULL;
    Vector3d p_surface;
    
    //make sure that p_original can be projected to the surface,otherwise the nullptr will be returned
    if(world_->project2surface(p_original(0),p_original(1),&p_surface))
    {
        node=new Node;
        node->plane_=new Plane(p_surface,world_,radius_fit_plane_,fit_plane_arg_);
        node->position_ = p_surface + h_surf_ * node->plane_->normal_vector;
    }
    return node;
}

void PFRRTStar::fitPlane(Node* node)
{
    Vector2d init_coord=node->plane_->init_coord;
    //cout << RowVector3d(node->plane_->normal_vector) << endl;
    //cout << RowVector3d(node->position_) << endl;
    delete node->plane_;
    node->plane_=NULL;
    Vector3d p_surface;
    if(world_->project2surface(init_coord(0),init_coord(1),&p_surface))
    {
        node->plane_=new Plane(p_surface,world_,radius_fit_plane_,fit_plane_arg_);
        //cout << RowVector3d(node->plane_->normal_vector) << endl;
        //cout << RowVector3d(node->position_) << endl;
        node->position_=p_surface + h_surf_ * node->plane_->normal_vector;
    }
}

void PFRRTStar::findNearNeighbors(Node* node_new,vector<pair<Node*,float>> &record) 
{ 
    for (const auto&node:tree_) 
    {
        if(EuclideanDistance(node_new,node) < neighbor_radius_ && world_->collisionFree(node_new,node) )
            record.push_back( pair<Node*,float>(node,calCostBetweenTwoNode(node_new,node)) );
    }
}

void PFRRTStar::findParent(Node* node_new,const vector<pair<Node*,float>> &record) 
{    
    Node* node_parent=NULL;
    float min_cost = INF;
    for(const auto&rec:record) 
    { 
        Node* node=rec.first;
        float tmp_cost=node->cost_+rec.second;
        if(tmp_cost < min_cost)
        {
            node_parent=node;
            min_cost=tmp_cost;
        }
    }
    node_new->parent_=node_parent;
    node_new->cost_=min_cost;
    node_parent->children_.push_back(node_new);
}

void PFRRTStar::reWire(Node* node_new,const vector<pair<Node*,float>> &record) 
{ 
    for (const auto&rec:record) 
    { 
        Node* node=rec.first;
        float tmp_cost=node_new->cost_+rec.second;//cost value if the new node is the parent
        float costdifference=node->cost_-tmp_cost;//compare the two and update if the latter is smaller,change new node to the parent node
        if(costdifference > 0) 
        {
            deleteChildren(node->parent_,node);
            node->parent_=node_new;
            node->cost_=tmp_cost;
            node_new->children_.push_back(node);
            updateChildrenCost(node,costdifference);
        }
    }
}

void PFRRTStar::deleteChildren(Node* node_parent,Node* node_children)
{
    for(vector<Node*>::iterator it=node_parent->children_.begin();it!=node_parent->children_.end();++it)
    {
        if(*it==node_children)
        {
            node_parent->children_.erase(it);
            break;
        }
    }
}

void PFRRTStar::updateChildrenCost(Node* &node_root, const float &costdifference) 
{
    for(auto&node:node_root->children_)
    {
        node->cost_ -= costdifference;
        updateChildrenCost(node,costdifference); 
    }
}

void PFRRTStar::closeCheck(Node* node)
{
    switch(planning_state_)
    {
        case Global:
        {
            if(EuclideanDistance(node,node_target_) < goal_threshold_ && world_->collisionFree(node,node_target_))
                close_check_record_.push_back(pair<Node*,float>(node,calCostBetweenTwoNode(node,node_target_)));
        }
        break;
        case Roll: 
        {
            if(EuclideanDistance(project2plane(node->position_),end_pos_2D_) < sub_goal_threshold_)
                close_check_record_.push_back(pair<Node*,float>(node,powf(EuclideanDistance(end_pos_2D_,project2plane(node->position_)),3)));
        }
        break;
        default:
        break;
    }
}

float PFRRTStar::calPathDis(const vector<Node*> &nodes)
{
    float dis=0.0f;
    for(size_t i=0;i < nodes.size()-1; i++)
        dis+=EuclideanDistance(nodes[i],nodes[i+1]);
    return dis;
}

void PFRRTStar::generatePath()
{
    switch(planning_state_)
    {
        case Global:
        {
            Node* node_choosed=NULL;
            float min_cost=path_.cost_;
            for (const auto&rec:close_check_record_) 
            {
                Node* node=rec.first;
                float tmp_cost=node->cost_+rec.second;
                if(tmp_cost < min_cost)
                {
                    min_cost=tmp_cost;
                    node_choosed=node;
                }
            }
            if(min_cost!=path_.cost_)
            {
                path_.nodes_.clear();
                path_.nodes_.push_back(node_target_);
                while(node_choosed!=NULL)
                {
                    path_.nodes_.push_back(node_choosed);
                    node_choosed=node_choosed->parent_;
                }
                path_.cost_=min_cost;
                path_.dis_=calPathDis(path_.nodes_);
                path_.type_=Path::Global;
            }
        }
        break;
        case Roll:
        {
            path_=Path();
            Node* sub_goal = NULL;
            float min_cost = INF;  
            for(const auto&rec:close_check_record_) 
            { 
                Node* node=rec.first;
                float tmp_cost=node->cost_+rec.second;
                if (tmp_cost < min_cost) 
                {
                    min_cost = tmp_cost; 
                    sub_goal = node;
                }
            }
            if(sub_goal!=NULL)
            {
                while(sub_goal!=NULL)
                {
                    path_.nodes_.push_back(sub_goal);
                    sub_goal=sub_goal->parent_;
                }
                path_.cost_=path_.nodes_.front()->cost_;
                path_.dis_=calPathDis(path_.nodes_);
                path_.type_=Path::Sub;
            }
            else
            {
                //If close_check_record is empty,adjust the threshold according to the current information of distance
                //so that next time it will more likely to get a solution
                float min_dis=INF;
                for(const auto&node:tree_)
                {
                    float tmp_dis=EuclideanDistance(project2plane(node->position_),end_pos_2D_);
                    if(tmp_dis<min_dis) min_dis=tmp_dis;
                }
                sub_goal_threshold_=min_dis+1.0f;
            }
        }
        break;
        default:
        break;
    }
}

Path PFRRTStar::planner(const int &max_iter,const double &max_time)
{
    if(planning_state_==Invalid)
    {
        ROS_ERROR("Illegal operation:the planner is at an invalid working state!!");
        return {};
    }
    double time_now=curr_time_;
    timeval start;gettimeofday(&start,NULL);
    while (curr_iter_ < max_iter && curr_time_ < max_time)
    {
        //Update current iteration
        curr_iter_++;

        //Update current time consuming
        timeval end;gettimeofday(&end,NULL);
        float ms=1000*(end.tv_sec-start.tv_sec)+0.001*(end.tv_usec-start.tv_usec);
        curr_time_=ms+time_now;

        //Sample to get a random 2D point
        Vector2d rand_point_2D=sample();

        //Find the nearest node to the random point
        Node* nearest_node= findNearest(rand_point_2D);
        Vector2d nearest_point_2D = project2plane(nearest_node->position_);

        //Expand from the nearest point to the random point
        Vector2d new_point_2D = steer(rand_point_2D,nearest_point_2D);

        //Based on the new 2D point,
        Node* new_node = fitPlane(new_point_2D); 

        if( new_node!=NULL//1.Fail to fit the plane,it will return a null pointer
            &&world_->isInsideBorder(new_node->position_)//2.The position is out of the range of the grid map.
          ) 
        {
            //Get the set of the neighbors of the new node in the tree
            vector<pair<Node*,float>> neighbor_record;
            findNearNeighbors(new_node,neighbor_record);

            //Select an appropriate parent node for the new node from the set.
            if(!neighbor_record.empty()) findParent(new_node,neighbor_record);
            //Different from other RRT algorithm,it is posible that the new node is too far away from the whole tree.If
            //so,discard the new node.
            else
            {
                delete new_node;
                continue;
            }

            //Add the new node to the tree
            tree_.push_back(new_node);

            //Rewire the tree to optimize it
            reWire(new_node,neighbor_record);

            //Check if the new node is close enough to the goal
            closeCheck(new_node);

            if(planning_state_==Global) generatePath();
        }
        else  
            delete new_node;
    }

    if(planning_state_==Roll) generatePath();

    visTree(tree_,tree_vis_pub_);
    pubTraversabilityOfTree(tree_tra_pub_);
    
    return path_;
}

void PFRRTStar::pubTraversabilityOfTree(Publisher* tree_tra_pub)
{
    if(tree_tra_pub==NULL) return;
    Float32MultiArray msg;
    for(const auto&node:tree_)
    {
        msg.data.push_back(node->position_(0));
        msg.data.push_back(node->position_(1));
        msg.data.push_back(node->position_(2));
        msg.data.push_back(node->plane_->traversability);
    }
    tree_tra_pub->publish(msg);
}
