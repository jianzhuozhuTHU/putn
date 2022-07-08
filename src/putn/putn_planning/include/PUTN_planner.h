#ifndef PUTN_PLANNER_H
#define PUTN_PLANNER_H

#include <utility>
#include "PUTN_vis.h"

namespace PUTN
{
namespace planner
{

/**
 * @brief represents four different working states of the planner.
 */
enum PlanningState{Global,Roll,WithoutGoal,Invalid}; 

class PFRRTStar
{
public:
    //ros related,which will not work unless the user assigns.
    ros::Publisher* tree_vis_pub_=NULL;
    ros::Publisher* goal_vis_pub_=NULL;
    ros::Publisher* tree_tra_pub_=NULL;

    PFRRTStar();
    PFRRTStar(const double &height,World* world);//Input the height of the robot center,and the array of grid map.
    ~PFRRTStar();

    /** 
     * @brief Set the origin and target for the planner.According to whether they are successfully projected to the
     *        surface,the planner will convert to 3 different working states:Global,Roll,Invalid.
     * @param Vector3d start_point
     * @param Vector3d end_point
     * @return void
     * @note In fact,only the x and y dimensions of the input point are used
     */
    void initWithGoal(const Eigen::Vector3d &start_pos,const Eigen::Vector3d &end_pos);

     /** 
     * @brief Only set the origin for the planner.According to whether it's successfully projected to the
     *        surface,the planner will convert to 2 different working states:WithoutGoal,Invalid.
     * @param Vector3d start_point
     * @return void
     */
    void initWithoutGoal(const Eigen::Vector3d &start_pos);

    /**
     * @brief Expand the tree to search the solution,and stop after reaching the max iterations or max time.
     * @param int max_iter
     * @param float max_time
     * @return Path(The solution to the goal)
     */
    Path planner(const int &max_iter,const double &max_time);

    int getCurrentIterations(){return curr_iter_;}

    double getCurrentTime(){return curr_time_;}

    void setFitPlaneRadius(const float &radius){radius_fit_plane_=radius;}
    
    void setFitPlaneArg(const FitPlaneArg &fit_plane_arg){fit_plane_arg_=fit_plane_arg;}

    void setStepSize(const double &step_size){step_size_=step_size;}

    void setGoalThre(const double &threshold){goal_threshold_=threshold;}

    void setGoalBiased(const double &goal_biased){goal_biased_=goal_biased;}

    void setNeighborRadius(const double &neighbor_radius){neighbor_radius_=neighbor_radius;}

    Node* origin(){return node_origin_;}
    Node* target(){return node_target_;}

    std::vector<Node*> tree(){return tree_;}
    Path path(){return path_;}
    PlanningState state(){return planning_state_;}

    /**
     * @brief Project a point to the surface and then fit a local plane on it.Generate a new node based on the plane.
     * @param Vector2d 
     * @return Node*
     */
    Node* fitPlane(const Eigen::Vector2d &p_original);
    Node* fitPlane(const Eigen::Vector3d &p_original){return fitPlane(project2plane(p_original));}

    /**
     * @brief According to the init coordinates stored by the node,update its information about plane and position.
     * @param Node*
     * @return void
     * @note Unlike the above function,it doesn't create new nodes,but updates the existing node
     */
    void fitPlane(Node* node);


protected:
//Data Members
    Node* node_origin_=NULL;
    Node* node_target_=NULL;

    int curr_iter_;
    double curr_time_;//(in ms)

    //To accelerate the speed of generating the initial solution,the tree will grow toward the target with it,a centain probability 
    double goal_biased_=0.15;

    double goal_threshold_=1.0;
    double sub_goal_threshold_=1.0;

    double inherit_threshold_=1.25;

    //step size used when generating new nodes
    double step_size_=0.2;

    //parameters related to function fitPlane
    float h_surf_;
    FitPlaneArg fit_plane_arg_={1.0,2000.0,0.0014,0.4,0.25,0.4,0.1152};
    double radius_fit_plane_=1.0;

    //radius used in function FindNeighbors
    float neighbor_radius_=1.0f;

    PlanningState planning_state_;

    World* world_;

    //used in function generatePath
    std::vector<std::pair<Node*,float>> close_check_record_;

    std::vector<Node*> tree_;

    Path path_;

    //record 2D information of target,when the target can't be projected to surface,it will be used in rolling-planning
    Eigen::Vector2d end_pos_2D_;

//Function Members

//----------funtions for inherit
    void updateNode(Node *node_input);

    bool inheritPath(Node* new_root,Path::Type type);

    void addInvalidNodes(Node* &node_input,const bool &ifdelete,std::vector<Node*> &invalid_nodes);

    void trimTree();

    bool inheritTree(Node* new_root);
//----------

//----------funtions for sample    

    float getRandomNum();

    Eigen::Vector2d getRandom2DPoint();

    Eigen::Vector3d sampleInEllipsoid();

    Eigen::Vector2d sampleInSector();

    /**
     * @brief Sample to get a random 2D point in various ways.It integrates all the above sampling functions
     * @param void
     * @return Vector2d
     */
    Eigen::Vector2d sample();
//----------

    Node* findNearest(const Eigen::Vector2d &point);

    Eigen::Vector2d steer(const Eigen::Vector2d &point_rand_projection, const Eigen::Vector2d &point_nearest);

    void findNearNeighbors(Node* node_new,std::vector<std::pair<Node*,float>> &record);

    void findParent(Node* node_new,const std::vector<std::pair<Node*,float>> &record);

    void reWire(Node* node_new,const std::vector<std::pair<Node*,float>> &record);

    void deleteChildren(Node* node_parent,Node* node_children);

    void updateChildrenCost(Node* &node_root, const float &costdifference);

    /**
     * @brief Check the node.If it has met the conditions set in advance(i.e.,it's close enough to the target),
     *        add it to the data member "close_check_record_".   
     * @param Node*
     * @return void
     */
    void closeCheck(Node* node);

    /**
     * @brief Read information from "close_check_record_".According to the node information stored in,the function
     *        will select the node with the smallest valuation funtion,and generate a path through the node   
     * @param void
     * @return void
     */
    void generatePath();

    float calPathDis(const std::vector<Node*> &nodes);

    void pubTraversabilityOfTree(ros::Publisher* tree_tra_pub);
};
}
}

#endif