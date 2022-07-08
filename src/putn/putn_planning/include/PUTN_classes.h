/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 */

#ifndef PUTN_CLASSES_H
#define PUTN_CLASSES_H

#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace PUTN
{

const float INF= std::numeric_limits<float>::max();
const float PI = 3.14151f;

class Node;
class Plane;
class World;

/**
 * @brief class for storing the info of vertex.The recorded information of parent and child nodes will be used to bulid
 * the tree.The cost value represents the sum of the cost values required to arrive from the root node of the tree.
 */
class Node 
{
public: 
    std::vector<Node*> children_;
    Node* parent_=NULL;

    Eigen::Vector3d position_;
    float cost_;

    Plane *plane_=NULL;

    Node();
    Node(const Node &node);
    ~Node();
};

/**
 * @brief class for storing information about a path from the start point to the end point.Besides
 * the set of nodes that describe the path,it also saves the Eucildean length and the cost value of
 * the path.The member variable "type_" is used to indecate whether the end point of this path is a
 * real goal or a temporary sub goal
 */
class Path
{
public:
    std::vector<Node*> nodes_;
    float dis_;
    float cost_;
    enum Type{Global,Sub,Empty}type_;

    Path();
    ~Path();
};

/**
 * @brief struct for setting parameters when fitting local plane
 */
struct FitPlaneArg
{
    double w_total_;
    double w_flatness_;
    double w_slope_;
    double w_sparsity_;
    double ratio_max_;
    double ratio_min_;
    double conv_thre_;
};

/**
 * @brief class for describing the local plane.The normal vector can be used to estimate the real 
 * position of the robot when moving on this plane,and the traversability can evaluate whether the
 * the robot is safe when moving on this plane.
 */
class Plane 
{
public: 
    Eigen::Vector2d init_coord;
    std::vector<Eigen::Vector3d> plane_pts;
    Eigen::Vector3d normal_vector;
    float traversability;

    Plane();
    Plane(const Eigen::Vector3d &p_surface,World* world,const double &radius,const FitPlaneArg &arg);
};

namespace visualization{void visWorld(World* world,ros::Publisher* world_vis_pub);}
/**
 * @brief Class for storing obstacles and world dimension.The information of obstacle is stored in a three-dimensional bool array.
 *        Before using the PF-RRT* algorithm,a suitable grid map must be built
 */
class World
{
public:
    friend void visualization::visWorld(World* world,ros::Publisher* world_vis_pub);

    //indicate whether the range of the grid map has been determined
    bool has_map_=false;

    World(const float &resolution=0.1f);
    ~World();

    /**
     * @brief Automatically determine the upperbound and lowerbound of the grid map according to the
     *        information of the input point cloud.
     * @param pcl::PointCloud<pcl::PointXYZ> point cloud input
     * @return void
     */
    void initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud);

    /**
     * @brief Manually specify the upperbound and lowerbound.
     * @param Vector3d
     * @param Vector3d
     * @return void
     */
    void initGridMap(const Eigen::Vector3d &lowerbound,const Eigen::Vector3d &upperbound);
    void setObs(const Eigen::Vector3d &point);

    /**
     * @brief Find the grid closet to the point and return the coordinate of its center
     * @param Vector3d
     * @return Vector3d
     */
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    bool isFree(const Eigen::Vector3d &point);
    bool isFree(const float &coord_x, const float &coord_y, const float &coord_z){return isFree(Eigen::Vector3d(coord_x,coord_y,coord_z));}

    /**
     * @brief Given a 2D coord,start from the lowerbound of the height of the grid map,search upward,
     *        and determine the boundary between the occupied area and the non occupied area as the 
     *        surface point.
     * @param float x(the first dimension)
     * @param float y(the second dimension)
     * @param Vector3d* p_surface(store the result of the projecting)
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    bool project2surface(const float &x,const float &y,Eigen::Vector3d* p_surface); 
    bool project2surface(const Eigen::Vector3d &p_original,Eigen::Vector3d* p_surface){return project2surface(p_original(0),p_original(1),p_surface);}

     /**
     * @brief Check if there is any obstacle between 2 nodes.
     * @param Node* node_start
     * @param Node* node_end
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    bool collisionFree(const Node* node_start,const Node* node_end);
    
    /**
     * @brief Check whether the given point is within the range of the grid map
     * @param Eigen::Vector3i(the index value obtained after discretization of the given point)
     * @return bool true(within range),false（out of range)
     */
    bool isInsideBorder(const Eigen::Vector3i &index);
    bool isInsideBorder(const Eigen::Vector3d &point){return isInsideBorder(coord2index(point));}

    /**
     * @brief get the low bound of the world
     * @param void
     * @return Vector3d
     */
    Eigen::Vector3d getLowerBound(){return lowerbound_;}

    /**
     * @brief get the up bound of the world
     * @param void
     * @return Vector3d
     */
    Eigen::Vector3d getUpperBound(){return upperbound_;}

    /**
     * @brief get resolution of the world
     * @param void
     * @return float
     */
    float getResolution(){return resolution_;} 

    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + lowerbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }

    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-lowerbound_)/resolution_).cast<int>();            
        return index;
    }
//protected:
    bool ***grid_map_=NULL;

    float resolution_;

    Eigen::Vector3i idx_count_;

    Eigen::Vector3d lowerbound_;
    Eigen::Vector3d upperbound_;

    void clearMap();
};

/**
 * @brief Given a 3D point,extract its x and y coordinates and return a 2D point
 * @param Vector3d 
 * @return Vector2d
 */
inline Eigen::Vector2d project2plane(const Eigen::Vector3d &p){return Eigen::Vector2d(p(0),p(1));}
inline Eigen::Vector2d project2plane(const float &x,const float &y){return Eigen::Vector2d(x,y);}

inline float EuclideanDistance(const Eigen::VectorXd &p,const Eigen::VectorXd &q){return (p-q).norm();}
inline float EuclideanDistance(const Node* p,const Node* q){return EuclideanDistance(p->position_,q->position_);}
inline float calCostBetweenTwoNode(const Node* n1,const Node* n2)
{
    float dis= EuclideanDistance(n1,n2);
    float cost=dis*(1.0f+0.1f*( 1/(1.0001f-n1->plane_->traversability) +  1/(1.0001f-n2->plane_->traversability) ));
    return cost;
}

template <typename T>
void clean_vector(std::vector<T*> &vec)
{
    for(auto &element:vec)
    {
        delete element;
        element=NULL;
    }
    vec.clear();
}

}

#endif
