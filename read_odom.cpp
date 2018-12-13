
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <dirent.h>
#include <pcl/filters/voxel_grid.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
using namespace g2o;
using namespace std;

G2O_USE_TYPE_GROUP(slam3d);
int _vertexnum = 0;
vector<VertexSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _vertices;
vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _odometryEdges;
vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _edges;
string _outFilename = "LidarOdom.g2o";
Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix<double, 6, 6>::Identity();

bool saveg2o(Eigen::Isometry3d curr) {
    VertexSE3 *v = new VertexSE3;
    v->setId(_vertexnum++);
    //添加顶点
    Eigen::Isometry3d t_v;
    //当前的t
    t_v = curr;
    v->setEstimate(t_v);
    if (_vertexnum == 1) {
        v->setFixed(true);
    }
    _vertices.push_back(v);
    if (_vertexnum > 1) {
        cout << "  connect between " << _vertexnum - 2 << "---" << _vertexnum - 1
             << endl;
        VertexSE3 *prev = _vertices[_vertexnum - 2]; // debug
        VertexSE3 *cur = _vertices[_vertexnum - 1];

        cout << "  __vertices.size() : after add :  " << _vertices.size()
             << endl; //debug
        Eigen::Isometry3d t_e = prev->estimate().inverse() * cur->estimate();
        Eigen::Isometry3d ttmmpp = t_e; // debug
        EdgeSE3 *e = new EdgeSE3;
        e->setVertex(0, prev); //debug
        e->setVertex(1, cur); //debug
        e->setMeasurement(ttmmpp); //debug
        e->setInformation(_information);
        //正常边两个都有的
        _odometryEdges.push_back(e);
        _edges.push_back(e);
    }
}

    bool savefile() {
        //ROS_ERROR("BEFORE SAVING G2O");
        if (_vertices.size() > 0) {
            //ROS_ERROR("SAVING G2O");
            ofstream fileOutputStream;
            //cerr << "Writing into " << _outFilename << endl;
            fileOutputStream.open(_outFilename.c_str());

            string vertexTag = Factory::instance()->tag(_vertices[0]);
            string edgeTag = Factory::instance()->tag(_edges[0]);
            //todo 去除下划线的操作? _outFilename = "-" ->cout
            ostream &fout = _outFilename != "-" ? fileOutputStream : cout;

            for (size_t i = 0; i < _vertices.size(); ++i) {
                VertexSE3 *v = _vertices[i];
                fout << vertexTag << " " << v->id() << " ";
                //fout << vertexTag << " " << _vertices[i]->id() << " ";
                v->write(fout);
                //_vertices[i]->write(fout);
                fout << endl;
            }

            for (size_t i = 0; i < _edges.size(); ++i) {
                fout << edgeTag << " "
                     << static_cast<VertexSE3 *>(_edges[i]->vertex(0))->id()
                     << " " << static_cast<VertexSE3 *>(_edges[i]->vertex(1))->id()
                     << " ";
                _edges[i]->write(fout);
                fout << endl;
            }
            return true;
        } else {
            return false;
        }
    }

int main(int argc,char** argv){
    ifstream in("/home/echo/small_program/5_read_odom_txt/build/odometry.txt");
    std::string filename;
    std::string line;
    int odom_count = 0;
    if(in) // 有该文件
    {
        while (getline (in, line)) // line中不包括每行的换行符
        {
            _vertexnum = odom_count;
            std::istringstream is(line);
            std::string str0,str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11;
            is>>str0>>str1>>str2>>str3>>str4>>str5>>str6>>str7>>str8>>str9>>str10>>str11;
            cout<<str2<<","<<str3<<","<<str4<<endl;
            Eigen::Isometry3d currodom = Eigen::Isometry3d::Identity();
            currodom(0,0)=atof(str0.data());
            currodom(0,1)=atof(str1.data());
            currodom(0,2)=atof(str2.data());
            currodom(0,3)=atof(str3.data());
            currodom(1,0)=atof(str4.data());
            currodom(1,1)=atof(str5.data());
            currodom(1,2)=atof(str6.data());
            currodom(1,3)=atof(str7.data());
            currodom(2,0)=atof(str8.data());
            currodom(2,1)=atof(str9.data());
            currodom(2,2)=atof(str10.data());
            currodom(2,3)=atof(str11.data());
            cout<<currodom.matrix()<<endl;
            saveg2o(currodom);
            odom_count ++;
        }
    }
    else // 没有该文件
    {
        cout <<"no such file" << endl;
    }
    savefile();
    return 0;
}