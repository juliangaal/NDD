
/**
  * @file SAC-IA+ICP.cpp
  * @author julian 
  * @date 12/23/22
*/

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>//体素下采样滤波
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/fpfh_omp.h> //fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>//sac_ia算法
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(const pointcloud::Ptr& input_cloud, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree)
{
    //-------------------------法向量估计-----------------------
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    n.setNumberOfThreads(8);//设置openMP的线程数
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8); //指定8核计算
    f.setInputCloud(input_cloud);
    f.setInputNormals(normals);
    f.setSearchMethod(tree);
    f.setKSearch(10);
    f.compute(*fpfh);
    return fpfh;
}

int main(int argc, char** argv)
{
    //计算sacia和icp的实践
    clock_t start_sac_ia, end_sac_ia,start_icp, end_icp ;
    
    pointcloud::Ptr source_cloud(new pointcloud);
    pointcloud::Ptr target_cloud(new pointcloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/julian/dev/loop_closure/vorplatz/00000.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/julian/dev/loop_closure/vorplatz/00310.pcd", *target_cloud);
    
    //---------------------------去除源点云的NAN点------------------------
    pcl::Indices indices_src; //保存去除的点的索引,后期没用
    pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
    cout << "remove *source_cloud nan" << endl;
    pcl::Indices indices_tgt; //保存去除的点的索引,后期没用
    pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
    cout << "remove *target_cloud nan" << endl;
    
    Eigen::Matrix4f t;
    t << 1, 0, 0, 25,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    pcl::transformPointCloud(*target_cloud, *target_cloud, t);
    
    //-------------------------点云下采样滤波-------------------------
    //原始点云
    pcl::VoxelGrid<pcl::PointXYZ> vg_source;
    vg_source.setLeafSize(0.5, 0.5, 0.5);
    vg_source.setInputCloud(source_cloud);
    pointcloud::Ptr vg_source_cloud(new pointcloud);
    vg_source.filter(*vg_source_cloud);
    cout << "down size *source_cloud from " << source_cloud->size() << " to " <<vg_source_cloud->size() << endl;
    //目标点云
    pcl::VoxelGrid<pcl::PointXYZ> vg_target;
    vg_target.setLeafSize(0.5, 0.5, 0.5);
    vg_target.setInputCloud(target_cloud);
    pointcloud::Ptr vg_target_cloud(new pointcloud);
    vg_target.filter(*vg_target_cloud);
    cout << "down size *target_cloud from " << target_cloud->size() << " to " << vg_target_cloud->size() << endl;
    
    //---------------计算FPFH------------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(vg_source_cloud, tree);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(vg_target_cloud, tree);
    cout<<"fpfh has finished"<<endl;
    //--------------采样一致性SAC_IA初始配准----------------------------
    start_sac_ia = clock();
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(vg_source_cloud);//将体素下采样后的点云作为输入
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(vg_target_cloud);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(0.1);//设置样本之间的最小距离
    //sac_ia.setNumberOfSamples(200);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
    sac_ia.setCorrespondenceRandomness(12); //在选择随机特征对应时，设置要使用的邻居的数量;
    //也就是计算协方差时选择的近邻点个数，该值越大，协防差越精确，但是计算效率越低.(可省)
    //sac_ia.setErrorFunction();//这个调用是可选的
    pointcloud::Ptr sac_ia_cloud(new pointcloud);
    sac_ia.align(*sac_ia_cloud);
    
    end_sac_ia = clock();
    //矩阵变换，将矩阵作用于source_cloud，并保存到sac_ia_cloud，这跟align()直接得到的sac_ia_cloud相同
    //pcl::transformPointCloud(*source_cloud, *sac_ia_cloud, sac_ia.getFinalTransformation());
    std::cerr << "calculate time is: " << float(end_sac_ia - start_sac_ia) / CLOCKS_PER_SEC <<"s"<< endl;
    std::cerr << std::endl << "SAC-IA has converged, score : " << sac_ia.getFitnessScore() << std::endl;
    std::cout << "transformation Matrix : " << std::endl << sac_ia.getFinalTransformation() << std::endl;
    //--------------ICP精配准----------------------------
    start_icp=clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    //将体素下采样后的点云作为输入
//    icp.setInputSource(vg_source_cloud);// 源点云
//    icp.setInputTarget(vg_target_cloud);// 目标点云
//    //该设置需要查看soource 和target的大小，以及距离大小，不然设置最大距离过小将会提前结束
//    //icp.setMaxCorrespondenceDistance(1);// 设置对应点对之间的最大距离（此值对配准结果影响较大）。
//    icp.setMaximumIterations(50);// 最大迭代次数
//    icp.setEuclideanFitnessEpsilon(0.5);// 设置收敛条件是均方误差和小于阈值， 停止迭代；
//    icp.setTransformationEpsilon(1e-10);// 为终止条件设置最小转换差异
//    icp.align(*icp_cloud,sac_ia.getFinalTransformation());//重要，设置初始矩阵，相当于将sac ia 和icp连接起来了
    
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon (0.001);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.5);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1.0);
    // Setting max number of registration iterations.
    ndt.setMaximumIterations(100);
    ndt.setInputCloud(vg_source_cloud);
    ndt.setInputTarget(vg_target_cloud);
    ndt.align(*icp_cloud, sac_ia.getFinalTransformation());
    
    end_icp=clock();
    std::cerr << "calculate time is: " << float(end_icp - start_icp) / CLOCKS_PER_SEC <<"s"<< endl;
    std::cerr << std::endl << "NDT has" << (ndt.hasConverged() ? "" : " not") << " converged, score : " << ndt.getFitnessScore() << std::endl;
    std::cout << "transformation Matrix : " << std::endl << ndt.getFinalTransformation() << std::endl;
    
    //-------------------可视化------------------------------------
    
    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    //--------创建两个显示窗口并设置背景颜色------------
    int v1=0,v2=1,v3=2;
    viewer.createViewPort(0, 0.66, 1, 1.0, v1);
    viewer.createViewPort(0, 0.33, 1, 0.66, v2);
    viewer.createViewPort(0, 0.0, 1, 0.33, v3);
    
    //-----------给点云添加颜色-------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> vg_target_color(vg_target_cloud,255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> vg_source_color(vg_source_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sac_ia_color(sac_ia_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_color(icp_cloud, 0, 255, 0);
    //----------添加点云到显示窗口----------------------
    viewer.addPointCloud(vg_target_cloud, vg_target_color, "source cloud",v1);
    viewer.addPointCloud(vg_source_cloud, vg_source_color, "target1 cloud",v1);
    viewer.addPointCloud(vg_target_cloud, vg_target_color, "target2 cloud", v2);
    viewer.addPointCloud(sac_ia_cloud, sac_ia_color, "sac_ia cloud",v2);
    viewer.addPointCloud(vg_target_cloud, vg_target_color, "target3 cloud", v3);
    viewer.addPointCloud(icp_cloud, icp_color, "icp cloud",v3);
    
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
    return 0;
}