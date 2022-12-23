/**
  * @file SAC-IA.cpp
  * @author julian
  * @date 12/21/22
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>//体素下采样滤波
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/fpfh_omp.h> //fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>//sac_ia算法
#include <pcl/visualization/pcl_visualizer.h>

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

void visualize_pcd(const pointcloud::Ptr& pcd_src, const pointcloud::Ptr& pcd_tgt, const pointcloud::Ptr &pcd_final)
{

    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    //--------创建两个显示窗口并设置背景颜色------------
    int v1,v2;
    viewer.createViewPort(0, 0.0, 1, 1.0, v1);
    viewer.createViewPort(1, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.setBackgroundColor(0.05, 0, 0, v2);
    //-----------给点云添加颜色-------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 255, 0);
    //----------添加点云到显示窗口----------------------
    
    viewer.addPointCloud(pcd_src, src_h, "source cloud",v1);
    viewer.addPointCloud(pcd_tgt, tgt_h, "target cloud",v1);
    viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud", v2);
    viewer.addPointCloud(pcd_final, final_h, "final cloud",v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
}

int main(int argc, char** argv)
{

    clock_t start, end, time;
    start = clock();
    pointcloud::Ptr source_cloud(new pointcloud);
    pointcloud::Ptr target_cloud(new pointcloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../data/000209.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../data/001684.pcd", *target_cloud);

    //---------------------------去除源点云的NAN点------------------------
    vector<int> indices_src; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
    cout << "remove *source_cloud nan" << endl;
    //-------------------------源点云下采样滤波-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vs;
    vs.setLeafSize(0.5, 0.5, 0.5);
    vs.setInputCloud(source_cloud);
    pointcloud::Ptr source(new pointcloud);
    vs.filter(*source);
    cout << "down size *source_cloud from " << source_cloud->size() << " to " <<source->size() << endl;

    //--------------------------去除目标点云的NAN点--------------------
    vector<int> indices_tgt; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
    cout << "remove *target_cloud nan" << endl;
    //----------------------目标点云下采样滤波-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vt;
    vt.setLeafSize(0.5, 0.5, 0.5);
    vt.setInputCloud(target_cloud);
    pointcloud::Ptr target(new pointcloud);
    vt.filter(*target);
    cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;
    //---------------计算源点云和目标点云的FPFH------------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);
    cout << "calculated fpfh features\n";

    //--------------采样一致性SAC_IA初始配准----------------------------
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(0.1);//设置样本之间的最小距离
    //sac_ia.setNumberOfSamples(200);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
    sac_ia.setCorrespondenceRandomness(6); //在选择随机特征对应时，设置要使用的邻居的数量;
    //也就是计算协方差时选择的近邻点个数，该值越大，协防差越精确，但是计算效率越低.(可省)
    //sac_ia.setErrorFunction();//这个调用是可选的
    pointcloud::Ptr align(new pointcloud);
    sac_ia.align(*align);
    end = clock();
    //    pcl::transformPointCloud(*source_cloud, *align, sac_ia.getFinalTransformation());
    //    // pcl::io::savePCDFile("crou_output.pcd", *align);
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC <<"s"<< endl;
    cout << "\nSAC_IA has converged, score is " << sac_ia.getFitnessScore() << endl;
    cout << "变换矩阵：\n" << sac_ia.getFinalTransformation() << endl;
    //-------------------可视化------------------------------------
    visualize_pcd(source_cloud, target_cloud, align);

    return 0;
}