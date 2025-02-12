//   R. Zhou and L. He. NDD: A 3D Point Cloud Descriptor  Based on Normal Distribution for Loop Closure Detection.

//THANKS:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.
//  G. Kim and A. Kim, “Scan context: Egocentric spatial descriptor for place recognition within 3d point cloud map,” 
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.


#include "NDD.h"

using namespace ndd;

double NDDManager::CorrBtnNDD(Eigen::MatrixXd &_ndd1, Eigen::MatrixXd &_ndd2)
{
    auto ave1 = _ndd1.mean();
    auto ave2 = _ndd2.mean();
    
    //correlation coefficient NDDs_corr
    auto A = _ndd1 - Eigen::MatrixXd::Ones(_ndd1.rows(), _ndd1.cols()) * ave1;
    auto B = _ndd2 - Eigen::MatrixXd::Ones(_ndd2.rows(), _ndd2.cols()) * ave2;
    auto R1 = A.cwiseProduct(B);
    
    double r1, r2, r3;
    r1 = R1.sum();
    r2 = A.squaredNorm();
    r3 = B.squaredNorm();
    
    double NDDs_corr = abs(r1 / sqrt(r2 * r3));
    
    return 1.0 - NDDs_corr;
    
} // CorrBtnNDD


int NDDManager::fastAlignUsingVkey(Eigen::MatrixXd &_vkey1, Eigen::MatrixXd &_vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 1000;
    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
    {
        Eigen::MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
        
        Eigen::MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
        
        double cur_diff_norm = vkey_diff.norm();
        if (cur_diff_norm < min_veky_diff_norm)
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }
    
    return argmin_vkey_shift;
    
} // fastAlignUsingVkey


std::pair<double, int> NDDManager::distanceBtnNDD(Eigen::MatrixXd &_ndd1, Eigen::MatrixXd &_ndd2)
{
    // 1. fast alignment
    Eigen::MatrixXd vkey_ndd1 = AlignmentKeyFromNDD(_ndd1);
    Eigen::MatrixXd vkey_ndd2 = AlignmentKeyFromNDD(_ndd2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_ndd1, vkey_ndd2);
    
    const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _ndd1.cols()); // a half of search range
    std::vector<int> shift_idx_search_space{argmin_vkey_shift};
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
    {
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + _ndd1.cols()) % _ndd1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + _ndd1.cols()) % _ndd1.cols());
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());
    
    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_dist = 1000;
    for (int num_shift: shift_idx_search_space)
    {
        Eigen::MatrixXd ndd2_shifted = circshift(_ndd2, num_shift);
        double cur_dist = CorrBtnNDD(_ndd1, ndd2_shifted);
        if (cur_dist < min_dist)
        {
            argmin_shift = num_shift;
            min_dist = cur_dist;
        }
    }
    
    return {min_dist, argmin_shift};
    
} // distanceBtnNDD


Eigen::MatrixXd NDDManager::makeNDD(pcl::PointCloud<NDDPointType> &_scan_down)
{
    TicToc t_making_desc;
    
    int num_pts_scan_down = static_cast<int>(_scan_down.points.size()); //down sample
    
    // main
    const int NO_POINT = -1000;
    Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
    
    //edit to vector (20x60 cells)
    std::array<Eigen::Matrix<double, 30, 3>, 1200> points_cell;
    // std::array<Eigen::Matrix<double, Dynamic, 3>, Dynamic> points_cell;
    Eigen::MatrixXd cell_bin_counter = Eigen::MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
    
    NDDPointType pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sector_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z;
        
        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);
        
        // if range is out of roi, pass
        if (azim_range > PC_MAX_RADIUS)
        {
            continue;
        }
        
        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        sector_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);
        
        int array_index = (ring_idx - 1) * PC_NUM_SECTOR + sector_idx - 1;
        int corresponding_counter = cell_bin_counter(ring_idx - 1, sector_idx - 1);
        
        // points_in_cell(i) = pt.x,pt.y,pt.z;
        if (corresponding_counter < max_num_in_cell)
        {
            points_cell[array_index](corresponding_counter, 0) = pt.x;
            points_cell[array_index](corresponding_counter, 1) = pt.y;
            points_cell[array_index](corresponding_counter, 2) = pt.z;
        } else
        {
            continue;
        }
        
        cell_bin_counter(ring_idx - 1, sector_idx - 1) = cell_bin_counter(ring_idx - 1, sector_idx - 1) + 1;
        desc(ring_idx - 1, sector_idx - 1) = 0; //init
    }
    
    // reset no points to zero (for cosine dist later)
    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
    {
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
        {
            if (desc(row_idx, col_idx) == NO_POINT)
            {
                desc(row_idx, col_idx) = 0;
            } else
            {
            
                Eigen::MatrixXd points_in_cell = points_cell[row_idx * PC_NUM_SECTOR + (col_idx)];
                for (int k = 1; k < points_in_cell.rows(); k++)
                {
                    if (abs(points_in_cell(k, 0)) < 1e-5)
                    {
                        points_in_cell = points_in_cell.block(1, 0, k - 1, 3);
                        break;
                    }
                }
            
                if (points_in_cell.rows() < 5)
                {
                    desc(row_idx, col_idx) = 0;
                } else
                {
                    Eigen::MatrixXd points_mean = points_in_cell.colwise().mean();
                
                    Eigen::MatrixXd points_cen = points_in_cell;
                    for (int i = 0; i < points_cen.rows(); i++)
                    {
                        points_cen.row(i) = points_cen.row(i) - points_mean;
                    }
                    //Seeking the zero-mean column vector matrix
                    Eigen::MatrixXd zeroMeanMat = points_in_cell;
                    //zero-mean column vec (from Eigen::MatrixXd to RowVectorXd)
                    Eigen::RowVectorXd meanVecRow(Eigen::RowVectorXd::Map(points_mean.data(), 3));
                    zeroMeanMat.rowwise() -= meanVecRow;
                    //cov
                    Eigen::MatrixXd points_covMat =
                        (zeroMeanMat.adjoint() * zeroMeanMat) / double(points_in_cell.rows() - 1);
                    //-points_cen' * points_in_cell_covinv * points_cen/2
                    Eigen::MatrixXd gaussianValue =
                        -0.5 * (points_cen * points_covMat.inverse() * points_cen.transpose());
                    Eigen::MatrixXd pd = gaussianValue.array().exp();
                    // std::cout << "sum_pds" << "\n"  << pd<<endl;
                    float sum_pds = pd.trace();
                
                    //Perform well by just using a single feature (probability density score) to describe a PC.
                    desc(row_idx, col_idx) = sum_pds;
                }
            }
        }
    }
    t_making_desc.toc("NDD making");
    
    return desc;
} // NDDManager::makeNDD


Eigen::MatrixXd NDDManager::SearchingKeyFromNDD(Eigen::MatrixXd &_desc)
{
    //  rowwise sum vector
    Eigen::MatrixXd searching_key(_desc.rows(), 1);
    for (int row_idx = 0; row_idx < _desc.rows(); row_idx++)
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        searching_key(row_idx, 0) = curr_row.sum();
    }
    
    return searching_key;
} // NDDManager::SearchingKeyFromNDD


Eigen::MatrixXd NDDManager::AlignmentKeyFromNDD(Eigen::MatrixXd &_desc)
{
    //columnwise sum vector
    Eigen::MatrixXd alignment_key(1, _desc.cols());
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++)
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        alignment_key(0, col_idx) = curr_col.sum();
    }
    
    return alignment_key;
} // NDDManager::AlignmentKeyFromNDD


void NDDManager::makeAndSaveNDDAndKeys(pcl::PointCloud<NDDPointType> &_scan_down)
{
    Eigen::MatrixXd ndd = makeNDD(_scan_down); // v1
    Eigen::MatrixXd SearchingKey = SearchingKeyFromNDD(ndd);
    Eigen::MatrixXd AlignmentKey = AlignmentKeyFromNDD(ndd);
    std::vector<float> NDD_invkey_vec = eig2stdvec(SearchingKey);
    
    polarcontexts_.push_back(ndd);
    polarcontext_invkeys_.push_back(SearchingKey);
    polarcontext_vkeys_.push_back(AlignmentKey);
    polarcontext_invkeys_mat_.push_back(NDD_invkey_vec);
} // NDDManager::makeAndSaveNDDAndKeys


std::tuple<int, int, float> NDDManager::detectLoopClosureID()
{
    int loop_id = -1; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")
    int curr_id = 0;
    
    auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)
    auto curr_desc = polarcontexts_.back(); // current observation (query)
    
    // KD-tree
    if (polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::cout.precision(3);
        std::cout << "[Not loop] < NUM_EXCLUDE_RECENT\n";
        return { curr_id, loop_id, 0.0 }; // Early return
    }
    
    // tree_ reconstruction (not mandatory to make everytime)
    if (tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
    {
        TicToc t_tree_construction;
        
        polarcontext_invkeys_to_search_.clear();
        polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(),
                                               polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT);
        
        polarcontext_tree_.reset();
        polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_,
                                                          10 /* max leaf */ );
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;
    
    double min_dist = 10000000; // init with large num
    int nn_align = 0;
    int nn_idx = 0;
    
    // knn search
    std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);
    std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);
    
    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
    knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10));
    t_tree_search.toc("Tree search");
    
    TicToc t_calc_dist;
    for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
    {
        Eigen::MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
        std::pair<double, int> corr_dist_result = distanceBtnNDD(curr_desc, polarcontext_candidate);
        
        double candidate_dist = corr_dist_result.first;
        int candidate_align = corr_dist_result.second;
        
        if (candidate_dist < min_dist)
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;
            
            nn_idx = candidate_indexes[candidate_iter_idx];
        }
    }
    t_calc_dist.toc("Distance calc");
    
    curr_id = polarcontexts_.size() - 1;
    
    /* 
     * loop threshold check
     */
    if (min_dist < DIST_THRES)
    {
        loop_id = nn_idx;
        std::cout.precision(3);
        std::cout << "[Loop found] Nearest distance: " << min_dist << " btn " << curr_id << " and "
                  << nn_idx << ".\n";
        std::cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg.\n";
    } else
    {
        std::cout.precision(3);
        std::cout << "[Not loop] Nearest distance: " << min_dist << " btn " << curr_id << " and "
                  << nn_idx << ".\n";
        std::cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg.\n";
    }
    
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    return { curr_id, loop_id, yaw_diff_rad };
} // NDDManager::detectLoopClosureID
