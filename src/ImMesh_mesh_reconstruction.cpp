/* 
This code is the implementation of our paper "ImMesh: An Immediate LiDAR Localization and Meshing Framework".

The source code of this package is released under GPLv2 license. We only allow it free for personal and academic usage. 

If you use any code of this repo in your academic research, please cite at least one of our papers:
[1] Lin, Jiarong, et al. "Immesh: An immediate lidar localization and meshing framework." IEEE Transactions on Robotics
   (T-RO 2023)
[2] Yuan, Chongjian, et al. "Efficient and probabilistic adaptive voxel mapping for accurate online lidar odometry."
    IEEE Robotics and Automation Letters (RA-L 2022)
[3] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled
    state Estimation and mapping package." IEEE International Conference on Robotics and Automation (ICRA 2022)

For commercial use, please contact me <ziv.lin.ljr@gmail.com> and Dr. Fu Zhang <fuzhang@hku.hk> to negotiate a 
different license.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#include "voxel_mapping.hpp"
#include "meshing/mesh_rec_display.hpp"
#include "meshing/mesh_rec_geometry.hpp"
#include "tools/tools_thread_pool.hpp"

extern Global_map       g_map_rgb_pts_mesh;
extern Triangle_manager g_triangles_manager;
extern int              g_current_frame;

extern double                       minimum_pts;
extern double                       g_meshing_voxel_size;
extern FILE *                       g_fp_cost_time;
extern FILE *                       g_fp_lio_state;
extern bool                         g_flag_pause;
extern const int                    number_of_frame;
extern int                          appending_pts_frame;
extern LiDAR_frame_pts_and_pose_vec g_eigen_vec_vec;

int        g_maximum_thread_for_rec_mesh;
std::mutex g_mutex_append_map;
std::mutex g_mutex_reconstruct_mesh;

extern double g_LiDAR_frame_start_time;
double        g_vx_map_frame_cost_time;
static double g_LiDAR_frame_avg_time;

struct Rec_mesh_data_package
{
    pcl::PointCloud< pcl::PointXYZI >::Ptr m_frame_pts;
    Eigen::Quaterniond                     m_pose_q;
    Eigen::Vector3d                        m_pose_t;
    int                                    m_frame_idx;
    Rec_mesh_data_package( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts, Eigen::Quaterniond pose_q, Eigen::Vector3d pose_t, int frame_idx )
    {
        m_frame_pts = frame_pts;
        m_pose_q = pose_q;
        m_pose_t = pose_t;
        m_frame_idx = frame_idx;
    }
};

std::mutex                                  g_mutex_data_package_lock;
std::list< Rec_mesh_data_package >          g_rec_mesh_data_package_list;
std::shared_ptr< Common_tools::ThreadPool > g_thread_pool_rec_mesh = nullptr;

extern int                                  g_enable_mesh_rec;
extern int                                  g_save_to_offline_bin;

LiDAR_frame_pts_and_pose_vec                                                                               g_ponintcloud_pose_vec;


void incremental_mesh_reconstruction( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts, Eigen::Quaterniond pose_q, Eigen::Vector3d pose_t, int frame_idx )
{
    while ( g_flag_pause )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    }

    Eigen::Matrix< double, 7, 1 > pose_vec;
    pose_vec.head< 4 >() = pose_q.coeffs().transpose();
    pose_vec.block( 4, 0, 3, 1 ) = pose_t;
    for ( int i = 0; i < frame_pts->points.size(); i++ )
    {
        g_eigen_vec_vec[ frame_idx ].first.emplace_back( frame_pts->points[ i ].x, frame_pts->points[ i ].y, frame_pts->points[ i ].z,
                                                         frame_pts->points[ i ].intensity );
    }
    g_eigen_vec_vec[ frame_idx ].second = pose_vec;
    // g_eigen_vec_vec.push_back( std::make_pair( empty_vec, pose_vec ) );
    // TODO : add time tic toc

    int                 append_point_step = std::max( ( int ) 1, ( int ) std::round( frame_pts->points.size() / appending_pts_frame ) );
    Common_tools::Timer tim, tim_total;
    g_mutex_append_map.lock();
    g_map_rgb_pts_mesh.append_points_to_global_map( *frame_pts, frame_idx, nullptr, append_point_step );
    std::unordered_set< std::shared_ptr< RGB_Voxel > > voxels_recent_visited = g_map_rgb_pts_mesh.m_voxels_recent_visited;
    g_mutex_append_map.unlock();

    std::atomic< int >    voxel_idx( 0 );
    
    std::mutex mtx_triangle_lock, mtx_single_thr;
    typedef std::unordered_set< std::shared_ptr< RGB_Voxel > >::iterator set_voxel_it;
    std::unordered_map< std::shared_ptr< RGB_Voxel >, Triangle_set >     removed_triangle_list;
    std::unordered_map< std::shared_ptr< RGB_Voxel >, Triangle_set >     added_triangle_list;
    g_mutex_reconstruct_mesh.lock();
    tim.tic();
    tim_total.tic();
    try
    {
        tbb::parallel_for_each( voxels_recent_visited.begin(), voxels_recent_visited.end(), [ & ]( const std::shared_ptr< RGB_Voxel > &voxel ) {
            // std::unique_lock<std::mutex> thr_lock(mtx_single_thr);
            // printf_line;
            if ( ( voxel->m_meshing_times >= 1 ) || ( voxel->m_new_added_pts_count < 0 ) )
            {
                return;
            }
            Common_tools::Timer tim_lock;
            tim_lock.tic();
            voxel->m_meshing_times++;
            voxel->m_new_added_pts_count = 0;
            vec_3 pos_1 = vec_3( voxel->m_pos[ 0 ], voxel->m_pos[ 1 ], voxel->m_pos[ 2 ] );

            // printf("Voxels [%d], (%d, %d, %d) ", count, pos_1(0), pos_1(1), pos_1(2) );
            std::unordered_set< std::shared_ptr< RGB_Voxel > > neighbor_voxels;
            neighbor_voxels.insert( voxel );
            g_mutex_append_map.lock();
            std::vector< RGB_pt_ptr > pts_in_voxels = retrieve_pts_in_voxels( neighbor_voxels );
            if ( pts_in_voxels.size() < 3 )
            {
                g_mutex_append_map.unlock();
                return;
            }
            g_mutex_append_map.unlock();
            // Voxel-wise mesh pull
            pts_in_voxels = retrieve_neighbor_pts_kdtree( pts_in_voxels );
            pts_in_voxels = remove_outlier_pts( pts_in_voxels, voxel );

            std::set< long > relative_point_indices;
            for ( RGB_pt_ptr tri_ptr : pts_in_voxels )
            {
                relative_point_indices.insert( tri_ptr->m_pt_index );
            }

            int iter_count = 0;
            g_triangles_manager.m_enable_map_edge_triangle = 0;

            pts_in_voxels.clear();
            for ( auto p : relative_point_indices )
            {
                pts_in_voxels.push_back( g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ] );
            }
           
            std::set< long > convex_hull_index, inner_pts_index;
            // mtx_triangle_lock.lock();
            voxel->m_short_axis.setZero();
            std::vector< long > add_triangle_idx = delaunay_triangulation( pts_in_voxels, voxel->m_long_axis, voxel->m_mid_axis,
                                                                               voxel->m_short_axis, convex_hull_index, inner_pts_index );

            for ( auto p : inner_pts_index )
            {
                if ( voxel->if_pts_belong_to_this_voxel( g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ] ) )
                {
                    g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ]->m_is_inner_pt = true;
                    g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ]->m_parent_voxel = voxel;
                }
            }

            for ( auto p : convex_hull_index )
            {
                g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ]->m_is_inner_pt = false;
                g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ]->m_parent_voxel = voxel;
            }
            Triangle_set triangles_sets = g_triangles_manager.find_relative_triangulation_combination( relative_point_indices );
            Triangle_set triangles_to_remove, triangles_to_add, existing_triangle;
            // Voxel-wise mesh commit
            triangle_compare( triangles_sets, add_triangle_idx, triangles_to_remove, triangles_to_add, &existing_triangle );
            
            // Refine normal index
            for ( auto triangle_ptr : triangles_to_add )
            {
                correct_triangle_index( triangle_ptr, g_eigen_vec_vec[ frame_idx ].second.block( 4, 0, 3, 1 ), voxel->m_short_axis );
            }
            for ( auto triangle_ptr : existing_triangle )
            {
                correct_triangle_index( triangle_ptr, g_eigen_vec_vec[ frame_idx ].second.block( 4, 0, 3, 1 ), voxel->m_short_axis );
            }

            std::unique_lock< std::mutex > lock( mtx_triangle_lock );
            
            removed_triangle_list.emplace( std::make_pair( voxel, triangles_to_remove ) );
            added_triangle_list.emplace( std::make_pair( voxel, triangles_to_add ) );
            
            voxel_idx++;
        } );
    }
    catch ( ... )
    {
        for ( int i = 0; i < 100; i++ )
        {
            cout << ANSI_COLOR_RED_BOLD << "Exception in tbb parallels..." << ANSI_COLOR_RESET << endl;
        }
        return;
    }

    double              mul_thr_cost_time = tim.toc( " ", 0 );
    Common_tools::Timer tim_triangle_cost;
    int                 total_delete_triangle = 0, total_add_triangle = 0;
    // Voxel-wise mesh push
    for ( auto &triangles_set : removed_triangle_list )
    {
        total_delete_triangle += triangles_set.second.size();
        g_triangles_manager.remove_triangle_list( triangles_set.second );
    }

    for ( auto &triangle_list : added_triangle_list )
    {
        Triangle_set triangle_idx = triangle_list.second;
        total_add_triangle += triangle_idx.size();
        for ( auto triangle_ptr : triangle_idx )
        {
            Triangle_ptr tri_ptr = g_triangles_manager.insert_triangle( triangle_ptr->m_tri_pts_id[ 0 ], triangle_ptr->m_tri_pts_id[ 1 ],
                                                                        triangle_ptr->m_tri_pts_id[ 2 ], 1 );
            tri_ptr->m_index_flip = triangle_ptr->m_index_flip;
        }
    }
    
    g_mutex_reconstruct_mesh.unlock();
   
    if ( g_fp_cost_time )
    {
        if ( frame_idx > 0 )
            g_LiDAR_frame_avg_time = g_LiDAR_frame_avg_time * ( frame_idx - 1 ) / frame_idx + ( g_vx_map_frame_cost_time ) / frame_idx;
        fprintf( g_fp_cost_time, "%d %lf %d %lf %lf\r\n", frame_idx, tim.toc( " ", 0 ), ( int ) voxel_idx.load(), g_vx_map_frame_cost_time,
                 g_LiDAR_frame_avg_time );
        fflush( g_fp_cost_time );
    }
    if ( g_current_frame < frame_idx )
    {
        g_current_frame = frame_idx;
    }
    else
    {
        if ( g_eigen_vec_vec[ g_current_frame + 1 ].second.size() > 7 )
        {
            g_current_frame++;
        }
    }
}




void service_reconstruct_mesh()
{
    if ( g_thread_pool_rec_mesh == nullptr )
    {
        g_thread_pool_rec_mesh = std::make_shared< Common_tools::ThreadPool >( g_maximum_thread_for_rec_mesh );
    }
    int drop_frame_num = 0;
    while ( 1 )
    {
        
            while ( g_rec_mesh_data_package_list.size() == 0 )
            {
                std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
            }

            g_mutex_data_package_lock.lock();
            while ( g_rec_mesh_data_package_list.size() > 1e5 )
            {
                cout << "Drop mesh frame [" << g_rec_mesh_data_package_list.front().m_frame_idx;
                printf( "], total_drop = %d, all_frame = %d\r\n", drop_frame_num++, g_rec_mesh_data_package_list.front().m_frame_idx );
                g_rec_mesh_data_package_list.pop_front();
            }
            if ( g_rec_mesh_data_package_list.size() > 10 )
            {
                cout << "Poor real-time performance, current buffer size = " << g_rec_mesh_data_package_list.size() << endl;
            }
            Rec_mesh_data_package data_pack_front = g_rec_mesh_data_package_list.front();
            g_rec_mesh_data_package_list.pop_front();
            g_mutex_data_package_lock.unlock();
            // ANCHOR - Comment follow line to disable meshing
            if ( g_enable_mesh_rec )
            {
                g_thread_pool_rec_mesh->commit_task( incremental_mesh_reconstruction, data_pack_front.m_frame_pts, data_pack_front.m_pose_q,
                                                     data_pack_front.m_pose_t, data_pack_front.m_frame_idx );
            }

        std::this_thread::sleep_for( std::chrono::microseconds( 10 ) );
    }
}
extern bool  g_flag_pause;
int          g_frame_idx = 0;
std::thread *g_rec_mesh_thr = nullptr;

void start_mesh_threads( int maximum_threads = 20 )
{
    if ( g_eigen_vec_vec.size() <= 0 )
    {
        g_eigen_vec_vec.resize( 1e6 );
    }
    if ( g_rec_mesh_thr == nullptr )
    {
        g_maximum_thread_for_rec_mesh = maximum_threads;
        g_rec_mesh_thr = new std::thread( service_reconstruct_mesh );
    }
}

void reconstruct_mesh_from_pointcloud( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts, double minimum_pts_distance )
{
    start_mesh_threads();
    cout << "=== reconstruct_mesh_from_pointcloud ===" << endl;
    cout << "Input pointcloud have " << frame_pts->points.size() << " points." << endl;
    pcl::PointCloud< pcl::PointXYZI >::Ptr all_cloud_ds( new pcl::PointCloud< pcl::PointXYZI > );

    pcl::VoxelGrid< pcl::PointXYZI > sor;
    sor.setInputCloud( frame_pts );
    sor.setLeafSize( minimum_pts_distance, minimum_pts_distance, minimum_pts_distance );
    sor.filter( *all_cloud_ds );

    cout << ANSI_COLOR_BLUE_BOLD << "Raw points number = " << frame_pts->points.size()
         << ", downsampled points number = " << all_cloud_ds->points.size() << ANSI_COLOR_RESET << endl;
    g_mutex_data_package_lock.lock();
    g_rec_mesh_data_package_list.emplace_back( all_cloud_ds, Eigen::Quaterniond::Identity(), vec_3::Zero(), 0 );
    g_mutex_data_package_lock.unlock();
}

void open_log_file()
{
    if ( g_fp_cost_time == nullptr || g_fp_lio_state == nullptr )
    {
        Common_tools::create_dir( std::string( Common_tools::get_home_folder() ).append( "/ImMesh_PGO" ).c_str() );
        std::string cost_time_log_name = std::string( Common_tools::get_home_folder() ).append( "/ImMesh_PGO/mesh_cost_time.log" );
        std::string lio_state_log_name = std::string( Common_tools::get_home_folder() ).append( "/ImMesh_PGO/lio_state.log" );
        // cout << ANSI_COLOR_BLUE_BOLD ;
        // cout << "Record cost time to log file:" << cost_time_log_name << endl;
        // cout << "Record LIO state to log file:" << cost_time_log_name << endl;
        // cout << ANSI_COLOR_RESET;
        g_fp_cost_time = fopen( cost_time_log_name.c_str(), "w+" );
        g_fp_lio_state = fopen( lio_state_log_name.c_str(), "w+" );
    }
}

std::vector< vec_4 > convert_pcl_pointcloud_to_vec( pcl::PointCloud< pcl::PointXYZI > &pointcloud )
{
    int                  pt_size = pointcloud.points.size();
    std::vector< vec_4 > eigen_pt_vec( pt_size );
    for ( int i = 0; i < pt_size; i++ )
    {
        eigen_pt_vec[ i ]( 0 ) = pointcloud.points[ i ].x;
        eigen_pt_vec[ i ]( 1 ) = pointcloud.points[ i ].y;
        eigen_pt_vec[ i ]( 2 ) = pointcloud.points[ i ].z;
        eigen_pt_vec[ i ]( 3 ) = pointcloud.points[ i ].intensity;
    }
    return eigen_pt_vec;
}

void Voxel_mapping::map_incremental_grow()
{
    start_mesh_threads( m_meshing_maximum_thread_for_rec_mesh );
    if ( m_use_new_map )
    {
        while ( g_flag_pause )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
        }
        // startTime = clock();
        pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar( new pcl::PointCloud< pcl::PointXYZI > );
        pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar_full( new pcl::PointCloud< pcl::PointXYZI > );

        std::vector< Point_with_var > pv_list;
        // TODO: saving pointcloud to file
        // pcl::io::savePCDFileBinary(Common_tools::get_home_folder().append("/r3live_output/").append("last_frame.pcd").c_str(), *m_feats_down_body);
        transformLidar( state.rot_end, state.pos_end, m_feats_down_body, world_lidar );
        for ( size_t i = 0; i < world_lidar->size(); i++ )
        {
            Point_with_var pv;
            pv.m_point << world_lidar->points[ i ].x, world_lidar->points[ i ].y, world_lidar->points[ i ].z;
            M3D point_crossmat = m_cross_mat_list[ i ];
            M3D var = m_body_cov_list[ i ];
            var = ( state.rot_end * m_extR ) * var * ( state.rot_end * m_extR ).transpose() +
                  ( -point_crossmat ) * state.cov.block< 3, 3 >( 0, 0 ) * ( -point_crossmat ).transpose() + state.cov.block< 3, 3 >( 3, 3 );
            pv.m_var = var;
            pv_list.push_back( pv );
        }

        // pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar( new pcl::PointCloud< pcl::PointXYZI > );
        std::sort( pv_list.begin(), pv_list.end(), var_contrast );
        updateVoxelMap( pv_list, m_max_voxel_size, m_max_layer, m_layer_init_size, m_max_points_size, m_min_eigen_value, m_feat_map );
        double vx_map_cost_time = omp_get_wtime();
        g_vx_map_frame_cost_time = ( vx_map_cost_time - g_LiDAR_frame_start_time ) * 1000.0;
        // cout << "vx_map_cost_time = " <<  g_vx_map_frame_cost_time << " ms" << endl;
#ifdef USE_LOOP_PGO
        if (has_loop_flag && (loop_container.back().second - consecutive_loop_begin > std_manager->config.skip_near_loop)) {
            int num_of_update = 0;
            // 从index=1开始，由于index=0的prior帧并没有进入这个函数。并且这里不包含最后一帧，因为无论gtsam的更新量有没有超过阈值，最后一帧都需要加入meshing
            for (int i = 1; i <= pc_pose_pgo.size() - 2; i++) {
                if (sqrt((pc_pose_pgo[i].second.matrix() - pose_update[i].matrix()).block<3, 1>(0, 3).squaredNorm() / 3) > pgo_pose_update_thres) {
                    Eigen::Affine3d temp_transform = pc_pose_pgo[i].second;
                    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                    transformLidar(temp_transform.linear(), temp_transform.translation(), pc_pose_pgo[i].first.makeShared(), temp_cloud);
                    // ANCHOR: 注意！这里pose_update只跟踪用于meshing的位姿，并不一定与pc_pose_pgo完全一致，一些变化不超出阈值的位姿，仍保留pose_odom的值
                    pose_update[i] = pc_pose_pgo[i].second;

                    g_mutex_data_package_lock.lock();
                    g_rec_mesh_data_package_list.emplace_back(temp_cloud, Eigen::Quaterniond(temp_transform.linear()), temp_transform.translation(), i - 1);
                    g_mutex_data_package_lock.unlock();
                    num_of_update++;
                }
            }
            cout << ANSI_COLOR_GREEN_BOLD << "[Mesh Update] Updated State: " << num_of_update << 
                ", Match ID: " << loop_container.back().first << ", Curr ID: " << loop_container.back().second << ANSI_COLOR_RESET << endl;
            consecutive_loop_begin = loop_container.back().second;
        }
        // 因为进入这个函数没有算prior那一帧，但是pose_vec里存有init priorFactor那一帧
        int pose_vec_index = g_frame_idx + 1;
        Eigen::Affine3d temp_transform = pc_pose_pgo[pose_vec_index].second;
        transformLidar(temp_transform.linear(), temp_transform.translation(), m_feats_undistort, world_lidar_full);

        g_mutex_data_package_lock.lock();
        g_rec_mesh_data_package_list.emplace_back(world_lidar_full, Eigen::Quaterniond(temp_transform.linear()), temp_transform.translation(), g_frame_idx);
        g_mutex_data_package_lock.unlock();
#else
        transformLidar( state.rot_end, state.pos_end, m_feats_undistort, world_lidar_full );
         
        g_mutex_data_package_lock.lock();
        // ANCHOR: meshing用data package
        g_rec_mesh_data_package_list.emplace_back( world_lidar_full, Eigen::Quaterniond( state.rot_end ), state.pos_end, g_frame_idx );
        g_mutex_data_package_lock.unlock();
#endif
        open_log_file();
        if ( g_fp_lio_state != nullptr )
        {
            dump_lio_state_to_log( g_fp_lio_state );
        }
        g_frame_idx++;
    }

    if ( !m_use_new_map )
    {
        for ( int i = 0; i < m_feats_down_size; i++ )
        {
            /* transform to world frame */
            pointBodyToWorld( m_feats_down_body->points[ i ], m_feats_down_world->points[ i ] );
        }
        
        // add_to_offline_bin( state, m_Lidar_Measures.lidar_beg_time, m_feats_down_world );
        
#ifdef USE_ikdtree
#ifdef USE_ikdforest
        ikdforest.Add_Points( feats_down_world->points, lidar_end_time );
#else
        m_ikdtree.Add_Points( m_feats_down_world->points, true );
#endif
#endif
    }
}

#ifdef USE_LOOP_PGO
void Voxel_mapping::initSAM() {
    prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
    odometry_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    gtsam::Vector robustNoiseVector(6);
    robustNoiseVector << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    robust_loop_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), gtsam::noiseModel::Diagonal::Variances(robustNoiseVector));
    
    gtsam::ISAM2Params parameters;
    parameters.relinearizeSkip = 1;
    parameters.relinearizeThreshold = 0.01;
    isam = std::make_shared<gtsam::ISAM2>(parameters);
}

void Voxel_mapping::get_cloud_for_std_matcher(pcl::PointCloud<pcl::PointXYZI>::Ptr &in) {
    in->clear();

    transformLidar(state.rot_end, state.pos_end, m_feats_undistort, in);

    std_desc::voxelFilter(in, std_manager->config.ds_size);
}

bool Voxel_mapping::get_std_feature_and_matching(const int frame_id) {
    bool has_loop = false;

    std_desc::STDFeature feature = std_manager->extract(key_frame_cloud);
    // cout << "ID: " << frame_id << " Feature Size: " << feature.descs.size() << " Cloud Size: " << key_frame_cloud->size() << endl;
    std_desc::LoopResult result;

    int64_t duration;
    if (frame_id > std_config.skip_near_num) {
        auto t1 = std::chrono::high_resolution_clock::now();
        result = std_manager->searchLoop(feature);
        auto t2 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    }
    
    std_manager->insert(feature);
    if (result.valid) {
        ROS_WARN("FIND MATCHED LOOP! Current ID: %lu, Loop ID: %lu, Match Score: %.4f, Time Cost: %lu ms", feature.id, result.match_id, result.match_score, duration);
        double score = std_manager->verifyGeoPlaneICP(feature.cloud, std_manager->cloud_vec[result.match_id], result.rotation, result.translation);
        has_loop = true;
        loop_container.emplace_back(result.match_id, feature.id);
        for (size_t j = 1; j <= std_manager->config.sub_frame_num; j++) {
            // 当前关键帧，是由过去10帧加起来的
            int src_frame = frame_id + j - std_manager->config.sub_frame_num;
            // 历史配对帧，由于feature id即match_id，是由STD内部计数，提取feature每10帧一个关键帧提取一次，因此match_id的计数总是由这个10倍的关系
            int tar_frame = result.match_id * std_manager->config.sub_frame_num + j;

            Eigen::Affine3d delta_pose = Eigen::Affine3d::Identity();
            delta_pose.translation() = result.translation;
            delta_pose.linear() = result.rotation;

            Eigen::Affine3d refined_src = delta_pose * pose_odom[src_frame];
            Eigen::Affine3d tar_pose = pose_odom[tar_frame];

            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(tar_frame, src_frame,
                                                        gtsam::Pose3(tar_pose.matrix()).between(gtsam::Pose3(refined_src.matrix())),
                                                        robust_loop_noise));
        }
    }
    key_frame_cloud->clear();

    return has_loop;
}

void Voxel_mapping::optimize_once_and_update() {
    isam->update(graph, initial);
    isam->update();

    graph.resize(0);
    initial.clear();

    gtsam::Values current_estimates = isam->calculateEstimate();
    assert(current_estimates.size() == pc_pose_pgo.size());

    for (int i = 0; i < current_estimates.size(); i++) {
        gtsam::Pose3 est = current_estimates.at<gtsam::Pose3>(i);
        pc_pose_pgo[i].second = Eigen::Affine3d(est.matrix());
    }
}

void Voxel_mapping::optimize_loop_and_update() {
    isam->update(graph, initial);
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();

    graph.resize(0);
    initial.clear();

    gtsam::Values current_estimates = isam->calculateEstimate();
    assert(current_estimates.size() == pc_pose_pgo.size());

    for (int i = 0; i < current_estimates.size(); i++) {
        gtsam::Pose3 est = current_estimates.at<gtsam::Pose3>(i);
        pc_pose_pgo[i].second = Eigen::Affine3d(est.matrix());
    }
}
#endif