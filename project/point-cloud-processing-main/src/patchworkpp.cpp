#include "percep_main/patchworkpp.hpp"

using namespace std;
using namespace patchwork;

bool point_z_cmp(PointXYZ a, PointXYZ b) { return a.z < b.z; }

Eigen::MatrixX3f PatchWorkpp::toEigenCloud(const vector<PointXYZ> &cloud) {
  Eigen::MatrixX3f dst(cloud.size(), 3);
  int j = 0;
  for (auto &p : cloud) {
    dst.row(j++) << p.x, p.y, p.z;
  }
  return dst;
}

Eigen::VectorXi PatchWorkpp::toIndices(const vector<PointXYZ> &cloud) {
  Eigen::VectorXi dst(cloud.size());
  int j = 0;
  for (auto &p : cloud) {
    dst.row(j++) << p.idx;
  }
  return dst;
}

void PatchWorkpp::addCloud(vector<PointXYZ> &cloud, vector<PointXYZ> &add) {
  cloud.insert(cloud.end(), add.begin(), add.end());
}

void PatchWorkpp::updateDist(double new_dist)
{
  params_.th_dist = new_dist;
}
void PatchWorkpp::flush_patches(vector<Zone> &czm) {
  for (int k = 0; k < params_.num_zones; k++) {
    for (int i = 0; i < params_.num_rings_each_zone[k]; i++) {
      for (int j = 0; j < params_.num_sectors_each_zone[k]; j++) {
        // czm[k][i][j].resize(MAX_POINTS, 3);
        czm[k][i][j].clear();
      }
    }
  }

  if (params_.verbose)
    cout << "\033[1;31m"
         << "PatchWorkpp::flush_patches() - Flushed patches successfully!"
         << "\033[0m" << endl;
}

void PatchWorkpp::estimate_plane(const vector<PointXYZ> &ground) {
  if (ground.empty()) return;

  Eigen::MatrixX3f eigen_ground(ground.size(), 3);
  int j = 0;
  for (auto &p : ground) {
    eigen_ground.row(j++) << p.x, p.y, p.z;
  }
  Eigen::MatrixX3f centered = eigen_ground.rowwise() - eigen_ground.colwise().mean();
  Eigen::MatrixX3f cov =
      (centered.adjoint() * centered) / static_cast<double>(eigen_ground.rows() - 1);

  pc_mean_.resize(3);
  pc_mean_ << eigen_ground.colwise().mean()(0), eigen_ground.colwise().mean()(1),
      eigen_ground.colwise().mean()(2);

  Eigen::JacobiSVD<Eigen::MatrixX3f> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  singular_values_ = svd.singularValues();

  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));

  if (normal_(2) < 0) {
    for (int i = 0; i < 3; i++) normal_(i) *= -1;
  }

  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean_.head<3>();
  
  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);  
  

}

void PatchWorkpp::extract_initial_seeds(const int zone_idx,
                                        const vector<PointXYZ> &p_sorted,
                                        vector<PointXYZ> &init_seeds,
                                        double th_seed) {
  init_seeds.clear();

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt    = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < p_sorted.size(); i++) {
      if (p_sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (int i = init_idx; i < p_sorted.size() && cnt < params_.num_lpr; i++) {
    sum += p_sorted[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0
  // cout << "\033[1;32m"<< "lpr_height_v:"<<lpr_height<< "\033[0m" << endl;
  double h_thr = lpr_height + params_.th_seeds;
  if (h_thr<-0.01) { h_thr = 0.1;}
  for (int i = 0; i < p_sorted.size(); i++) {
    if (p_sorted[i].z < h_thr && p_sorted[i].z>-0.1) {
      init_seeds.push_back(p_sorted[i]);
    }
  }
}

void PatchWorkpp::extract_initial_seeds(const int zone_idx,
                                        const vector<PointXYZ> &p_sorted,
                                        vector<PointXYZ> &init_seeds) {
  init_seeds.clear();

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt    = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < p_sorted.size(); i++) {
      if (p_sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (int i = init_idx; i < p_sorted.size() && cnt < params_.num_lpr; i++) {
    sum += p_sorted[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0

  // cout << "\033[1;32m"<< "lpr_height:"<<lpr_height<< "\033[0m" << endl;
  double h_thr = lpr_height + params_.th_seeds;
  if (h_thr<-0.01) { h_thr = 0.1;}
  for (int i = 0; i < p_sorted.size(); i++) {
    if (p_sorted[i].z < h_thr && p_sorted[i].z>-0.1){
      init_seeds.push_back(p_sorted[i]);
    }
  }
}

void PatchWorkpp::estimateGround(Eigen::MatrixXf &cloud_in) {
  cloud_ground_.clear();
  cloud_nonground_.clear();
  
  if (params_.verbose)
    cout << "\033[1;32m"
         << "PatchWorkpp::estimateGround() - Estimation starts !"
         << "\033[0m" << endl;



  auto start1 = std::chrono::high_resolution_clock::now();
  // 1. Reflected Noise Removal (RNR)
  if (params_.enable_RNR) reflected_noise_removal(cloud_in);

  auto end1 = std::chrono::high_resolution_clock::now();
  auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
  // cout<<"RNR  Time taken: "<<duration1.count()<<"ms"<<endl;

  // 2. Concentric Zone Model (CZM)
  flush_patches(ConcentricZoneModel_);

  auto end2 = std::chrono::high_resolution_clock::now();
  auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - end1);
  // cout<<"FLUSH  Time taken: "<<duration2.count()<<"ms"<<endl;

  pc2czm(cloud_in, ConcentricZoneModel_);

  auto end3 = std::chrono::high_resolution_clock::now();
  auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(end3 - end2);
  // cout<<"CZM  Time taken: "<<duration3.count()<<"ms"<<endl;

  int concentric_idx = 0;

  centers_.clear();
  centers_vec.clear();
  normals_.clear();
  normals_vec.clear();
  d_vec.clear();
  
  std::vector<patchwork::RevertCandidate> candidates;
  std::vector<double> ringwise_flatness;

  std::chrono::milliseconds sort_duration(0),pca_duration(0),gle_duration(0),tgr_duration(0),update_duration(0);
  for (int zone_idx = 0; zone_idx < params_.num_zones; ++zone_idx) {
    auto zone = ConcentricZoneModel_[zone_idx];
    for (int ring_idx = 0; ring_idx < params_.num_rings_each_zone[zone_idx]; ++ring_idx) {
      for (int sector_idx = 0; sector_idx < params_.num_sectors_each_zone[zone_idx]; ++sector_idx) {
        // if (params_.verbose){cout<<"\033[1;31m"<<"zone.size:"<<zone[ring_idx][sector_idx].size()<<"\033[0m"<<endl;}
        if (zone[ring_idx][sector_idx].size() < params_.num_min_pts) {
          addCloud(cloud_nonground_, zone[ring_idx][sector_idx]);
          cout<<"\033[1;31m"<<"zone, nonground"<<"\033[0m"<<endl;
          continue;
        }
        
        // --------- region-wise sorting (faster than global sorting method) ---------------- //

        auto start11 = std::chrono::high_resolution_clock::now();
        sort(zone[ring_idx][sector_idx].begin(), zone[ring_idx][sector_idx].end(), point_z_cmp);
        auto end11 = std::chrono::high_resolution_clock::now();
        auto duration11 = std::chrono::duration_cast<std::chrono::milliseconds>(end11 - start11);
        sort_duration += duration11;

        // ---------------------------------------------------------------------------------- //
        if (zone[ring_idx][sector_idx].empty()) { 
          // cout<<"ring_idx:"<<ring_idx<<" sector_idx:"<<sector_idx<<endl;
          continue;}

        auto start22 = std::chrono::high_resolution_clock::now();
        extract_piecewiseground(
            zone_idx, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);

        auto end22 = std::chrono::high_resolution_clock::now();
        auto duration22 = std::chrono::duration_cast<std::chrono::milliseconds>(end22 - start22);
        pca_duration +=  duration22;
        centers_.push_back(PointXYZ(pc_mean_(0), pc_mean_(1), pc_mean_(2)));
        normals_.push_back(PointXYZ(normal_(0), normal_(1), normal_(2)));
        centers_vec.push_back(pc_mean_);
        normals_vec.push_back(normal_);
        d_vec.push_back(d_);


        auto start33 = std::chrono::high_resolution_clock::now();
        // Status of each patch
        // used in checking uprightness, elevation, and flatness, respectively
        const double ground_uprightness = normal_(2);
        const double ground_elevation   = pc_mean_(2);
        const double ground_flatness    = singular_values_.minCoeff();
        const double line_variable      = singular_values_(1) != 0
                                              ? singular_values_(0) / singular_values_(1)
                                              : std::numeric_limits<double>::max();

        double heading = 0.0;
        for (int i = 0; i < 3; i++) heading += pc_mean_(i) * normal_(i);

        /*
            About 'is_heading_outside' condition, heading should be smaller than 0 theoretically.
            ( Imagine the geometric relationship between the surface normal vector on the ground
           plane and the vector connecting the sensor origin and the mean point of the ground plane
           )

            However, when the patch is far awaw from the sensor origin,
            heading could be larger than 0 even if it's ground due to lack of amount of ground plane
           points.

            Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near
           condition )
        */
        bool is_upright         = ground_uprightness > params_.uprightness_thr;
        bool is_near_zone       = concentric_idx < params_.num_rings_of_interest;
        bool is_heading_outside = heading < 0.5;

        bool is_not_elevated = false;
        bool is_flat         = false;
        
        if (concentric_idx < params_.num_rings_of_interest) {
          is_not_elevated = ground_elevation < params_.elevation_thr[concentric_idx];
          is_flat         = ground_flatness < params_.flatness_thr[concentric_idx];
        }

        if (params_.verbose) {
          
          cout<<"concentric_idx: "<<concentric_idx<<
          " ground_uprightness: "<<ground_uprightness<<" ? uprightness_thr:"<<params_.uprightness_thr<<
          " ground_elevation: "<<ground_elevation<<" ? elevation_thr:"<<params_.elevation_thr[concentric_idx]<<
          " ground_flatness:"<<ground_flatness<<" ? flatness_thr:"<<params_.flatness_thr[concentric_idx]<<
          " heading: "<<heading<<endl;
          cout<<"is_upright:"<<is_upright<<" is_near_zone:"<<is_near_zone<<
              " is_heading_outside:"<<is_heading_outside<<" is_not_elevated:"<<is_not_elevated<<" is_flat:"<<is_flat<<endl;
        }          
       
        /*
            Store the elevation & flatness variables
            for A-GLE (Adaptive Ground Likelihood Estimation)
            and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
        */
        if (is_upright && is_not_elevated && is_near_zone) {
          update_elevation_[concentric_idx].push_back(ground_elevation);
          update_flatness_[concentric_idx].push_back(ground_flatness);

          ringwise_flatness.push_back(ground_flatness);
        }

        // Ground estimation based on conditions
        if (!is_upright) {
          if (params_.verbose){cout<<"\033[1;31m"<<"not upright, nonground"<<"\033[0m"<<endl;}
          addCloud(cloud_nonground_, regionwise_ground_);
        } else if (!is_near_zone) {
          if (params_.verbose){cout<<"\033[1;31m"<<"not near_zone, ground"<<"\033[0m"<<endl;}
          addCloud(cloud_ground_, regionwise_ground_);
        } else if (!is_heading_outside) {
          if (params_.verbose){cout<<"\033[1;31m"<<"not heading_outside, nonground"<<"\033[0m"<<endl;}
          addCloud(cloud_nonground_, regionwise_ground_);
        } else if (is_not_elevated || is_flat) {
          if (params_.verbose){cout<<"\033[1;31m"<<"not elevated or not flat, ground"<<"\033[0m"<<endl;}
          addCloud(cloud_ground_, regionwise_ground_);
        } else {
          if (params_.verbose){cout<<"\033[1;31m"<<"candidate"<<"\033[0m"<<endl;}
          patchwork::RevertCandidate candidate(concentric_idx,
                                               sector_idx,
                                               ground_flatness,
                                               line_variable,
                                               pc_mean_,
                                               regionwise_ground_);
          candidates.push_back(candidate);
        }
        // addCloud(cloud_ground_, regionwise_ground_);
        // Every regionwise_nonground is considered nonground.
        addCloud(cloud_nonground_, regionwise_nonground_);

        auto end33 = std::chrono::high_resolution_clock::now();
        auto duration33 = std::chrono::duration_cast<std::chrono::milliseconds>(end33 - start33);
        gle_duration += duration33;

      }

      auto start44 = std::chrono::high_resolution_clock::now();
      if (!candidates.empty()) {
        if (params_.enable_TGR) {
          temporal_ground_revert(ringwise_flatness, candidates, concentric_idx);
        } else {
          for (auto candidate : candidates) {
            addCloud(cloud_nonground_, candidate.regionwise_ground);
          }
        }

        candidates.clear();
        ringwise_flatness.clear();
      }

      auto end44 = std::chrono::high_resolution_clock::now();
      auto duration44 = std::chrono::duration_cast<std::chrono::milliseconds>(end44 - start44);
      tgr_duration += duration44;
      concentric_idx++;
    }
  }

  auto start55 = std::chrono::high_resolution_clock::now();
  update_elevation_thr();
  update_flatness_thr();
  auto end55 = std::chrono::high_resolution_clock::now();
  auto duration55 = std::chrono::duration_cast<std::chrono::milliseconds>(end55 - start55);
  update_duration += duration55;

  // cout<<"SORT  Time taken: "<<sort_duration.count()<<"ms"<<endl;
  // cout<<"PCA  Time taken: "<<pca_duration.count()<<"ms"<<endl;
  // cout<<"GLE  Time taken: "<<gle_duration.count()<<"ms"<<endl;
  // cout<<"TGR  Time taken: "<<tgr_duration.count()<<"ms"<<endl;
  // cout<<"UPDATE  Time taken: "<<update_duration.count()<<"ms"<<endl;

  if (params_.verbose)
    cout << "\033[1;32m"
         << "PatchWorkpp::estimateGround() - Estimation is finished !"
         << "\033[0m" << endl;
}

void PatchWorkpp::update_elevation_thr(void) {
  for (int i = 0; i < params_.num_rings_of_interest; i++) {
    if (update_elevation_[i].empty()) continue;

    double update_mean = 0.0, update_stdev = 0.0;
    calc_mean_stdev(update_elevation_[i], update_mean, update_stdev);
    if (i == 0) {
      params_.elevation_thr[i] = update_mean + 3 * update_stdev;
      params_.sensor_height    = -update_mean;
    } else {
      params_.elevation_thr[i] = update_mean + 2 * update_stdev;
    }

    // if (params_.verbose) cout << "elevation threshold [" << i << "]: " <<
    // params_.elevation_thr[i] << endl;

    int exceed_num = update_elevation_[i].size() - params_.max_elevation_storage;
    if (exceed_num > 0)
      update_elevation_[i].erase(update_elevation_[i].begin(),
                                 update_elevation_[i].begin() + exceed_num);
  }
}

void PatchWorkpp::update_flatness_thr(void) {
  for (int i = 0; i < params_.num_rings_of_interest; i++) {
    if (update_flatness_[i].empty()) break;
    if (update_flatness_[i].size() <= 1) break;

    double update_mean = 0.0, update_stdev = 0.0;
    calc_mean_stdev(update_flatness_[i], update_mean, update_stdev);
    params_.flatness_thr[i] = update_mean + update_stdev;

    // if (params_.verbose) { cout << "flatness threshold [" << i << "]: " <<
    // params_.flatness_thr[i] << endl; }

    int exceed_num = update_flatness_[i].size() - params_.max_flatness_storage;
    if (exceed_num > 0)
      update_flatness_[i].erase(update_flatness_[i].begin(),
                                update_flatness_[i].begin() + exceed_num);
  }
}

void PatchWorkpp::reflected_noise_removal(Eigen::MatrixXf &cloud_in) {
  if (cloud_in.cols() < 4) {
    cout << "RNR requires intensity information !" << endl;
    return;
  }

  int cnt = 0;
  for (int i = 0; i < cloud_in.rows(); i++) {
    double r =
        sqrt(cloud_in.row(i)(0) * cloud_in.row(i)(0) + cloud_in.row(i)(1) * cloud_in.row(i)(1));
    double z                = cloud_in.row(i)(2);
    double ver_angle_in_deg = atan2(z, r) * 180 / M_PI;

    if (ver_angle_in_deg < params_.RNR_ver_angle_thr && z < -params_.sensor_height - 0.8 &&
        cloud_in.row(i)(3) < params_.RNR_intensity_thr) {
      cloud_nonground_.push_back(
          PointXYZ(cloud_in.row(i)(0), cloud_in.row(i)(1), cloud_in.row(i)(2), i));
      cloud_in.row(i)(2) = std::numeric_limits<float>::min();
      cnt++;
    }
  }

  if (params_.verbose)
    cout << "PatchWorkpp::reflected_noise_removal() - Number of Noises : " << cnt << endl;
}

void PatchWorkpp::temporal_ground_revert(std::vector<double> ring_flatness,
                                         std::vector<patchwork::RevertCandidate> candidates,
                                         int concentric_idx) {
  if (params_.verbose)
    std::cout << "\033[1;34m"
              << "=========== Temporal Ground Revert (TGR) ==========="
              << "\033[0m" << endl;

  double mean_flatness = 0.0, stdev_flatness = 0.0;
  calc_mean_stdev(ring_flatness, mean_flatness, stdev_flatness);

  if (params_.verbose) {
    cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
         << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness
         << std::endl;
  }

  for (auto candidate : candidates) {
    // Debug
    if (params_.verbose) {
      cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
           << " / flatness: " << candidate.ground_flatness
           << " / line_variable: " << candidate.line_variable
           << " / ground_num : " << candidate.regionwise_ground.size() << "\033[0m" << endl;
    }

    double mu_flatness = mean_flatness + 1.5 * stdev_flatness;
    double prob_flatness =
        1 / (1 + exp((candidate.ground_flatness - mu_flatness) / (mu_flatness / 10)));

    if (
        candidate.regionwise_ground.size() > 50 &&
        candidate.ground_flatness < params_.th_dist * params_.th_dist)
      prob_flatness = 1.0;

    double prob_line = 1.0;
    if (candidate.line_variable > 8.0) {  //&& candidate.line_dir > M_PI/4)//
      // if (params_.verbose) cout << "line_dir: " << candidate.line_dir << endl;
      prob_line = 0.0;
    }

    bool revert = prob_line * prob_flatness > 0.5;

    if (concentric_idx < params_.num_rings_of_interest) {
      if (revert) {
        if (params_.verbose) {
          cout << "\033[1;32m"
               << "REVERT TRUE"
               << "\033[0m" << endl;
        }

        addCloud(cloud_ground_, candidate.regionwise_ground);
      } else {
        if (params_.verbose) {
          cout << "\033[1;31m"
               << "FINAL REJECT"
               << "\033[0m" << endl;
        }
        addCloud(cloud_nonground_, candidate.regionwise_ground);
      }
    }
  }

  if (params_.verbose)
    std::cout << "\033[1;34m"
              << "===================================================="
              << "\033[0m" << endl;
}

// For adaptive
void PatchWorkpp::extract_piecewiseground(const int zone_idx,
                                          const vector<PointXYZ> &src,
                                          vector<PointXYZ> &dst,
                                          vector<PointXYZ> &non_ground_dst) {
                                       

  // 0. Initialization
  if (!ground_pc_.empty()) ground_pc_.clear();
  if (!dst.empty()) dst.clear();
  if (!non_ground_dst.empty()) non_ground_dst.clear();

  // 1. Region-wise Vertical Plane Fitting (R-VPF)
  // : removes potential vertical plane under the ground plane
  vector<PointXYZ> src_wo_verticals;
  src_wo_verticals = src;
  cloud_visualize.clear();
  if (params_.enable_RVPF) {
    for (int i = 0; i < params_.num_iter; i++) {
      extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, params_.th_seeds_v);
      
      estimate_plane(ground_pc_);
      //地面点云垂直
      if (zone_idx == 0 && normal_(2) < params_.uprightness_thr) {
        vector<PointXYZ> src_tmp;
        src_tmp = src_wo_verticals;
        src_wo_verticals.clear();

        for (auto point : src_tmp) {
          double distance = calc_point_to_plane_d(point, normal_, d_);
          
          if (abs(distance) < params_.th_dist_v) {
            non_ground_dst.push_back(point);
          } else {
            src_wo_verticals.push_back(point);
          }
        }
      } else
        break;
    }
  }
  // cout<<"pc_mean_1:"<<pc_mean_<<" normal_1:"<<normal_<<endl;
  // 2. Region-wise Ground Plane Fitting (R-GPF)
  // : fits the ground plane
  cloud_visualize = ground_pc_;
  extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
  estimate_plane(ground_pc_);   //计算normal_,d_
  // cout<<"pc_mean_:"<<pc_mean_<<endl;
  // cout<<" normal_:"<<normal_<<endl;
  // cout<<" d_:"<<d_<<endl;
  for (int i = 0; i < params_.num_iter; i++) {
    ground_pc_.clear();

    for (auto point : src_wo_verticals) {
      double distance = calc_point_to_plane_d(point, normal_, d_);
      if (i < params_.num_iter - 1) {
        if (distance < params_.th_dist) {
          ground_pc_.push_back(point);
        }
      } else {
        if (distance < params_.th_dist) {
          dst.push_back(point);
        } else {
          // cout<<"distance:"<<distance<<" < th_dist:"<<params_.th_dist<<endl;
          non_ground_dst.push_back(point);
        }
      }
    }
    
    if (i < params_.num_iter - 1) {
      estimate_plane(ground_pc_);
    } else {
      estimate_plane(dst);
    }
  }
  
  // cout<<"pc_mean_:"<<pc_mean_<<" normal_:"<<normal_<<endl;
  if (dst.size() + non_ground_dst.size() != src.size()) {
    cout << "\033[1;33m"
         << "Points are Missing/Adding !!! Please Check !! "
         << "\033[0m" << endl;
    cout << "gnd size: " << dst.size() << ", non gnd size: " << non_ground_dst.size()
         << ", src: " << src.size() << endl;
  }
}

double PatchWorkpp::calc_point_to_plane_d(PointXYZ p, Eigen::VectorXf normal, double d) {
    if (normal.size() < 3) {
        
        return 0.0;
    }
  return normal(0) * p.x + normal(1) * p.y + normal(2) * p.z + d;
}

void PatchWorkpp::calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev) {
  if (vec.size() <= 1) return;

  mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

  for (int i = 0; i < vec.size(); i++) {
    stdev += (vec.at(i) - mean) * (vec.at(i) - mean);
  }
  stdev /= vec.size() - 1;
  stdev = sqrt(stdev);
}

double PatchWorkpp::xy2theta(const double &x, const double &y) {  // 0 ~ 2 * PI
  double angle = atan2(y, x);
  return angle > 0 ? angle : 2 * M_PI + angle;
}

double PatchWorkpp::xy2radius(const double &x, const double &y) {
  // return sqrt(pow(x, 2) + pow(y, 2));
  return sqrt(x * x + y * y);
}

void PatchWorkpp::pc2czm(const Eigen::MatrixXf &src, std::vector<Zone> &czm) {
  double max_range = params_.max_range, min_range = params_.min_range;
  double min_range_0 = min_ranges_[0], min_range_1 = min_ranges_[1], min_range_2 = min_ranges_[2],
         min_range_3 = min_ranges_[3];
  int num_ring_0 = params_.num_rings_each_zone[0], num_sector_0 = params_.num_sectors_each_zone[0];
  int num_ring_1 = params_.num_rings_each_zone[1], num_sector_1 = params_.num_sectors_each_zone[1];
  int num_ring_2 = params_.num_rings_each_zone[2], num_sector_2 = params_.num_sectors_each_zone[2];
  int num_ring_3 = params_.num_rings_each_zone[3], num_sector_3 = params_.num_sectors_each_zone[3];

  for (int i = 0; i < src.rows(); i++) {
    float x = src.row(i)(0), y = src.row(i)(1), z = src.row(i)(2);

    if (z == std::numeric_limits<float>::min()) continue;
    czm[0][0][0].emplace_back(PointXYZ(x, y, z, i));
    //是否加入patch策略
 //////////////////////////////
    // double r = xy2radius(x, y);
    // int ring_idx, sector_idx;
    // if ((r <= max_range) && (r > min_range)) {
    //   // double theta = xy2theta(pt.x, pt.y);
    //   double theta = xy2theta(x, y);
      
    //   if (r < min_range_1) {  // In First rings
    //     ring_idx   = min(static_cast<int>(((r - min_range_0) / ring_sizes_[0])), num_ring_0 - 1);
    //     sector_idx = min(static_cast<int>((theta / sector_sizes_[0])), num_sector_0 - 1);
    //     czm[0][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
    //   } 
    //   else if (r < min_range_2) {
    //     ring_idx   = min(static_cast<int>(((r - min_range_1) / ring_sizes_[1])), num_ring_1 - 1);
    //     sector_idx = min(static_cast<int>((theta / sector_sizes_[1])), num_sector_1 - 1);
    //     czm[1][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
    //   } 
    //   else if (r < min_range_3) {
    //     ring_idx   = min(static_cast<int>(((r - min_range_2) / ring_sizes_[2])), num_ring_2 - 1);
    //     sector_idx = min(static_cast<int>((theta / sector_sizes_[2])), num_sector_2 - 1);
    //     czm[2][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
    //   } 
    //   else {  // Far!
    //     ring_idx   = min(static_cast<int>(((r - min_range_3) / ring_sizes_[3])), num_ring_3 - 1);
    //     sector_idx = min(static_cast<int>((theta / sector_sizes_[3])), num_sector_3 - 1);
    //     czm[3][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
    //   }
    // } else {
    //   cloud_nonground_.push_back(PointXYZ(x, y, z, i));
    // }
   //////////////////////////////    
  }
  if (params_.verbose)
    cout << "\033[1;33m"
         << "PatchWorkpp::pc2czm() - Divides pointcloud into the concentric zone model successfully"
         << "\033[0m" << endl;
}
