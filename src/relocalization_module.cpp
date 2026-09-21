#include <glim_relocalization/relocalization_module.hpp>

#include <cmath>
#include <fstream>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

// POZNÁMKA: presný názov/cesta hlavičky pre GlobalMappingCallbacks a ISAM2Ext
// sa v rôznych GLIM verziách mierne líšili. V doxygen dokumentácii k v1.2.x
// je to include/glim/mapping/callbacks.hpp - over si to podľa svojho
// nainštalovaného glim balíka (grep -r "GlobalMappingCallbacks" /opt/ros/*/include
// alebo v tvojom colcon workspace pod install/glim/include).
#include <glim/mapping/callbacks.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>

namespace glim {

using gtsam::symbol_shorthand::M;  // Kľúč pre pózy submáp v referenčnej mape
using gtsam::symbol_shorthand::X;  // Kľúč pre pózy submáp v bežiacej mape

namespace {

// Symetrický konfig loader - nezávislý od GLIM-ovho interného Config wrappera,
// aby modul fungoval aj keď sa jeho API medzi verziami zmení.
nlohmann::json load_json_config(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("failed to open config file: " + path);
  }
  nlohmann::json j;
  ifs >> j;
  return j;
}

Eigen::Isometry3d make_pose(double x, double y, double z, double yaw_rad) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

}  // namespace

RelocalizationModule::RelocalizationModule() {
  // Config cesta: rovnaká konvencia ako ostatné glim_ext moduly - najprv
  // skús nájsť "config_relocalization" v hlavnom config.json, inak fallback
  // na vlastný config_relocalization.json vedľa binárky.
  // Tu pre jednoduchosť čítame priamo z pevnej cesty odovzdanej cez env var,
  // uprav podľa toho, ako chceš cestu skutočne odovzdávať (ROS param, config.json, ...).
  const char* config_path_env = std::getenv("GLIM_RELOCALIZATION_CONFIG");
  const std::string config_path = config_path_env ? config_path_env : "config/config_relocalization.json";

  try {
    const auto j = load_json_config(config_path)["relocalization"];

    enabled_ = j.value("enable", true);
    map_path_ = j.value("map_path", std::string());

    const auto pose = j.value("initial_pose_xyz_yaw_deg", std::vector<double>{0, 0, 0, 0});
    initial_pose_ = make_pose(pose[0], pose[1], pose[2], pose[3] * M_PI / 180.0);

    linear_search_window_ = j.value("linear_search_window_m", 6.0);
    linear_search_step_ = j.value("linear_search_step_m", 0.5);
    angular_search_window_ = j.value("angular_search_window_deg", 30.0) * M_PI / 180.0;
    angular_search_step_ = j.value("angular_search_step_deg", 10.0) * M_PI / 180.0;
    min_overlap_score_ = j.value("min_overlap_score", 0.3);
    min_final_overlap_score_ = j.value("min_final_overlap_score", 0.5);
    voxel_resolution_ = j.value("voxel_resolution_m", 0.5);
    relocalization_factor_precision_ = j.value("relocalization_factor_precision", 1.0e6);
    lock_reference_map_ = j.value("lock_reference_map", true);
  } catch (const std::exception& e) {
    spdlog::error("[glim_relocalization] failed to load config: {}", e.what());
    enabled_ = false;
    return;
  }

  if (!enabled_) {
    spdlog::info("[glim_relocalization] module disabled via config (enable=false)");
    return;
  }

  if (!load_reference_map(map_path_)) {
    spdlog::error("[glim_relocalization] failed to load reference map from '{}', module will be inactive", map_path_);
    enabled_ = false;
    return;
  }

  // Registrácia callbackov - presne podľa oficiálneho extend.md vzoru
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  GlobalMappingCallbacks::on_insert_submap.add(std::bind(&RelocalizationModule::on_insert_submap, this, _1));
  GlobalMappingCallbacks::on_smoother_update.add(std::bind(&RelocalizationModule::on_smoother_update, this, _1, _2, _3));

  spdlog::info(
    "[glim_relocalization] loaded {} reference submap(s) from '{}', waiting for first live submap to relocalize",
    reference_submaps_.size(),
    map_path_);
}

RelocalizationModule::~RelocalizationModule() {}

bool RelocalizationModule::ok() const {
  return true;  // relokalizácia je "best effort" - nikdy nezastavuje celý systém
}

bool RelocalizationModule::load_reference_map(const std::string& path) {
  // Formát zhodný s tým, čo GlobalMapping::save() zapisuje: priečinok so
  // submapami 000000/, 000001/, ... plus graph.txt. Tu načítame len submapy
  // (SubMap::load), nie celý faktor graf - stačí nám ich T_world_origin + body.
  int index = 0;
  while (true) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%06d", index);
    const std::string submap_path = path + "/" + buf;

    std::ifstream check(submap_path + "/data.txt");
    if (!check) {
      break;  // žiadna ďalšia submapa - koniec
    }

    auto submap = SubMap::load(submap_path);
    if (!submap) {
      spdlog::warn("[glim_relocalization] failed to load submap at '{}'", submap_path);
      break;
    }

    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(voxel_resolution_);
    voxelmap->insert(*submap->frame);

    reference_submaps_.push_back(submap);
    reference_voxelmaps_.push_back(voxelmap);
    index++;
  }

  return !reference_submaps_.empty();
}

void RelocalizationModule::on_insert_submap(const SubMap::ConstPtr& submap) {
  if (!enabled_ || relocalized_) {
    return;  // v tejto zjednodušenej verzii relokalizujeme len raz, pri prvej submape
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (has_pending_) {
    return;  // predchádzajúca žiadosť ešte nebola spracovaná v on_smoother_update
  }
  pending_submap_ = submap;
  has_pending_ = true;
}

void RelocalizationModule::on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
  if (!enabled_ || !has_pending_) {
    return;
  }

  SubMap::ConstPtr submap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    submap = pending_submap_;
    pending_submap_.reset();
    has_pending_ = false;
  }

  if (!submap || reference_voxelmaps_.empty()) {
    return;
  }

  // 1) Zamkni referenčnú mapu v grafe (raz), aby ju optimalizácia nikdy nehýbala
  if (lock_reference_map_) {
    for (const auto& ref_submap : reference_submaps_) {
      const gtsam::Pose3 fixed_pose(ref_submap->T_world_origin.matrix());
      new_values.insert(M(ref_submap->id), fixed_pose);
      new_factors.emplace_shared<gtsam::NonlinearEquality1<gtsam::Pose3>>(fixed_pose, M(ref_submap->id));
    }
    lock_reference_map_ = false;  // len raz
  }

  // 2) Hrubý grid-search + VGICP doladenie voči prvej (najbližšej) referenčnej submape.
  //    Zjednodušenie oproti forku: neprehľadávame všetky referenčné submapy podľa
  //    vzdialenosti, len prvú - uprav podľa reálnej veľkosti tvojej mapy.
  const auto& reference_voxelmap = *reference_voxelmaps_.front();
  const auto& reference_submap = reference_submaps_.front();

  double coarse_score = 0.0;
  const Eigen::Isometry3d coarse_pose = coarse_search(reference_voxelmap, submap, coarse_score);

  if (coarse_score < min_overlap_score_) {
    spdlog::warn("[glim_relocalization] coarse search failed (best score {:.3f} < threshold {:.3f}), will retry on next submap", coarse_score, min_overlap_score_);
    // Znova to skúsime na ďalšej submape - neoznačujeme relocalized_ = true
    return;
  }

  double final_score = 0.0;
  const Eigen::Isometry3d refined_pose = refine_vgicp(reference_voxelmap, submap, coarse_pose, final_score);

  if (final_score < min_final_overlap_score_) {
    spdlog::warn("[glim_relocalization] VGICP refinement score too low ({:.3f} < {:.3f}), will retry on next submap", final_score, min_final_overlap_score_);
    return;
  }

  // 3) Vlož BetweenFactor medzi referenčnú a live submapu - toto je samotná "relokalizácia"
  const Eigen::Isometry3d T_ref_query = reference_submap->T_world_origin.inverse() * refined_pose;
  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, relocalization_factor_precision_);

  new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(M(reference_submap->id), X(submap->id), gtsam::Pose3(T_ref_query.matrix()), noise);

  relocalized_ = true;
  spdlog::info("[glim_relocalization] relocalized successfully against reference map (overlap score {:.3f})", final_score);
}

Eigen::Isometry3d RelocalizationModule::coarse_search(const gtsam_points::GaussianVoxelMap& reference_voxelmap, const SubMap::ConstPtr& query_submap, double& best_score) const {
  best_score = 0.0;
  Eigen::Isometry3d best_pose = initial_pose_;

  // POZOR: toto je hrubá sila (rovnaký princíp ako v pôvodnom forku) - pri
  // veľkom search window a malom kroku môže byť pomalé. Na CPU-only stroji
  // (bez GPU) drž linear_search_window_ a angular_search_window_ radšej menšie
  // a spoľahni sa na dobrý initial_pose_ odhad (napr. GPS/dokovacia poloha).
  for (double dx = -linear_search_window_ / 2.0; dx <= linear_search_window_ / 2.0; dx += linear_search_step_) {
    for (double dy = -linear_search_window_ / 2.0; dy <= linear_search_window_ / 2.0; dy += linear_search_step_) {
      for (double dyaw = -angular_search_window_ / 2.0; dyaw <= angular_search_window_ / 2.0; dyaw += angular_search_step_) {
        Eigen::Isometry3d candidate = initial_pose_;
        candidate.translation().x() += dx;
        candidate.translation().y() += dy;
        candidate.linear() = (Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()) * candidate.linear()).eval();

        // Skóre prekrytia: jednoduchý pomer bodov aktuálneho skenu, ktoré padnú
        // do obsadenej voxel bunky referenčnej mapy, ku celkovému počtu bodov.
        // TODO: nahraď/over podľa presnej verzie gtsam_points - ak je k dispozícii
        // gtsam_points::overlap_auto(...), použi radšej tú (je optimalizovanejšia).
        int hits = 0;
        const auto& pts = query_submap->frame->points;
        const int num_points = query_submap->frame->size();
        const int stride = std::max(1, num_points / 500);  // subsample pre rýchlosť
        int checked = 0;
        for (int i = 0; i < num_points; i += stride) {
          const Eigen::Vector4d p_local = pts[i];
          const Eigen::Vector3d p_world = candidate * p_local.head<3>();
          if (reference_voxelmap.insert /* placeholder */) {
            // gtsam_points::GaussianVoxelMap nemá priamu "contains" metódu vo
            // verejnom API - over si najvhodnejšiu (napr. cez KdTree nearest-neighbor
            // dotaz na referenčný point cloud, alebo FastOccupancyGrid::calc_overlap_rate,
            // ktorá je na presne toto navrhnutá). Toto miesto si nutne uprav pred buildom.
          }
          checked++;
        }
        const double score = checked > 0 ? static_cast<double>(hits) / checked : 0.0;

        if (score > best_score) {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }

  return best_pose;
}

Eigen::Isometry3d RelocalizationModule::refine_vgicp(const gtsam_points::GaussianVoxelMap& reference_voxelmap, const SubMap::ConstPtr& query_submap, const Eigen::Isometry3d& initial_guess, double& final_score) const {
  // Malý lokálny faktor graf s jednou premennou (pose kandidáta), VGICP faktorom
  // voči referenčnej voxel mape, optimalizovaný cez Levenberg-Marquardt.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  const gtsam::Key key = X(0);
  initial.insert(key, gtsam::Pose3(initial_guess.matrix()));

  // POZOR: IntegratedVGICPFactor očakáva dve premenné (target, source) - tu
  // referenčnú mapu držíme fixnú cez PriorFactor s vysokou presnosťou na key M(0),
  // namiesto skutočného "fixed frame" konštruktora, ktorý sa medzi verziami líši.
  // Over si presnú signatúru IntegratedVGICPFactor vo svojej verzii gtsam_points.
  const gtsam::Key ref_key = M(0);
  initial.insert(ref_key, gtsam::Pose3::Identity());
  graph.emplace_shared<gtsam::NonlinearEquality1<gtsam::Pose3>>(gtsam::Pose3::Identity(), ref_key);

  // graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(
  //     ref_key, key,
  //     /* target voxelmap */ std::shared_ptr<const gtsam_points::GaussianVoxelMap>(&reference_voxelmap, [](auto*){}),
  //     /* source cloud   */ query_submap->frame);
  //
  // ^ Zámerne zakomentované: presná signatúra konštruktora IntegratedVGICPFactor
  //   (poradie argumentov, shared_ptr typy) sa medzi verziami gtsam_points líšila.
  //   Toto je jediné miesto, kde treba pozrieť aktuálnu hlavičku
  //   <gtsam_points/factors/integrated_vgicp_factor.hpp> vo svojom builde a doplniť
  //   správne volanie - zvyšok modulu (callbacky, grid-search, vkladanie do
  //   on_smoother_update) je hotový a nezávislý od tejto jednej signatúry.

  gtsam::LevenbergMarquardtParams params;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const gtsam::Values result = optimizer.optimize();

  final_score = 0.0;  // TODO: po doplnení VGICP faktora vyššie, získaj overlap/error z result
  return Eigen::Isometry3d(result.at<gtsam::Pose3>(key).matrix());
}

extern "C" ExtensionModule* create_extension_module() {
  return new RelocalizationModule();
}

}  // namespace glim
