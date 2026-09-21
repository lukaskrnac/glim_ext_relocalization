#pragma once

#include <atomic>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/util/extension_module.hpp>
#include <glim/mapping/sub_map.hpp>

namespace gtsam {
class NonlinearFactorGraph;
class Values;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class GaussianVoxelMap;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Zjednodušený relokalizačný extension modul pre GLIM.
 *
 * Princíp (inšpirované forkom se7oluti0n/glim_localization, ale bez
 * nahrádzania GlobalMappingBase - pripája sa cez oficiálne callbacky):
 *
 *  1. Pri štarte načíta prebuilt referenčnú mapu (submapy uložené cez
 *     GlobalMapping::save) a postaví z nej voxel mriežky.
 *  2. Cez GlobalMappingCallbacks::on_insert_submap zachytí prvú (a voliteľne
 *     ďalšie) submapu z bežiaceho mapovania.
 *  3. Spraví hrubý grid-search (X/Y/yaw) okolo počiatočného odhadu pozície,
 *     každého kandidáta oskóruje prekrytím voči referenčnej mape.
 *  4. Najlepšieho kandidáta doladí cez VGICP (gtsam_points::IntegratedVGICPFactor
 *     + lokálna GTSAM optimalizácia).
 *  5. Cez GlobalMappingCallbacks::on_smoother_update vloží výsledok ako
 *     BetweenFactor priamo do bežiaceho iSAM2 grafu, a voliteľne "zamkne"
 *     referenčnú mapu cez NonlinearEquality1, aby sa neoptimalizovala.
 *
 * POZNÁMKA: toto je štartovacia kostra, nie hotový, otestovaný modul.
 * Očakávaj drobné úpravy include ciest / API signatúr podľa presnej
 * verzie glim/gtsam_points, s ktorou to budeš stavať.
 */
class RelocalizationModule : public ExtensionModule {
public:
  RelocalizationModule();
  virtual ~RelocalizationModule() override;

  virtual bool ok() const override;

private:
  // Callback z GlobalMappingCallbacks::on_insert_submap
  void on_insert_submap(const SubMap::ConstPtr& submap);

  // Callback z GlobalMappingCallbacks::on_smoother_update
  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values);

  // Načíta referenčnú mapu z config["map_path"] a postaví voxel mriežky
  bool load_reference_map(const std::string& path);

  // Hrubý grid-search (X/Y/yaw) okolo initial_pose_, vráti najlepšieho kandidáta a jeho skóre
  Eigen::Isometry3d coarse_search(const std::shared_ptr<const gtsam_points::GaussianVoxelMap>& reference_voxelmap, const SubMap::ConstPtr& query_submap, double& best_score) const;

  // VGICP doladenie okolo coarse_search výsledku
  Eigen::Isometry3d refine_vgicp(const std::shared_ptr<const gtsam_points::GaussianVoxelMap>& reference_voxelmap, const SubMap::ConstPtr& query_submap, const Eigen::Isometry3d& initial_guess, double& final_score) const;

private:
  mutable std::mutex mutex_;

  bool enabled_ = false;
  bool relocalized_ = false;
  bool lock_reference_map_ = true;

  // Parametre z config_relocalization.json
  std::string map_path_;
  Eigen::Isometry3d initial_pose_ = Eigen::Isometry3d::Identity();
  double linear_search_window_ = 6.0;
  double linear_search_step_ = 0.5;
  double angular_search_window_ = 30.0 * M_PI / 180.0;
  double angular_search_step_ = 10.0 * M_PI / 180.0;
  double min_overlap_score_ = 0.3;
  double min_final_overlap_score_ = 0.5;
  double voxel_resolution_ = 0.5;
  double relocalization_factor_precision_ = 1.0e6;

  // Referenčná (prebuilt) mapa - submapa/mapy + ich voxel mriežky
  std::vector<SubMap::Ptr> reference_submaps_;
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMap>> reference_voxelmaps_;

  // Pending submapa čakajúca na spracovanie v on_smoother_update
  // (relokalizácia sa NEROBÍ priamo v on_insert_submap callbacku, lebo ten
  // beží vo vlastnom mapovacom vlákne a chceme injektovať faktory presne
  // v momente pred optimalizáciou)
  SubMap::ConstPtr pending_submap_;
  std::atomic<bool> has_pending_{false};
};

}  // namespace glim
