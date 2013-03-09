cmake_minimum_required(VERSION 2.6)
project(non-rigid-tracking)

set(CMAKE_CXX_FLAGS "-Wall")

# All #includes relative to top level.
include_directories(.)
# For generated protobuf files.
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include(depend.cmake)

add_subdirectory(videoseg)
add_subdirectory(util)

add_executable(visualize-keypoints
  visualize_keypoints.cpp
  read_image.cpp
  sift_position.cpp
  sift_position_reader.cpp
  draw_sift_position.cpp
  random_color.cpp
  hsv.cpp)
target_link_libraries(visualize-keypoints
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(match-features
  match_features.cpp
  descriptor.cpp
  classifier.cpp
  find_matches.cpp
  find_unique_matches.cpp
  find_matches_util.cpp
  match.cpp
  match_result.cpp
  unique_match_result.cpp
  descriptor_reader.cpp
  unique_match_result_writer.cpp
  match_result_writer.cpp)
target_link_libraries(match-features
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(visualize-matches
  visualize_matches.cpp
  read_image.cpp
  match.cpp
  draw_matches.cpp
  sift_position.cpp
  random_color.cpp
  hsv.cpp
  match_reader.cpp
  draw_sift_position.cpp
  sift_position_reader.cpp)
target_link_libraries(visualize-matches
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(adjacent-matches-to-tracks
  adjacent_matches_to_tracks.cpp
  read_image.cpp
  sift_position.cpp
  match.cpp
  match_reader.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(adjacent-matches-to-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(track-features-bidir
  track_features_bidir.cpp
  read_image.cpp
  flow.cpp
  warp.cpp
  util.cpp
  similarity_warper.cpp
  similarity_warp.cpp
  sift_position.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp
  random_color.cpp
  hsv.cpp)
target_link_libraries(track-features-bidir
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES})

add_executable(extract-sift-tracks
  extract_sift_tracks.cpp
  read_image.cpp
  sift_position.cpp
  extract_sift.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp
  descriptor_writer.cpp
  descriptor.cpp)
target_link_libraries(extract-sift-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(extract-sift-test
  extract_sift_test.cpp
  read_image.cpp
  sift_feature.cpp
  sift_position.cpp
  detect_sift.cpp
  extract_sift.cpp
  descriptor.cpp)
target_link_libraries(extract-sift-test
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(pca-descriptor
  pca_descriptor.cpp
  random_color.cpp
  hsv.cpp
  descriptor_reader.cpp
  descriptor.cpp)
target_link_libraries(pca-descriptor
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(visualize-tracks
  visualize_tracks.cpp
  sift_position.cpp
  scale_space_position.cpp
  draw_sift_position.cpp
  sift_feature_drawer.cpp
  translation_feature_drawer.cpp
  scale_space_feature_drawer.cpp
  read_image.cpp
  random_color.cpp
  hsv.cpp
  sift_position_reader.cpp
  scale_space_position_reader.cpp)
target_link_libraries(visualize-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(merge-tracks
  merge_tracks.cpp
  sift_position.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(merge-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(select-active-tracks
  select_active_tracks.cpp
  sift_position.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(select-active-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(effect-of-sift-threshold
  effect_of_sift_threshold.cpp
  match.cpp
  descriptor.cpp
  descriptor_reader.cpp)
target_link_libraries(effect-of-sift-threshold
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(find-keypoints
  find_keypoints.cpp
  read_image.cpp
  detect_sift.cpp
  sift_position.cpp
  sift_feature.cpp
  sift_feature_writer.cpp
  sift_position_writer.cpp
  descriptor_writer.cpp
  descriptor.cpp)
target_link_libraries(find-keypoints
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(optimal-triangulation-test
  optimal_triangulation_test.cpp
  optimal_triangulation.cpp
  roots.cpp
  geometry.cpp)
target_link_libraries(optimal-triangulation-test
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${GSL_LIBRARIES})

add_executable(evaluate-matches
  evaluate_matches.cpp
  match.cpp
  match_reader.cpp
  matrix_reader.cpp
  optimal_triangulation.cpp
  roots.cpp)
target_link_libraries(evaluate-matches
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${GSL_LIBRARIES})

add_executable(undistort-image
  undistort_image.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  camera_properties_reader.cpp
  matrix_reader.cpp
  distortion.cpp
  read_image.cpp)
target_link_libraries(undistort-image
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(undistort-points
  undistort_points.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  camera_properties_reader.cpp
  matrix_reader.cpp
  distortion.cpp)
target_link_libraries(undistort-points
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(cameras-to-fund-mat
  cameras_to_fund_mat.cpp
  camera_pose.cpp
  camera_pose_reader.cpp
  world_point_reader.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  distortion.cpp
  camera_properties_reader.cpp
  matrix_reader.cpp
  geometry.cpp
  matrix_writer.cpp)
target_link_libraries(cameras-to-fund-mat
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(effect-of-matching-parameters
  effect_of_matching_parameters.cpp
  read_lines.cpp
  stats.cpp)
target_link_libraries(effect-of-matching-parameters
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(select-long-tracks
  select_long_tracks.cpp
  sift_position.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(select-long-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(visualize-multiview-tracks
  visualize_multiview_tracks.cpp
  read_lines.cpp
  image_index.cpp
  random_color.cpp
  hsv.cpp
  scale_space_position.cpp
  draw_scale_space_position.cpp
  scale_space_position_reader.cpp
  read_image.cpp)
target_link_libraries(visualize-multiview-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(select-distinctive-matches
  select_distinctive_matches.cpp
  unique_match_result.cpp
  match_result.cpp
  unique_match_result_reader.cpp
  unique_match_result_writer.cpp)
target_link_libraries(select-distinctive-matches
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(track-matches-to-multiview-tracks
  track_matches_to_multiview_tracks.cpp
  match.cpp
  scale_space_position.cpp
  image_index.cpp
  match_reader.cpp
  scale_space_position_reader.cpp
  scale_space_position_writer.cpp)
target_link_libraries(track-matches-to-multiview-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(merge-multiview-tracks
  merge_multiview_tracks.cpp
  sift_position.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp
  image_index.cpp)
target_link_libraries(merge-multiview-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(select-active-multiview-tracks
  select_active_multiview_tracks.cpp
  sift_position.cpp
  image_index.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(select-active-multiview-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(multiview-index-tracks-to-features
  multiview_index_tracks_to_features.cpp
  image_index.cpp
  sift_position.cpp
  read_lines.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(multiview-index-tracks-to-features
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(visualize-multiview-multitracks
  visualize_multiview_multitracks.cpp
  read_lines.cpp
  image_index.cpp
  random_color.cpp
  hsv.cpp
  sift_position.cpp
  draw_sift_position.cpp
  sift_position_reader.cpp
  read_image.cpp)
target_link_libraries(visualize-multiview-multitracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(find-max-cliques
  find_max_cliques.cpp
  read_lines.cpp
  match.cpp
  sift_feature.cpp
  sift_position.cpp
  descriptor.cpp
  image_index.cpp
  match_reader.cpp
  sift_feature_reader.cpp
  descriptor_reader.cpp
  sift_position_reader.cpp
  sift_feature_writer.cpp
  descriptor_writer.cpp
  sift_position_writer.cpp)
target_link_libraries(find-max-cliques
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(partition-graph
  partition_graph.cpp
  read_lines.cpp
  match.cpp
  sift_feature.cpp
  sift_position.cpp
  descriptor.cpp
  image_index.cpp
  feature_index.cpp
  match_reader.cpp
  sift_feature_reader.cpp
  descriptor_reader.cpp
  sift_position_reader.cpp
  sift_feature_writer.cpp
  descriptor_writer.cpp
  sift_position_writer.cpp)
target_link_libraries(partition-graph
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(visualize-multiview-multitracks-time-slice
  visualize_multiview_multitracks_time_slice.cpp
  read_lines.cpp
  image_index.cpp
  random_color.cpp
  hsv.cpp
  sift_position.cpp
  draw_sift_position.cpp
  sift_position_reader.cpp
  read_image.cpp)
target_link_libraries(visualize-multiview-multitracks-time-slice
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(cluster-descriptors
  cluster_descriptors.cpp
  kmeans.cpp
  random.cpp
  image_index.cpp
  sift_feature.cpp
  sift_position.cpp
  descriptor.cpp
  read_lines.cpp
  sift_feature_reader.cpp
  sift_position_reader.cpp
  descriptor_reader.cpp)
target_link_libraries(cluster-descriptors
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(sparse-mat-unittest
  sparse_mat_unittest.cpp
  sparse_mat.cpp)
target_link_libraries(sparse-mat-unittest
  ${GTEST_BOTH_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(visualize-some-multiview-tracks
  visualize_some_multiview_tracks.cpp
  random.cpp
  image_index.cpp
  random_color.cpp
  hsv.cpp
  sift_position.cpp
  draw_sift_position.cpp
  read_lines.cpp
  sift_position_reader.cpp
  read_image.cpp)
target_link_libraries(visualize-some-multiview-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(select-rigid-matches
    select_rigid_matches.cpp
    optimal_triangulation.cpp
    roots.cpp
    match_result.cpp
    match_result_reader.cpp
    matrix_reader.cpp
    match_result_writer.cpp)
target_link_libraries(select-rigid-matches
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${GSL_LIBRARIES}
  ${LAPACK_LIBRARIES})

add_executable(select-tracked-matches
    select_tracked_matches.cpp
    sift_position.cpp
    match_result.cpp
    match_result_reader.cpp
    sift_position_reader.cpp
    match_result_writer.cpp)
target_link_libraries(select-tracked-matches
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(matches-to-multiview-tracks
  matches_to_multiview_tracks.cpp
  read_lines.cpp
  match.cpp
  sift_feature.cpp
  sift_position.cpp
  descriptor.cpp
  image_index.cpp
  match_reader.cpp
  feature_index.cpp
  sift_feature_reader.cpp
  descriptor_reader.cpp
  sift_position_reader.cpp
  sift_feature_writer.cpp
  descriptor_writer.cpp
  sift_position_writer.cpp)
target_link_libraries(matches-to-multiview-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(agglomerative-cluster
  agglomerative_cluster.cpp
  read_lines.cpp
  sift_feature.cpp
  sift_position.cpp
  descriptor.cpp
  match_result.cpp
  image_index.cpp
  feature_index.cpp
  camera.cpp
  camera_pose.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  distortion.cpp
  camera_reader.cpp
  camera_properties_reader.cpp
  camera_pose_reader.cpp
  matrix_reader.cpp
  world_point_reader.cpp
  match_result_reader.cpp
  sift_feature_reader.cpp
  descriptor_reader.cpp
  sift_position_reader.cpp
  sift_feature_writer.cpp
  descriptor_writer.cpp
  sift_position_writer.cpp
  find_smooth_trajectory.cpp)
target_link_libraries(agglomerative-cluster
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES})

add_executable(feature-sets-unittest
  feature_sets_unittest.cpp
  feature_index.cpp
  image_index.cpp)
target_link_libraries(feature-sets-unittest
  ${GTEST_BOTH_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(train-appearance-classifiers
  train_appearance_classifiers.cpp
  descriptor.cpp
  classifier.cpp
  descriptor_reader.cpp
  classifier_writer.cpp)
target_link_libraries(train-appearance-classifiers
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${LIBLINEAR_LIBRARIES})

add_executable(match-features-using-classifiers
  match_features_using_classifiers.cpp
  descriptor.cpp
  classifier.cpp
  find_unique_matches.cpp
  find_matches.cpp
  find_matches_util.cpp
  match.cpp
  match_result.cpp
  unique_match_result.cpp
  descriptor_reader.cpp
  classifier_reader.cpp
  match_result_writer.cpp
  unique_match_result_writer.cpp)
target_link_libraries(match-features-using-classifiers
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(static-camera-to-moving
  static_cameras_to_moving.cpp
  camera.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  distortion.cpp
  camera_pose.cpp
  camera_writer.cpp
  camera_properties_writer.cpp
  camera_pose_writer.cpp
  world_point_writer.cpp
  world_point_reader.cpp
  matrix_reader.cpp
  matrix_writer.cpp
  camera_properties_reader.cpp
  camera_pose_reader.cpp)
target_link_libraries(static-camera-to-moving
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(combine-matches
  combine_matches.cpp
  match.cpp
  match_result.cpp
  match_reader.cpp
  match_result_reader.cpp
  match_writer.cpp
  match_result_writer.cpp)
target_link_libraries(combine-matches
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(filter-matches
  filter_matches.cpp
  match.cpp
  match_result.cpp
  unique_match_result.cpp
  match_result_reader.cpp
  unique_match_result_reader.cpp
  match_result_writer.cpp)
target_link_libraries(filter-matches
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(display-distorted-epipolar-line
  display_distorted_epipolar_line.cpp
  distortion.cpp
  distorted_epipolar_lines.cpp
  sift_position.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  distortion.cpp
  sift_position_reader.cpp
  camera_properties_reader.cpp
  matrix_reader.cpp
  read_image.cpp)
target_link_libraries(display-distorted-epipolar-line
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(extract-multiview-examples
  extract_multiview_examples.cpp
  distortion.cpp
  distorted_epipolar_lines.cpp
  sift_feature.cpp
  sift_position.cpp
  descriptor.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  distortion.cpp
  extract_sift.cpp
  sift_feature_reader.cpp
  sift_position_reader.cpp
  descriptor_reader.cpp
  camera_properties_reader.cpp
  matrix_reader.cpp
  read_lines.cpp
  read_image.cpp)
target_link_libraries(extract-multiview-examples
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(offline-classifier-tracking
  offline_classifier_tracking.cpp
  viterbi.cpp
  image_file_sequence.cpp
  admm_tracking.cpp
  dynamic_program_tracker.cpp
  dynamic_program_occlusion_tracker.cpp
  util.cpp
  read_image.cpp)
target_link_libraries(offline-classifier-tracking
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(viterbi-unittest
  viterbi_unittest.cpp
  viterbi.cpp)
target_link_libraries(viterbi-unittest
  ${GTEST_BOTH_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(display-admm-tracking
  display_admm_tracking.cpp
  viterbi.cpp
  read_image.cpp)
target_link_libraries(display-admm-tracking
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(choose-keypoints
  choose_keypoints.cpp
  sift_position.cpp
  draw_sift_position.cpp
  read_image.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(choose-keypoints
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(select-keypoint-subset
  select_keypoint_subset.cpp
  sift_position.cpp
  sift_position_reader.cpp
  sift_position_writer.cpp)
target_link_libraries(select-keypoint-subset
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(restrict-tracks-to-box
  restrict_tracks_to_box.cpp
  scale_space_position.cpp
  scale_space_feature_drawer.cpp
  random_color.cpp
  read_image.cpp
  scale_space_position_reader.cpp
  scale_space_position_writer.cpp)
target_link_libraries(restrict-tracks-to-box
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(choose-points
  choose_points.cpp
  match.cpp
  random_color.cpp
  read_image.cpp
  read_lines.cpp
  match_writer.cpp)
target_link_libraries(choose-points
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(manually-filter-tracks
  manually_filter_tracks.cpp
  scale_space_position.cpp
  scale_space_feature_drawer.cpp
  random_color.cpp
  read_image.cpp
  scale_space_position_reader.cpp
  scale_space_position_writer.cpp)
target_link_libraries(manually-filter-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(merge-forward-and-reverse-tracks
  merge_forward_and_reverse_tracks.cpp
  scale_space_position.cpp
  scale_space_position_reader.cpp
  scale_space_position_writer.cpp)
target_link_libraries(merge-forward-and-reverse-tracks
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(track-live
  track_live.cpp
  flow.cpp
  warp.cpp
  util.cpp
  translation_warp.cpp
  translation_warper.cpp
  similarity_warp.cpp
  similarity_warper.cpp)
target_link_libraries(track-live
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES})

add_executable(find-multiview-track
  find_multiview_track.cpp
  quantize_ray.cpp
  viterbi.cpp
  camera.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  camera_pose.cpp
  distortion.cpp
  read_lines.cpp
  matrix_reader.cpp
  camera_properties_reader.cpp
  camera_pose_reader.cpp
  world_point_reader.cpp)
target_link_libraries(find-multiview-track
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(display-distorted-epipolar-line-segments
  display_distorted_epipolar_line_segments.cpp
  quantize_ray.cpp
  distortion.cpp
  camera.cpp
  camera_properties.cpp
  axis_aligned_ellipse.cpp
  camera_pose.cpp
  distortion.cpp
  camera_properties_reader.cpp
  camera_pose_reader.cpp
  matrix_reader.cpp
  world_point_reader.cpp
  read_lines.cpp
  read_image.cpp)
target_link_libraries(display-distorted-epipolar-line-segments
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBS})

#add_executable(spectral-partition-graph
#  spectral_partition_graph.cpp
#  normalized_cut.cpp
#  sparse_mat.cpp
#  read_lines.cpp
#  match.cpp
#  sift_feature.cpp
#  sift_position.cpp
#  descriptor.cpp
#  image_index.cpp
#  feature_index.cpp
#  match_reader.cpp
#  sift_feature_reader.cpp
#  descriptor_reader.cpp
#  sift_position_reader.cpp
#  sift_feature_writer.cpp
#  descriptor_writer.cpp
#  sift_position_writer.cpp)
#target_link_libraries(spectral-partition-graph
#  ${GLOG_LIBRARIES}
#  ${GFLAGS_LIBRARIES}
#  ${OpenCV_LIBS}
#  arpack gfortran)
