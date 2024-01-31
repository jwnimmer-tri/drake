# s#(include "drake/geometry/)(deformable_mesh|geometry_frame|geometry_ids|geometry_instance|geometry_properties|geometry_roles|geometry_set|geometry_version|internal_frame|kinematics_vector|make_mesh_for_deformable|mesh_deformer|mesh_traits|polygon_surface_mesh|proximity_properties|read_obj|rgba|shape_specification|shape_to_string|triangle_surface_mesh|utilities|volume_mesh)\.#\1common/\2.#;
# 
# s#(include "drake/geometry/)(collision_filter_declaration|collision_filter_manager|geometry_state|internal_geometry|proximity_engine|query_object|scene_graph|scene_graph_inspector)\.#\1scene_graph/\2.#;
# 
# s#(include "drake/geometry/)(drake_visualizer|drake_visualizer_params|meshcat|meshcat_animation|meshcat_file_storage_internal|meshcat_graphviz|meshcat_internal|meshcat_point_cloud_visualizer|meshcat_types_internal|meshcat_visualizer|meshcat_visualizer_params)\.#\1meshcat/\2\.#;
# 
# s#(include "drake/geometry/)proximity/(deformable_mesh|geometry_frame|geometry_ids|geometry_instance|geometry_properties|geometry_roles|geometry_set|geometry_version|internal_frame|kinematics_vector|make_mesh_for_deformable|mesh_deformer|mesh_traits|polygon_surface_mesh|proximity_properties|read_obj|rgba|shape_specification|shape_to_string|triangle_surface_mesh|utilities|volume_mesh)\.#\1common/\2.#;
# 
# s#/meshcat/meshcat/#/meshcat/#g;
# s#/scene_graph/scene_graph/#/scene_graph/#g;
# 
# s#/common/deformable_mesh_with_bvh#/proximity/deformable_mesh_with_bvh#g;
# s#/common/make_mesh_for_deformable#/proximity/make_mesh_for_deformable#g;
# s#/proximity/obj_to_surface_mesh#/common/obj_to_surface_mesh#g;
# s#/scene_graph/collision_filter_declaration#/common/collision_filter_declaration#g;
# s#/proximity/volume_to_surface_mesh#/common/volume_to_surface_mesh#g;
# 
# s#/proximity/(mesh_field_linear|polygon_surface_mesh_field|triangle_surface_mesh_field|volume_mesh_field)#/common/\1#g;
#
# s#geometry/meshcat/(meshcat_point_cloud_visualizer|meshcat_visualizer|meshcat_visualizer_params|drake_visualizer_params|drake_visualizer)\.#meshcat/\1.#g;
#
#s#systems/sensors/(camera_info.h|image_file_format.h|image.h|image_io.h|image_io_internal.h|vtk_diagnostic_event_observer.h|vtk_image_reader_writer.h|pixel_types.h)#perception/common/\1#g;
#
#s#geometry/proximity/(sorted_triplet.h)#common/\1#g;

s#multibody/meshcat/(contact_visualizer.h|contact_visualizer_params.h|hydroelastic_contact_visualizer.h|joint_sliders.h|point_contact_visualizer.h)#visualization/\1#g;
