#include "drake/multibody/tree/joint_actuator.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace {

constexpr double kPositiveEffortLimit = 1e3;
constexpr double kZeroEffortLimit = 0;
constexpr double kNegativeEffortLimit = -2;

GTEST_TEST(JointActuatorTest, JointActuatorLimitTest) {
  auto tree_pointer = std::make_unique<internal::MultibodyTree<double>>();
  internal::MultibodyTree<double>& tree = *tree_pointer;

  // Spatial inertia for adding body. The actual value is not important for
  // these tests and therefore we do not initialize it.
  const SpatialInertia<double> M_B;  // Default construction is ok for this.

  // Add bodies so we can add joints to them.
  const auto body1 = &tree.AddBody<RigidBody>("body1", M_B);
  const auto body2 = &tree.AddBody<RigidBody>("body2", M_B);
  const auto body3 = &tree.AddBody<RigidBody>("body3", M_B);

  // Add a prismatic joint between the world and body1:
  const Joint<double>& body1_world =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism1", tree.world_body().body_frame(), body1->body_frame(),
          Eigen::Vector3d(0, 0, 1)));

  tree.AddJointActuator("act1", body1_world, kPositiveEffortLimit);
  // Validate the actuator effort limit has been set up correctly.
  const auto& actuator1 = tree.GetJointActuatorByName("act1");
  EXPECT_EQ(actuator1.effort_limit(), kPositiveEffortLimit);

  // Unit test PD controller APIs.
  EXPECT_FALSE(actuator1.has_controller());
  JointActuator<double>& mutable_actuator1 =
      tree.get_mutable_joint_actuator(actuator1.index());
  PdControllerGains gains{.p = 1000, .d = 100};
  mutable_actuator1.set_controller_gains(gains);
  EXPECT_TRUE(actuator1.has_controller());

  // Throw if the effort limit is set to 0.
  const Joint<double>& body2_body1 =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism2", body1->body_frame(), body2->body_frame(),
          Eigen::Vector3d(0, 0, 1)));
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.AddJointActuator("act2", body2_body1, kNegativeEffortLimit),
      "Effort limit must be strictly positive!");

  // Throw if the effort limit is set to be negative.
  const Joint<double>& body3_body2 =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism3", body2->body_frame(), body3->body_frame(),
          Eigen::Vector3d(0, 0, 1)));
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.AddJointActuator("act3", body3_body2, kZeroEffortLimit),
      "Effort limit must be strictly positive!");

  DRAKE_EXPECT_THROWS_MESSAGE(actuator1.input_start(),
                              ".*after the MultibodyPlant is finalized.");
  DRAKE_EXPECT_THROWS_MESSAGE(actuator1.num_inputs(),
                              ".*after the MultibodyPlant is finalized.");

  const auto body4 = &tree.AddBody<RigidBody>("body4", M_B);
  const Joint<double>& body4_world =
      tree.AddJoint(std::make_unique<PlanarJoint<double>>(
          "planar4", tree.world_body().body_frame(), body4->body_frame(),
          Eigen::Vector3d{0, 0, 0.1}));

  const auto& actuator4 =
      tree.AddJointActuator("act4", body4_world, kPositiveEffortLimit);

  tree.Finalize();

  EXPECT_EQ(tree.num_actuated_dofs(), 4);
  EXPECT_EQ(actuator4.input_start(), 1);
  EXPECT_EQ(actuator4.num_inputs(), 3);

  std::vector<JointActuatorIndex> expected_actuator_indices{
      actuator1.index(), actuator4.index()};
  EXPECT_EQ(tree.GetJointActuatorIndices(), expected_actuator_indices);
}

GTEST_TEST(JointActuatorTest, RemoveJointActuatorTest) {
  auto tree_pointer = std::make_unique<internal::MultibodyTree<double>>();
  internal::MultibodyTree<double>& tree = *tree_pointer;

  // Spatial inertia for adding body. The actual value is not important for
  // these tests and therefore we do not initialize it.
  const SpatialInertia<double> M_B;  // Default construction is ok for this.

  // Add model instances.
  const auto model_instance1 = tree.AddModelInstance("instance1");
  const auto model_instance2 = tree.AddModelInstance("instance2");

  // Add bodies so we can add joints to them.
  const auto body1 = &tree.AddRigidBody("body1", model_instance1, M_B);
  const auto body2 = &tree.AddRigidBody("body2", model_instance1, M_B);
  const auto body3 = &tree.AddRigidBody("body3", model_instance1, M_B);
  const auto body4 = &tree.AddRigidBody("body4", model_instance2, M_B);

  // Add a prismatic joint between the world and body1:
  const Joint<double>& body1_world =
      tree.AddJoint(std::make_unique<PlanarJoint<double>>(
          "planar1", tree.world_body().body_frame(), body1->body_frame(),
          Eigen::Vector3d(0, 0, 1)));

  const JointActuator<double>& actuator1 = tree.AddJointActuator(
      "duplicate_name", body1_world, kPositiveEffortLimit);

  const Joint<double>& body2_body1 =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism2", body1->body_frame(), body2->body_frame(),
          Eigen::Vector3d(0, 0, 1)));
  const JointActuator<double>& actuator2 =
    tree.AddJointActuator("act2", body2_body1, kPositiveEffortLimit);

  const Joint<double>& body3_body2 =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism3", body2->body_frame(), body3->body_frame(),
          Eigen::Vector3d(0, 0, 1)));
  const JointActuator<double>& actuator3 =
    tree.AddJointActuator("act3", body3_body2, kPositiveEffortLimit);

  const Joint<double>& body4_world =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism3", tree.world_body().body_frame(), body4->body_frame(),
          Eigen::Vector3d(0, 0, 1)));
  const JointActuator<double>& actuator4 =
    tree.AddJointActuator("duplicate_name", body4_world, kPositiveEffortLimit);

  JointActuatorIndex actuator1_index = actuator1.index();
  JointActuatorIndex actuator2_index = actuator2.index();
  JointActuatorIndex actuator3_index = actuator3.index();
  JointActuatorIndex actuator4_index = actuator4.index();

  EXPECT_EQ(tree.num_actuators(), 4);
  EXPECT_EQ(tree.num_actuated_dofs(), 3 + 1 + 1 + 1);
  std::vector<JointActuatorIndex> expected_indices{
      actuator1_index, actuator2_index, actuator3_index, actuator4_index};
  EXPECT_EQ(tree.GetJointActuatorIndices(), expected_indices);

  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.HasJointActuatorNamed("duplicate_name"),
      ".*appears in multiple model instances.*instance1.*instance2.*");
  EXPECT_TRUE(tree.HasJointActuatorNamed("duplicate_name", model_instance1));
  EXPECT_TRUE(tree.HasJointActuatorNamed("duplicate_name", model_instance2));

  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.GetJointActuatorByName("duplicate_name"),
      ".*appears in multiple model instances.*instance1.*instance2.*");
  EXPECT_EQ(
      tree.GetJointActuatorByName("duplicate_name", model_instance1).index(),
      actuator1_index);
  EXPECT_EQ(
      tree.GetJointActuatorByName("duplicate_name", model_instance2).index(),
      actuator4_index);

  // Remove the first actuator.
  tree.RemoveJointActuator(actuator1);
  EXPECT_EQ(tree.num_actuators(), 3);
  EXPECT_EQ(tree.num_actuated_dofs(), 1 + 1 + 1);
  EXPECT_FALSE(tree.has_joint_actuator(actuator1_index));
  expected_indices = std::vector<JointActuatorIndex>{
      actuator2_index, actuator3_index, actuator4_index};
  EXPECT_EQ(tree.GetJointActuatorIndices(), expected_indices);
  EXPECT_TRUE(tree.HasJointActuatorNamed("duplicate_name"));
  EXPECT_FALSE(
      tree.HasJointActuatorNamed("duplicate_name", world_model_instance()));
  EXPECT_FALSE(tree.HasJointActuatorNamed("duplicate_name", model_instance1));
  EXPECT_TRUE(tree.HasJointActuatorNamed("duplicate_name", model_instance2));
  EXPECT_EQ(
      tree.GetJointActuatorByName("duplicate_name").index(),
      actuator4_index);
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.GetJointActuatorByName("duplicate_name", model_instance1),
      ".*There is no JointActuator named 'duplicate_name' in the model "
      "instance named 'instance1', but one does exist in other model "
      "instances.*instance2.*");
  EXPECT_EQ(
      tree.GetJointActuatorByName("duplicate_name", model_instance2).index(),
      actuator4_index);

  // Replace the third actuator.
  tree.RemoveJointActuator(actuator3);
  EXPECT_EQ(tree.num_actuators(), 2);
  EXPECT_EQ(tree.num_actuated_dofs(), 2);
  EXPECT_FALSE(tree.has_joint_actuator(actuator3_index));
  expected_indices =
      std::vector<JointActuatorIndex>{actuator2_index, actuator4_index};
  EXPECT_EQ(tree.GetJointActuatorIndices(), expected_indices);
  EXPECT_FALSE(tree.HasJointActuatorNamed("act3"));
  // Add a new actuator with the same name as the first.
  const JointActuator<double>& new_actuator3 =
      tree.AddJointActuator("act3", body3_body2, 2 * kPositiveEffortLimit);
  JointActuatorIndex new_actuator3_index = new_actuator3.index();
  EXPECT_EQ(tree.num_actuators(), 3);
  EXPECT_EQ(tree.num_actuated_dofs(), 1 + 1 + 1);
  EXPECT_TRUE(tree.has_joint_actuator(new_actuator3_index));
  expected_indices.push_back(new_actuator3_index);
  EXPECT_EQ(tree.GetJointActuatorIndices(), expected_indices);
  EXPECT_TRUE(tree.HasJointActuatorNamed("act3"));
  EXPECT_TRUE(tree.HasJointActuatorNamed("act3", model_instance1));
  EXPECT_EQ(tree.GetJointActuatorByName("act3").index(), new_actuator3.index());
  EXPECT_EQ(tree.GetJointActuatorByName("act3", model_instance1).index(),
            new_actuator3.index());

  tree.Finalize();

  EXPECT_EQ(actuator2.input_start(), 0);
  EXPECT_EQ(actuator4.input_start(), 1);
  EXPECT_EQ(new_actuator3.input_start(), 2);
  EXPECT_EQ(actuator2.num_inputs(), 1);
  EXPECT_EQ(actuator4.num_inputs(), 1);
  EXPECT_EQ(new_actuator3.num_inputs(), 1);
  EXPECT_EQ(tree.num_actuators(model_instance1), 2);
  EXPECT_EQ(tree.num_actuated_dofs(model_instance1), 2);
  EXPECT_EQ(tree.num_actuators(model_instance2), 1);
  EXPECT_EQ(tree.num_actuated_dofs(model_instance2), 1);
  std::vector<JointActuatorIndex> expected_model1_instances{
      actuator2_index, new_actuator3_index};
  EXPECT_EQ(tree.GetJointActuatorIndices(model_instance1),
            expected_model1_instances);

  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.RemoveJointActuator(actuator2),
      "Post-finalize calls to 'RemoveJointActuator.*' are not allowed.*");

  // Confirm that removed joint logic is preserved after cloning.
  std::unique_ptr<internal::MultibodyTree<double>> clone =
      tree.CloneToScalar<double>();
  EXPECT_EQ(clone->num_actuators(), 3);
  EXPECT_EQ(clone->num_actuated_dofs(), 1 + 1 + 1);
  EXPECT_TRUE(clone->has_joint_actuator(new_actuator3_index));
  EXPECT_EQ(clone->GetJointActuatorIndices(), expected_indices);
  EXPECT_TRUE(clone->HasJointActuatorNamed("act3"));
  EXPECT_TRUE(clone->HasJointActuatorNamed("act3", model_instance1));
  EXPECT_EQ(clone->GetJointActuatorByName("act3").index(),
            new_actuator3.index());
  EXPECT_EQ(clone->GetJointActuatorByName("act3", model_instance1).index(),
            new_actuator3.index());
  EXPECT_EQ(actuator2.input_start(), 0);
  EXPECT_EQ(actuator4.input_start(), 1);
  EXPECT_EQ(new_actuator3.input_start(), 2);
  EXPECT_EQ(actuator2.num_inputs(), 1);
  EXPECT_EQ(actuator4.num_inputs(), 1);
  EXPECT_EQ(new_actuator3.num_inputs(), 1);
  EXPECT_EQ(clone->num_actuators(model_instance1), 2);
  EXPECT_EQ(clone->num_actuated_dofs(model_instance1), 2);
  EXPECT_EQ(clone->num_actuators(model_instance2), 1);
  EXPECT_EQ(clone->num_actuated_dofs(model_instance2), 1);
  EXPECT_EQ(clone->GetJointActuatorIndices(model_instance1),
            expected_model1_instances);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
