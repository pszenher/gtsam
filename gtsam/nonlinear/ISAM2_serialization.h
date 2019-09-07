#pragma once
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

// https://bitbucket.org/gtborg/gtsam/issues/307/serialization-of-gtsam-classes-such-as
GTSAM_VALUE_EXPORT(Pose3);
GTSAM_VALUE_EXPORT(Point3);
GTSAM_VALUE_EXPORT(Pose2);
GTSAM_VALUE_EXPORT(Point2);
BOOST_CLASS_EXPORT_GUID(PriorFactor<Pose3>, "gtsam_PriorFactor_Pose3");
BOOST_CLASS_EXPORT_GUID(BetweenFactor<Pose3>, "gtsam_BetweenFactor_Pose3");
BOOST_CLASS_EXPORT_GUID(PriorFactor<Pose2>, "gtsam_PriorFactor_Pose2");
BOOST_CLASS_EXPORT_GUID(BetweenFactor<Pose2>, "gtsam_BetweenFactor_Pose2");
BOOST_CLASS_EXPORT_GUID(noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(noiseModel::mEstimator::Cauchy, "gtsam_noiseModel_Cauchy");
BOOST_CLASS_EXPORT_GUID(noiseModel::mEstimator::DCS, "gtsam_noiseModel_DCS");
BOOST_CLASS_EXPORT_GUID(noiseModel::mEstimator::Huber, "gtsam_noiseModel_Huber");
BOOST_CLASS_EXPORT_GUID(noiseModel::Robust, "gtsam_noiseModel_Robust");
BOOST_CLASS_EXPORT_GUID(SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(SharedDiagonal, "gtsam_SharedDiagonal");
BOOST_CLASS_EXPORT_GUID(JacobianFactor, "gtsam_JacobianFactor");
BOOST_CLASS_EXPORT_GUID(HessianFactor, "gtsam_HessianFactor");
BOOST_CLASS_EXPORT_GUID(GaussianConditional, "gtsam_GaussianConditional");

namespace gtsam
{
std::string serializeValues(const Values &values)
{
  return serializeBinary(values);
}

Values deserializeValues(const std::string &buffer)
{
  Values values;
  deserializeBinary(buffer, values);
  return values;
}

std::string serializeNonlinearFactorGraph(const NonlinearFactorGraph &graph)
{
  return serializeBinary(graph);
}

NonlinearFactorGraph deserializeNonlinearFactorGraph(const std::string &buffer)
{
  NonlinearFactorGraph graph;
  deserializeBinary(buffer, graph);
  return graph;
}

std::string serializeISAM2(const ISAM2 &isam2)
{
  return serializeBinary(isam2);
}

ISAM2 deserializeISAM2(const std::string &buffer)
{
  ISAM2 isam2;
  deserializeBinary(buffer, isam2);
  return isam2;
}
}