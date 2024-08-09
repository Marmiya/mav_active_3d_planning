#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NAIVE_EVALUATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NAIVE_EVALUATOR_H_

#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
  namespace trajectory_evaluator {
    class PotentialEvaluator : public SimulatedSensorEvaluator {
        public:
        explicit PotentialEvaluator(PlannerI& planner);  // NOLINT

        void setupFromParamMap(Module::ParamMap* param_map) override;

        protected:
        static ModuleFactoryRegistry::Registration<PotentialEvaluator> registration;

        // Override virtual methods
        bool computeGainFromVisibleVoxels(TrajectorySegment* traj_in) override;
        };

    }  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NAIVE_EVALUATOR_H_