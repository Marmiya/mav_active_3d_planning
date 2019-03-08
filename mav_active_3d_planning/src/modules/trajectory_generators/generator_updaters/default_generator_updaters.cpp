#include "mav_active_3d_planning/modules/trajectory_generators/generator_updaters/default_generator_updaters.h"

#include <vector>
#include <algorithm>
#include <memory>

namespace mav_active_3d_planning {
    namespace generator_updaters {

        // ResetTree
        bool ResetTree::updateSegments(TrajectorySegment *root) {
            root->children.clear();
            return true;
        }

        // RecheckCollision
        RecheckCollision::RecheckCollision(TrajectoryGenerator *parent) : GeneratorUpdater(parent) {}

        bool RecheckCollision::updateSegments(TrajectorySegment *root) {
            checkSingle(root);
            return true;
        }

        bool RecheckCollision::isCollided(const mav_msgs::EigenTrajectoryPointVector &trajectory) {
            for (int i = 0; i < trajectory.size(); ++i) {
                if (!parent_->checkTraversable(trajectory[i].position_W)) {
                    return true;
                }
            }
            return false;
        }

        void RecheckCollision::checkSingle(TrajectorySegment *segment) {
            // Recursive removal
            int j = 0;
            for (int i = 0; i < segment->children.size(); ++i) {
                if (isCollided(segment->children[j]->trajectory)) {
                    segment->children.erase(segment->children.begin() + j);
                } else {
                    j++;
                }
            }
            // remaining children
            for (int i = 0; i < segment->children.size(); ++i) {
                checkSingle(segment->children[i].get());
            }
        }

    } // namespace generator_updaters
} // namepsace mav_active_3d_planning
