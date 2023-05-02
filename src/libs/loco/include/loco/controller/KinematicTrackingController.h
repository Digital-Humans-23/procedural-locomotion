#pragma once

#include <loco/controller/LocomotionController.h>
#include <loco/kinematics/IK_Solver.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/robot/RB.h>
#include <loco/robot/RBJoint.h>

namespace crl::loco {

/**
 * A controller that kinematically "tracks" the objectives output by a
 * locomotion trajectory generator
 */
class KinematicTrackingController : public LocomotionController {
public:
    std::shared_ptr<LeggedRobot> robot;
    std::shared_ptr<IK_Solver> ikSolver = nullptr;

public:
    /**
     * constructor
     */
    KinematicTrackingController(const std::shared_ptr<LocomotionTrajectoryPlanner> &planner) : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = std::make_shared<IK_Solver>(robot);
    }

    /**
     * destructor
     */
    ~KinematicTrackingController(void) override = default;

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }

    void computeAndApplyControlSignals(double dt) override {
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.
        P3D targetPos = planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(planner->getSimTime() + dt);

        robot->setRootState(targetPos, targetOrientation);

        // now we solve inverse kinematics for each limbs
        for (uint i = 0; i < robot->getLimbCount(); i++) {
            P3D target = planner->getTargetLimbEEPositionAtTime(robot->getLimb(i), planner->getSimTime() + dt);

            ikSolver->addEndEffectorTarget(robot->getLimb(i)->eeRB, robot->getLimb(i)->ee->endEffectorOffset, target);
        }

        ikSolver->solve();
    }

    void advanceInTime(double dt) override {
        planner->advanceInTime(dt);
    }

    void drawDebugInfo(gui::Shader *shader) override {
        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl::loco