#pragma once

#include <crl-basic/gui/renderer.h>
#include <crl-basic/utils/trajectory.h>
#include <loco/planner/BodyFrame.h>
#include <loco/planner/FootFallPattern.h>
#include <loco/planner/LocomotionPlannerHelpers.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/robot/RB.h>
#include <loco/robot/RBJoint.h>
#include <loco/robot/RBUtils.h>

namespace crl::loco {

/**
 * A basic trajectory generator for locomotion tasks. We will assume a simple
 * structure for the robot, namely a whole lotta legs connected to a trunk.
 * This generator can be querried for high level objectives, such as target
 * position for the feet, position/orientation/velocity for the trunk, etc...
 *
 * This simplest of models assumes the trunk motion follows the motion of the body frame
 *
 * Note: it is convenient to specify some quantities in a coordinate frame that is
 * located at the CoM of the trunk and shares its heading, but otherwise ignores
 * pitch, roll, parasitic components of its motion (e.g. periodic, transient or
 * accidental fluctuations in yaw/tangential location of COM that should not be
 * affecting the robot's COM position or heading). We call this the body frame.
 *
 */

class SimpleLocomotionTrajectoryPlanner : public LocomotionTrajectoryPlanner {
protected:
    //store cartesian trajectories for each foot
    std::map<const shared_ptr<RobotLimb>, Trajectory3D> limbTrajectories;

    //store reference trajectory for the robot's body frame
    bFrameReferenceMotionPlan bFrameMotionPlan;
    LimbMotionProperties lmProps;
    FootstepPlan fsp;

    //assumption is that walking happens on flat ground here
    double groundHeight = 0;

public:
    /**
         * constructor
         */
    SimpleLocomotionTrajectoryPlanner(const std::shared_ptr<LeggedRobot>& bot) : LocomotionTrajectoryPlanner(bot), bFrameMotionPlan(bot) {
        generateTrajectoriesFromCurrentState();
    }

    void initializeMotionPlan(double dt) {
        //set properties/targets needed to generate body frame motion trajectory
        bFrameMotionPlan.dt = dt;
        bFrameMotionPlan.targetbFrameHeight = groundHeight + trunkHeight;
        bFrameMotionPlan.targetForwardSpeed = speedForward;
        bFrameMotionPlan.targetSidewaysSpeed = speedSideways;
        bFrameMotionPlan.targetTurngingSpeed = turningSpeed;
        bFrameMotionPlan.tStart = simTime;
        bFrameMotionPlan.tEnd = simTime + tPlanningHorizon + tPlanningHorizonBuffer;

        lmProps.stepWidthOffsetX = stepWidthModifier;
        lmProps.swingFootHeight = targetStepHeight;
    }

    void generateBFrameTrajectory() {
        //now generate the motion of the body frame - and do account for the differences between the planned motion of the trunk and the reference motion for the body frame
        bFrameMotionPlan.generateTrajectory();
    }

    void generateSteppingLocations() {
        //and the contact locations for the limbs
        bFrameMotionPlan.populateFootstepPlan(fsp, lmProps, &cpm, groundHeight);
    }

    void generateLimbTrajectories(double dt) {
        //and full motion trajectories for each limb
        for (uint i = 0; i < robot->getLimbCount(); i++) {
            limbTrajectories[robot->getLimb(i)] = fsp.generateLimbTrajectory(robot->getLimb(i), lmProps, simTime, simTime + tPlanningHorizon, dt, groundHeight);
        }
    }

    virtual void generateTrajectoriesFromCurrentState(double dt = 1 / 30.0) {
        initializeMotionPlan(dt);

        generateBFrameTrajectory();

        generateSteppingLocations();

        generateLimbTrajectories(dt);
    }

    virtual P3D getTargetLimbEEPositionAtTime(const std::shared_ptr<RobotLimb>& l, double t) {
        return P3D() + limbTrajectories[l].evaluate_linear(t);
    }

    virtual P3D getTargetTrunkPositionAtTime(double t) {
        return P3D() + bFrameMotionPlan.bFramePosTrajectory.evaluate_linear(t);  // +RBGlobals::worldUp.cross(robot->forward) * 0.05 * sin(5 * t);
    }

    virtual double getTargetTrunkHeadingAtTime(double t) {
        return bFrameMotionPlan.bFrameHeadingTrajectory.evaluate_linear(t) + trunkYaw;
    }

    virtual Quaternion getTargetTrunkOrientationAtTime(double t) {
        return getRotationQuaternion(getTargetTrunkHeadingAtTime(t), V3D(0, 1, 0)) *
               getRotationQuaternion(trunkPitch, RBGlobals::worldUp.cross(robot->getForward())) * getRotationQuaternion(trunkRoll, robot->getForward());
    }

    virtual void drawTrajectories(gui::Shader* shader) {
        for (int i = 0; i < bFrameMotionPlan.bFramePosTrajectory.getKnotCount(); i++)
            drawSphere(P3D() + bFrameMotionPlan.bFramePosTrajectory.getKnotValue(i), 0.02, *shader);

        for (uint i = 0; i < robot->getLimbCount(); i++)
            for (int j = 0; j < limbTrajectories[robot->getLimb(i)].getKnotCount(); j++)
                drawSphere(P3D() + limbTrajectories[robot->getLimb(i)].getKnotValue(j), 0.01, *shader, V3D(1, 1, 0));
    }
};

}  // namespace crl::loco
