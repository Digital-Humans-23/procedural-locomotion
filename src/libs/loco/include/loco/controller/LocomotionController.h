#pragma once

#include <crl-basic/gui/renderer.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>

namespace crl::loco {

/**
 * Abstract class for legged locomotion controllers. It has trajectory planner
 * as a member. Derived class of this class should implement planning and
 * control logic.
 */
class LocomotionController {
public:
    std::shared_ptr<LocomotionTrajectoryPlanner> planner = nullptr;

public:
    LocomotionController(const std::shared_ptr<LocomotionTrajectoryPlanner>& planner) : planner(planner) {}

    virtual ~LocomotionController(){};

    /**
     * Generate motion trajectory with timestep size dt.
     */
    virtual void generateMotionTrajectories(double dt) = 0;

    /**
     * Compute and apply control signal with timestep size dt.
     */
    virtual void computeAndApplyControlSignals(double dt) = 0;

    /**
     * Call this function after applying control signal.
     */
    virtual void advanceInTime(double dt) = 0;

    /**
     * Draw control options to ImGui.
     */
    virtual void drawControllerOptions() {
        ImGui::Text("No option available");
    };

    /**
     * Draw some useful information for debugging.
     * e.g. contact, velocity, acceleration etc.
     */
    virtual void drawDebugInfo(gui::Shader* shader) = 0;

    /**
     * Plot some useful information for debugging.
     * e.g. joint torque etc.
     */
    virtual void plotDebugInfo() = 0;
};

}  // namespace crl::loco
