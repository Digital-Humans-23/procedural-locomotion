#pragma once

#include <crl-basic/gui/application.h>

#include "loco/controller/KinematicTrackingController.h"
#include "loco/planner/GaitPlanner.h"
#include "loco/planner/SimpleLocomotionTrajectoryPlanner.h"
#include "menu.h"

namespace locoApp {

class App : public crl::gui::ShadowApplication {
public:
    App() : crl::gui::ShadowApplication("Locomotion App") {
        this->showConsole = true;
        this->automanageConsole = true;
        this->showPlots = true;
        this->show_world_frame = false;

        // setup
        setupRobotAndController();
    }

    ~App() override = default;

    void process() override {
        // add gait plan
        controller_->planner->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));

        double simTime = 0;
        while (simTime < 1.0 / targetFramerate) {
            simTime += dt;
            controller_->computeAndApplyControlSignals(dt);
            controller_->advanceInTime(dt);
        }

        // generate motion plan
        controller_->generateMotionTrajectories();

        // adjust light and camera
        const auto &center = robot_->getTrunk()->getWorldCoordinates(crl::P3D());
        if (followRobotWithCamera) {
            camera.target.x = (float)center.x;
            camera.target.z = (float)center.z;
        }
        light.target.x() = center.x;
        light.target.z() = center.z;
    }

    void restart() override {
        setupRobotAndController();
    }

    void drawObjectsWithShadows(const crl::gui::Shader &shader) override {
        ShadowApplication::drawObjectsWithShadows(shader);
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        robot_->draw(shader);
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        robot_->draw(shader);

        if (drawDebugInfo)
            controller_->drawDebugInfo(&basicShader);
    }

    bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            processIsRunning = !processIsRunning;
        }
        if (key == GLFW_KEY_ENTER) {
            if (!processIsRunning)
                process();
        }

        // joystick command
        bool dirty = false;
        if (key == GLFW_KEY_UP) {
            planner_->speedForward += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_DOWN) {
            planner_->speedForward -= 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_LEFT) {
            planner_->turningSpeed += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_RIGHT) {
            planner_->turningSpeed -= 0.1;
            dirty = true;
        }

        if (dirty) {
            planner_->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));
            controller_->generateMotionTrajectories();
            return true;
        }

        return false;
    }

    template <typename T>
    void drawComboMenu(const std::string &menuName, const std::vector<T> &options, uint &selected) {
        if (ImGui::BeginCombo(menuName.c_str(), options[selected].name.c_str())) {
            for (uint n = 0; n < options.size(); n++) {
                bool is_selected = (selected == n);
                if (ImGui::Selectable(options[n].name.c_str(), is_selected)) {
                    selected = n;
                    setupRobotAndController();
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        ImGui::Checkbox("Follow Robot with Camera", &followRobotWithCamera);
        if (ImGui::CollapsingHeader("Character")) {
            drawComboMenu("Model##character", modelOptions, selectedModel);
        }
        if (ImGui::CollapsingHeader("Draw")) {
            if (ImGui::Checkbox("Show meshes", &robot_->showMeshes)) {
                robot_->showSkeleton = !robot_->showMeshes;
            }
            ImGui::Checkbox("Show end effectors", &robot_->showEndEffectors);
            ImGui::Checkbox("Draw debug info", &drawDebugInfo);
        }

        ImGui::End();

        planner_->visualizeContactSchedule();
        planner_->visualizeParameters();
    }

    void drawImPlot() override {
        crl::gui::ShadowApplication::drawImPlot();

        ImGui::Begin("Plots");
        // here, you can draw plots
        ImGui::End();
    }

    virtual bool drop(int count, const char **fileNames) override {
        return true;
    }

private:
    void setupRobotAndController() {
        const auto &m = modelOptions[selectedModel];
        const char *rbsFile = m.filePath.c_str();
        robot_ = std::make_shared<crl::loco::LeggedRobot>(rbsFile);
        robot_->setRootState(crl::P3D(0, m.baseTargetHeight, 0));
        if (m.type == ModelOption::Type::DOG) {
            robot_->showMeshes = false;
            robot_->showSkeleton = true;
            gaitPlanner_ = std::make_shared<crl::loco::QuadrupedalGaitPlanner>();
        } else {
            gaitPlanner_ = std::make_shared<crl::loco::BipedalGaitPlanner>();
        }

        // add legs
        for (int i = 0; i < m.legs.size(); i++) {
            robot_->addLimb(m.legs[i].first, m.legs[i].second);
        }

        // setup planner and controller
        planner_ = std::make_shared<crl::loco::SimpleLocomotionTrajectoryPlanner>(robot_);
        planner_->trunkHeight = m.baseTargetHeight;
        planner_->targetStepHeight = m.swingFootHeight;
        controller_ = std::make_shared<crl::loco::KinematicTrackingController>(planner_);

        // generate plan
        planner_->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));
        controller_->generateMotionTrajectories();
    }

public:
    // simulation
    std::shared_ptr<crl::loco::LeggedRobot> robot_ = nullptr;
    std::shared_ptr<crl::loco::GaitPlanner> gaitPlanner_ = nullptr;
    std::shared_ptr<crl::loco::LocomotionTrajectoryPlanner> planner_ = nullptr;
    std::shared_ptr<crl::loco::KinematicTrackingController> controller_ = nullptr;

    // parameters
    double dt = 1 / 60.0;

    // options
    uint selectedModel = 0;
    bool followRobotWithCamera = true;
    bool drawDebugInfo = true;
};

}  // namespace locoApp
