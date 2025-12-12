package com.ck4911;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav = new QuestNav();

    // TODO: Insert actual offsets here
    private static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(/* x, y, z, yaw, pitch, roll */);

    private Pose3d latestRobotPose = new Pose3d();

    private static final QuestNavSubsystem INSTANCE = new QuestNavSubsystem();

    public static QuestNavSubsystem getInstance() {
        return INSTANCE;
    }

    private QuestNavSubsystem() {}

    @Override
    public void periodic() {

        // REQUIRED per QuestNav docs
        questNav.commandPeriodic();

        // Read all new pose frames
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            // Grab newest pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();


            // Convert to robot pose
            Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

            // Store for other subsystems
            latestRobotPose = robotPose;
        }
    }

    public Pose3d getLatestRobotPose() {
        return latestRobotPose;
    }
}
