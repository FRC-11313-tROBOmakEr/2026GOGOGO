package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

//import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.LimelightHelpers;

public class Target {
    public Pose2d TargetPose = new Pose2d();
    public static HashMap<Integer, List<Pose2d>> hubMap = new HashMap<>();
    private boolean isValidTarget = false;

    private final Pose2d RED_GOAL = new Pose2d(8.27, 4.035, new Rotation2d());
    private final Pose2d BLUE_GOAL = new Pose2d(4.03, 4.035, new Rotation2d());

    public void updateTargetStatus(int aprilTagsID) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red
                    && ((aprilTagsID >= 2 && aprilTagsID <= 5) || (aprilTagsID >= 8 && aprilTagsID <= 11))) {
                isValidTarget = true;
                TargetPose = RED_GOAL;
            } else if (alliance.get() == Alliance.Blue
                    && ((aprilTagsID >= 18 && aprilTagsID <= 21) || (aprilTagsID >= 24 && aprilTagsID <= 27))) {
                isValidTarget = true;
                TargetPose = BLUE_GOAL;
            } else {
                isValidTarget = false;
            }
        }
    }

    public Rotation2d getAimingRotation(Pose2d robotPose) {
        double deltaX = TargetPose.getX() - robotPose.getX();
        double deltaY = TargetPose.getY() - robotPose.getY();

        return new Rotation2d(Math.atan2(deltaY, deltaX));
    }
//姊姊有看到我嗎
    public double getDistanceToTarget(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(TargetPose.getTranslation());
    }

    public boolean isTargetValid() {
        return isValidTarget;
    }
}
