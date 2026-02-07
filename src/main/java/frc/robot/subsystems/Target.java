package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.PointWheelsAt;

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
    private boolean targetisblue;
    private final PointWheelsAt point = new PointWheelsAt();

    private final Pose2d REDPose = new Pose2d(8.27, 4.035, new Rotation2d(180));
    private final Pose2d BLUEPose = new Pose2d(4.03, 4.035, new Rotation2d(0));

    public void updateTargetStatus(double aprilTagsID) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red
                    && ((aprilTagsID >= 2 && aprilTagsID <= 5) || (aprilTagsID >= 8 && aprilTagsID <= 11))) {
                isValidTarget = true;
                TargetPose = REDPose;
                targetisblue = false;

            } else if (alliance.get() == Alliance.Blue
                    && ((aprilTagsID >= 18 && aprilTagsID <= 21) || (aprilTagsID >= 24 && aprilTagsID <= 27))) {
                isValidTarget = true;
                TargetPose = BLUEPose;
                targetisblue = true;
            } else {
                isValidTarget = false;
            }
        }
    }

    public Rotation2d getAimingRotation(Pose2d robotPose) {

        double redx = REDPose.getX() - robotPose.getX();
        double redy = REDPose.getY() - robotPose.getY();

        double bluex = BLUEPose.getX() - robotPose.getX();
        double bluey = BLUEPose.getY() - robotPose.getY();

        if (!targetisblue) {
            return new Rotation2d(Math.atan2(redy, redx));
        } 
        else {
            return new Rotation2d(Math.atan2(bluey, bluex));
        }
    }

    public double getDistanceToTarget(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(TargetPose.getTranslation());
    }

    public boolean isTargetValid() {
        return isValidTarget;
    }


}
