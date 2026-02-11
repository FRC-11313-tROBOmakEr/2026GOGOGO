
package frc.robot.command.shoot;

import frc.robot.subsystems.Shooter;
import frc.robot.LimelightHelpers;
import frc.robot.Target;
import frc.robot.Constants.VisionConstants;
//import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;





public class Shootout extends SequentialCommandGroup {
  public PointWheelsAt point = new PointWheelsAt();
  public Target target = new Target();
  public Shootout(Shooter shooter) {

    addRequirements(shooter);
    addCommands(
    // new InstantCommand(() -> target.getTargetStatus(LimelightHelpers.getFiducialID(VisionConstants.LLName))),
    //new RunCommand(() -> swerve.applyRequest(() -> point.withModuleDirection(target.getAimingRotation(LimelightHelpers.getBotPose2d(VisionConstants.LLName))))),
    new InstantCommand(() -> shooter.Shooter_Out(),shooter),
    Commands.waitSeconds(0.8),
    new InstantCommand(() -> shooter.indexerRun()),
    new InstantCommand(() -> shooter.conveyorRun())
        
    );
  }
}
