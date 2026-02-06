
package frc.robot.Command.Shoot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Target;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;





public class Shooterout extends SequentialCommandGroup {
  public Shooterout(Shooter shooter, Target target) {
    addRequirements(shooter);
    addCommands(
    new InstantCommand(() -> target.getAimingRotation(LimelightHelpers.getBotPose2d(VisionConstants.LLName))),
    new InstantCommand(() -> target.updateTargetStatus(LimelightHelpers.getFiducialID(VisionConstants.LLName))),
    new RunCommand(() -> shooter.Shooter_Out(),shooter),
    new InstantCommand(() -> shooter.angle_out(),shooter),
    Commands.waitSeconds(0.8),
    new RunCommand(() -> shooter.IndexrWorking(),shooter)
    );
  }
}
