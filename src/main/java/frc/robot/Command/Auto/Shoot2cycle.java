package frc.robot.Command.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;
public class Shoot2cycle extends SequentialCommandGroup {

  public final Shooter shooter;
  public final Intake intake;

  public Shoot2cycle (Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(shooter);
    addRequirements(intake);
    addCommands(
    new ParallelCommandGroup(
        new InstantCommand(() -> shooter.Shooter_Out(), shooter),
        new WaitCommand(5.0)
    ),
  
    new ParallelCommandGroup(
        new InstantCommand(() -> intake.intakeOut(), intake),
        new WaitCommand(5.0)
    ),
    
    new ParallelCommandGroup(
        new InstantCommand(() -> shooter.Shooter_Out(), shooter),
        new WaitCommand(5.0)
    ),

    new InstantCommand(() -> intake.intakeOut(), intake)
    );
    }   
  }
