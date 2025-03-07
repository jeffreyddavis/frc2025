package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;

public class DunkAuto extends SequentialCommandGroup {

  public DunkAuto(Intake intake, Elevator elevator, Drive drive, Arm arm) {

    addRequirements(intake, elevator);
    addCommands(
        Commands.runOnce(() -> arm.dunk(), arm),
        Commands.waitSeconds(.1),
        Commands.runOnce(() -> elevator.DunkAuto(), elevator),
        Commands.runOnce(() -> intake.ShootCoralAuto(), intake),
        
        //Commands.runOnce(() -> drive.goBackward(.5), drive),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> intake.stop(), intake),
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm)
        );
  }
}
