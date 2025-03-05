package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.Stow;

public class Dunk extends SequentialCommandGroup {

  public Dunk(Intake intake, Elevator elevator, Drive drive, Arm arm) {

    addCommands(Commands.either(
      new SequentialCommandGroup(
        Commands.runOnce(() -> intake.ShootAlgae(), intake),
        
        Commands.waitSeconds(.5),
        
        Commands.runOnce(() -> drive.goBackward(1), drive),
        
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> intake.stop(), intake),

        Commands.runOnce(() -> drive.stop(), drive),
        new Stow(arm, elevator, intake)

      )
      , 
      new SequentialCommandGroup(
        
        Commands.runOnce(() -> arm.dunk(), arm),
        Commands.waitSeconds(.1),
        Commands.runOnce(() -> elevator.dunk(), elevator),
        
        Commands.waitSeconds(.4),
        Commands.runOnce(() -> intake.ShootCoral(), intake),
        Commands.runOnce(() -> drive.goBackward(.5), drive),
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> intake.stop(), intake),
        Commands.runOnce(() -> drive.stop(), drive),
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
        new Stow(arm, elevator, intake)
        
      ), () -> (elevator.getLocation() <= (Constants.Elevator.HeightL2 - 1000)) 
    ));


    
  }
}
