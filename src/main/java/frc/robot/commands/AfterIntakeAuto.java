package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AfterIntakeAuto extends SequentialCommandGroup {

  public AfterIntakeAuto(Intake intake) {

    addRequirements(intake);
    addCommands(

      Commands.waitSeconds(.8),
      
      Commands.runOnce(() -> intake.stop(), intake)   
    );
  }
}
//