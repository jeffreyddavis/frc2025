package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AlgaeProcessor extends SequentialCommandGroup {

  public AlgaeProcessor(Arm arm, Intake intake, Elevator elevator) {

    addCommands(
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.Processor)),
        Commands.waitUntil(() -> arm.isAtLocation()),
        Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.ProcessorHeight), elevator)
    );

  }
}
