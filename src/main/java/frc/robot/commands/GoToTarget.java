package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class GoToTarget extends SequentialCommandGroup {

  public GoToTarget(RobotContainer rob, Arm arm, Elevator elevator) {
    addCommands(
    Commands.select(Map.ofEntries(
        Map.entry(1, new GoToL1(arm,elevator)),
        Map.entry(2, new GoToL2(arm,elevator)),
        Map.entry(3, new GoToL3(arm,elevator)),
        Map.entry(4, new GoToL4(arm,elevator))
        
    ), () -> { return rob.currentTargetLevel;})
    );
    
  }
}
