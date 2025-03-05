package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.addons.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;

public class AutoStationIntake extends SequentialCommandGroup {

  public AutoStationIntake(Intake intake, Arm arm, Elevator elevator, Drive drive) {



    addCommands(
        PathFind.ToPose(FieldConstants.CoralStation.rightCenterFace.rotateBy(new Rotation2d(180))),
        new StationIntake(intake, arm, elevator)
      
      );
  }
}
