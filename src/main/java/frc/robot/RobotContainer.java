// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.addons.QuestNav;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.Targets;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // public final QuestNav NavigationSystem = new QuestNav();

  public final Elevator SeaElevator = new Elevator();
  public final Arm theArm = new Arm(SeaElevator);
  public final Intake AlgaeYoinker = new Intake();
  private final SendableChooser<Command> autoChooser;
  public final QuestNav insanity = new QuestNav();
  // public final Hopper MCHopper = new Hopper();
  public final Climber CageAscender = new Climber();

  public final Drive drive;
  //public final Vision vision;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_testController =
      new CommandXboxController(OperatorConstants.kTestingControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("GoToL1", new GoToScore(theArm, SeaElevator, Targets.L1));
    NamedCommands.registerCommand("score", new score(AlgaeYoinker));

    NamedCommands.registerCommand("GoToL4", new GoToScore(theArm, SeaElevator, Targets.L4));
    NamedCommands.registerCommand("dunk", new DunkAuto(AlgaeYoinker, SeaElevator, null));
    NamedCommands.registerCommand("GoToRest", new GoToRest(theArm, SeaElevator));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

//        vision =
//            new Vision(
//                drive::addVisionMeasurement,
//                new VisionIOPhotonVision(
//                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
//                new VisionIOPhotonVision(
//                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
 //               new VisionIOLimelight(VisionConstants.camera2Name, drive::getRotation),
  //              new VisionIOLimelight(VisionConstants.camera3Name, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
/*
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
 */
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
   //     vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    defaultCommands();
    configureBindings();
  }

  private void defaultCommands() {
    // SeaElevator.setDefaultCommand(Commands.run(() -> SeaElevator.GoToTarget(Elevator.Targets.L1),
    // SeaElevator));
    SeaElevator.setDefaultCommand(Commands.run(() -> SeaElevator.stop(), SeaElevator));
    theArm.setDefaultCommand(Commands.run(() -> theArm.stop(), theArm));
    AlgaeYoinker.setDefaultCommand(Commands.run(() -> AlgaeYoinker.stop(), AlgaeYoinker));
    // MCHopper.setDefaultCommand(Commands.run(() -> MCHopper.stop(), MCHopper));
    CageAscender.setDefaultCommand(Commands.run(() -> CageAscender.idle(), CageAscender));
  }

  private void configureBindings() {

    m_testController.a().whileTrue(Commands.run(() -> CageAscender.releaseCage(), CageAscender));
    //  m_testController.x().whileTrue(Commands.run(() -> SeaElevator.testUp(), SeaElevator));

    // m_testController.y().whileTrue(Commands.run(() -> SeaElevator.testDown(), SeaElevator));
    m_testController.povUp().whileTrue(Commands.run(() -> AlgaeYoinker.testIn(), AlgaeYoinker));
    m_testController.povDown().whileTrue(Commands.run(() -> AlgaeYoinker.testOut(), AlgaeYoinker));

    m_testController
        .povRight()
        .whileTrue(Commands.run(() -> AlgaeYoinker.holdAlgae(), AlgaeYoinker));

    m_testController.povLeft().whileTrue(Commands.run(() -> theArm.hold(), theArm));

    // m_testController.leftBumper().whileTrue(Commands.run(() -> MCHopper.testIn(), MCHopper));
    // m_testController.rightBumper().whileTrue(Commands.run(() -> MCHopper.testOut(), MCHopper));

    m_testController
        .leftTrigger()
        .whileTrue(Commands.run(() -> CageAscender.testDown(), CageAscender));
    m_testController
        .rightTrigger()
        .whileTrue(Commands.run(() -> CageAscender.testUp(), CageAscender));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_testController.getLeftY(),
            () -> -m_testController.getLeftX(),
            () -> -m_testController.getRightX()));

    // m_driverController.button(Constants.controller.ShootButton).onTrue(new Dunk(AlgaeYoinker,
    // SeaElevator, swerve));
    // m_driverController.button(Constants.controller.EjectButton).onTrue(new Eject(AlgaeYoinker));
    // m_driverController.button(Constants.controller.GoToL1).onTrue(new GoToScore(theArm,
    // SeaElevator, Targets.L1));
    // m_driverController.button(Constants.controller.GoToL2).onTrue(new GoToScore(theArm,
    // SeaElevator, Targets.L2));
    // m_driverController.button(Constants.controller.GoToL3).onTrue(new GoToScore(theArm,
    // SeaElevator, Targets.L3));
    // m_driverController.button(Constants.controller.GoToL4).onTrue(new GoToScore(theArm,
    // SeaElevator, Targets.L4));
    // m_driverController.button(Constants.controller.ThrowAlgaeTheButton).onTrue(new
    // ThrowAlgae(theArm, AlgaeYoinker, SeaElevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // ...

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return null;
  }
}
