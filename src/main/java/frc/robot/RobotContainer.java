// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import javax.crypto.SealedObject;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.addons.QuestNav;
import frc.robot.addons.ScoringLocations;
import frc.robot.Constants;
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

  public final Elevator SeaElevator = new Elevator();
  public final Arm theArm = new Arm(SeaElevator);
  public final Intake AlgaeYoinker = new Intake();
  private final SendableChooser<Command> autoChooser;
  public final QuestNav insanity = new QuestNav();
  public final Climber CageAscender = new Climber();
  public final Drive drive;

  public final Vision vision;

  private GyroIOPigeon2 pigeon = new GyroIOPigeon2();

  @AutoLogOutput
  public int currentTargetLevel = 4;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_testController =
      new CommandXboxController(OperatorConstants.kTestingControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                pigeon,
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                /*new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),*/
                new VisionIOLimelight(VisionConstants.camera2Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera3Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera4Name, drive::getRotation)
                 );

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    // new VisionIOPhotonVisionSim(
               //     VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                //new VisionIOPhotonVisionSim(
                 //   VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
 
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    
    NamedCommands.registerCommand("GoToL1", new GoToL1(theArm, SeaElevator));
   // NamedCommands.registerCommand("score", new score(AlgaeYoinker));

    NamedCommands.registerCommand("GoToL4", new GoToL4(theArm, SeaElevator));
    
    NamedCommands.registerCommand("GoToL3", new GoToL3(theArm, SeaElevator));
    NamedCommands.registerCommand("dunk", new DunkAuto(AlgaeYoinker, SeaElevator, theArm));
    NamedCommands.registerCommand("intake", new IntakeAuto(AlgaeYoinker, theArm, SeaElevator));
    
    NamedCommands.registerCommand("afterintake", new AfterIntakeAuto(AlgaeYoinker, theArm, SeaElevator));
    NamedCommands.registerCommand("resetQuest", Commands.runOnce(() -> resetInsanity()));
    NamedCommands.registerCommand("Stow", new Stow(theArm, SeaElevator, AlgaeYoinker));
    NamedCommands.registerCommand("resendPosition", Commands.runOnce(() -> drive.allowUpdates = true));

    NamedCommands.registerCommand("autoScore", Commands.defer(() -> new LineUpGoToTarget(drive, theArm, SeaElevator, this, m_driverController), Set.of(drive, theArm, SeaElevator)));
    NamedCommands.registerCommand("autoIntake", new LineUpIntake(drive, AlgaeYoinker, theArm, SeaElevator, m_driverController));
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);


    onStart();
    // Configure the trigger bindings
    defaultCommands();
    configureBindings();
    
  }

  public void onStart() {
    theArm.stop();
    SeaElevator.stop();
    AlgaeYoinker.stop();

  }

  private void defaultCommands() {

    CageAscender.setDefaultCommand(Commands.run(() -> CageAscender.idle(), CageAscender));
  //  SeaElevator.setDefaultCommand(Commands.run(() -> SeaElevator.stop(), SeaElevator));


    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getRawAxis(1),
            () -> -m_driverController.getRawAxis(0),
            () -> -(m_driverController.getRawAxis(2) * spinSpeed)));
  }
  


  public void resetInsanity() {
    insanity.resetPose(drive.getPose());
    vision.isAllowedToSend = false;
  }

  public void toggleVision() {
    vision.isAllowedToSend = !vision.isAllowedToSend;
  }

  public void resetGyro() {
    drive.resetGyro();
  }

  public double spinSpeed = 1;
  public void spinFast() {
    spinSpeed = 4;
  }
  public void spinNormal() {
    spinSpeed = 1;
  }


  private void configureBindings() {


     


    m_driverController.button(5).onTrue(new GetLowAlgae(theArm, AlgaeYoinker, SeaElevator));
    m_driverController.button(6).onTrue(new GetHighAlgae(theArm, AlgaeYoinker, SeaElevator));
    m_driverController.button(10).onTrue(new AlgaeProcessor(theArm, AlgaeYoinker, SeaElevator)); 
    m_driverController.button(7).onTrue(new ThrowAlgae(theArm, AlgaeYoinker, SeaElevator));

   m_driverController.button(4).onTrue(new Stow(theArm, SeaElevator, AlgaeYoinker));
   m_driverController.button(3).onTrue(Commands.defer(() -> new LineUpGoToTarget(drive, theArm, SeaElevator, this, m_driverController),Set.of(drive, theArm, SeaElevator)));
   
   //m_driverController.button(3).onTrue(new GoToTarget(this, theArm, SeaElevator));
   
   m_driverController.button(1).onTrue(new Dunk(AlgaeYoinker, SeaElevator, theArm).andThen(new Stow(theArm, SeaElevator, AlgaeYoinker)));

   m_driverController.button(7).onTrue(new GetFloorAlgae(theArm, AlgaeYoinker, SeaElevator));

   m_driverController.button(8).onTrue(Commands.runOnce(() -> pigeon.resetGyro()));//.andThen(Commands.runOnce(() -> resetGyro())));

   m_driverController.button(9)
            .whileTrue(Commands.runEnd(() -> spinFast(), () -> spinNormal())); 

   m_driverController.povUp().onTrue(Commands.runOnce(() -> {
    this.currentTargetLevel+=1;
    if (this.currentTargetLevel > 4) this.currentTargetLevel = 1;
   } ).andThen(new GoToTarget(this, theArm, SeaElevator)));

   m_driverController.povDown().onTrue(Commands.runOnce(() -> {
    this.currentTargetLevel-=1;
    if (this.currentTargetLevel < 1) this.currentTargetLevel = 4;
   } ).andThen(new GoToTarget(this, theArm, SeaElevator)));

   m_driverController
      .button(2)
      .onTrue(new LineUpIntake(drive, AlgaeYoinker, theArm, SeaElevator, m_driverController));
   
      // isRed:  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red

   // m_testController.rightBumper().whileTrue(Commands.run(() -> CageAscender.releaseCage(), CageAscender));

   //m_testController.rightBumper().onTrue(new ReefLineUp(drive, theArm, SeaElevator));
   m_testController.y().whileTrue(Commands.run(() -> SeaElevator.testUp(), theArm));
   m_testController.x().whileTrue(Commands.run(() -> SeaElevator.testDown(), theArm));
  
   m_testController.b().whileTrue(Commands.run(() -> theArm.testUp(), theArm));
   
   m_testController.a().whileTrue(Commands.run(() -> theArm.testDown(), theArm));


    m_testController
      .povLeft()
      .whileTrue(Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, .5, 0)), drive));

    m_testController
        .povRight()
        .whileTrue(Commands.run(() -> AlgaeYoinker.holdAlgae(), AlgaeYoinker));

    m_testController
        .pov(0)
        .whileTrue(Commands.run(() -> CageAscender.testDown(), CageAscender));
    m_testController
        .pov(180)
        .whileTrue(Commands.run(() -> CageAscender.testUp(), CageAscender));

    

    m_driverController
      .button(14)
      .onTrue(new Eject(AlgaeYoinker));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // ...

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return null;
  }
}
