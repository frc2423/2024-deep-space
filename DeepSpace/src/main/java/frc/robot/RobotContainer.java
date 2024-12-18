// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.ClawCommands;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */

public class RobotContainer {
  String deployDirectory = (Robot.isSimulation()) ? "neo" : "swerve";
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), deployDirectory));
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  IntakeCommands intakeCommands = new IntakeCommands(intakeSubsystem, elevatorSubsystem);
  ClawSubsystem clawSubsystem = new ClawSubsystem();
  ClawCommands clawCommands = new ClawCommands(clawSubsystem);

  public static ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(7);

  // A chooser for autonomous commands
  SendableChooser<String> m_chooser = new SendableChooser<>();

  XboxController driverXbox = new XboxController(0); //BAZINFA
  XboxController operator = new XboxController(1);//hi :-)
  boolean isPanel = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    SmartDashboard.putData("intake", intakeSubsystem);

    Command driveFieldOrientedAngularVelocity = getTeleopDriveCommand();

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    SmartDashboard.putData("Elevator", elevator);
  }

  private Command getTeleopDriveCommand() {
    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
        () -> {
          double y = MathUtil.applyDeadband(
              -driverXbox.getLeftY(),
              OperatorConstants.LEFT_Y_DEADBAND);
          if (PoseTransformUtils.isRedAlliance()) {
            y *= -1;
          }
          return m_yspeedLimiter.calculate(y);
        },
        () -> {
          double x = MathUtil.applyDeadband(
              -driverXbox.getLeftX(),
              OperatorConstants.LEFT_X_DEADBAND);
          if (PoseTransformUtils.isRedAlliance()) {
            x *= -1;
          }
          return m_xspeedLimiter.calculate(x);
        },
        () -> -driverXbox.getRightX());
    return driveFieldOrientedAngularVelocity; // :P
  }

  private void configureBindings() {
    new JoystickButton(driverXbox, XboxController.Button.kStart.value)
        .onTrue((new InstantCommand(drivebase::zeroGyro)));

    // new JoystickButton(driverXbox, XboxController.Button.kA.value)
    //      .onTrue(elevator.goDown());

    //  new JoystickButton(driverXbox, XboxController.Button.kY.value)
    //      .onTrue(elevator.goUp());
    new Trigger(() -> operator.getPOV() == 270).whileTrue(elevator.goToSetpoint((isPanel) ? Constants.SetpointConstants.ROCKET_BOTTOM_PANEL : Constants.SetpointConstants.ROCKET_BOTTOM_BALLZ));
    new Trigger(() -> operator.getPOV() == 0).whileTrue(elevator.goToSetpoint((isPanel) ? Constants.SetpointConstants.ROCKET_MIDDLE_PANEL : Constants.SetpointConstants.ROCKET_MIDDLE_BALLZ));
    new Trigger(() -> operator.getPOV() == 90).whileTrue(elevator.goToSetpoint((isPanel) ? Constants.SetpointConstants.ROCKET_TOP_PANEL : Constants.SetpointConstants.ROCKET_TOP_BALLZ));
    // new Trigger(() -> operator.getPOV() == 0).whileTrue(elevator.goUp());

    
  

   
    new JoystickButton(driverXbox, XboxController.Button.kY.value)
        .onTrue(intakeCommands.intakeIn());

    new JoystickButton(driverXbox, XboxController.Button.kX.value)
        .onTrue(intakeCommands.intakeOut());

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .onTrue(elevator.goLittleUp());

     new JoystickButton(operator, XboxController.Button.kRightBumper.value)
        .onTrue(elevator.goLittleDown());
    
    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
        .whileTrue(intakeCommands.intakeStop());

    new JoystickButton(driverXbox, XboxController.Button.kA.value)
        .whileTrue(clawCommands.clawRelease());

    new JoystickButton(driverXbox, XboxController.Button.kB.value)
        .whileTrue(clawCommands.clawStop());
  }
  

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAuto(m_chooser.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void zeroGyro() {
    drivebase.zeroGyro();
  }
}


