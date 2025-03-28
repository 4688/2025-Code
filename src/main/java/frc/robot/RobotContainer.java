// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.coral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick buttons = new CommandJoystick(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
                                                                                
  private final Arm arm = new Arm();
  private final limelight limelight = new limelight();
  private final coral help = new coral(arm);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<Command> autoChooser;
  public RobotContainer()
  {
    
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    //NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("lvl1",arm.gotolevel1());
    NamedCommands.registerCommand("lvl2",arm.gotolevel2());
    NamedCommands.registerCommand("lvl3",arm.gotolevel3());
    NamedCommands.registerCommand("Algaelvl",arm.gotoAlgaelevel1());
    NamedCommands.registerCommand("shootCoral",help);
    NamedCommands.registerCommand("goDown", arm.gotoLow());
     autoChooser = AutoBuilder.buildAutoChooser();
     SmartDashboard.putData("Autos",autoChooser);

 
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
   // Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        //driveDirectAngleKeyboard);
    Command zeroNavx = Commands.runOnce(() -> drivebase.zeroGyro());
    Command slowDown= Commands.runOnce(()-> drivebase.getSwerveDrive().setMaximumAllowableSpeeds(Constants.MAX_SPEED*0.25,1));
    Command slowDownHalf = Commands.runOnce(()-> drivebase.getSwerveDrive().setMaximumAllowableSpeeds(Constants.MAX_SPEED*0.5, 2));
    Command normalSpeed= Commands.runOnce(()-> drivebase.getSwerveDrive().setMaximumAllowableSpeeds(Constants.MAX_SPEED,4));
      //Trigger armHigh = new Trigger(null, arm.slowItdown());
      //armHigh.whileTrue(slowDownHalf).whileFalse(normalSpeed);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      driverXbox.button(7).whileTrue(zeroNavx);
      driverXbox.rightTrigger().whileTrue(slowDown).whileFalse(normalSpeed);
      driverXbox.rightBumper().whileTrue(arm.goUp()).whileFalse(arm.ElevatorHold());
      driverXbox.leftBumper().whileTrue(arm.goDown()).whileFalse(arm.ElevatorHold());
      //algae intake/outtake
      driverXbox.b().whileTrue(arm.AlgaeIntake()).whileFalse(arm.AlgaeHeld());
      driverXbox.x().whileTrue(arm.AlgaeOuttake()).whileFalse(arm.AlgaeHeld());
      driverXbox.povLeft().whileTrue(arm.coralleft()).whileFalse(arm.coralstop());
      driverXbox.povRight().whileTrue(arm.coralright()).whileFalse(arm.coralstop());
      // driverXbox.y().whileTrue(arm.climbUp()).whileFalse(arm.climbStop());
      //driverXbox.a().whileTrue(arm.climbDown()).whileFalse(arm.climbStop());
      buttons.button(1).whileTrue(arm.gotolevel1()).whileFalse(arm.ElevatorHold());
      buttons.button(2).whileTrue(arm.gotolevel2()).whileFalse(arm.ElevatorHold());
      buttons.button(3).whileTrue(arm.gotolevel3()).whileFalse(arm.ElevatorHold());
      buttons.button(4).whileTrue(arm.gotoAlgaelevel1()).whileFalse(arm.ElevatorHold());
      buttons.button(5).whileTrue(arm.gotoAlgaelevel2()).whileFalse(arm.ElevatorHold());
      //driverXbox.button(8).whileTrue(drivebase.driveCommand());
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}