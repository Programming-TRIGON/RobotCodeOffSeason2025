// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandfactories.CoralCollectionCommands;
import frc.trigon.robot.commands.commandfactories.CoralEjectionCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectPoseEstimator;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.climber.Climber;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.elevator.Elevator;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.intake.Intake;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.transporter.Transporter;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;
import lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final PoseEstimator ROBOT_POSE_ESTIMATOR = new PoseEstimator();
    public static final ObjectPoseEstimator CORAL_POSE_ESTIMATOR = new ObjectPoseEstimator(
            CameraConstants.OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS,
            ObjectPoseEstimator.DistanceCalculationMethod.ROTATION_AND_TRANSLATION,
            SimulatedGamePieceConstants.GamePieceType.CORAL,
            CameraConstants.OBJECT_DETECTION_CAMERA
    );
    public static final Swerve SWERVE = new Swerve();
    public static final Arm ARM = new Arm();
    public static final Climber CLIMBER = new Climber();
    public static final Elevator ELEVATOR = new Elevator();
    public static final Intake INTAKE = new Intake();
    public static final Transporter TRANSPORTER = new Transporter();

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        initializeGeneralSystems();
        buildAutoChooser();
        configureBindings();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(GeneralCommands.getFieldRelativeDriveCommand());
        ARM.setDefaultCommand(ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.REST));
        CLIMBER.setDefaultCommand(ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.REST));
        ELEVATOR.setDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST));
        INTAKE.setDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REST));
        TRANSPORTER.setDefaultCommand(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.REST));
    }

    /**
     * Initializes the general systems of the robot.
     * Some systems need to be initialized at the start of the robot code so that others can use their functions.
     * For example, the LEDConstants need to be initialized so that the other systems can use them.
     */
    private void initializeGeneralSystems() {
        Flippable.init();
        LEDConstants.init();
        PathPlannerConstants.init();
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());

        OperatorConstants.SPAWN_CORAL_TRIGGER.onTrue(new InstantCommand(() -> SimulationFieldHandler.updateCoralSpawning(new Pose3d(ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose()))));
        OperatorConstants.CORAL_COLLECTION_TRIGGER.whileTrue(CoralCollectionCommands.getCoralCollectionCommand());
        OperatorConstants.DRIVER_CONTROLLER.rightTrigger().whileTrue(CoralEjectionCommands.getCoralEjectionCommand());
        OperatorConstants.DRIVER_CONTROLLER.a().whileTrue(CoralCollectionCommands.getLoadCoralCommand());
        OperatorConstants.DRIVER_CONTROLLER.x().whileTrue(ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.SCORE_L4));
    }

    private void configureSysIDBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(Commands.idle(subsystem));
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}