package frc.trigon.robot.subsystems.armelevator;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.endeffector.EndEffectorCommands;
import frc.trigon.robot.subsystems.endeffector.EndEffectorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.commands.ArmCalibrationCommand;
import lib.commands.ExecuteEndCommand;
import lib.commands.GearRatioCalculationCommand;
import lib.commands.NetworkTablesCommand;

import java.util.Set;
import java.util.function.Supplier;

public class ArmElevatorCommands {
    public static Command getDebuggingCommand(boolean ignoreConstraints) {
        return new NetworkTablesCommand(
                (targetArmAngleDegrees, targetElevatorPositionMeters) -> {
                    RobotContainer.ARM_ELEVATOR.setTargetArmAngle(Rotation2d.fromDegrees(targetArmAngleDegrees), ignoreConstraints);
                    RobotContainer.ARM_ELEVATOR.setTargetElevatorPositionMeters(targetElevatorPositionMeters, ignoreConstraints);
                },
                true,
                Set.of(RobotContainer.ARM_ELEVATOR),
                "Debugging/ArmTargetPositionDegrees",
                "Debugging/ElevatorTargetPositionMeters"
        );
    }

    public static Command getArmCalibrationCommand() {
        return new ArmCalibrationCommand(
                () -> RobotContainer.ARM_ELEVATOR.getCurrentArmAngle().getRotations(),
                voltage -> ArmElevatorConstants.ARM_MASTER_MOTOR.setControl(new VoltageOut(voltage).withEnableFOC(true)),
                RobotContainer.ARM_ELEVATOR
        );
    }

    public static Command getArmGearRatioCalulationCommand() {
        return new GearRatioCalculationCommand(
                ArmElevatorConstants.ARM_MASTER_MOTOR,
                ArmElevatorConstants.ANGLE_ENCODER,
                0.5,
                RobotContainer.ARM_ELEVATOR
        );
    }

    public static Command resetElevatorPositionCommand() {
        return new ParallelCommandGroup(
                new ExecuteEndCommand(
                        () -> RobotContainer.ARM_ELEVATOR.setTargetArmState(ArmElevatorConstants.ArmElevatorState.ZERO_ELEVATOR, false),
                        () -> {
                        }
                ).until(() -> RobotContainer.ARM_ELEVATOR.atState(ArmElevatorConstants.ArmElevatorState.ZERO_ELEVATOR)),
                new ExecuteEndCommand(
                        () -> RobotContainer.ARM_ELEVATOR.setElevatorVoltage(OperatorConstants.DRIVER_CONTROLLER.getRightY() * 2),
                        () -> {
                        },
                        RobotContainer.ARM_ELEVATOR
                ),
                SwerveCommands.getOpenLoopFieldRelativeDriveCommand(() -> 0, () -> 0, () -> 0),
                EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.EJECT)
        ).finallyDo(() -> {
            RobotContainer.ARM_ELEVATOR.resetElevatorPosition();
            RobotContainer.END_EFFECTOR.setTargetState(EndEffectorConstants.EndEffectorState.REST);
        });
    }

    public static Command getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState targetState) {
        return getSetTargetStateCommand(() -> targetState);
    }

    public static Command getSetTargetStateCommand(Supplier<ArmElevatorConstants.ArmElevatorState> targetState) {
        return getSetTargetStateCommand(targetState, () -> false);
    }

    public static Command getSetTargetStateCommand(Supplier<ArmElevatorConstants.ArmElevatorState> targetState, Supplier<Boolean> isStateReversed) {
        return new SequentialCommandGroup(
                getPrepareForStateCommand(targetState, isStateReversed)
                        .onlyIf(() -> targetState.get().ignoreConstraints && targetState.get().prepareState != null)
                        .until(() -> RobotContainer.ARM_ELEVATOR.atState(targetState.get().prepareState, isStateReversed.get())),
                new ExecuteEndCommand(
                        () -> RobotContainer.ARM_ELEVATOR.setTargetState(targetState.get(), isStateReversed.get()),
                        RobotContainer.ARM_ELEVATOR::stop,
                        RobotContainer.ARM_ELEVATOR
                )
        );
    }

    public static Command getStayInPlaceCommand() {
        return new ExecuteEndCommand(
                () -> {
                    RobotContainer.ARM_ELEVATOR.setTargetArmAngle(RobotContainer.ARM_ELEVATOR.getCurrentArmAngle(), true);
                    RobotContainer.ARM_ELEVATOR.setTargetElevatorPositionMeters(RobotContainer.ARM_ELEVATOR.getCurrentElevatorPositionMeters(), true);
                },
                RobotContainer.ARM_ELEVATOR::stop,
                RobotContainer.ARM_ELEVATOR
        );
    }

    public static Command getPrepareForStateCommand(Supplier<ArmElevatorConstants.ArmElevatorState> targetState) {
        return getPrepareForStateCommand(targetState, () -> false);
    }

    public static Command getPrepareForStateCommand(Supplier<ArmElevatorConstants.ArmElevatorState> targetState, Supplier<Boolean> isStateReversed) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ARM_ELEVATOR.prepareToState(targetState.get(), isStateReversed.get()),
                RobotContainer.ARM_ELEVATOR::stop,
                RobotContainer.ARM_ELEVATOR
        );
    }

    public static Command getDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.REST_WITH_CORAL),
                getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.REST),
                RobotContainer.END_EFFECTOR::hasGamePiece
        );
    }
}
