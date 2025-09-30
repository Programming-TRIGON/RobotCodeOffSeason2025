package frc.trigon.robot.subsystems.armelevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
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

    public static Command getArmGearRatioCalulationCommand() {
        return new GearRatioCalculationCommand(
                ArmElevatorConstants.ARM_MASTER_MOTOR,
                ArmElevatorConstants.ANGLE_ENCODER,
                0.5,
                RobotContainer.ARM_ELEVATOR
        );
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
        return new ConditionalCommand(
                getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.REST_WITH_CORAL),
                getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.REST),
                RobotContainer.END_EFFECTOR::hasGamePiece
        );
    }
}
