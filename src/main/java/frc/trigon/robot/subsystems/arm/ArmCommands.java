package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.trigon.robot.RobotContainer;
import lib.commands.ExecuteEndCommand;
import lib.commands.GearRatioCalculationCommand;
import lib.commands.NetworkTablesCommand;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ArmCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees, targetVoltage) -> RobotContainer.ARM.setTargetState(
                        Rotation2d.fromDegrees(targetAngleDegrees),
                        targetVoltage
                ),
                false,
                Set.of(RobotContainer.ARM),
                "Debugging/ArmTargetPositionDegrees",
                "Debugging/EndEffectorTargetVoltage"
        );
    }

    public static Command getGearRatioCalulationCommand() {
        return new GearRatioCalculationCommand(
                ArmConstants.ARM_MASTER_MOTOR,
                ArmConstants.ANGLE_ENCODER,
                0.5,
                RobotContainer.ARM
        );
    }

    public static Command getSetTargetStateCommand(Supplier<ArmConstants.ArmState> targetState, BooleanSupplier isStateReversed) {
        return new FunctionalCommand(
                () -> RobotContainer.ARM.setEndEffectorState(targetState.get()),
                () -> RobotContainer.ARM.setArmState(targetState.get(), isStateReversed.getAsBoolean()),
                interrupted -> RobotContainer.ARM.stop(),
                () -> false,
                RobotContainer.ARM
        );
    }

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState, BooleanSupplier isStateReversed) {
        return new FunctionalCommand(
                () -> RobotContainer.ARM.setEndEffectorState(targetState),
                () -> RobotContainer.ARM.setArmState(targetState, isStateReversed.getAsBoolean()),
                interrupted -> RobotContainer.ARM.stop(),
                () -> false,
                RobotContainer.ARM
        );
    }

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState) {
        return getSetTargetStateCommand(targetState, false);
    }

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState, boolean isStateReversed) {
        return new FunctionalCommand(
                () -> RobotContainer.ARM.setEndEffectorState(targetState),
                () -> RobotContainer.ARM.setArmState(targetState, isStateReversed),
                interrupted -> RobotContainer.ARM.stop(),
                () -> false,
                RobotContainer.ARM
        );
    }

    public static Command getPrepareForStateCommand(Supplier<ArmConstants.ArmState> targetState, BooleanSupplier isStateReversed) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ARM.setPrepareState(targetState.get(), isStateReversed.getAsBoolean()),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getPrepareForStateCommand(ArmConstants.ArmState targetState, BooleanSupplier isStateReversed) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ARM.setPrepareState(targetState, isStateReversed.getAsBoolean()),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getPrepareForStateCommand(ArmConstants.ArmState targetState) {
        return getPrepareForStateCommand(targetState, false);
    }

    public static Command getPrepareForStateCommand(ArmConstants.ArmState targetState, boolean isStateReversed) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ARM.setPrepareState(targetState, isStateReversed),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getRestCommand() {
        return new ConditionalCommand(
                getSetTargetStateCommand(ArmConstants.ArmState.REST_WITH_CORAL),
                getSetTargetStateCommand(ArmConstants.ArmState.REST),
                RobotContainer.ARM::hasGamePiece
        );
    }
}
