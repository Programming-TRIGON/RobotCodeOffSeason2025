package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import lib.commands.GearRatioCalculationCommand;
import lib.commands.NetworkTablesCommand;

import java.util.Set;

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

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState, boolean isStateReversed) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.setTargetState(targetState, isStateReversed),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.setTargetState(targetState),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getPrepareStateCommand(ArmConstants.ArmState targetState, boolean isStateReversed) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.prepareForState(targetState, isStateReversed),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getPrepareStateCommand(ArmConstants.ArmState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.prepareForState(targetState),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }
}
