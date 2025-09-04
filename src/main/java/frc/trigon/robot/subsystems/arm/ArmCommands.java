package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.GearRatioCalculationCommand;
import trigon.commands.NetworkTablesCommand;

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

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState, boolean reverseState) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.setTargetStateWithReverseStates(targetState, reverseState),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }
}
