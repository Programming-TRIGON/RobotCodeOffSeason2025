package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class ArmCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double[] targetStates) -> RobotContainer.ARM.setTargetState(
                        Rotation2d.fromDegrees(targetStates[0]),
                        targetStates[1]
                ),
                false,
                Set.of(RobotContainer.ARM),
                "Debugging/ElevatorTargetPositionRotations"
        );
    }

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.setTargetState(targetState),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }
}
