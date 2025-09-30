package frc.trigon.robot.subsystems.endeffector;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import frc.trigon.robot.subsystems.MotorSubsystem;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import org.littletonrobotics.junction.AutoLogOutput;

public class EndEffector extends MotorSubsystem {
    private final TalonFXMotor endEffectorMotor = EndEffectorConstants.END_EFFECTOR_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(EndEffectorConstants.FOC_ENABLED);

    @Override
    public void stop() {
        endEffectorMotor.stopMotor();
    }

    @Override
    public void updateMechanism() {
        EndEffectorConstants.END_EFFECTOR_MECHANISM.update(endEffectorMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }

    @Override
    public void updatePeriodically() {
        endEffectorMotor.update();
        EndEffectorConstants.DISTANCE_SENSOR.updateSensor();
    }

    @AutoLogOutput(key = "EndEffector/HasCoral")
    public boolean hasGamePiece() {
        return EndEffectorConstants.COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    @AutoLogOutput(key = "EndEffector/GamePieceInEndEffector")
    public boolean isGamePieceInEndEffector() {
        return SimulationFieldHandler.isCoralInEndEffector();
    }

    public Translation3d calculateLinearEndEffectorVelocity() {
        double velocityMetersPerSecond = endEffectorMotor.getSignal(TalonFXSignal.VELOCITY) * 2 * Math.PI * EndEffectorConstants.WHEEL_RADIUS_METERS;
        return RobotContainer.ARM_ELEVATOR.calculateLinearArmVelocity().plus(
                new Translation3d(
                        RobotContainer.ARM_ELEVATOR.getCurrentArmAngle().getCos() * velocityMetersPerSecond,
                        RobotContainer.ARM_ELEVATOR.getCurrentArmAngle().getSin() * velocityMetersPerSecond,
                        0
                )
        );
    }

    @AutoLogOutput(key = "EndEffector/IsEjecting")
    public boolean isEjecting() {
        return endEffectorMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) > 2;
    }

    void setTargetState(EndEffectorConstants.EndEffectorState targetState) {
        setEndEffectorTargetVoltage(targetState.targetVoltage);
    }

    void setTargetState(double targetVoltage) {
        setEndEffectorTargetVoltage(targetVoltage);
    }

    private void setEndEffectorTargetVoltage(double targetVoltage) {
        EndEffectorConstants.END_EFFECTOR_MECHANISM.setTargetVelocity(targetVoltage);
        endEffectorMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }
}
