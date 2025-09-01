package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = ElevatorConstants.MASTER_MOTOR,
            followerMotor = ElevatorConstants.FOLLOWER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, ElevatorConstants.DEFAULT_MAXIMUM_VELOCITY, ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION, ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION * 10).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.REST;

    public Elevator() {
        setName("Elevator");
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ElevatorConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
        followerMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .linearPosition(Units.Meters.of(getPositionRotations()))
                .linearVelocity(Units.MetersPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void sysIdDrive(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public void updateMechanism() {
        ElevatorConstants.MECHANISM.update(
                getPositionsMeters(),
                rotationsToMeters(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    public double rotationsToMeters(double positionsRotations) {
        return Conversions.rotationsToDistance(positionsRotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    public double metersToRotations(double positionsMeters) {
        return Conversions.distanceToRotations(positionsMeters, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;
        scalePositionRequestSpeed(targetState.speedScalar);
        setTargetPositionRotations(metersToRotations(targetState.targetPositionMeters));
    }

    private void scalePositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = ElevatorConstants.DEFAULT_MAXIMUM_VELOCITY;
        positionRequest.Acceleration = ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    void setTargetPositionRotations(double targetPositionRotations) {
        masterMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    private double getPositionRotations() {
        return masterMotor.getSignal(TalonFXSignal.POSITION);
    }

    private double getPositionsMeters() {
        return rotationsToMeters(getPositionRotations());
    }
}
