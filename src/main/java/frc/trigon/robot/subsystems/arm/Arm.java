package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import lib.hardware.phoenix6.cancoder.CANcoderSignal;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import org.littletonrobotics.junction.Logger;

public class Arm extends MotorSubsystem {
    private final TalonFXMotor
            armMasterMotor = ArmConstants.ARM_MASTER_MOTOR,
            endEffectorMotor = ArmConstants.END_EFFECTOR_MOTOR;
    private final CANcoderEncoder angleEncoder = ArmConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ArmConstants.ARM_DEFAULT_MAXIMUM_VELOCITY,
            ArmConstants.ARM_DEFAULT_MAXIMUM_ACCELERATION,
            ArmConstants.ARM_DEFAULT_MAXIMUM_JERK
    ).withEnableFOC(ArmConstants.FOC_ENABLED);
    public boolean isStateReversed = false;
    private ArmConstants.ArmState targetState = ArmConstants.ArmState.REST;

    public Arm() {
        setName("Arm");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ArmConstants.ARM_SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        armMasterMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        armMasterMotor.stopMotor();
        endEffectorMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .angularPosition(Units.Rotations.of(getAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(armMasterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(armMasterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        ArmConstants.ARM_MECHANISM.update(
                Rotation2d.fromRotations(getAngle().getRotations() + ArmConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET),
                Rotation2d.fromRotations(armMasterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + ArmConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET)
        );
        ArmConstants.END_EFFECTOR_MECHANISM.update(endEffectorMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));

        Logger.recordOutput("Poses/Components/ArmPose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        armMasterMotor.update();
        endEffectorMotor.update();
        angleEncoder.update();
        ArmConstants.DISTANCE_SENSOR.updateSensor();
        Logger.recordOutput("Arm/CurrentPositionDegrees", getAngle().getDegrees());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        armMasterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public boolean isArmAboveSafeZone() {
        return getAngle().getDegrees() >= ArmConstants.MAXIMUM_ARM_SAFE_ZONE.getDegrees();
    }

    public boolean atState(ArmConstants.ArmState targetState, boolean isStateReversed) {
        if (isStateReversed)
            return this.targetState == targetState && atTargetAngle(isStateReversed);
        return atState(targetState);
    }

    public boolean atState(ArmConstants.ArmState targetState) {
        return this.targetState == targetState && atTargetAngle();
    }

    public boolean atTargetAngle(boolean isStateReversed) {
        if (isStateReversed) {
            final double currentToTargetStateDifferenceDegrees = Math.abs(subtractFrom360Degrees(targetState.targetAngle).minus(getAngle()).getDegrees());
            return currentToTargetStateDifferenceDegrees < ArmConstants.ANGLE_TOLERANCE.getDegrees();
        }
        return atTargetAngle();
    }

    public boolean atTargetAngle() {
        final double currentToTargetStateDifferenceDegrees = Math.abs(targetState.targetAngle.minus(getAngle()).getDegrees());
        return currentToTargetStateDifferenceDegrees < ArmConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }

    void setTargetState(ArmConstants.ArmState targetState) {
        this.isStateReversed = false;
        this.targetState = targetState;
        setTargetState(targetState, false);
    }

    void setTargetState(ArmConstants.ArmState targetState, boolean isStateReversed) {
        this.isStateReversed = isStateReversed;
        this.targetState = targetState;

        if (isStateReversed) {
            setTargetState(
                    subtractFrom360Degrees(targetState.targetAngle)
                    , targetState.targetEndEffectorVoltage
            );
            return;
        }
        setTargetState(
                targetState.targetAngle,
                targetState.targetEndEffectorVoltage);
    }

    void setTargetState(Rotation2d targetAngle, double targetVoltage) {
        setTargetAngle(targetAngle);
        setTargetVoltage(targetVoltage);
    }

    void prepareForState(ArmConstants.ArmState targetState) {
        prepareForState(targetState, false);
    }

    void prepareForState(ArmConstants.ArmState targetState, boolean isStateReversed) {
        this.isStateReversed = isStateReversed;
        this.targetState = targetState;

        if (isStateReversed) {
            setTargetAngle(subtractFrom360Degrees(targetState.targetAngle));
            return;
        }
        setTargetAngle(targetState.targetAngle);
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        final double distanceFromSafeZone = RobotContainer.ELEVATOR.getPositionMeters() - ElevatorConstants.MINIMUM_ELEVATOR_SAFE_ZONE_METERS;
        final double cosMinimumAngle = distanceFromSafeZone / ArmConstants.ARM_LENGTH_METERS;
        final Rotation2d minimumSafeAngle = Rotation2d.fromRadians(RobotContainer.ELEVATOR.isElevatorAboveSafeZone() ? 0 : Math.acos(cosMinimumAngle));
        armMasterMotor.setControl(positionRequest.withPosition(Math.max(targetAngle.getRotations(), minimumSafeAngle.getRotations())));
    }

    private void setTargetVoltage(double targetVoltage) {
        ArmConstants.END_EFFECTOR_MECHANISM.setTargetVelocity(targetVoltage);
        endEffectorMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public static Rotation2d subtractFrom360Degrees(Rotation2d angleToSubtract) {
        return Rotation2d.fromDegrees(360 - angleToSubtract.getDegrees());
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d armTransform = new Transform3d(
                new Translation3d(0, 0, RobotContainer.ELEVATOR.getPositionMeters()),
                new Rotation3d(0, getAngle().getRadians(), 0)
        );
        return ArmConstants.ARM_VISUALIZATION_ORIGIN_POINT.transformBy(armTransform);
    }
}