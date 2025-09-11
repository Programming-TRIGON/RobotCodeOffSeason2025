package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import lib.hardware.misc.servo.Servo;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import org.littletonrobotics.junction.Logger;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor motor = ClimberConstants.MOTOR;
    private final Servo
            rightServo = ClimberConstants.RIGHT_SERVO,
            leftServo = ClimberConstants.LEFT_SERVO;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(ClimberConstants.FOC_ENABLED);
    private ClimberConstants.ClimberState targetState = ClimberConstants.ClimberState.REST;

    public Climber() {
        setName("Climber");
    }

    @Override
    public void stop() {
        motor.stopMotor();
        setServoPowers(0);
    }

    @Override
    public void updateMechanism() {
        ClimberConstants.MECHANISM.update(
                Rotation2d.fromRotations(getPositionRotations() * ClimberConstants.CLIMBER_VISUALIZATION_POSITION_SCALAR),
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) * ClimberConstants.CLIMBER_VISUALIZATION_POSITION_SCALAR)
        );

        Logger.recordOutput("Poses/Components/ClimberPose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        rightServo.update();
        leftServo.update();
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("ClimberMotor")
                .angularPosition(Units.Rotations.of(getPositionRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        setTargetVoltage(targetDrivePower);
    }

    public boolean atState(ClimberConstants.ClimberState targetState) {
        return this.targetState == targetState && atTargetState();
    }

    public boolean atTargetState() {
        return Math.abs(getPositionRotations() - targetState.targetPositionRotations) < ClimberConstants.CLIMBER_TOLERANCE_ROTATIONS;
    }

    public boolean hasCage() {
        return ClimberConstants.HAS_CAGE_BOOLEAN_EVENT.getAsBoolean();
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetPositionRotations, targetState.targetServoPower, targetState.isAffectedByRobotWeight);
    }

    void setTargetState(double targetPositionRotations, double targetServoPower, boolean isAffectedByRobotWeight) {
        final MotionMagicVoltage positionRequest = this.positionRequest
                .withPosition(targetPositionRotations)
                .withSlot(isAffectedByRobotWeight ? ClimberConstants.ON_CAGE_PID_SLOT : ClimberConstants.GROUNDED_PID_SLOT);
        motor.setControl(positionRequest);
        setServoPowers(targetServoPower);
    }

    void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private void setServoPowers(double power) {
        rightServo.set(power);
        leftServo.set(-power);
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(-Rotation2d.fromRotations(getPositionRotations()).getRadians() * ClimberConstants.CLIMBER_VISUALIZATION_POSITION_SCALAR, 0, 0)
        );

        return ClimberConstants.CLIMBER_VISUALIZATION_ORIGIN_POINT.transformBy(climberTransform);
    }

    private double getPositionRotations() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }
}
