package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import trigon.hardware.RobotHardwareStats;
import trigon.hardware.misc.simplesensor.SimpleSensor;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.hardware.simulation.SimpleMotorSimulation;
import trigon.hardware.simulation.SingleJointedArmSimulation;
import trigon.utilities.mechanisms.SingleJointedArmMechanism2d;
import trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class IntakeConstants {
    private static final int
            INTAKE_MOTOR_ID = 9,
            ANGLE_MOTOR_ID = 10,
            MAXIMUM_ANGLE_BEAM_BREAK_CHANNEL = 0,
            MINIMUM_ANGLE_BEAM_BREAK_CHANNEL = 1,
            DISTANCE_SENSOR_CHANNEL = 2;
    private static final String
            INTAKE_MOTOR_NAME = "IntakeMotor",
            ANGLE_MOTOR_NAME = "IntakeAngleMotor",
            MAXIMUM_ANGLE_BEAM_BREAK_NAME = "IntakeMaximumAngleBeamBreak",
            MINIMUM_ANGLE_BEAM_BREAK_NAME = "IntakeMinimumAngleBeamBreak",
            DISTANCE_SENSOR_NAME = "IntakeDistanceSensor";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final SimpleSensor
            MAXIMUM_ANGLE_BEAM_BREAK = SimpleSensor.createDigitalSensor(MAXIMUM_ANGLE_BEAM_BREAK_CHANNEL, MAXIMUM_ANGLE_BEAM_BREAK_NAME),
            MINIMUM_ANGLE_BEAM_BREAK = SimpleSensor.createDigitalSensor(MINIMUM_ANGLE_BEAM_BREAK_CHANNEL, MINIMUM_ANGLE_BEAM_BREAK_NAME),
            DISTANCE_SENSOR = SimpleSensor.createDutyCycleSensor(DISTANCE_SENSOR_CHANNEL, DISTANCE_SENSOR_NAME);

    private static final double
            INTAKE_MOTOR_GEAR_RATIO = 4,
            ANGLE_MOTOR_GEAR_RATIO = 28;
    private static final double
            DISTANCE_SENSOR_SCALING_SLOPE = 0.0002,
            DISTANCE_SENSOR_SCALING_INTERCEPT_POINT = -200;
    static final boolean FOC_ENABLED = true;

    private static final int
            INTAKE_MOTOR_AMOUNT = 1,
            ANGLE_MOTOR_AMOUNT = 1;
    private static final DCMotor
            INTAKE_GEARBOX = DCMotor.getFalcon500(INTAKE_MOTOR_AMOUNT),
            ANGLE_GEARBOX = DCMotor.getKrakenX60(ANGLE_MOTOR_AMOUNT);

    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            INTAKE_LENGTH_METERS = 0.44, //TODO: get actual numbers
            INTAKE_MASS_KILOGRAMS = 8;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-15), //TODO: get actual numbers
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(50);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SimpleMotorSimulation INTAKE_SIMULATION = new SimpleMotorSimulation(
            INTAKE_GEARBOX,
            INTAKE_MOTOR_GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final SingleJointedArmSimulation ANGLE_MOTOR_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            INTAKE_LENGTH_METERS,
            INTAKE_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );
    private static final DoubleSupplier
            MAXIMUM_ANGLE_BEAM_BREAK_SIMULATION_SUPPLIER = () -> 0,
            MINIMUM_ANGLE_BEAM_BREAK_SIMULATION_SUPPLIER = () -> 0,
            DISTANCE_SENSOR_SIMULATION_SUPPLIER = () -> SimulationFieldHandler.isHoldingGamePiece() ? 0 : 1000;

    static final SysIdRoutine.Config ANGLE_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    static final Pose3d INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0) //TODO: get actual numbers
    );
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d INTAKE_MECHANISM = new SpeedMechanism2d(
            "IntakeMechanism",
            MAXIMUM_DISPLAYABLE_VELOCITY
    );
    static final SingleJointedArmMechanism2d ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            "IntakeAngleMechanism",
            INTAKE_LENGTH_METERS,
            Color.kRed
    );

    public static final double
            COLLECTION_RUMBLE_DURATION_SECONDS = 0.7,
            COLLECTION_RUMBLE_POWER = 1;
    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.5);

    static {
        configureIntakeMotor();
        configureAngleMotor();
        configureBeamBreaks();
        configureDistanceSensor();
    }

    private static void configureIntakeMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.RotorToSensorRatio = INTAKE_MOTOR_GEAR_RATIO;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        INTAKE_MOTOR.applyConfiguration(config);
        INTAKE_MOTOR.setPhysicsSimulation(INTAKE_SIMULATION);

        INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 1 : 0; //TODO: calibrate
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 12 / config.Slot0.kV : 2;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 8 : 8;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINIMUM_ANGLE.getRotations();

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_ANGLE.getRotations();

        config.CurrentLimits.SupplyCurrentLimit = 60;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureBeamBreaks() {
        MINIMUM_ANGLE_BEAM_BREAK.setSimulationSupplier(MINIMUM_ANGLE_BEAM_BREAK_SIMULATION_SUPPLIER);
        MAXIMUM_ANGLE_BEAM_BREAK.setSimulationSupplier(MAXIMUM_ANGLE_BEAM_BREAK_SIMULATION_SUPPLIER);
    }

    private static void configureDistanceSensor() {
        DISTANCE_SENSOR.setSimulationSupplier(DISTANCE_SENSOR_SIMULATION_SUPPLIER);
        DISTANCE_SENSOR.setScalingConstants(DISTANCE_SENSOR_SCALING_SLOPE, DISTANCE_SENSOR_SCALING_INTERCEPT_POINT);
    }

    public enum IntakeState {
        REST(0, MAXIMUM_ANGLE),
        COLLECT(5, MINIMUM_ANGLE),
        EJECT(-5, MINIMUM_ANGLE);

        public final double targetVoltage;
        public final Rotation2d targetAngle;

        IntakeState(double targetVoltage, Rotation2d targetAngle) {
            this.targetVoltage = targetVoltage;
            this.targetAngle = targetAngle;
        }
    }
}