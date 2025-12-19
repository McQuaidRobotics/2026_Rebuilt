package sham.shamController.unitSafeControl;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.AngleUnit;
import edu.wpi.first.units.measure.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularVelocityUnit;
import edu.wpi.first.units.measure.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.PerUnit;
import edu.wpi.first.units.measure.Unit;
import edu.wpi.first.units.measure.VoltageUnit;
import edu.wpi.first.units.measure.measure.Angle;
import edu.wpi.first.units.measure.measure.AngularAcceleration;
import edu.wpi.first.units.measure.measure.AngularVelocity;
import edu.wpi.first.units.measure.measure.Per;
import edu.wpi.first.units.measure.measure.Time;

public sealed interface UnitFeedforward<O extends Unit> {
    public void logGains(EpilogueBackend logger);

    public O calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate);

    public O calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate);

    public O calculate(AngularVelocity goalRate);

    public default O calculate(
            Angle position, AngularVelocity goalRate, AngularAcceleration goalRateRate) {
        return calculate(goalRate, goalRateRate);
    }

    public default O calculate(
            Angle position, AngularVelocity goalRate, AngularVelocity nextGoalRate) {
        return calculate(goalRate, nextGoalRate);
    }

    public default O calculate(Angle position, AngularVelocity goalRate) {
        return calculate(goalRate);
    }

    public O calculateStatics(Angle position, double sign);

    public static final class SimpleFeedforward<O extends Unit> implements UnitFeedforward<O> {
        private final edu.wpi.first.math.controller.SimpleMotorFeedforward internalFeedforward;
        private final O outputUnit;
        final O kS;
        final Per<O, AngularVelocityUnit> kV;
        final Per<O, AngularAccelerationUnit> kA;

        public SimpleFeedforward(
                O kS,
                Per<O, AngularVelocityUnit> kV,
                Per<O, AngularAccelerationUnit> kA,
                Time dt) {
            outputUnit = kS.baseUnit();
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            internalFeedforward =
                    new edu.wpi.first.math.controller.SimpleMotorFeedforward(
                            kS.baseUnitMagnitude(),
                            kV.in(PerUnit.combine(outputUnit, RadiansPerSecond)),
                            kA.in(PerUnit.combine(outputUnit, RadiansPerSecondPerSecond)),
                            dt.in(Seconds));
        }

        public static SimpleFeedforward<VoltageUnit> forVoltage(
                AngleUnit inputUnit, double kS, double kV, double kA, Time dt) {
            return new SimpleFeedforward<VoltageUnit>(
                    Volts.of(kS),
                    Volts.per(inputUnit.per(Seconds)).ofNative(kV),
                    Volts.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
                    dt);
        }

        public static SimpleFeedforward<CurrentUnit> forCurrent(
                AngleUnit inputUnit, double kS, double kV, double kA, Time dt) {
            return new SimpleFeedforward<CurrentUnit>(
                    Amps.of(kS),
                    Amps.per(inputUnit.per(Seconds)).ofNative(kV),
                    Amps.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
                    dt);
        }

        @Override
        public void logGains(EpilogueBackend logger) {
            logger.log("kS", kS.baseUnitMagnitude());
            logger.log("kV", kV.baseUnitMagnitude());
            logger.log("kA", kA.baseUnitMagnitude());
            logger.log("outputUnit", outputUnit.name());
            logger.log("inputUnit", Radian.name());
            logger.log("feedforwardType", "Flywheel");
        }

        @SuppressWarnings({"unchecked", "removal"})
        @Override
        public O calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
            return (O)
                    outputUnit.of(
                            internalFeedforward.calculate(
                                    goalRate.in(RadiansPerSecond),
                                    goalRateRate.in(RadiansPerSecondPerSecond)));
        }

        @SuppressWarnings("unchecked")
        @Override
        public O calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
            return (O)
                    outputUnit.of(
                            internalFeedforward.calculateWithVelocities(
                                    goalRate.in(RadiansPerSecond),
                                    nextGoalRate.in(RadiansPerSecond)));
        }

        @SuppressWarnings({"unchecked"})
        @Override
        public O calculate(AngularVelocity goalRate) {
            return (O)
                    outputUnit.of(internalFeedforward.calculate(goalRate.in(RadiansPerSecond)));
        }

        @Override
        public O calculateStatics(Angle position, double sign) {
            return kS.times(sign);
        }
    }

    public static final class ElevatorFeedforward<O extends Unit> implements UnitFeedforward<O> {
        private final edu.wpi.first.math.controller.ElevatorFeedforward internalFeedforward;
        private final O outputUnit;
        final O kS, kG;
        final Per<O, AngularVelocityUnit> kV;
        final Per<O, AngularAccelerationUnit> kA;

        public ElevatorFeedforward(
                O kS,
                O kG,
                Per<O, AngularVelocityUnit> kV,
                Per<O, AngularAccelerationUnit> kA,
                Time dt) {
            outputUnit = kS.baseUnit();
            internalFeedforward =
                    new edu.wpi.first.math.controller.ElevatorFeedforward(
                            kS.baseUnitMagnitude(),
                            kG.baseUnitMagnitude(),
                            kV.in(PerUnit.combine(outputUnit, RadiansPerSecond)),
                            kA.in(PerUnit.combine(outputUnit, RadiansPerSecondPerSecond)),
                            dt.in(Seconds));
            this.kS = kS;
            this.kG = kG;
            this.kV = kV;
            this.kA = kA;
        }

        public static ElevatorFeedforward<VoltageUnit> forVoltage(
                AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
            return new ElevatorFeedforward<VoltageUnit>(
                    Volts.of(kS),
                    Volts.of(kG),
                    Volts.per(inputUnit.per(Seconds)).ofNative(kV),
                    Volts.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
                    dt);
        }

        public static ElevatorFeedforward<CurrentUnit> forCurrent(
                AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
            return new ElevatorFeedforward<CurrentUnit>(
                    Amps.of(kS),
                    Amps.of(kG),
                    Amps.per(inputUnit.per(Seconds)).ofNative(kV),
                    Amps.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
                    dt);
        }

        @Override
        public void logGains(EpilogueBackend logger) {
            logger.log("kS", kS);
            logger.log("kG", kG);
            logger.log("kV", kV);
            logger.log("kA", kA);
            logger.log("outputUnit", outputUnit.name());
            logger.log("inputUnit", Meters.name());
            logger.log("feedforwardType", "Elevator");
        }

        @SuppressWarnings({"unchecked", "removal"})
        @Override
        public O calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
            return (O)
                    outputUnit.of(
                            internalFeedforward.calculate(
                                    goalRate.in(RadiansPerSecond),
                                    goalRateRate.in(RadiansPerSecondPerSecond)));
        }

        @SuppressWarnings("unchecked")
        @Override
        public O calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
            return (O)
                    outputUnit.of(
                            internalFeedforward.calculateWithVelocities(
                                    goalRate.in(RadiansPerSecond),
                                    nextGoalRate.in(RadiansPerSecond)));
        }

        @SuppressWarnings({"unchecked"})
        @Override
        public O calculate(AngularVelocity goalRate) {
            return (O)
                    outputUnit.of(internalFeedforward.calculate(goalRate.in(RadiansPerSecond)));
        }

        @Override
        public O calculateStatics(Angle position, double sign) {
            return kS.times(sign).plus(kG);
        }
    }

    public static final class ArmFeedforward<O extends Unit> implements UnitFeedforward<O> {
        private final edu.wpi.first.math.controller.ArmFeedforward internalFeedforward;
        private final O outputUnit;
        final O kS, kG;
        final Per<O, AngularVelocityUnit> kV;
        final Per<O, AngularAccelerationUnit> kA;

        public ArmFeedforward(
                O kS,
                O kG,
                Per<O, AngularVelocityUnit> kV,
                Per<O, AngularAccelerationUnit> kA,
                Time dt) {
            outputUnit = kS.baseUnit();
            internalFeedforward =
                    new edu.wpi.first.math.controller.ArmFeedforward(
                            kS.baseUnitMagnitude(),
                            kG.baseUnitMagnitude(),
                            kV.in(PerUnit.combine(outputUnit, RadiansPerSecond)),
                            kA.in(PerUnit.combine(outputUnit, RadiansPerSecondPerSecond)),
                            dt.in(Seconds));
            this.kS = kS;
            this.kG = kG;
            this.kV = kV;
            this.kA = kA;
        }

        public static ArmFeedforward<VoltageUnit> forVoltage(
                AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
            return new ArmFeedforward<VoltageUnit>(
                    Volts.of(kS),
                    Volts.of(kG),
                    Volts.per(inputUnit.per(Seconds)).ofNative(kV),
                    Volts.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
                    dt);
        }

        public static ArmFeedforward<CurrentUnit> forCurrent(
                AngleUnit inputUnit, double kS, double kG, double kV, double kA, Time dt) {
            return new ArmFeedforward<CurrentUnit>(
                    Amps.of(kS),
                    Amps.of(kG),
                    Amps.per(inputUnit.per(Seconds)).ofNative(kV),
                    Amps.per(inputUnit.per(Seconds).per(Seconds)).ofNative(kA),
                    dt);
        }

        @Override
        public void logGains(EpilogueBackend logger) {
            logger.log("kS", kS);
            logger.log("kG", kG);
            logger.log("kV", kV);
            logger.log("kA", kA);
            logger.log("outputUnit", outputUnit.name());
            logger.log("inputUnit", Radian.name());
            logger.log("feedforwardType", "Arm");
        }

        @Override
        public O calculate(AngularVelocity goalRate) {
            throw new UnsupportedOperationException();
        }

        @Override
        public O calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
            throw new UnsupportedOperationException();
        }

        @Override
        public O calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
            throw new UnsupportedOperationException();
        }

        @Override
        @SuppressWarnings({"unchecked", "removal"})
        public O calculate(
                Angle currentAngle, AngularVelocity goalRate, AngularAcceleration goalRateRate) {
            return (O)
                    outputUnit.of(
                            internalFeedforward.calculate(
                                    currentAngle.in(Radian),
                                    goalRate.in(RadiansPerSecond),
                                    goalRateRate.in(RadiansPerSecondPerSecond)));
        }

        @Override
        @SuppressWarnings("unchecked")
        public O calculate(
                Angle currentAngle, AngularVelocity goalRate, AngularVelocity nextGoalRate) {
            return (O)
                    outputUnit.of(
                            internalFeedforward.calculateWithVelocities(
                                    currentAngle.in(Radian),
                                    goalRate.in(RadiansPerSecond),
                                    nextGoalRate.in(RadiansPerSecond)));
        }

        @Override
        @SuppressWarnings({"unchecked"})
        public O calculate(Angle currentAngle, AngularVelocity goalRate) {
            return (O)
                    outputUnit.of(
                            internalFeedforward.calculate(
                                    currentAngle.in(Radian), goalRate.in(RadiansPerSecond)));
        }

        @Override
        public O calculateStatics(Angle position, double sign) {
            return kS.times(sign).plus(kG.times(Math.cos(position.in(Radian))));
        }
    }
}
