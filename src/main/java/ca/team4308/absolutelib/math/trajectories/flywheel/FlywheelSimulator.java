package ca.team4308.absolutelib.math.trajectories.flywheel;

import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;

/**
 * Simulates flywheel shooter performance.
 * Models ball-wheel interaction, energy transfer, slip, and exit velocity.
 */
public class FlywheelSimulator {
    
    /**
     * Result of a flywheel simulation.
     */
    public static class SimulationResult {
        public final double exitVelocityMps;
        public final double energyTransferEfficiency;
        public final double slipRatio;
        public final double requiredWheelRpm;
        public final double requiredMotorRpm;
        public final double motorPowerPercent;
        public final double spinUpTimeSeconds;
        public final double currentDrawAmps;
        public final double storedEnergyJoules;
        public final double contactTimeMs;
        public final double ballSpinRpm;
        public final boolean isAchievable;
        public final String limitingFactor;
        
        public SimulationResult(double exitVelocityMps, double energyTransferEfficiency,
                                double slipRatio, double requiredWheelRpm, double requiredMotorRpm,
                                double motorPowerPercent,
                                double spinUpTimeSeconds, double currentDrawAmps,
                                double storedEnergyJoules, double contactTimeMs, double ballSpinRpm,
                                boolean isAchievable, String limitingFactor) {
            this.exitVelocityMps = exitVelocityMps;
            this.energyTransferEfficiency = energyTransferEfficiency;
            this.slipRatio = slipRatio;
            this.requiredWheelRpm = requiredWheelRpm;
            this.requiredMotorRpm = requiredMotorRpm;
            this.motorPowerPercent = motorPowerPercent;
            this.spinUpTimeSeconds = spinUpTimeSeconds;
            this.currentDrawAmps = currentDrawAmps;
            this.storedEnergyJoules = storedEnergyJoules;
            this.contactTimeMs = contactTimeMs;
            this.ballSpinRpm = ballSpinRpm;
            this.isAchievable = isAchievable;
            this.limitingFactor = limitingFactor;
        }
        
        /**
         * Creates an unachievable result with error message.
         */
        public static SimulationResult unachievable(String reason) {
            return new SimulationResult(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, reason);
        }
        
        @Override
        public String toString() {
            if (!isAchievable) {
                return String.format("UNACHIEVABLE: %s", limitingFactor);
            }
            return String.format("%.0f%% power (%.0f RPM wheel), %.1f m/s exit",
                motorPowerPercent * 100, requiredWheelRpm, exitVelocityMps);
        }
    }
    
    private final FlywheelConfig config;
    private final GamePiece gamePiece;
    
    /**
     * Creates a simulator for the given flywheel and game piece.
     */
    public FlywheelSimulator(FlywheelConfig config, GamePiece gamePiece) {
        this.config = config;
        this.gamePiece = gamePiece;
    }
    
    /**
     * Simulates the flywheel achieving a target exit velocity.
     * 
     * @param targetExitVelocityMps Desired ball exit velocity in m/s
     * @return Simulation result with all parameters
     */
    public SimulationResult simulateForVelocity(double targetExitVelocityMps) {
        double compressionRatio = config.getCompressionRatio();
        double ballCOR = gamePiece.getCoefficientOfRestitution();
        double efficiency = config.getMaterial().getEnergyTransferEfficiency(compressionRatio, ballCOR);
        
        efficiency *= getArrangementEfficiencyFactor();
        
        double slip = estimateSlip(targetExitVelocityMps);
        
        double effectiveTransfer = efficiency * (1.0 - slip * 0.5);
        if (effectiveTransfer < 0.1) {
            return SimulationResult.unachievable("Energy transfer too low");
        }
        
        double requiredSurfaceVelocity = targetExitVelocityMps / effectiveTransfer;
        
        double requiredWheelRpm = config.getRpmForVelocity(requiredSurfaceVelocity);
        double requiredMotorRpm = config.getMotorRpmForWheelRpm(requiredWheelRpm);
        
        double maxMotorRpm = config.getMotor().getFreeSpeedRpm();
        double motorPowerPercent = requiredMotorRpm / maxMotorRpm;
        
        double maxWheelRpm = config.getMaxWheelRpm();
        if (requiredWheelRpm > maxWheelRpm) {
            return SimulationResult.unachievable(
                String.format("Required %.0f RPM exceeds max %.0f RPM", requiredWheelRpm, maxWheelRpm));
        }
        
        if (requiredMotorRpm > maxMotorRpm * 0.95) {
            return SimulationResult.unachievable(
                String.format("Motor RPM %.0f too close to free speed %.0f", requiredMotorRpm, maxMotorRpm));
        }
        
        double spinUpTime = config.estimateSpinUpTime(requiredWheelRpm);
        double currentDraw = config.estimateCurrentDraw(requiredWheelRpm);
        double storedEnergy = config.getStoredEnergy(requiredWheelRpm);
        double contactTime = estimateContactTime(requiredSurfaceVelocity);
        double ballSpin = estimateBallSpin(requiredWheelRpm, efficiency);
        
        double currentPerMotor = currentDraw / (config.getMotorsPerWheel() * config.getWheelCount());
        String limitingFactor = "None";
        if (currentPerMotor > 40) {
            limitingFactor = String.format("High current: %.1fA/motor", currentPerMotor);
        }
        
        return new SimulationResult(
            targetExitVelocityMps, efficiency, slip,
            requiredWheelRpm, requiredMotorRpm, motorPowerPercent,
            spinUpTime, currentDraw, storedEnergy,
            contactTime * 1000, ballSpin,
            true, limitingFactor
        );
    }
    
    /**
     * Simulates the maximum achievable exit velocity.
     */
    public SimulationResult simulateMaxVelocity() {
        double maxSurfaceVelocity = config.getSurfaceVelocity(config.getMaxWheelRpm() * 0.8);
        double compressionRatio = config.getCompressionRatio();
        double ballCOR = gamePiece.getCoefficientOfRestitution();
        double efficiency = config.getMaterial().getEnergyTransferEfficiency(compressionRatio, ballCOR);
        efficiency *= getArrangementEfficiencyFactor();
        
        double maxExitVelocity = maxSurfaceVelocity * efficiency;
        
        return simulateForVelocity(maxExitVelocity);
    }
    
    /**
     * Simulates at a specific wheel RPM.
     */
    public SimulationResult simulateAtRpm(double wheelRpm) {
        if (wheelRpm > config.getMaxWheelRpm()) {
            return SimulationResult.unachievable("RPM exceeds maximum");
        }
        
        double surfaceVelocity = config.getSurfaceVelocity(wheelRpm);
        double compressionRatio = config.getCompressionRatio();
        double ballCOR = gamePiece.getCoefficientOfRestitution();
        double efficiency = config.getMaterial().getEnergyTransferEfficiency(compressionRatio, ballCOR);
        efficiency *= getArrangementEfficiencyFactor();
        
        double slip = estimateSlip(surfaceVelocity * efficiency);
        double exitVelocity = surfaceVelocity * efficiency * (1.0 - slip * 0.5);
        
        double motorRpm = config.getMotorRpmForWheelRpm(wheelRpm);
        double motorPowerPercent = motorRpm / config.getMotor().getFreeSpeedRpm();
        
        double spinUpTime = config.estimateSpinUpTime(wheelRpm);
        double currentDraw = config.estimateCurrentDraw(wheelRpm);
        double storedEnergy = config.getStoredEnergy(wheelRpm);
        double contactTime = estimateContactTime(surfaceVelocity);
        double ballSpin = estimateBallSpin(wheelRpm, efficiency);
        
        return new SimulationResult(
            exitVelocity, efficiency, slip,
            wheelRpm, motorRpm, motorPowerPercent,
            spinUpTime, currentDraw, storedEnergy,
            contactTime * 1000, ballSpin,
            true, "None"
        );
    }
    
    /**
     * Calculates efficiency factor based on wheel arrangement.
     */
    private double getArrangementEfficiencyFactor() {
        switch (config.getArrangement()) {
            case SINGLE:
                return 0.85;
            case DUAL_PARALLEL:
                return 0.90;
            case DUAL_OVER_UNDER:
                return 0.95;
            case STACKED:
                return 0.92;
            case DIFFERENTIAL:
                return 0.88;
            default:
                return 0.90;
        }
    }
    
    /**
     * Estimates slip ratio based on configuration and velocity.
     */
    private double estimateSlip(double exitVelocity) {
        double compressionRatio = config.getCompressionRatio();
        WheelMaterial material = config.getMaterial();
        
        double velocityFactor = exitVelocity / 30.0;
        
        double compressionFactor = 1.0 - compressionRatio * 3.0;
        compressionFactor = Math.max(0.5, Math.min(1.0, compressionFactor));
        
        double gripFactor = material.getGripFactor();
        
        double slip = velocityFactor * compressionFactor * (1.0 - gripFactor);
        
        return Math.max(0, Math.min(0.5, slip));
    }
    
    /**
     * Estimates ball-wheel contact time.
     */
    private double estimateContactTime(double surfaceVelocity) {
        double contactLength = gamePiece.getDiameterMeters() * config.getCompressionRatio();
        contactLength = Math.max(contactLength, 0.01);
        
        if (surfaceVelocity < 0.1) {
            return 0.1;
        }
        
        return contactLength / surfaceVelocity;
    }
    
    /**
     * Estimates induced ball spin from flywheel.
     */
    private double estimateBallSpin(double wheelRpm, double efficiency) {
        switch (config.getArrangement()) {
            case SINGLE:
                return -wheelRpm * 0.3 * efficiency;
            case DIFFERENTIAL:
                return wheelRpm * 0.5;
            case DUAL_OVER_UNDER:
                return wheelRpm * 0.05;
            default:
                return wheelRpm * 0.1;
        }
    }
    
    /**
     * Scores this flywheel configuration for a target velocity.
     * Higher score = better configuration.
     */
    public double scoreConfiguration(double targetVelocityMps) {
        SimulationResult result = simulateForVelocity(targetVelocityMps);
        
        if (!result.isAchievable) {
            return 0;
        }
        
        double score = 100.0;
        
        score += result.energyTransferEfficiency * 20;
        
        score += (1.0 - result.slipRatio) * 15;
        
        score += Math.max(0, 10 - result.spinUpTimeSeconds * 5);
        
        double currentPerMotor = result.currentDrawAmps / 
            (config.getMotorsPerWheel() * config.getWheelCount());
        score += Math.max(0, 10 - currentPerMotor / 5);
        
        double rpmMargin = 1.0 - result.requiredWheelRpm / config.getMaxWheelRpm();
        score += rpmMargin * 15;
        
        double ballKE = 0.5 * gamePiece.getMassKg() * targetVelocityMps * targetVelocityMps;
        double shotsAvailable = result.storedEnergyJoules / ballKE;
        score += Math.min(10, shotsAvailable);
        
        return score;
    }
    
    public FlywheelConfig getConfig() {
        return config;
    }
    
    public GamePiece getGamePiece() {
        return gamePiece;
    }
}
