package ca.team4308.absolutelib.math.trajectories.flywheel;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.motor.FRCMotors;
import ca.team4308.absolutelib.math.trajectories.motor.MotorSpec;

/**
 * Generates and evaluates multiple flywheel configurations.
 * Uses systematic variation of parameters to find optimal designs.
 */
public class FlywheelGenerator {
    
    /**
     * Result of flywheel generation and evaluation.
     */
    public static class GenerationResult {
        public final List<ScoredConfig> configurations;
        public final ScoredConfig bestConfig;
        public final int totalGenerated;
        public final int achievableCount;
        
        public GenerationResult(List<ScoredConfig> configurations, int totalGenerated) {
            this.configurations = configurations;
            this.totalGenerated = totalGenerated;
            this.achievableCount = configurations.size();
            this.bestConfig = configurations.isEmpty() ? null : configurations.get(0);
        }
    }
    
    /**
     * A configuration with its score.
     */
    public static class ScoredConfig {
        public final FlywheelConfig config;
        public final FlywheelSimulator.SimulationResult simulation;
        public final double score;
        
        public ScoredConfig(FlywheelConfig config, FlywheelSimulator.SimulationResult simulation, double score) {
            this.config = config;
            this.simulation = simulation;
            this.score = score;
        }
        
        @Override
        public String toString() {
            return String.format("Score: %.1f - %s", score, config.toString());
        }
    }
    
    /**
     * Generation parameters.
     */
    public static class GenerationParams {
        // Wheel diameter range (inches)
        public double minDiameterInches = 3.0;
        public double maxDiameterInches = 6.0;
        public double diameterStepInches = 0.5;
        
        // Durometer range
        public int minDurometer = 30;
        public int maxDurometer = 70;
        public int durometerStep = 10;
        
        // Compression ratio range
        public double minCompression = 0.05;
        public double maxCompression = 0.20;
        public double compressionStep = 0.05;
        
        // Gear ratio range
        public double minGearRatio = 0.5;
        public double maxGearRatio = 2.0;
        public double gearRatioStep = 0.25;
        
        // Arrangements to consider
        public FlywheelConfig.WheelArrangement[] arrangements = {
            FlywheelConfig.WheelArrangement.DUAL_OVER_UNDER,
            FlywheelConfig.WheelArrangement.SINGLE,
            FlywheelConfig.WheelArrangement.DUAL_PARALLEL
        };
        
        // Motors to consider
        public MotorSpec[] motors = FRCMotors.SHOOTER_MOTORS;
        
        // Motors per wheel options
        public int[] motorsPerWheelOptions = {1, 2};
        
        // Maximum configurations to generate
        public int maxConfigurations = 500;
        
        // Top configurations to return
        public int topResults = 10;
        
        public static GenerationParams defaultParams() {
            return new GenerationParams();
        }
        
        public static GenerationParams quickScan() {
            GenerationParams params = new GenerationParams();
            params.diameterStepInches = 1.0;
            params.durometerStep = 20;
            params.compressionStep = 0.10;
            params.gearRatioStep = 0.5;
            params.maxConfigurations = 100;
            return params;
        }
        
        public static GenerationParams detailed() {
            GenerationParams params = new GenerationParams();
            params.diameterStepInches = 0.25;
            params.durometerStep = 5;
            params.compressionStep = 0.025;
            params.gearRatioStep = 0.1;
            params.maxConfigurations = 2000;
            params.topResults = 20;
            return params;
        }
    }
    
    private final GamePiece gamePiece;
    private final GenerationParams params;
    
    /**
     * Creates a generator with default parameters.
     */
    public FlywheelGenerator(GamePiece gamePiece) {
        this(gamePiece, GenerationParams.defaultParams());
    }
    
    /**
     * Creates a generator with specified parameters.
     */
    public FlywheelGenerator(GamePiece gamePiece, GenerationParams params) {
        this.gamePiece = gamePiece;
        this.params = params;
    }
    
    /**
     * Generates all flywheel configurations within parameter ranges.
     */
    public List<FlywheelConfig> generateAll() {
        List<FlywheelConfig> configs = new ArrayList<>();
        int configNum = 0;
        
        for (FlywheelConfig.WheelArrangement arrangement : params.arrangements) {
            for (double diameter = params.minDiameterInches; 
                 diameter <= params.maxDiameterInches; 
                 diameter += params.diameterStepInches) {
                
                for (int durometer = params.minDurometer;
                     durometer <= params.maxDurometer;
                     durometer += params.durometerStep) {
                    
                    for (double compression = params.minCompression;
                         compression <= params.maxCompression;
                         compression += params.compressionStep) {
                        
                        for (double gearRatio = params.minGearRatio;
                             gearRatio <= params.maxGearRatio;
                             gearRatio += params.gearRatioStep) {
                            
                            for (MotorSpec motor : params.motors) {
                                for (int motorsPerWheel : params.motorsPerWheelOptions) {
                                    
                                    if (configNum >= params.maxConfigurations) {
                                        return configs;
                                    }
                                    
                                    int wheelCount = getWheelCountForArrangement(arrangement);
                                    
                                    FlywheelConfig config = FlywheelConfig.builder()
                                        .name(String.format("Config-%04d", configNum++))
                                        .arrangement(arrangement)
                                        .wheelDiameterInches(diameter)
                                        .wheelWidthInches(diameter * 0.5) // Typical width ratio
                                        .durometer(durometer)
                                        .compressionRatio(compression)
                                        .wheelCount(wheelCount)
                                        .motor(motor)
                                        .motorsPerWheel(motorsPerWheel)
                                        .gearRatio(gearRatio)
                                        .build();
                                    
                                    configs.add(config);
                                }
                            }
                        }
                    }
                }
            }
        }
        
        return configs;
    }
    
    /**
     * Generates and evaluates configurations for a target velocity.
     * Returns top configurations sorted by score.
     * 
     * @param targetVelocityMps Target ball exit velocity in m/s
     * @return Generation result with scored configurations
     */
    public GenerationResult generateAndEvaluate(double targetVelocityMps) {
        List<FlywheelConfig> allConfigs = generateAll();
        List<ScoredConfig> scoredConfigs = new ArrayList<>();
        
        for (FlywheelConfig config : allConfigs) {
            FlywheelSimulator simulator = new FlywheelSimulator(config, gamePiece);
            FlywheelSimulator.SimulationResult result = simulator.simulateForVelocity(targetVelocityMps);
            
            if (result.isAchievable) {
                double score = simulator.scoreConfiguration(targetVelocityMps);
                scoredConfigs.add(new ScoredConfig(config, result, score));
            }
        }
        
        // Sort by score descending
        scoredConfigs.sort(Comparator.comparingDouble((ScoredConfig sc) -> sc.score).reversed());
        
        // Return top results
        int resultCount = Math.min(params.topResults, scoredConfigs.size());
        return new GenerationResult(
            scoredConfigs.subList(0, resultCount),
            allConfigs.size()
        );
    }
    
    /**
     * Generates configurations optimized for a velocity range.
     * 
     * @param minVelocityMps Minimum target velocity
     * @param maxVelocityMps Maximum target velocity
     * @return Generation result favoring configurations good across the range
     */
    public GenerationResult generateForVelocityRange(double minVelocityMps, double maxVelocityMps) {
        List<FlywheelConfig> allConfigs = generateAll();
        List<ScoredConfig> scoredConfigs = new ArrayList<>();
        
        double midVelocity = (minVelocityMps + maxVelocityMps) / 2.0;
        
        for (FlywheelConfig config : allConfigs) {
            FlywheelSimulator simulator = new FlywheelSimulator(config, gamePiece);
            
            // Test at min, mid, and max velocities
            FlywheelSimulator.SimulationResult resultMin = simulator.simulateForVelocity(minVelocityMps);
            FlywheelSimulator.SimulationResult resultMid = simulator.simulateForVelocity(midVelocity);
            FlywheelSimulator.SimulationResult resultMax = simulator.simulateForVelocity(maxVelocityMps);
            
            // All must be achievable
            if (resultMin.isAchievable && resultMid.isAchievable && resultMax.isAchievable) {
                double scoreMin = simulator.scoreConfiguration(minVelocityMps);
                double scoreMid = simulator.scoreConfiguration(midVelocity);
                double scoreMax = simulator.scoreConfiguration(maxVelocityMps);
                
                // Weight middle velocity higher
                double avgScore = (scoreMin + 2 * scoreMid + scoreMax) / 4.0;
                
                // Use mid result for display
                scoredConfigs.add(new ScoredConfig(config, resultMid, avgScore));
            }
        }
        
        scoredConfigs.sort(Comparator.comparingDouble((ScoredConfig sc) -> sc.score).reversed());
        
        int resultCount = Math.min(params.topResults, scoredConfigs.size());
        return new GenerationResult(
            scoredConfigs.subList(0, resultCount),
            allConfigs.size()
        );
    }
    
    /**
     * Creates preset configurations known to work well for FRC shooters.
     * Please add your own build for the current robot. these are just examples from other teams bots.
     */
    public List<FlywheelConfig> generatePresets() {
        List<FlywheelConfig> presets = new ArrayList<>();
        
        presets.add(FlywheelConfig.builder()
            .name("Classic Dual 4-inch")
            .arrangement(FlywheelConfig.WheelArrangement.DUAL_OVER_UNDER)
            .wheelDiameterInches(4.0)
            .wheelWidthInches(2.0)
            .material(WheelMaterial.GREEN_COMPLIANT)
            .compressionRatio(0.12)
            .wheelCount(2)
            .motor(FRCMotors.KRAKEN_X60)
            .motorsPerWheel(1)
            .gearRatio(1.0)
            .build());
        
        presets.add(FlywheelConfig.builder()
            .name("High-Speed Single")
            .arrangement(FlywheelConfig.WheelArrangement.SINGLE)
            .wheelDiameterInches(6.0)
            .wheelWidthInches(3.0)
            .material(WheelMaterial.STANDARD)
            .compressionRatio(0.15)
            .wheelCount(1)
            .motor(FRCMotors.KRAKEN_X60)
            .motorsPerWheel(2)
            .gearRatio(0.75)
            .build());
        
        presets.add(FlywheelConfig.builder()
            .name("Compact NEO Dual")
            .arrangement(FlywheelConfig.WheelArrangement.DUAL_OVER_UNDER)
            .wheelDiameterInches(3.0)
            .wheelWidthInches(1.5)
            .material(WheelMaterial.SOFT_COMPLIANT)
            .compressionRatio(0.10)
            .wheelCount(2)
            .motor(FRCMotors.NEO)
            .motorsPerWheel(1)
            .gearRatio(1.5)
            .build());
        
        presets.add(FlywheelConfig.builder()
            .name("High-Grip Dual")
            .arrangement(FlywheelConfig.WheelArrangement.DUAL_OVER_UNDER)
            .wheelDiameterInches(4.0)
            .wheelWidthInches(2.5)
            .material(WheelMaterial.SOFT_COMPLIANT)
            .compressionRatio(0.18)
            .wheelCount(2)
            .motor(FRCMotors.KRAKEN_X60)
            .motorsPerWheel(1)
            .gearRatio(1.0)
            .build());
        
        presets.add(FlywheelConfig.builder()
            .name("Differential Spin")
            .arrangement(FlywheelConfig.WheelArrangement.DIFFERENTIAL)
            .wheelDiameterInches(4.0)
            .wheelWidthInches(2.0)
            .material(WheelMaterial.STEALTH_BLUE)
            .compressionRatio(0.10)
            .wheelCount(2)
            .motor(FRCMotors.KRAKEN_X60)
            .motorsPerWheel(1)
            .gearRatio(1.0)
            .build());
        
        presets.add(FlywheelConfig.builder()
            .name("Triple Stack")
            .arrangement(FlywheelConfig.WheelArrangement.STACKED)
            .wheelDiameterInches(4.0)
            .wheelWidthInches(1.5)
            .material(WheelMaterial.MEDIUM_SOFT)
            .compressionRatio(0.12)
            .wheelCount(3)
            .motor(FRCMotors.KRAKEN_X60)
            .motorsPerWheel(1)
            .gearRatio(1.0)
            .build());
        
        return presets;
    }
    
    /**
     * Evaluates preset configurations.
     */
    public GenerationResult evaluatePresets(double targetVelocityMps) {
        List<FlywheelConfig> presets = generatePresets();
        List<ScoredConfig> scoredConfigs = new ArrayList<>();
        
        for (FlywheelConfig config : presets) {
            FlywheelSimulator simulator = new FlywheelSimulator(config, gamePiece);
            FlywheelSimulator.SimulationResult result = simulator.simulateForVelocity(targetVelocityMps);
            
            if (result.isAchievable) {
                double score = simulator.scoreConfiguration(targetVelocityMps);
                scoredConfigs.add(new ScoredConfig(config, result, score));
            }
        }
        
        scoredConfigs.sort(Comparator.comparingDouble((ScoredConfig sc) -> sc.score).reversed());
        
        return new GenerationResult(scoredConfigs, presets.size());
    }
    
    private int getWheelCountForArrangement(FlywheelConfig.WheelArrangement arrangement) {
        switch (arrangement) {
            case SINGLE:
                return 1;
            case DUAL_PARALLEL:
            case DUAL_OVER_UNDER:
            case DIFFERENTIAL:
                return 2;
            case STACKED:
                return 3;
            default:
                return 2;
        }
    }
    
    public GamePiece getGamePiece() {
        return gamePiece;
    }
    
    public GenerationParams getParams() {
        return params;
    }
}
