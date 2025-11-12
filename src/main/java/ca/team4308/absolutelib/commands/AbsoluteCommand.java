package ca.team4308.absolutelib.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Command wrapper that can run normally or with an optional timeout.
 * - new AbsoluteCommand(inner)           -> no timeout
 * - new AbsoluteCommand(inner, 2.5)      -> 2.5s timeout
 * - new AbsoluteCommand(inner, supplier) -> dynamic timeout
 * - AbsoluteCommand.of(inner)
 * - AbsoluteCommand.withTimeout(inner, seconds)
 */
public class AbsoluteCommand extends Command {
    private final Command inner;
    private final Timer timer = new Timer();
    private DoubleSupplier secondsSupplier; 

    public AbsoluteCommand(Command inner) {
        this(inner, (DoubleSupplier) null);
    }

    public AbsoluteCommand(Command inner, double seconds) {
        this(inner, () -> seconds);
    }

    public AbsoluteCommand(Command inner, DoubleSupplier secondsSupplier) {
        this.inner = inner;
        this.secondsSupplier = secondsSupplier;
    }

    public static AbsoluteCommand of(Command inner) {
        return new AbsoluteCommand(inner);
    }

    public static AbsoluteCommand withTimeout(Command inner, double seconds) {
        return new AbsoluteCommand(inner, seconds);
    }

    public AbsoluteCommand withTimeout(DoubleSupplier supplier) {
        this.secondsSupplier = supplier;
        return this;
    }

    public AbsoluteCommand withoutTimeout() {
        this.secondsSupplier = null;
        return this;
    }

    private boolean timeoutEnabled() {
        if (secondsSupplier == null) return false;
        double s = Math.max(0.0, secondsSupplier.getAsDouble());
        return Double.isFinite(s) && s > 0.0;
    }

    @Override
    public void initialize() {
        if (timeoutEnabled()) {
            timer.reset();
            timer.start();
        } else {
            timer.stop();
            timer.reset();
        }
        inner.initialize();
    }

    @Override
    public void execute() {
        inner.execute();
    }

    @Override
    public void end(boolean interrupted) {
        inner.end(interrupted);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (inner.isFinished()) return true;
        if (timeoutEnabled()) {
            double limit = Math.max(0.0, secondsSupplier.getAsDouble());
            return limit > 0.0 && timer.hasElapsed(limit);
        }
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return inner.getRequirements();
    }

    @Override
    public boolean runsWhenDisabled() {
        return inner.runsWhenDisabled();
    }
}
