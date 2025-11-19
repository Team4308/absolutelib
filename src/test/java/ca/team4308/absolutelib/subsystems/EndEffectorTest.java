package ca.team4308.absolutelib.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;

@Disabled("Requires HAL")
public class EndEffectorTest {

    @Test
    public void testInstantiation() {
        EndEffector ee = new EndEffector();
        assertNotNull(ee);
    }
}
