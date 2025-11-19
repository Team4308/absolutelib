package ca.team4308.absolutelib.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;

@Disabled("Requires HAL")
public class VisionTest {

    @Test
    public void testInstantiation() {
        Vision vision = new Vision();
        assertNotNull(vision);
    }
}
