package childposepredictor;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Child {
    private Translation3d translation;
    private Color color;
    // the idea will be to only hit kids who hold a sign with a red dot and we should hit that sign
    public enum Color {
        RED,
        GREEN,
    }

    public Child(Translation3d translation, Color color) {
        this.translation = translation;
        this.color = color;
    }

    public boolean shouldHitChild() { // less then 5 away and holding red
        return this.color == Color.RED && this.translation.getDistance(new Translation3d()) < 5.0;
    }

    public void setTranslation(Translation3d translation) {
        this.translation = translation;
    }

    public void setColor(Color color) {
        this.color = color;
    }
}
