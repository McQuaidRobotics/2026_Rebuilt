package igknighters.controllers;

import igknighters.subsystems.Subsystems;
import java.util.function.DoubleSupplier;

public abstract class Controller {
    public abstract DoubleSupplier getTranslationX();

    public abstract DoubleSupplier getTranslationY();

    public abstract DoubleSupplier getRotationX();

    public abstract DoubleSupplier getRotationY();

    public abstract DoubleSupplier getThrottle();

    public abstract DoubleSupplier getShotModifier();

    public abstract void rumble(double magnitude);

    public abstract void bind(Subsystems subsystems);
}
