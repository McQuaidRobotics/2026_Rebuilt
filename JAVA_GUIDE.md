# FRC 2026 Java Programming Guide

---

## 1. The Singleton Pattern

A **Singleton** is a design pattern that ensures a class has only **one instance** and provides a global point of access to it.

### Why use it?

In robot code, some things only exist once. For example, you only have one robot field, one `CommandScheduler`, and often one of each subsystem.

### Example in our code

```java
public class FieldVisualizer {
    // 1. Private constructor prevents other classes from making new ones
    private FieldVisualizer() {}

    // 2. A private static class to hold the single instance safely
    private static class SingletonHelper {
        private static final FieldVisualizer INSTANCE = new FieldVisualizer();
    }

    // 3. A public method to get that one instance
    public static FieldVisualizer getInstance() {
        return SingletonHelper.INSTANCE;
    }
}
```

### Pros and Cons

* **Pros:** Prevents multiple instances from conflicting (e.g., two pieces of code trying to control the same hardware), easy to access from anywhere.
* **Cons:** Can make testing harder because the "state" stays around, can hide dependencies between classes.

---

## 2. The `static` Keyword

The `static` keyword means that a member (variable or method) belongs to the **class itself**, rather than to a specific **instance** (object) of the class.

### Why use static?

Look at `SubsystemConstants.java`. We use `static` for things like `MOTOR_ID` because the ID of a motor is a "global truth"—it doesn't change based on how many objects you create.

### How it works

* **Static Variables:** Shared across all instances. You access them using the class name: `SubsystemConstants.kIntake.kRollers.LEADER_MOTOR_ID`.
* **Static Methods:** Can be called without creating an object.
* **Static Blocks:** Used for complex initialization of static variables.

    ```java

    static {
        if (disableAllLogs) {
            disableChainsawLogs = true;
        }
    }
    ```

### Constructors and Static

* **Constructors** run every time you create a new object (`new MyClass()`).
* **Static** members exist even if you **never** call a constructor.
* You cannot use `this` inside a static method because `this` refers to a specific object instance.

---

## 3. Abstract Classes

An `abstract` class is a "half-finished" class. You cannot create an object from it directly; you must "extend" it with a real class.

### Why use abstract classes?

We use this for our hardware abstraction. The `Example` class defines *what* a mechanism can do, while `ExampleReal` and `ExampleSim` define *how* it does it.

```java
public abstract class Example {
    public abstract void setSpeed(double speed); // No "body" {}, just a requirement
}
```

---

## 4. Enums

An `enum` (enumeration) is a special "class" that represents a group of **constants** (unchangeable variables).

### Why use enums?

It makes code much more readable than using numbers.

```java
public enum DebugType {
    SHOOTER, SWERVE, INTAKE
}
```

Instead of saying `if (mode == 1)`, we say `if (mode == DebugType.SHOOTER)`.

---

## 5. Basic Syntax Refresher

### Classes and Objects

* **Class:** The blueprint (e.g., `Swerve.java`).
* **Object/Instance:** The actual thing built from the blueprint (e.g., the `swerve` object in `Subsystems.java`).

### Constructors

The method that runs when you create an object. It always has the same name as the class.

```java
public class Led {
    public Led(int length, int port) { // This is the constructor
        // Setup logic here
    }
}
```

### Access Modifiers

* `public`: Anyone can see/use this.
* `private`: Only this class can see/use this. (Great for hiding complex hardware details!)
* `protected`: This class and its children can see/use this.

### Final

If a variable is `final`, its value cannot be changed after it is set. We use `public static final` for constants.
