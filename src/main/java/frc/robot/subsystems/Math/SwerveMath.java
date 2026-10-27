package frc.robot.subsystems.Math;

import org.joml.Math;
import org.joml.Vector2f;

public class SwerveMath {
  public static final float Pi = 3.1415927f;
  public static final float HalfPi = Pi * 0.5f;
  public static final float Tau = Pi * 2.0f;

  private SwerveMath() {}

  public static void fieldToRobot(float fieldX, float fieldY, float headingRad, Vector2f out) {
    float cos = Math.cos(headingRad);
    float sin = Math.sin(headingRad);

    out.x = cos * fieldX + sin * fieldY;
    out.y = -sin * fieldX + cos * fieldY;
  }

  public static void robotToField(float robotX, float robotY, float headingRad, Vector2f out) {
    float cos = Math.cos(headingRad);
    float sin = Math.sin(headingRad);

    out.x = cos * robotX - sin * robotY;
    out.y = sin * robotX + cos * robotY;
  }

  public static float signSquare(float value) {
    if (value < 0.0f) {
      return -value * value;
    }

    return value * value;
  }

  public static float headingHoldOmega(
      float targetHeadingRad, float headingRad, float kP, float maxOmegaRadPerSec) {
    return clamp(wrapRad(targetHeadingRad - headingRad) * kP, -maxOmegaRadPerSec, maxOmegaRadPerSec);
  }

  public static void applyNormalCompensation(
      Vector2f velocity, float omegaRadPerSec, float maxOmegaRadPerSec, float k, Vector2f out) {
    out.set(velocity);

    float speedSquared = velocity.lengthSquared();

    if (k == 0.0f || speedSquared == 0.0f || maxOmegaRadPerSec <= 0.0f) {
      return;
    }

    float speed = Math.sqrt(speedSquared);
    float omegaScale = clamp(omegaRadPerSec / maxOmegaRadPerSec, -1.0f, 1.0f);
    float normalMagnitude = k * speed * omegaScale;
    float normalX = velocity.y / speed;
    float normalY = -velocity.x / speed;

    out.x += normalX * normalMagnitude;
    out.y += normalY * normalMagnitude;

    float compensatedSpeed = out.length();

    if (compensatedSpeed > 0.0f) {
      out.mul(speed / compensatedSpeed);
    }
  }

  public static void arcDisplacement(
      float previousHeadingRad, float headingRad, float distanceM, Vector2f out) {
    float deltaHeading = wrapRad(headingRad - previousHeadingRad);

    if (Math.abs(deltaHeading) < 0.00001f) {
      out.x = Math.cos(headingRad) * distanceM;
      out.y = Math.sin(headingRad) * distanceM;
      return;
    }

    float radius = distanceM / deltaHeading;
    float previousRadiusAngle = previousHeadingRad - HalfPi;
    float currentRadiusAngle = headingRad - HalfPi;
    float previousX = Math.cos(previousRadiusAngle) * radius;
    float previousY = Math.sin(previousRadiusAngle) * radius;
    float currentX = Math.cos(currentRadiusAngle) * radius;
    float currentY = Math.sin(currentRadiusAngle) * radius;

    out.x = currentX - previousX;
    out.y = currentY - previousY;
  }

  public static void calculateModuleTargets(
      float robotX,
      float robotY,
      float omega,
      Vector2f centerOfRotation,
      Vector2f[] modulePositions,
      float[] speeds,
      float[] angles) {
    for (int i = 0; i < modulePositions.length; i++) {
      Vector2f position = modulePositions[i];
      float radiusX = position.x - centerOfRotation.x;
      float radiusY = position.y - centerOfRotation.y;
      float wheelX = robotX - omega * radiusY;
      float wheelY = robotY + omega * radiusX;

      speeds[i] = Math.sqrt(wheelX * wheelX + wheelY * wheelY);
      angles[i] = Math.atan2(wheelY, wheelX);
    }
  }

  public static float desaturate(float[] speeds, float maxSpeed) {
    float highest = maxAbs(speeds);

    if (highest <= maxSpeed) {
      return 1.0f;
    }

    float scale = maxSpeed / highest;

    for (int i = 0; i < speeds.length; i++) {
      speeds[i] *= scale;
    }

    return scale;
  }

  public static void limitVelocity(
      Vector2f wanted,
      Vector2f current,
      float dt,
      float maxVelocity,
      float maxForwardAccel,
      float maxFrontAccel,
      float maxSideAccel,
      float maxSkidAccel,
      Vector2f acceleration) {
    if (dt <= 0.0f) {
      current.set(wanted);
      return;
    }

    acceleration.set(wanted).sub(current).div(dt);
    limitAcceleration(
        acceleration,
        current,
        maxVelocity,
        maxForwardAccel,
        maxFrontAccel,
        maxSideAccel,
        maxSkidAccel);
    current.fma(dt, acceleration);
  }

  public static void limitAcceleration(
      Vector2f acceleration,
      Vector2f currentVelocity,
      float maxVelocity,
      float maxForwardAccel,
      float maxFrontAccel,
      float maxSideAccel,
      float maxSkidAccel) {
    float currentSpeed = currentVelocity.length();
    float speedRatio = clamp(currentSpeed / maxVelocity, 0.0f, 1.0f);
    float forwardLimit = maxForwardAccel * (1.0f - speedRatio);
    float acceleratingWithVelocity =
        acceleration.x * currentVelocity.x + acceleration.y * currentVelocity.y;

    if (acceleratingWithVelocity > 0.0f) {
      clampMagnitude(acceleration, forwardLimit);
    }

    acceleration.x = clamp(acceleration.x, -maxFrontAccel, maxFrontAccel);
    acceleration.y = clamp(acceleration.y, -maxSideAccel, maxSideAccel);

    clampMagnitude(acceleration, maxSkidAccel);
  }

  public static int nearestModule(Vector2f[] modulePositions, float x, float y) {
    int nearest = 0;
    float bestDistance = distanceSquared(modulePositions[0], x, y);

    for (int i = 1; i < modulePositions.length; i++) {
      float distance = distanceSquared(modulePositions[i], x, y);

      if (distance < bestDistance) {
        nearest = i;
        bestDistance = distance;
      }
    }

    return nearest;
  }

  public static float wrapRad(float radians) {
    while (radians >= Pi) {
      radians -= Tau;
    }

    while (radians < -Pi) {
      radians += Tau;
    }

    return radians;
  }

  public static float clamp(float value, float min, float max) {
    if (value < min) {
      return min;
    }

    if (value > max) {
      return max;
    }

    return value;
  }

  public static void clampMagnitude(Vector2f vector, float maxMagnitude) {
    float lengthSquared = vector.lengthSquared();
    float maxSquared = maxMagnitude * maxMagnitude;

    if (lengthSquared <= maxSquared || lengthSquared == 0.0f) {
      return;
    }

    vector.mul(maxMagnitude / Math.sqrt(lengthSquared));
  }

  private static float maxAbs(float[] values) {
    float highest = 0.0f;

    for (int i = 0; i < values.length; i++) {
      float value = Math.abs(values[i]);

      if (value > highest) {
        highest = value;
      }
    }

    return highest;
  }

  private static float distanceSquared(Vector2f position, float x, float y) {
    float dx = position.x - x;
    float dy = position.y - y;
    return dx * dx + dy * dy;
  }
}
