// This bot got me from Bronze to Gold

using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// - Target is based on next checkpoint position.
/// - Target is adjusted based on component of velocity perpendicular direction to checkpoint.
/// - Boost if distance > threshold and adjustment vector is smaller than threshold.
/// </summary>
class Player
{
    public const int WIDTH = 16000;
    public const int HEIGHT = 9000;
    public const int MIN_BOOST_DISTANCE = 4000;
    public const double MAX_BOOST_ADJUSTMENT_VECTOR = 800;
    public const double DEG_TO_RAD = Math.PI / 180;
    public const double RAD_TO_DEG = 180 / Math.PI;
    double TARGET_ADJUSTMENT_INDEX = 5.0;
    public const double NO_THRUST_THRESHOLD_DEG = 75;

    Vec2 target;

    static void Main(string[] args)
    {
        Player player = new Player();
        player.Run();
    }

    private void Run()
    {
        string[] inputs;

        // loop state
        int turn = 0;
        Vec2 prevPosition = null;

        while (true)
        {
            // Parse input
            inputs = Console.ReadLine().Split(' ');
            int x = int.Parse(inputs[0]);
            int y = int.Parse(inputs[1]);
            int nextCheckpointX = int.Parse(inputs[2]); // x position of the next check point
            int nextCheckpointY = int.Parse(inputs[3]); // y position of the next check point
            int nextCheckpointDist = int.Parse(inputs[4]); // distance to the next checkpoint
            double nextCheckpointAngle = int.Parse(inputs[5]) * DEG_TO_RAD; // angle between your pod orientation and the direction of the next checkpoint
            inputs = Console.ReadLine().Split(' ');
            int opponentX = int.Parse(inputs[0]);
            int opponentY = int.Parse(inputs[1]);

            // Create Vectors
            Vec2 position = new Vec2(x, y);
            if (prevPosition == null)
            {
                prevPosition = position;
            }
            Vec2 velocity = position.Sub(prevPosition);
            Vec2 nextCheckpoint = new Vec2(nextCheckpointX, nextCheckpointY);
            Vec2 opponent = new Vec2(opponentX, opponentY);

            // Calculate command
            Command command = CalcCommand(position, velocity, nextCheckpoint, opponent, nextCheckpointAngle, nextCheckpointDist, turn);

            // Write command
            Console.WriteLine(command.ToString());

            // Update loop state
            prevPosition = position;
            turn++;
        }
    }

    private Command CalcCommand(Vec2 position, Vec2 velocity, Vec2 nextCheckpoint, Vec2 opponent, double nextCheckPointAngle, double nextCheckpointDist, int turn)
    {
        // Find component of velocity perpendicular to direction to checkpoint
        Vec2 checkPointDir = nextCheckpoint.Sub(position).Normalized();
        Vec2 velocityPerpendicular = velocity.Sub(checkPointDir.Scaled(velocity.Dot(checkPointDir)));
        Vec2 adjustmentVector = velocityPerpendicular.Scaled(-TARGET_ADJUSTMENT_INDEX);

        // Calculate target adjustment
        Vec2 adjustedTarget = nextCheckpoint.Add(adjustmentVector);

        // Calculate thrust
        string thrust = "100";
        string msg = null;
        if (Math.Abs(nextCheckPointAngle) * RAD_TO_DEG > NO_THRUST_THRESHOLD_DEG)
        {
            thrust = "0";
            msg = "Stopped";
        }
        else if (nextCheckpointDist > MIN_BOOST_DISTANCE && adjustmentVector.Length() < MAX_BOOST_ADJUSTMENT_VECTOR)
        {
            thrust = "BOOST";
            msg = "BOOST";
        }

        // double cosThea = Math.Cos(nextCheckPointAngle);
        // double cosThetaAdjusted = Math.Pow(cosThea, THRUST_EXPONENT);
        // int thrust = (int)(Math.Clamp(cosThetaAdjusted * 100, 0, 100));

        // Diagnostic
        Console.Error.WriteLine($"NextCheckpointAngle={nextCheckPointAngle * RAD_TO_DEG}");


        return new Command(thrust, adjustedTarget, msg);
    }
}

record Command(string Thrust, Vec2 Target, string? Msg = null)
{
    public override string ToString()
    {
        string message = Msg != null ? $" {Msg}" : "";
        return $"{(int)Target.X} {(int)Target.Y} {Thrust}{message}";
    }
}

record Vec2(double X, double Y)
{
    public double Length()
    {
        return Math.Sqrt(X * X + Y * Y);
    }

    public Vec2 Normalized()
    {
        double len = Length();
        return new Vec2(X / len, Y / len);
    }

    public Vec2 Sub(Vec2 other)
    {
        return new Vec2(X - other.X, Y - other.Y);
    }

    public Vec2 Add(Vec2 other)
    {
        return new Vec2(X + other.X, Y + other.Y);
    }

    public double Dot(Vec2 other)
    {
        return X * other.X + Y * other.Y;
    }

    public double AngleTo(Vec2 other)
    {
        // Compute dot and cross product
        double dot = Dot(other);
        double cross = X * other.Y - Y * other.X;

        // Compute the signed angle
        return Math.Atan2(cross, dot);
    }

    public Vec2 Rotated(double angle)
    {
        double cos = Math.Cos(angle);
        double sin = Math.Sin(angle);

        return new Vec2(
            X * cos - Y * sin,
            X * sin + Y * cos
        );
    }

    public Vec2 Scaled(double scale)
    {
        return new Vec2(X * scale, Y * scale);
    }
}