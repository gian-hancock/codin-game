using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Test program for experimenting with the turning game mechanics.
class Player
{
    public const int WIDTH = 16000;
    public const int HEIGHT = 9000;
    string THRUST = "0";
    // string THRUST = "100";
    // string THRUST = "BOOST";

    double TURN_ANGLE = 180;
    // double TURN_ANGLE = 90;

    Vec2 target;

    static void Main(string[] args)
    {
        Player player = new Player();
        player.Run();
    }

    private void Run()
    {
        string[] inputs;

        int turn = 0;

        while (true)
        {
            // Parse input
            inputs = Console.ReadLine().Split(' ');
            int x = int.Parse(inputs[0]);
            int y = int.Parse(inputs[1]);
            int nextCheckpointX = int.Parse(inputs[2]); // x position of the next check point
            int nextCheckpointY = int.Parse(inputs[3]); // y position of the next check point
            int nextCheckpointDist = int.Parse(inputs[4]); // distance to the next checkpoint
            int nextCheckpointAngle = int.Parse(inputs[5]); // angle between your pod orientation and the direction of the next checkpoint
            inputs = Console.ReadLine().Split(' ');
            int opponentX = int.Parse(inputs[0]);
            int opponentY = int.Parse(inputs[1]);

            // Create Vectors
            Vec2 position = new Vec2(x, y);
            Vec2 nextCheckpoint = new Vec2(nextCheckpointX, nextCheckpointY);
            Vec2 opponent = new Vec2(opponentX, opponentY);

            // Calculate command
            Command command = CalcCommand(position, nextCheckpoint, opponent, turn++);

            // Write command
            Console.WriteLine($"{(int)command.Target.X} {(int)command.Target.Y} {command.Thrust}");
        }
    }

    private Command CalcCommand(Vec2 position, Vec2 nextCheckpoint, Vec2 opponent, int turn)
    {
        // For now, just test chaning the target position randomly every 20 turns and see what happens
        Random rand = new Random();
        if (turn % 8 == 0)
        {
            target = new Vec2(rand.Next(0, WIDTH), rand.Next(0, HEIGHT));
        }

        return new Command("0", target);
    }
}

record Command(string Thrust, Vec2 Target);

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
}