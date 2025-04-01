#pragma warning disable CA1050

// ReSharper disable RedundantUsingDirective
using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;
// ReSharper enable RedundantUsingDirective

public static class Program
{
    public static void Main(string[] args)
    {
        World world = World.FromStdin();
        GameState gameState = new(world, null, null);
        while (true)
        {
            TurnInfo turnInfo = TurnInfo.FromStdin();
            gameState = gameState.WithUpdatedPods(turnInfo);
            Action[] actions = gameState.DecideAction();
            foreach (Action action in actions)
            {
                Console.WriteLine(action);
            }
        }
        // ReSharper disable once FunctionNeverReturns
    }
    
    public static string NextConsoleLine() => Console.ReadLine() 
                                              ?? throw new NullReferenceException("Couldn't read line from console");
}

public record World(int Laps, int CheckpointCount, Vec2[] Checkpoints)
{
    // Game Constants
    public const int Height = 9000;
    public const int Width = 16000;

    public static World FromStdin()
    {
        int laps = int.Parse(Program.NextConsoleLine());
        int checkpointCount = int.Parse(Program.NextConsoleLine());
        var checkpoints = new Vec2[checkpointCount];
        for (int i = 0; i < checkpointCount; i++)
        {
            string[] inputs = Program.NextConsoleLine().Split(' ');
            checkpoints[i] = new Vec2(double.Parse(inputs[0]), double.Parse(inputs[1]));
        }
        return new World(laps, checkpointCount, checkpoints);
    }
}

public record GameState(World World, Pod[]? Team1Pods, Pod[]? Team2Pods)
{
    /// <summary>Affects how much pod targets are adjusted</summary>
    public const double TargetAdjustmentIndex = 5.0;
    public const int BoostDistanceThreshold = 4000;
    public const double BoostAdjustmentVecLenThreshold = 800;
    public const double BoostDegreesFromTargetThreshold = 5;
    public const double NoThrustThresholdDeg = 105;
    public GameState WithUpdatedPods(TurnInfo turnInfo)
    {
        return this with
        {
            Team1Pods = turnInfo.Team1Pods,
            Team2Pods = turnInfo.Team2Pods,
        };
    }

    public Action[] DecideAction()
    {
        // Invariants
        if (Team1Pods == null) throw new InvalidOperationException("Team1Pods is null");
        if (Team2Pods == null) throw new InvalidOperationException("Team2Pods is null");
        if (Team1Pods.Length != Team2Pods.Length) throw new InvalidOperationException("Team1Pods.Length != Team2Pods.Length");
        
        var actions = new Action[Team1Pods.Length];
        for (int i = 0; i < actions.Length; i++)
        {
            Pod pod = Team1Pods[i];
            
            // Calculate intermediate values
            Vec2 checkpointPos = World.Checkpoints[pod.NextCheckPointId];
            Vec2 dCheckpoint = checkpointPos.Sub(pod.Pos); // delta from pod to checkpoint
            double checkpointDist = dCheckpoint.Length();
            Vec2 paraVel = pod.Vel.Projected(dCheckpoint); // Component of velocity parallel to direction of checkpoint
            Vec2 perpVel = pod.Vel.Sub(paraVel); // Component of velocity perpendicular to direction of checkpoint
            
            // Calculate target
            Vec2 adjustmentVec = perpVel.Scaled(-TargetAdjustmentIndex);
            Vec2 target = checkpointPos.Add(adjustmentVec);
            
            // Calculate intermediate values depend on target
            Vec2 dTarget = target.Sub(pod.Pos);
            // FIXME: (-180, 180) angle
            double fromPodPosToTargetDeg = Math.Atan2(dTarget.Y, dTarget.X) * Util.RadToDeg;
            // FIXME: (-180, 180) angle
            double fromPodDirToTargetDeg = Util.AngleBetweenDeg(pod.RotDeg, fromPodPosToTargetDeg);
            double podDirDegreesFromTarget = Math.Abs(fromPodDirToTargetDeg); 
            
            // Calculate thrust
            string thrust = "100";
            string? msg = null;
            if (podDirDegreesFromTarget > NoThrustThresholdDeg)
            {
                thrust = "0";
                msg = $"Stopped";
            }
            else if (checkpointDist > BoostDistanceThreshold && adjustmentVec.Length() < BoostAdjustmentVecLenThreshold && podDirDegreesFromTarget < BoostDegreesFromTargetThreshold)
            {
                thrust = "BOOST";
                msg = "BOOST";
            }
            
            // Store action for this pod
            actions[i] = new Action(thrust, target, msg);
            
            // Diagnostics
            Console.Error.WriteLine($"Bot {i}:\n  Vel={pod.Vel}\n  Pos={pod.Pos}\n  CheckpointPos={checkpointPos}\n  dCheckpoint={dCheckpoint}\n  perpVel={perpVel}\n  paraVel={paraVel}\n  rot={pod.RotDeg}");
        }
        
        return actions;
    }
}

public record TurnInfo(Pod[] Team1Pods, Pod[] Team2Pods)
{
    public static TurnInfo FromStdin()
    {
        var team1Pods = new Pod[2];
        for (int i = 0; i < 2; i++)
        {
            team1Pods[i] = ReadPodFromStdin();
        }
        var team2Pods = new Pod[2];
        for (int i = 0; i < 2; i++)
        {
            team2Pods[i] = ReadPodFromStdin();
        }
        
        return new TurnInfo(team1Pods, team2Pods);

        Pod ReadPodFromStdin()
        {
            string[] inputs = Program.NextConsoleLine().Split(' ');
            Vec2 pos = new(int.Parse(inputs[0]), int.Parse(inputs[1]));
            Vec2 vel = new(int.Parse(inputs[2]), int.Parse(inputs[3]));
            double rot = double.Parse(inputs[4]);
            int nextCheckpointId = int.Parse(inputs[5]);
            Pod pod = new(pos, vel, rot, nextCheckpointId);
            return pod;
        }
    }
}

// FIXME: RotDeg (0, 360) angle
public record Pod(Vec2 Pos, Vec2 Vel, double RotDeg, int NextCheckPointId);

public record Action(string Thrust, Vec2 Target, string? Msg = null)
{
    public override string ToString()
    {
        string msg = Msg == null ? "" : $" {Msg}";
        return $"{(int)Target.X} {(int)Target.Y} {Thrust}{msg}";
    }
}

public record Vec2(double X, double Y)
{
    public override string ToString()
    {
        return $"({X:F2}, {Y:F2}))";
    }

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
    
    public Vec2 Projected(Vec2 other)
    {
        double dotProduct = Dot(other);
        double otherLengthSquared = other.Dot(other);

        if (otherLengthSquared < 1e-9) // Avoid division by zero for near-zero vectors
        {
            return new Vec2(0, 0);
        }

        double scalar = dotProduct / otherLengthSquared;
        return new Vec2(other.X * scalar, other.Y * scalar);
    }
}

public class Util
{
    public const double DegToRad = Math.PI / 180;
    public const double RadToDeg = 180 / Math.PI;
    
    public static double AngleBetweenDeg(double a, double b)
    {
        if (a < -180) throw new ArgumentException("a must be > -180.");
        if (b < -180) throw new ArgumentException("b must be > -180.");
        
        // Normalise to angles in range (0, 360). This handles (-180, 180) and (0, 360) conventions.
        a = (a + 360) % 360;
        b = (b + 360) % 360;
        
        
        double result = b - a; // A value in the range (-360, 360)
        result = (result + 540) % 360 - 180; // A value in the range (-180, 180)
        return result;
    }
}