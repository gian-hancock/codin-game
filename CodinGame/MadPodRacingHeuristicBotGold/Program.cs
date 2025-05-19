#pragma warning disable CA1050

/*
TODO:
    - Incorporate Knowledge of next checkpoint
        - Switch to next checkpoint early
            - Predict position if target is switched up to 8 frames early
                - Set target to next checkpoint
                    - loop 8
                        - perform target selection
                        - perform rotation
                        - perform acceleration
                        - Perform translation
                        - Perform friction 0.85
                        - Truncate speed x and y
                        - Round position x and y
        - target driving line based on next checkpoint
    - Teammate avoidance: Teammate who is on loosing position dodges winning teammate in upcoming collision
        - If (v1.dot(v2) < 0 && collisionLikely())
            - Calculate collision point
            - calculate 2 directions pependicular to collision point
            - choose the one which is closest to current velocity to set as target
    - Blocker: Teammate which falls behind targets opponent leading bot.
    - Improve shield usage
        - Sometimes it doesn't activate when I think it should?
        - It shouldn't activate when the bump will make the pod travel in the target direction.
*/


// There's some weirdness where I need explicit imports on the CodinGame platform that I don't need locally 
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
        Console.Error.WriteLine(world);
        int turn = 0;
        GameState gameState = new(
            world,
            0,
            0,
            new int[4],
            Enumerable.Repeat(1, 4).ToArray());
        while (true)
        {
            TurnInfo turnInfo = TurnInfo.FromStdin();
            (gameState, Action[] actions) = gameState.Next(turnInfo, turn);
            foreach (Action action in actions)
            {
                Console.WriteLine(action);
            }

            turn++;
        }
        // ReSharper disable once FunctionNeverReturns
    }

    public static string NextConsoleLine() => Console.ReadLine()
                                              ?? throw new NullReferenceException("Couldn't read line from console");
}

public record GameState(World World, int BoostTurn, int Turn, int[] LapsCompleted, int[] LastNextCheckpoint)
{
    /// <summary>Affects how much pod targets are adjusted</summary>
    public const double TargetAdjustmentIndex = 5.0;

    public const int BoostDistanceThreshold = 5000;
    public const double BoostAdjustmentVecLenThreshold = 800;
    public const double BoostDegreesFromTargetThreshold = 5;
    public const double NoThrustThresholdDeg = 105;
    public const double CollisionEstimationRadius = World.PodSize - 18;
    public const int BoostDelay = 25;
    public const double BlockTargetCloseSpeed = 150;
    public const double ScoringNeutralDistanceThreshold = World.Width;
    // Very roughly experimentally determined.

    public double EstimatedOvershoot(double speed)
    {
        return 8 * speed - 0.00125 * (speed * speed);
    }

    public (GameState gameState, Action[] actions) Next(TurnInfo turnInfo, int turn)
    {
        // Invariants
        if (turnInfo.Pods.Length != 4) throw new InvalidOperationException("There must be 4 pods");

        // Updated state variables
        int newBoostTurn = BoostTurn;
        int[] newLapsCompleted = [..LapsCompleted];
        int[] newLastNextCheckpoint = new int[4];

        // Count laps (this information isn't given to us)
        for (int podIdx = 0; podIdx < turnInfo.Pods.Length; podIdx++)
        {
            bool reachedNewCheckpoint = turnInfo.Pods[podIdx].NextCheckPointId != LastNextCheckpoint[podIdx];
            bool isAtFirstCheckpoint = turnInfo.Pods[podIdx].NextCheckPointId == 1;
            if (reachedNewCheckpoint && isAtFirstCheckpoint) newLapsCompleted[podIdx]++;
            newLastNextCheckpoint[podIdx] = turnInfo.Pods[podIdx].NextCheckPointId;
        }

        // Calculate scores
        double[] scores = new double[4];
        for (int podIdx = 0; podIdx < turnInfo.Pods.Length; podIdx++)
        {
            Pod pod = turnInfo.Pods[podIdx];
            double checkpointDistance = pod.Pos.Sub(World.Checkpoints[pod.NextCheckPointId]).Length();
            double pointsPerCheckpoint = 1.0 / World.Checkpoints.Length;
            double lapScore = newLapsCompleted[podIdx];
            int checkpointsCompletedThisLap =
                (pod.NextCheckPointId + World.Checkpoints.Length - 1) % World.Checkpoints.Length;
            double checkpointScore = pointsPerCheckpoint * checkpointsCompletedThisLap;
            double checkpointProgressScore = pointsPerCheckpoint -
                                             pointsPerCheckpoint * checkpointDistance / ScoringNeutralDistanceThreshold;
            scores[podIdx] = lapScore + checkpointScore + checkpointProgressScore;
            Console.Error.WriteLine(
                $"score: {lapScore:F2} + {checkpointScore:F2} + {checkpointProgressScore:F2} = {scores[podIdx]:F2}");
        }

        for (int podIdx = 0; podIdx < 4; podIdx++)
        {
            // TODO:
            // 1. Count laps
            // 2. create score: (lap)+(normalised dis to next checkpoint)
            // 3. trailing pod dodges leading pot (to prevent interference)
            // 4. trailing pod interrupts opponent's leading pod
            //   - either target just ahead of opponent.
            //   - target the checkpoint opponent is up to.
        }

        var actions = new Action[2];
        for (int podIdx = 0; podIdx < actions.Length; podIdx++)
        {
            Pod pod = turnInfo.Pods[podIdx];

            // FIXME: Hack to test blocker   
            // if (podIdx == 1)
            // {
            //     Pod targetPod = turnInfo.Pods[2];
            //     double targetPodDis = targetPod.Pos.Sub(pod.Pos).Length();
            //     Vec2 blockTarget = targetPod.Pos.Add(targetPod.Vel.Scaled(targetPodDis / BlockTargetCloseSpeed));
            //     actions[podIdx] = new Action("100", blockTarget);
            //     continue;
            // }

            // Calculate intermediate values
            Vec2 checkpointPos = World.Checkpoints[pod.NextCheckPointId];
            Vec2 dCheckpoint = checkpointPos.Sub(pod.Pos); // delta from pod to checkpoint
            double checkpointDist = dCheckpoint.Length();
            Vec2 paraVel = pod.Vel.Projected(dCheckpoint); // Component of velocity parallel to direction of checkpoint
            Vec2 perpVel = pod.Vel.Sub(paraVel); // Component of velocity perpendicular to direction of checkpoint

            string? msg = null;

            // Prematurely advance to next checkpoint if we are already going to hit current checkpoint
            bool willHitCheckpoint =
                Util.CollisionLikely(pod.Pos, pod.Vel, checkpointPos, World.CheckpointRadius, 50, 5);
            if (willHitCheckpoint)
            {
                checkpointPos = World.Checkpoints[(pod.NextCheckPointId + 1) % World.Checkpoints.Length];
                msg = "SKIP";
            }

            // Overshoot correction
            double overshootDist = EstimatedOvershoot(paraVel.Length()) - checkpointDist;
            Vec2 overshoot = overshootDist > 0
                ? paraVel.Normalized().Scaled(overshootDist)
                : new Vec2(0, 0);
            // Overshoot which affects making it to the subsequent checkpoint
            Vec2 effectiveOvershoot = overshoot.Projected(World.ToNextCheckPoint[pod.NextCheckPointId]);
            Vec2 overshootCorrection = new Vec2(0, 0);
            if (effectiveOvershoot.Dot(World.ToNextCheckPoint[pod.NextCheckPointId]) < 0)
            {
                overshootCorrection = effectiveOvershoot.Scaled(-1);
            }

            // Drift correction
            Vec2 driftCorrection = perpVel.Scaled(-TargetAdjustmentIndex);

            // Calculate target
            Vec2 target = checkpointPos.Add(driftCorrection).Add(overshootCorrection);

            // Calculate intermediate values depending on target
            Vec2 dTarget = target.Sub(pod.Pos);
            // FIXME: (-180, 180) angle
            double fromPodPosToTargetDeg = Math.Atan2(dTarget.Y, dTarget.X) * Util.RadToDeg;
            // FIXME: (-180, 180) angle
            double fromPodDirToTargetDeg = Util.AngleBetweenDeg(pod.RotDeg, fromPodPosToTargetDeg);
            double podDirDegreesFromTarget = Math.Abs(fromPodDirToTargetDeg);

            // Estimated next positions
            var opponentNextPosEst = new Vec2[2];
            for (int opponentIdx = 0; opponentIdx < opponentNextPosEst.Length; opponentIdx++)
            {
                Pod opponentPod = turnInfo.Pods[2 + opponentIdx]; // NOTE: first opponent pod at Pods[2]
                opponentNextPosEst[opponentIdx] = opponentPod.Pos.Add(opponentPod.Vel);
            }

            Vec2 nextPosEst = pod.Pos.Add(pod.Vel);

            // Estimate whether collisions will occur
            bool collWith1 = Util.CircleCollision(nextPosEst, CollisionEstimationRadius, opponentNextPosEst[0],
                CollisionEstimationRadius);
            bool collWith2 = Util.CircleCollision(nextPosEst, CollisionEstimationRadius, opponentNextPosEst[1],
                CollisionEstimationRadius);

            // Calculate thrust
            string thrust = "100";

            // Shield rules
            if (collWith1 || collWith2)
            {
                thrust = "SHIELD";
                msg = "SHIELD";
            }
            // No thrust rules
            else if (podDirDegreesFromTarget > NoThrustThresholdDeg)
            {
                thrust = "0";
                // msg = $"Stopped";
            }
            // Boost rules
            else if (Turn >= newBoostTurn
                     && checkpointDist > BoostDistanceThreshold
                     && driftCorrection.Length() < BoostAdjustmentVecLenThreshold
                     && podDirDegreesFromTarget < BoostDegreesFromTargetThreshold)
            {
                newBoostTurn = Turn + BoostDelay;
                thrust = "BOOST";
                // msg = "BOOST";
            }

            // Store action for this pod
            actions[podIdx] = new Action(thrust, target, msg);

            // Diagnostics
            Console.Error.WriteLine($@"Bot {podIdx}:
  Vel={pod.Vel}
  Pos={pod.Pos}
  CheckpointPos={checkpointPos}
  dCheckpoint={dCheckpoint}
  perpVel={perpVel}
  paraVel={paraVel}
  rot={pod.RotDeg}
  col=({collWith1}, {collWith2})
  overshootDist={overshootDist}
  overshoot={overshoot}
  subsequentCheckpoint={World.ToNextCheckPoint[pod.NextCheckPointId]}
  effectiveOvershoot={effectiveOvershoot}
  overshootCorrection={overshootCorrection}
  scores={string.Join(", ", scores.Select(s => s.ToString("F2")))}");
        }

        return (
            this with
            {
                BoostTurn = newBoostTurn,
                Turn = Turn + 1,
                LastNextCheckpoint = newLastNextCheckpoint,
                LapsCompleted = newLapsCompleted,
            },
            actions);
    }
}

public record TurnInfo(Pod[] Pods)
{
    public static TurnInfo FromStdin()
    {
        var pods = new Pod[4];
        for (int i = 0; i < 4; i++)
        {
            pods[i] = ReadPodFromStdin();
        }

        return new TurnInfo(pods);

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

public record World(int Laps, Vec2[] Checkpoints, Vec2[] ToNextCheckPoint)
{
    // Game Constants
    public const int Height = 9000;
    public const int Width = 16000;
    public const int PodSize = 400;
    public const int CheckpointRadius = 600;

    public static World FromStdin()
    {
        int laps = int.Parse(Program.NextConsoleLine());
        int checkpointCount = int.Parse(Program.NextConsoleLine());
        var checkpoints = new Vec2[checkpointCount];
        var toNextCheckPoint = new Vec2[checkpointCount];
        for (int i = 0; i < checkpointCount; i++)
        {
            string[] inputs = Program.NextConsoleLine().Split(' ');
            checkpoints[i] = new Vec2(double.Parse(inputs[0]), double.Parse(inputs[1]));
        }

        for (int i = 0; i < checkpointCount; i++)
        {
            Vec2 pos = checkpoints[i];
            Vec2 nextPos = checkpoints[(i + 1) % checkpointCount];
            toNextCheckPoint[i] = nextPos.Sub(pos);
        }

        return new World(laps, checkpoints, toNextCheckPoint);
    }

    public override string ToString()
    {
        return $@"World {{
    Laps = {Laps},
    Checkpoints = [{string.Join(", ", Checkpoints.Select(x => $"{x.X} {x.Y}"))}],
    ToNextCheckPoint = [{string.Join(", ", ToNextCheckPoint.Select(x => $"{x.X} {x.Y}"))}],
}}";
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
        return len < Util.Epsilon
            ? new Vec2(0, 0)
            : new Vec2(X / len, Y / len);
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

        if (otherLengthSquared < Util.Epsilon) // Avoid division by zero for near-zero vectors
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
    public const double Epsilon = 1e-9;

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

    public static bool CircleCollision(Vec2 p1, double r1, Vec2 p2, double r2)
    {
        double distance = p1.Sub(p2).Length();
        return distance <= r1 + r2;
    }

    public static bool CollisionLikely(Vec2 position, Vec2 velocity, Vec2 targetPos, int targetRadius,
        int radiusPerTick, int maxTicks)
    {
        int ticks = 0;
        while (targetRadius > 0)
        {
            if (++ticks > maxTicks) break;
            
            bool collision = CircleCollision(position, targetRadius, targetPos, 0);
            if (collision) return true;

            position = position.Add(velocity);
            targetRadius -= radiusPerTick;
        }

        return false;
    }
}