using System.Globalization;
using UnityEngine;

namespace SkyLead.Simulator
{
    public enum CommandKind
    {
        Command,
        StreamOn,
        TakeOff,
        Land,
        Move,
        Turn,
        Photo,
        State,
    }

    /// <summary>One Tello SDK command of docs/simulator.md, section 4, parsed and checked against its limits.</summary>
    public readonly struct DroneCommand
    {
        // The real Tello's limits (docs/simulator.md, section 4).
        public const float MinMoveCm = 20f;
        public const float MaxMoveCm = 500f;
        public const float MinTurnDeg = 1f;
        public const float MaxTurnDeg = 360f;

        public readonly CommandKind Kind;
        /// <summary>For a move: the direction in the drone's own frame, x right, y forward, z up.</summary>
        public readonly Vector3 Direction;
        /// <summary>cm for a move; degrees for a turn, clockwise positive like cw.</summary>
        public readonly float Amount;

        DroneCommand(CommandKind kind, Vector3 direction = default, float amount = 0f)
        {
            Kind = kind;
            Direction = direction;
            Amount = amount;
        }

        /// <summary>Parse one line; on failure <paramref name="error"/> is the reply the contract gives.</summary>
        public static bool TryParse(string line, out DroneCommand command, out string error)
        {
            command = default;
            error = null;
            string[] words = line.Trim().Split(' ', System.StringSplitOptions.RemoveEmptyEntries);
            if (words.Length == 1)
            {
                switch (words[0])
                {
                    case "command": command = new DroneCommand(CommandKind.Command); return true;
                    case "streamon": command = new DroneCommand(CommandKind.StreamOn); return true;
                    case "takeoff": command = new DroneCommand(CommandKind.TakeOff); return true;
                    case "land": command = new DroneCommand(CommandKind.Land); return true;
                    case "photo": command = new DroneCommand(CommandKind.Photo); return true;
                    case "state?": command = new DroneCommand(CommandKind.State); return true;
                }
            }
            else if (words.Length == 2 && float.TryParse(words[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float value))
            {
                Vector3? direction = words[0] switch
                {
                    "forward" => new Vector3(0f, 1f, 0f),
                    "back" => new Vector3(0f, -1f, 0f),
                    "right" => new Vector3(1f, 0f, 0f),
                    "left" => new Vector3(-1f, 0f, 0f),
                    "up" => new Vector3(0f, 0f, 1f),
                    "down" => new Vector3(0f, 0f, -1f),
                    _ => null,
                };
                if (direction != null)
                {
                    if (value < MinMoveCm || value > MaxMoveCm)
                        return OutOfRange(out error);
                    command = new DroneCommand(CommandKind.Move, direction.Value, value);
                    return true;
                }
                if (words[0] is "cw" or "ccw")
                {
                    if (value < MinTurnDeg || value > MaxTurnDeg)
                        return OutOfRange(out error);
                    command = new DroneCommand(CommandKind.Turn, amount: words[0] == "cw" ? value : -value);
                    return true;
                }
            }
            error = "error unknown command";
            return false;
        }

        static bool OutOfRange(out string error)
        {
            error = "error out of range";
            return false;
        }
    }
}
