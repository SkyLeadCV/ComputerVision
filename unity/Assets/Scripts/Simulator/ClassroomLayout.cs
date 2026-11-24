using UnityEngine;

namespace SkyLead.Simulator
{
    /// <summary>A person placed in the room: floor position in room cm and heading in degrees, clockwise from +y.</summary>
    public readonly struct PersonPlacement
    {
        public readonly Vector2 PositionCm;
        public readonly float HeadingDeg;

        public PersonPlacement(float xCm, float yCm, float headingDeg)
        {
            PositionCm = new Vector2(xCm, yCm);
            HeadingDeg = headingDeg;
        }
    }

    /// <summary>
    /// The measured classroom of docs/classroom.md, in its room frame: cm, origin at the bottom-left outer corner,
    /// x right, y up the plan, z up. Everything the simulator builds reads its sizes from here.
    /// </summary>
    public static class ClassroomLayout
    {
        public const float WidthCm = 460f;
        public const float DepthCm = 560f;
        public const float CeilingCm = 300f;
        public const float TileCm = 60f;

        // The whole closed part of the plan, from the floor to the ceiling: anything that blocks the way is solid,
        // so the height of the real tables does not matter.
        public static readonly Vector2 TablesMinCm = new(120f, 0f);
        public static readonly Vector2 TablesMaxCm = new(370f, 310f);

        public static readonly Vector2[] FreeFloorCm =
        {
            new(0f, 0f), new(120f, 0f), new(120f, 310f), new(370f, 310f),
            new(370f, 0f), new(460f, 0f), new(460f, 560f), new(0f, 560f),
        };

        public static readonly Vector2 TakeOffPadCm = new(60f, 60f);
        public const float TakeOffHeadingDeg = 0f;
        // docs/simulator.md: inside the 76-83 cm the real drone holds.
        public const float TakeOffHeightCm = 80f;

        // Both in the top band: a lying person (about 170 x 50 cm) in an arm would block it, and the right arm
        // leaves the drone only a 42 cm lane.
        public static readonly PersonPlacement[] Victims =
        {
            new(100f, 450f, 90f),
            new(360f, 470f, 90f),
        };

        // Waits against the top wall, above the tables block and facing it: clear of the lines up both arms.
        public static readonly PersonPlacement Rescuer = new(230f, 530f, 180f);

        /// <summary>Room cm to Unity metres: Unity has y up, so the room's (x, y, z) becomes (x, z, y), as in the map.</summary>
        public static Vector3 ToUnity(float xCm, float yCm, float zCm) => new Vector3(xCm, zCm, yCm) / 100f;

        /// <summary>A room heading (clockwise from +y) as a Unity rotation: Unity's yaw is also clockwise seen from above.</summary>
        public static Quaternion HeadingToUnity(float headingDeg) => Quaternion.Euler(0f, headingDeg, 0f);
    }
}
