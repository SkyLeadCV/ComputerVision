using System.Globalization;
using UnityEngine;

namespace SkyLead.Simulator
{
    /// <summary>
    /// The simulated Tello of docs/simulator.md, section 3: it flies the commands at the real drone's pace, stops at
    /// anything in its way, and reports its state as the Tello does. It never tells anyone where it is.
    /// Positions are in the room frame of <see cref="ClassroomLayout"/>: cm, x right, y up the plan, z up.
    /// </summary>
    public sealed class SimulatedDrone : MonoBehaviour
    {
        [Tooltip("Only colliders on these layers stop the drone: the classroom, never the map drawn in the same scene.")]
        [SerializeField] LayerMask obstacles = Physics.DefaultRaycastLayers;

        [Tooltip("1 is the real drone's pace. Higher values fly the same moves faster.")]
        [SerializeField, Min(0.1f)] float timeScale = 1f;

        // docs/protocol.md's drone_radius_cm: the sphere swept along each move.
        public const float RadiusCm = 9f;
        // The pace of explorer.py's SyntheticDrone (SYNTHETIC_FORWARD_SPEED_CM_S, SYNTHETIC_YAW_SPEED_DEG_S), so a
        // flight takes about as long in both stand-ins.
        public const float SpeedCmS = 50f;
        public const float YawSpeedDegS = 90f;
        // A drone stopped exactly at the contact would start its next sweep already touching, which a sphere cast does
        // not report, so it stops this short of it.
        const float ContactGapCm = 1f;
        // The contract keeps the battery full in v1.
        const int BatteryPct = 100;

        Vector3 positionCm;
        float headingDeg;
        Vector3 targetCm;
        float targetHeadingDeg;
        string pendingReply;

        public bool Flying { get; private set; }

        public bool Busy => pendingReply != null;

        public Vector3 PositionCm => positionCm;

        /// <summary>Heading as the Tello reports yaw: from -180 to 180, 0 at take-off, clockwise positive.</summary>
        public float YawDeg => Mathf.Repeat(headingDeg + 180f, 360f) - 180f;

        public float TimeScale
        {
            get => timeScale;
            set => timeScale = value;
        }

        /// <summary>A new flight: landed on the take-off pad, facing the take-off heading (docs/simulator.md, section 2).</summary>
        public void ResetToPad()
        {
            Vector2 pad = ClassroomLayout.TakeOffPadCm;
            positionCm = targetCm = new Vector3(pad.x, pad.y, 0f);
            headingDeg = targetHeadingDeg = ClassroomLayout.TakeOffHeadingDeg;
            Flying = false;
            pendingReply = null;
            ApplyPose();
        }

        /// <summary>
        /// Start a flight command. Returns its reply at once when there is nothing to fly, or null when the drone starts
        /// moving: <see cref="Step"/> gives the reply when the move ends, as the Tello only replies then.
        /// </summary>
        public string Begin(DroneCommand command)
        {
            switch (command.Kind)
            {
                case CommandKind.Command:
                case CommandKind.StreamOn:
                    return "ok";
                case CommandKind.TakeOff:
                    if (Flying)
                        return "ok";
                    Flying = true;
                    return StartMove(new Vector3(positionCm.x, positionCm.y, ClassroomLayout.TakeOffHeightCm), headingDeg, "ok");
                case CommandKind.Land:
                    if (!Flying)
                        return "error not flying";
                    Flying = false;
                    return StartMove(new Vector3(positionCm.x, positionCm.y, 0f), headingDeg, "ok");
                case CommandKind.Turn:
                    if (!Flying)
                        return "error not flying";
                    return StartMove(positionCm, headingDeg + command.Amount, "ok");
                case CommandKind.Move:
                    if (!Flying)
                        return "error not flying";
                    Vector3 direction = ToRoom(command.Direction);
                    float freeCm = FreeDistanceCm(direction, command.Amount);
                    bool hits = freeCm < command.Amount;
                    return StartMove(positionCm + direction * freeCm, headingDeg, hits ? "error collision" : "ok");
                default:
                    return "error unknown command";
            }
        }

        /// <summary>Fly the current move for dt seconds. Returns true, with the reply, when the move has ended.</summary>
        public bool Step(float dt, out string reply)
        {
            reply = null;
            if (pendingReply == null)
                return false;
            float scaled = dt * timeScale;
            positionCm = Vector3.MoveTowards(positionCm, targetCm, SpeedCmS * scaled);
            headingDeg = Mathf.MoveTowards(headingDeg, targetHeadingDeg, YawSpeedDegS * scaled);
            ApplyPose();
            if (positionCm != targetCm || !Mathf.Approximately(headingDeg, targetHeadingDeg))
                return false;
            reply = pendingReply;
            pendingReply = null;
            return true;
        }

        /// <summary>The Tello state packet restricted to the fields of docs/simulator.md, section 6.</summary>
        public string StateLine() => string.Format(CultureInfo.InvariantCulture,
            "pitch:0;roll:0;yaw:{0};tof:{1};bat:{2};", Mathf.RoundToInt(YawDeg), Mathf.RoundToInt(positionCm.z), BatteryPct);

        void Awake() => ResetToPad();

        string StartMove(Vector3 toCm, float toHeadingDeg, string reply)
        {
            targetCm = toCm;
            targetHeadingDeg = toHeadingDeg;
            pendingReply = reply;
            return null;
        }

        /// <summary>A direction in the drone's frame (x right, y forward, z up) in the room frame, for the current heading.</summary>
        Vector3 ToRoom(Vector3 body)
        {
            float heading = Mathf.Deg2Rad * headingDeg;
            var forward = new Vector3(Mathf.Sin(heading), Mathf.Cos(heading), 0f);
            var right = new Vector3(Mathf.Cos(heading), -Mathf.Sin(heading), 0f);
            return right * body.x + forward * body.y + Vector3.forward * body.z;
        }

        /// <summary>How far the drone can fly along a room direction before its sphere touches an obstacle.</summary>
        float FreeDistanceCm(Vector3 directionCm, float distanceCm)
        {
            // Colliders moved from code only reach the physics scene on the next simulation step otherwise.
            Physics.SyncTransforms();
            Vector3 origin = ClassroomLayout.ToUnity(positionCm.x, positionCm.y, positionCm.z);
            Vector3 direction = ClassroomLayout.ToUnity(directionCm.x, directionCm.y, directionCm.z).normalized;
            if (!Physics.SphereCast(origin, RadiusCm / 100f, direction, out RaycastHit hit, distanceCm / 100f, obstacles,
                    QueryTriggerInteraction.Ignore))
                return distanceCm;
            return Mathf.Max(0f, hit.distance * 100f - ContactGapCm);
        }

        void ApplyPose() => transform.SetPositionAndRotation(
            ClassroomLayout.ToUnity(positionCm.x, positionCm.y, positionCm.z), ClassroomLayout.HeadingToUnity(headingDeg));
    }
}
