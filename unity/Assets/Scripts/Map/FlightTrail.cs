using System.Collections.Generic;
using UnityEngine;
using static SkyLead.Map.HudStyle;

namespace SkyLead.Map
{
    /// <summary>
    /// The path the drone has flown, from the move messages, and the drone itself: a chevron at flight height with its
    /// safety circle on the floor. A move arrives once it is over, so the drone glides to it at the real drone's pace
    /// instead of jumping, and the trail grows behind it.
    /// </summary>
    public sealed class FlightTrail : MonoBehaviour
    {
        [Tooltip("Unlit, vertex-coloured material for the trail and the drone.")]
        [SerializeField] Material overlayMaterial;

        // The agreed flight height, used until the first observation gives the real one; move carries no z.
        const float DefaultHeightCm = 80f;
        // docs/protocol.md's 9 + 15 cm, until session_start gives them.
        const float DefaultSafetyRadiusCm = 24f;
        // The real drone's pace (SimulatedDrone and explorer.py's SyntheticDrone), so the glide lasts as long as the move.
        const float SpeedCmS = 50f;
        const float YawSpeedDegS = 90f;
        const float TrailWidthM = 0.03f;
        const float OutlineWidthM = 0.01f;
        const float ChevronLengthM = 0.3f;
        // Overlays float just above the floor so they never flicker against it.
        const float FloorLiftM = 0.01f;
        // A pure turn repeats the position up to float rounding of the brain's sin and cos.
        const float SamePointM = 0.01f;

        readonly List<Vector3> points = new();
        float heightCm = DefaultHeightCm;
        Vector2 shownCm, targetCm;
        float shownYawDeg, targetYawDeg;
        LineRenderer trail;
        Transform drone;
        Transform footprint;
        LineRenderer drop;

        /// <summary>Every place the trail turns, in Unity metres; the drone may still be gliding towards the last one.</summary>
        public IReadOnlyList<Vector3> Points => points;

        public Transform Drone => EnsureDrone();

        public float SafetyRadiusCm { get; private set; } = DefaultSafetyRadiusCm;

        /// <summary>True while the drone is still gliding to the last pose it was given.</summary>
        public bool Moving => shownCm != targetCm || shownYawDeg != targetYawDeg;

        /// <summary>How much faster than the real drone to glide, so a backlog of messages is caught up.</summary>
        public float Pace { get; set; } = 1f;

        /// <summary>Clear the trail and size the drone's safety circle for a new session.</summary>
        public void StartSession(SessionStart session)
        {
            points.Clear();
            heightCm = DefaultHeightCm;
            shownCm = targetCm = Vector2.zero;
            shownYawDeg = targetYawDeg = 0f;
            SafetyRadiusCm = session.drone_radius_cm + session.safety_margin_cm;
            Redraw();
            if (drone)
                Remove(drone.gameObject);
            drone = null;
        }

        public void ShowPose(Pose pose)
        {
            heightCm = pose.z;
            GoTo(new Vector2(pose.x, pose.y), pose.yaw_deg);
        }

        public void AddMove(Move move)
        {
            // Turns leave the drone where it was, so only forward steps lengthen the trail. The start is added too,
            // so a lost move leaves a straight jump instead of a gap.
            Append(move.@from);
            Append(move.to);
            GoTo(new Vector2(move.to.x, move.to.y), move.to.yaw_deg);
        }

        void GoTo(Vector2 positionCm, float yawDeg)
        {
            targetCm = positionCm;
            // The shortest way round, as the drone turns: cw 270 and ccw 90 end the same.
            targetYawDeg = shownYawDeg + Mathf.DeltaAngle(shownYawDeg, yawDeg);
            if (!Application.isPlaying)
            {
                shownCm = targetCm;
                shownYawDeg = targetYawDeg;
            }
            PlaceDrone();
            Redraw();
        }

        void Update()
        {
            if (!Moving)
                return;
            float dt = Time.deltaTime * Pace;
            shownCm = Vector2.MoveTowards(shownCm, targetCm, SpeedCmS * dt);
            shownYawDeg = Mathf.MoveTowards(shownYawDeg, targetYawDeg, YawSpeedDegS * dt);
            PlaceDrone();
            Redraw();
        }

        void Append(PlanarPose pose)
        {
            Vector3 point = VoxelGrid.ToUnity(pose.x, pose.y, heightCm);
            if (points.Count > 0)
            {
                Vector3 last = points[^1];
                if (new Vector2(point.x - last.x, point.z - last.z).magnitude < SamePointM)
                    return;
            }
            points.Add(point);
        }

        void PlaceDrone()
        {
            Transform marker = EnsureDrone();
            marker.gameObject.SetActive(true);
            // Unity's positive y rotation is clockwise seen from above, like yaw_deg (docs/protocol.md).
            marker.SetLocalPositionAndRotation(VoxelGrid.ToUnity(shownCm.x, shownCm.y, heightCm), Quaternion.Euler(0f, shownYawDeg, 0f));
            float heightM = heightCm / 100f;
            footprint.localPosition = new Vector3(0f, FloorLiftM - heightM, 0f);
            drop.SetPosition(1, new Vector3(0f, FloorLiftM - heightM, 0f));
        }

        /// <summary>The trail up to the drone: while it glides, the last leg ends where the drone is, not where it goes.</summary>
        void Redraw()
        {
            if (!trail)
            {
                trail = MapShapes.Line(transform, "Trail", overlayMaterial, TrailWidthM, false);
                trail.startColor = trail.endColor = OnMap(DroneBlue);
                trail.sortingOrder = MapLayers.Trail;
            }
            int count = points.Count;
            trail.positionCount = count;
            for (int i = 0; i < count; i++)
                trail.SetPosition(i, points[i]);
            if (Moving && count > 0)
                trail.SetPosition(count - 1, VoxelGrid.ToUnity(shownCm.x, shownCm.y, heightCm));
        }

        Transform EnsureDrone()
        {
            if (drone)
                return drone;
            drone = new GameObject("Drone") { hideFlags = HideFlags.DontSave }.transform;
            drone.SetParent(transform, false);
            MapShapes.MeshObject(drone, "Chevron", ChevronMesh(), overlayMaterial).sortingOrder = MapLayers.Drone;

            drop = MapShapes.Line(drone, "Drop line", overlayMaterial, OutlineWidthM / 2f, false);
            drop.positionCount = 2;
            drop.SetPosition(0, Vector3.zero);
            drop.startColor = OnMap(new Color(DroneBlue.r, DroneBlue.g, DroneBlue.b, 0.5f));
            drop.endColor = OnMap(new Color(DroneBlue.r, DroneBlue.g, DroneBlue.b, 0.1f));

            footprint = new GameObject("Footprint") { hideFlags = HideFlags.DontSave }.transform;
            footprint.SetParent(drone, false);
            MapShapes.MeshObject(footprint, "Safety area", MapShapes.Disc(SafetyRadiusCm / 100f,
                new Color(DroneBlue.r, DroneBlue.g, DroneBlue.b, 0.18f), new Color(DroneBlue.r, DroneBlue.g, DroneBlue.b, 0.08f)), overlayMaterial).sortingOrder = MapLayers.Drone;
            LineRenderer safety = MapShapes.Line(footprint, "Safety circle", overlayMaterial, OutlineWidthM, true);
            MapShapes.SetCircle(safety, SafetyRadiusCm / 100f);
            safety.startColor = safety.endColor = OnMap(new Color(DroneBlue.r, DroneBlue.g, DroneBlue.b, 0.7f));
            safety.sortingOrder = MapLayers.Drone;

            drone.gameObject.SetActive(false);
            return drone;
        }

        static Mesh ChevronMesh()
        {
            float half = ChevronLengthM / 2f;
            var tip = new Vector3(0f, 0f, half);
            var right = new Vector3(half * 0.8f, 0f, -half);
            var notch = new Vector3(0f, 0f, -half * 0.4f);
            var left = new Vector3(-half * 0.8f, 0f, -half);
            var mesh = new Mesh { name = "Chevron", hideFlags = HideFlags.DontSave };
            mesh.SetVertices(new[] { tip, right, notch, left });
            Color blue = OnMap(DroneBlue);
            mesh.SetColors(new[] { blue, blue, blue, blue });
            // Clockwise seen from above, so the chevron faces the map's cameras.
            mesh.SetTriangles(new[] { 0, 1, 2, 0, 2, 3 }, 0);
            mesh.SetNormals(new[] { Vector3.up, Vector3.up, Vector3.up, Vector3.up });
            return mesh;
        }

        static void Remove(Object target)
        {
            if (Application.isPlaying)
                Destroy(target);
            else
                DestroyImmediate(target);
        }
    }
}
