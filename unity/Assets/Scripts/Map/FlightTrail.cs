using System.Collections.Generic;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>
    /// The path the drone has flown, drawn from the move messages; the drone itself, as a chevron at flight height
    /// with its safety circle projected on the floor; and the floor its latest photo proved free.
    /// </summary>
    public sealed class FlightTrail : MonoBehaviour
    {
        [Tooltip("Bright vertex-coloured material: trail, chevron and outlines.")]
        [SerializeField] Material glowMaterial;

        [Tooltip("Faint vertex-coloured material: the latest scan on the floor.")]
        [SerializeField] Material holoMaterial;

        // The agreed flight height, used until the first observation gives the real one; move carries no z.
        const float DefaultHeightCm = 80f;
        // docs/protocol.md's 9 + 15 cm, until session_start gives them.
        const float DefaultSafetyRadiusCm = 24f;
        const float TrailWidthM = 0.035f;
        const float OutlineWidthM = 0.012f;
        const float ChevronLengthM = 0.32f;
        // Overlays float just above the floor so they never flicker against its faces.
        const float FloorLiftM = 0.01f;
        // A pure turn repeats the position up to float rounding of the brain's sin and cos.
        const float SamePointM = 0.01f;

        static readonly Color Amber = new(1f, 0.4f, 0.04f);
        static readonly Color ScanCyan = new(0.3f, 0.95f, 0.9f);

        readonly List<Vector3> points = new();
        readonly List<Vector3> scanRim = new();
        float heightCm = DefaultHeightCm;
        LineRenderer trail;
        Transform drone;
        Transform footprint;
        LineRenderer drop;
        MeshFilter scanFill;
        LineRenderer scanEdge;

        public IReadOnlyList<Vector3> Points => points;

        /// <summary>Where each ray of the latest photo stopped, on the floor; a ray that saw no floor stays at the drone.</summary>
        public IReadOnlyList<Vector3> ScanRim => scanRim;

        public Transform Drone => EnsureDrone();

        public float SafetyRadiusCm { get; private set; } = DefaultSafetyRadiusCm;

        /// <summary>Clear the trail and scan, and size the drone's safety circle for a new session.</summary>
        public void StartSession(SessionStart session)
        {
            points.Clear();
            scanRim.Clear();
            heightCm = DefaultHeightCm;
            SafetyRadiusCm = session.drone_radius_cm + session.safety_margin_cm;
            RedrawTrail();
            if (drone)
                Remove(drone.gameObject);
            drone = null;
            if (scanFill)
                Remove(scanFill.transform.parent.gameObject);
            scanFill = null;
        }

        public void ShowObservation(Observation observation)
        {
            Pose pose = observation.pose;
            heightCm = pose.z;
            PlaceDrone(pose.x, pose.y, pose.yaw_deg);
            DrawScan(observation);
        }

        public void AddMove(Move move)
        {
            // Turns leave the drone where it was, so only forward steps lengthen the trail. The start is added too,
            // so a lost move leaves a straight jump instead of a gap.
            Append(move.@from);
            Append(move.to);
            PlaceDrone(move.to.x, move.to.y, move.to.yaw_deg);
            RedrawTrail();
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

        void PlaceDrone(float xCm, float yCm, float yawDeg)
        {
            Transform marker = EnsureDrone();
            marker.gameObject.SetActive(true);
            // Unity's positive y rotation is clockwise seen from above, like yaw_deg (docs/protocol.md).
            marker.SetLocalPositionAndRotation(VoxelGrid.ToUnity(xCm, yCm, heightCm), Quaternion.Euler(0f, yawDeg, 0f));
            float heightM = heightCm / 100f;
            footprint.localPosition = new Vector3(0f, FloorLiftM - heightM, 0f);
            drop.SetPosition(1, new Vector3(0f, FloorLiftM - heightM, 0f));
        }

        /// <summary>
        /// The floor this photo proved free, as a fan from the point under the camera to each ray's stop. It stays where
        /// the photo was taken, so it shows exactly what the map has just carved, walls included.
        /// </summary>
        void DrawScan(Observation observation)
        {
            Pose pose = observation.pose;
            Rays rays = observation.rays;
            scanRim.Clear();
            for (int i = 0; i < rays.angle_deg.Length; i++)
            {
                // A ray that saw no floor proves nothing free (docs/protocol.md), so it collapses onto the drone.
                float distanceCm = rays.stop_cm[i] > rays.entry_cm[i] ? rays.stop_cm[i] : 0f;
                float heading = Mathf.Deg2Rad * (pose.yaw_deg + rays.angle_deg[i]);
                scanRim.Add(VoxelGrid.ToUnity(pose.x + distanceCm * Mathf.Sin(heading), pose.y + distanceCm * Mathf.Cos(heading), 0f));
            }

            EnsureScan();
            Vector3 apex = VoxelGrid.ToUnity(pose.x, pose.y, 0f);
            var vertices = new List<Vector3> { apex };
            var colors = new List<Color> { new(ScanCyan.r, ScanCyan.g, ScanCyan.b, 0.05f) };
            var triangles = new List<int>();
            for (int i = 0; i < scanRim.Count; i++)
            {
                vertices.Add(scanRim[i]);
                colors.Add(new Color(ScanCyan.r, ScanCyan.g, ScanCyan.b, 0.3f));
                // Rays run left to right, so apex, ray, next ray is clockwise seen from above.
                if (i > 0)
                    triangles.AddRange(new[] { 0, i, i + 1 });
            }
            Mesh mesh = scanFill.sharedMesh;
            mesh.Clear();
            mesh.SetVertices(vertices);
            mesh.SetColors(colors);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            scanEdge.positionCount = scanRim.Count;
            scanEdge.SetPositions(scanRim.ToArray());
        }

        void EnsureScan()
        {
            if (scanFill)
                return;
            var scan = new GameObject("Latest scan") { hideFlags = HideFlags.DontSave }.transform;
            scan.SetParent(transform, false);
            scan.localPosition = new Vector3(0f, FloorLiftM, 0f);
            var mesh = new Mesh { name = "Latest scan", hideFlags = HideFlags.DontSave };
            scanFill = MapShapes.MeshObject(scan, "Fill", mesh, holoMaterial).GetComponent<MeshFilter>();
            scanEdge = MapShapes.Line(scan, "Edge", glowMaterial, OutlineWidthM, false);
            scanEdge.startColor = scanEdge.endColor = new Color(ScanCyan.r, ScanCyan.g, ScanCyan.b, 0.8f);
        }

        void RedrawTrail()
        {
            if (!trail)
            {
                trail = MapShapes.Line(transform, "Trail", glowMaterial, TrailWidthM, false);
                // Older legs fade, so the direction of travel reads at a glance.
                var gradient = new Gradient();
                gradient.SetKeys(new[] { new GradientColorKey(Amber, 0f), new GradientColorKey(Amber, 1f) },
                    new[] { new GradientAlphaKey(0.2f, 0f), new GradientAlphaKey(1f, 1f) });
                trail.colorGradient = gradient;
            }
            trail.positionCount = points.Count;
            for (int i = 0; i < points.Count; i++)
                trail.SetPosition(i, points[i]);
        }

        Transform EnsureDrone()
        {
            if (drone)
                return drone;
            drone = new GameObject("Drone") { hideFlags = HideFlags.DontSave }.transform;
            drone.SetParent(transform, false);
            MapShapes.MeshObject(drone, "Chevron", ChevronMesh(), glowMaterial);

            drop = MapShapes.Line(drone, "Drop line", glowMaterial, OutlineWidthM / 2f, false);
            drop.positionCount = 2;
            drop.SetPosition(0, Vector3.zero);
            drop.startColor = new Color(Amber.r, Amber.g, Amber.b, 0.6f);
            drop.endColor = new Color(Amber.r, Amber.g, Amber.b, 0.1f);

            footprint = new GameObject("Footprint") { hideFlags = HideFlags.DontSave }.transform;
            footprint.SetParent(drone, false);
            MapShapes.MeshObject(footprint, "Glow", MapShapes.Disc(SafetyRadiusCm / 100f,
                new Color(Amber.r, Amber.g, Amber.b, 0.35f), new Color(Amber.r, Amber.g, Amber.b, 0.05f)), holoMaterial);

            LineRenderer safety = MapShapes.Line(footprint, "Safety circle", glowMaterial, OutlineWidthM, true);
            MapShapes.SetCircle(safety, SafetyRadiusCm / 100f);
            safety.startColor = safety.endColor = Amber;

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
            mesh.SetColors(new[] { Amber, Amber, Amber, Amber });
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
