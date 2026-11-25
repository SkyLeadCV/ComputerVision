using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using static SkyLead.Map.HudStyle;

namespace SkyLead.Map
{
    /// <summary>
    /// The map seen from above as a clean plan instead of blocks: the floor every photo proved free, as the union of
    /// their smooth fans; the obstacles seen, as the dots of the voxels painted as surface; and the latest photo, split
    /// into its blind zone and the floor it saw, between the two edges of the camera's view.
    /// </summary>
    [RequireComponent(typeof(VoxelMap))]
    public sealed class FloorPlan : MonoBehaviour
    {
        [Tooltip("Unlit, vertex-coloured material shared by every layer of the plan.")]
        [SerializeField] Material overlayMaterial;

        // Each layer sits a few millimetres above the one below it, so none flickers against another.
        const float UnexploredLiftM = 0.001f;
        const float FloorLiftM = 0.002f;
        const float WallLiftM = 0.005f;
        const float ScanLiftM = 0.007f;
        // Three quarters of a 10 cm voxel, so neighbouring dots overlap and merge into one continuous wall line.
        const float WallDotRadiusM = 0.075f;
        const float EdgeWidthM = 0.012f;
        // How far the edges of the camera's view are drawn: past the trusted range, so they read as the view's limits.
        const float ViewEdgeM = 3.5f;
        // The latest photo shows while its floor is uncovered and fades once the drone moves on, so the view is only on
        // screen while it means something: what the camera saw from where it stood.
        const float ScanFadeS = 0.4f;
        const float BlindAlpha = 0.45f;
        const float SeenAlpha = 0.45f;
        const float EdgeAlpha = 0.8f;

        readonly List<Vector3> floorVertices = new();
        readonly List<int> floorTriangles = new();
        readonly List<Vector3> scanRim = new();
        Mesh floorMesh, currentMesh, wallMesh, blindMesh, seenMesh;
        LineRenderer leftEdge, rightEdge;
        GameObject walls;
        float scanOpacity = 1f;
        float scanTarget = 1f;

        /// <summary>Where each ray of the latest photo stopped, on the floor; a ray that saw no floor stays at the drone.</summary>
        public IReadOnlyList<Vector3> ScanRim => scanRim;

        public void Clear()
        {
            Ensure();
            floorVertices.Clear();
            floorTriangles.Clear();
            scanRim.Clear();
            floorMesh.Clear();
            foreach (Mesh mesh in new[] { currentMesh, wallMesh, blindMesh, seenMesh })
                mesh.Clear();
            leftEdge.positionCount = rightEdge.positionCount = 0;
        }

        /// <summary>
        /// Draw a photo's scan up to a fraction of each ray, from the edge of its blind zone outwards, so the floor it
        /// proves free is revealed gradually instead of appearing at once.
        /// </summary>
        public void ShowScan(Observation observation, float progress)
        {
            Ensure();
            Pose pose = observation.pose;
            Rays rays = observation.rays;
            Vector3 apex = Floor(pose.x, pose.y);
            var blind = new List<Vector3> { apex };
            var seenInner = new List<Vector3>();
            var seenOuter = new List<Vector3>();
            scanRim.Clear();
            for (int i = 0; i < rays.angle_deg.Length; i++)
            {
                Vector2 direction = Direction(pose, rays.angle_deg[i]);
                bool sawFloor = rays.stop_cm[i] > rays.entry_cm[i];
                // A ray that saw no floor proves nothing free (docs/protocol.md), so it collapses onto the drone.
                float reach = sawFloor ? Mathf.Lerp(rays.entry_cm[i], rays.stop_cm[i], progress) : 0f;
                scanRim.Add(Floor(pose.x + reach * direction.x, pose.y + reach * direction.y));
                blind.Add(Floor(pose.x + rays.entry_cm[i] * direction.x, pose.y + rays.entry_cm[i] * direction.y));
                seenInner.Add(sawFloor ? blind[^1] : apex);
                seenOuter.Add(scanRim[^1]);
            }

            Fan(currentMesh, apex, scanRim, FreeFloor);
            Fan(blindMesh, apex, blind.GetRange(1, blind.Count - 1), BlindZone);
            Band(seenMesh, seenInner, seenOuter, SeenNow);
            Edge(leftEdge, pose, rays.angle_deg[0]);
            Edge(rightEdge, pose, rays.angle_deg[^1]);
            scanOpacity = scanTarget = 1f;
            ApplyScanOpacity();
        }

        /// <summary>Fade the latest photo out, as the drone leaves the place it was taken from.</summary>
        public void FadeScanOut() => scanTarget = 0f;

        void Update()
        {
            if (scanOpacity == scanTarget)
                return;
            scanOpacity = Mathf.MoveTowards(scanOpacity, scanTarget, Time.deltaTime / ScanFadeS);
            ApplyScanOpacity();
        }

        void ApplyScanOpacity()
        {
            blindMesh.SetColors(Repeat(new Color(BlindZone.r, BlindZone.g, BlindZone.b, BlindAlpha * scanOpacity), blindMesh.vertexCount));
            seenMesh.SetColors(Repeat(new Color(SeenNow.r, SeenNow.g, SeenNow.b, SeenAlpha * scanOpacity), seenMesh.vertexCount));
            var fade = new Gradient();
            fade.SetKeys(new[] { new GradientColorKey(OnMap(DroneBlue), 0f), new GradientColorKey(OnMap(DroneBlue), 1f) },
                new[] { new GradientAlphaKey(EdgeAlpha * scanOpacity, 0f), new GradientAlphaKey(0f, 1f) });
            leftEdge.colorGradient = rightEdge.colorGradient = fade;
        }

        /// <summary>Keep a finished photo's fan as part of the proven floor.</summary>
        public void Commit(Observation observation)
        {
            ShowScan(observation, 1f);
            int first = floorVertices.Count;
            floorVertices.Add(currentMesh.vertices[0]);
            floorVertices.AddRange(scanRim);
            for (int i = 1; i < scanRim.Count; i++)
                floorTriangles.AddRange(new[] { first, first + i, first + i + 1 });
            floorMesh.Clear();
            floorMesh.SetVertices(floorVertices);
            floorMesh.SetColors(Repeat(FreeFloor, floorVertices.Count));
            floorMesh.SetTriangles(floorTriangles, 0);
            floorMesh.RecalculateBounds();
            currentMesh.Clear();
        }

        /// <summary>Redraw the obstacles from the voxels painted as surface, which later photos may have cleared.</summary>
        public void RefreshWalls(VoxelGrid grid)
        {
            Ensure();
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            const int segments = 8;
            for (int x = 0; x < grid.Size.x; x++)
            for (int y = 0; y < grid.Size.y; y++)
            {
                if (!HasSurface(grid, x, y))
                    continue;
                Vector3 centre = grid.CellCornerToUnity(x, y, 0) + new Vector3(0.5f, 0f, 0.5f) * (VoxelGrid.VoxelCm / 100f);
                int first = vertices.Count;
                vertices.Add(centre);
                for (int i = 0; i < segments; i++)
                {
                    float angle = 2f * Mathf.PI * i / segments;
                    vertices.Add(centre + new Vector3(Mathf.Sin(angle), 0f, Mathf.Cos(angle)) * WallDotRadiusM);
                    triangles.AddRange(new[] { first, first + 1 + i, first + 1 + (i + 1) % segments });
                }
            }
            wallMesh.Clear();
            wallMesh.indexFormat = IndexFormat.UInt32;
            wallMesh.SetVertices(vertices);
            wallMesh.SetColors(Repeat(Obstacle, vertices.Count));
            wallMesh.SetTriangles(triangles, 0);
            wallMesh.RecalculateBounds();
        }

        /// <summary>The wall dots belong to the plan from above; in the 3D view the voxels show the walls.</summary>
        public void ShowWalls(bool visible)
        {
            Ensure();
            walls.SetActive(visible);
        }

        static bool HasSurface(VoxelGrid grid, int x, int y)
        {
            for (int z = 0; z < grid.Size.z; z++)
            {
                if (grid[x, y, z] == Voxel.Surface)
                    return true;
            }
            return false;
        }

        static Vector2 Direction(Pose pose, float angleDeg)
        {
            float heading = (pose.yaw_deg + angleDeg) * Mathf.Deg2Rad;
            return new Vector2(Mathf.Sin(heading), Mathf.Cos(heading));
        }

        static Vector3 Floor(float xCm, float yCm) => VoxelGrid.ToUnity(xCm, yCm, 0f);

        static List<Color> Repeat(Color color, int count)
        {
            color = OnMap(color);
            var colors = new List<Color>(count);
            for (int i = 0; i < count; i++)
                colors.Add(color);
            return colors;
        }

        /// <summary>A fan from the apex through the rim points, which run clockwise seen from above as the rays do.</summary>
        static void Fan(Mesh mesh, Vector3 apex, List<Vector3> rim, Color color)
        {
            var vertices = new List<Vector3> { apex };
            vertices.AddRange(rim);
            var triangles = new List<int>();
            for (int i = 1; i < rim.Count; i++)
                triangles.AddRange(new[] { 0, i, i + 1 });
            Set(mesh, vertices, triangles, color);
        }

        /// <summary>The strip between two rows of points, ray by ray.</summary>
        static void Band(Mesh mesh, List<Vector3> inner, List<Vector3> outer, Color color)
        {
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            for (int i = 0; i < inner.Count; i++)
            {
                vertices.Add(inner[i]);
                vertices.Add(outer[i]);
                if (i == 0)
                    continue;
                int a = 2 * (i - 1), b = a + 1, c = a + 2, d = a + 3;
                triangles.AddRange(new[] { a, b, d, a, d, c });
            }
            Set(mesh, vertices, triangles, color);
        }

        static void Set(Mesh mesh, List<Vector3> vertices, List<int> triangles, Color color)
        {
            mesh.Clear();
            mesh.SetVertices(vertices);
            mesh.SetColors(Repeat(color, vertices.Count));
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();
        }

        static void Edge(LineRenderer line, Pose pose, float angleDeg)
        {
            Vector2 direction = Direction(pose, angleDeg);
            line.positionCount = 2;
            line.SetPosition(0, Floor(pose.x, pose.y));
            line.SetPosition(1, Floor(pose.x + ViewEdgeM * 100f * direction.x, pose.y + ViewEdgeM * 100f * direction.y));
        }

        void Ensure()
        {
            if (floorMesh != null)
                return;
            Mesh ground = Layer("Unexplored ground", UnexploredLiftM, MapLayers.Unexplored, out _);
            // The voxel map's whole floor: anything inside it not yet proven free is unexplored, and drawn so.
            var map = GetComponent<VoxelMap>();
            Vector2 min = map.OriginCm, max = min + new Vector2(map.SizeVoxels.x, map.SizeVoxels.y) * VoxelGrid.VoxelCm;
            Set(ground, new List<Vector3> { Floor(min.x, min.y), Floor(min.x, max.y), Floor(max.x, max.y), Floor(max.x, min.y) },
                new List<int> { 0, 1, 2, 0, 2, 3 }, Unexplored);
            floorMesh = Layer("Proven floor", FloorLiftM, MapLayers.Floor, out _);
            floorMesh.indexFormat = IndexFormat.UInt32;
            currentMesh = Layer("Floor being revealed", FloorLiftM, MapLayers.Floor, out _);
            wallMesh = Layer("Obstacles seen", WallLiftM, MapLayers.Walls, out walls);
            blindMesh = Layer("Blind zone", ScanLiftM, MapLayers.Scan, out _);
            seenMesh = Layer("Floor seen now", ScanLiftM, MapLayers.Scan, out _);
            leftEdge = MapShapes.Line(transform, "View edge left", overlayMaterial, EdgeWidthM, false);
            rightEdge = MapShapes.Line(transform, "View edge right", overlayMaterial, EdgeWidthM, false);
            foreach (LineRenderer edge in new[] { leftEdge, rightEdge })
            {
                edge.transform.localPosition = new Vector3(0f, ScanLiftM, 0f);
                edge.sortingOrder = MapLayers.Scan;
            }
        }

        Mesh Layer(string name, float lift, int order, out GameObject layer)
        {
            var mesh = new Mesh { name = name, hideFlags = HideFlags.DontSave };
            MeshRenderer renderer = MapShapes.MeshObject(transform, name, mesh, overlayMaterial);
            // Transparent layers are sorted by distance, which flickers when they lie a few millimetres apart.
            renderer.sortingOrder = order;
            layer = renderer.gameObject;
            layer.transform.localPosition = new Vector3(0f, lift, 0f);
            return mesh;
        }
    }
}
