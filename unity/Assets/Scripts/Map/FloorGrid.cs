using System.Collections.Generic;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>
    /// The classroom's 60 cm tile grid, faint on the floor, so the map shows its scale. It reaches past the block and
    /// fades out, like a planning table; inside unseen solid it stays hidden.
    /// </summary>
    [ExecuteAlways, RequireComponent(typeof(VoxelMap))]
    public sealed class FloorGrid : MonoBehaviour
    {
        [SerializeField] Material holoMaterial;

        // The classroom's tiles (docs/classroom.md). The take-off pad sits on a tile corner, so lines fall on multiples.
        const float TileCm = 60f;
        // Two tiles past the block on each side, where the lines fade to nothing.
        const float MarginCm = 2f * TileCm;
        const float LineWidthM = 0.008f;
        const float FloorLiftM = 0.004f;

        static readonly Color Cyan = new(0.25f, 0.9f, 0.85f, 0.22f);

        GameObject gridObject;

        void OnEnable()
        {
            var map = GetComponent<VoxelMap>();
            Vector2 min = map.OriginCm;
            Vector2 max = min + new Vector2(map.SizeVoxels.x, map.SizeVoxels.y) * VoxelGrid.VoxelCm;
            float x0 = Mathf.Floor((min.x - MarginCm) / TileCm) * TileCm, x1 = Mathf.Ceil((max.x + MarginCm) / TileCm) * TileCm;
            float y0 = Mathf.Floor((min.y - MarginCm) / TileCm) * TileCm, y1 = Mathf.Ceil((max.y + MarginCm) / TileCm) * TileCm;

            var vertices = new List<Vector3>();
            var colors = new List<Color>();
            var triangles = new List<int>();
            for (float x = x0; x <= x1; x += TileCm)
                AddLine(new Vector2(x, y0), new Vector2(x, y1), vertices, colors, triangles);
            for (float y = y0; y <= y1; y += TileCm)
                AddLine(new Vector2(x0, y), new Vector2(x1, y), vertices, colors, triangles);

            var mesh = new Mesh { name = "Floor grid", hideFlags = HideFlags.DontSave };
            mesh.SetVertices(vertices);
            mesh.SetColors(colors);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            gridObject = MapShapes.MeshObject(transform, "Floor grid", mesh, holoMaterial).gameObject;
            gridObject.transform.localPosition = new Vector3(0f, FloorLiftM, 0f);
        }

        void OnDisable()
        {
            if (gridObject == null)
                return;
            if (Application.isPlaying)
                Destroy(gridObject);
            else
                DestroyImmediate(gridObject);
        }

        /// <summary>A thin strip from a to b (protocol cm) that fades in over the first margin and out over the last.</summary>
        static void AddLine(Vector2 a, Vector2 b, List<Vector3> vertices, List<Color> colors, List<int> triangles)
        {
            Vector2 along = (b - a).normalized;
            Vector2 side = new Vector2(along.y, -along.x) * (LineWidthM * 50f);
            float length = (b - a).magnitude;
            float[] stops = { 0f, MarginCm, length - MarginCm, length };
            float[] alphas = { 0f, 1f, 1f, 0f };
            int first = vertices.Count;
            for (int i = 0; i < stops.Length; i++)
            {
                Vector2 centre = a + along * stops[i];
                var color = new Color(Cyan.r, Cyan.g, Cyan.b, Cyan.a * alphas[i]);
                vertices.Add(VoxelGrid.ToUnity(centre.x - side.x, centre.y - side.y, 0f));
                vertices.Add(VoxelGrid.ToUnity(centre.x + side.x, centre.y + side.y, 0f));
                colors.Add(color);
                colors.Add(color);
            }
            for (int i = 0; i < stops.Length - 1; i++)
            {
                int left = first + i * 2, right = left + 1, nextLeft = left + 2, nextRight = left + 3;
                // Both windings, so the strip shows from above whichever way the line runs.
                triangles.AddRange(new[] { left, nextLeft, right, right, nextLeft, nextRight });
                triangles.AddRange(new[] { left, right, nextLeft, right, nextRight, nextLeft });
            }
        }
    }
}
