using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>The flat shapes the map overlays are drawn with. Colours go in the vertices, so one material serves all.</summary>
    public static class MapShapes
    {
        const int CircleSegments = 64;

        /// <summary>A child line in local space. Every overlay object is rebuilt on each run, so none is saved.</summary>
        public static LineRenderer Line(Transform parent, string name, Material material, float widthM, bool loop)
        {
            var lineObject = new GameObject(name) { hideFlags = HideFlags.DontSave };
            lineObject.transform.SetParent(parent, false);
            var line = lineObject.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.loop = loop;
            line.widthMultiplier = widthM;
            line.sharedMaterial = material;
            line.numCornerVertices = 4;
            line.numCapVertices = 4;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.positionCount = 0;
            return line;
        }

        public static void SetCircle(LineRenderer line, float radiusM)
        {
            line.positionCount = CircleSegments;
            for (int i = 0; i < CircleSegments; i++)
            {
                float angle = 2f * Mathf.PI * i / CircleSegments;
                line.SetPosition(i, new Vector3(Mathf.Sin(angle), 0f, Mathf.Cos(angle)) * radiusM);
            }
        }

        public static MeshRenderer MeshObject(Transform parent, string name, Mesh mesh, Material material)
        {
            var meshObject = new GameObject(name) { hideFlags = HideFlags.DontSave };
            meshObject.transform.SetParent(parent, false);
            meshObject.AddComponent<MeshFilter>().sharedMesh = mesh;
            var renderer = meshObject.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = material;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            return renderer;
        }

        /// <summary>A flat disc facing up whose colour fades from the centre to the rim.</summary>
        public static Mesh Disc(float radiusM, Color centre, Color rim)
        {
            var vertices = new Vector3[CircleSegments + 1];
            var colors = new Color[CircleSegments + 1];
            var triangles = new int[CircleSegments * 3];
            colors[0] = centre;
            for (int i = 0; i < CircleSegments; i++)
            {
                float angle = 2f * Mathf.PI * i / CircleSegments;
                vertices[i + 1] = new Vector3(Mathf.Sin(angle), 0f, Mathf.Cos(angle)) * radiusM;
                colors[i + 1] = rim;
                // Clockwise seen from above, so the disc faces the map's cameras.
                triangles[i * 3] = 0;
                triangles[i * 3 + 1] = i + 1;
                triangles[i * 3 + 2] = (i + 1) % CircleSegments + 1;
            }
            return NewMesh("Disc", vertices, colors, triangles);
        }

        static Mesh NewMesh(string name, Vector3[] vertices, Color[] colors, int[] triangles)
        {
            var mesh = new Mesh { name = name, hideFlags = HideFlags.DontSave };
            mesh.SetVertices(vertices);
            mesh.SetColors(colors);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
