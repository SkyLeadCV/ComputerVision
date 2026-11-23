using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SkyLead.Map.Editor
{
    /// <summary>
    /// Builds the map scene from code, so it can be recreated identically instead of hand-editing scene YAML,
    /// and renders a recorded flight to images from the command line, to check the map without opening the editor.
    /// </summary>
    public static class MapSceneBuilder
    {
        const string ScenePath = "Assets/Scenes/Map.unity";
        const string MaterialFolder = "Assets/Materials/Map";

        static readonly Color SolidColor = new(0.55f, 0.57f, 0.60f);
        static readonly Color SurfaceColor = new(0.95f, 0.55f, 0.20f);
        // Close to the classroom's pale green-grey tiles.
        static readonly Color FloorColor = new(0.78f, 0.84f, 0.78f);
        static readonly Color BackgroundColor = new(0.08f, 0.09f, 0.10f);
        // A neutral ambient: an empty scene's default sky light is bluish and washes out the map's colours.
        static readonly Color AmbientColor = new(0.45f, 0.45f, 0.45f);

        const int CaptureWidthPx = 1280;
        const int CaptureHeightPx = 1024;
        const float TopViewMarginM = 0.3f;

        [MenuItem("SkyLead/Build Map Scene")]
        public static void BuildScene()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = AmbientColor;

            var sun = new GameObject("Sun").AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.transform.rotation = Quaternion.Euler(55f, -30f, 0f);

            var mapObject = new GameObject("Voxel Map");
            var map = mapObject.AddComponent<VoxelMap>();
            var mapSettings = new SerializedObject(map);
            mapSettings.FindProperty("solidMaterial").objectReferenceValue = MaterialAsset("Solid", SolidColor);
            mapSettings.FindProperty("surfaceMaterial").objectReferenceValue = MaterialAsset("Surface", SurfaceColor);
            mapSettings.FindProperty("floorMaterial").objectReferenceValue = MaterialAsset("Floor", FloorColor);
            mapSettings.ApplyModifiedPropertiesWithoutUndo();

            var replay = mapObject.AddComponent<FlightReplay>();
            var replaySettings = new SerializedObject(replay);
            replaySettings.FindProperty("map").objectReferenceValue = map;
            replaySettings.ApplyModifiedPropertiesWithoutUndo();

            var camera = new GameObject("Camera") { tag = "MainCamera" }.AddComponent<Camera>();
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = BackgroundColor;
            Bounds block = BlockBounds(map);
            // Above and behind the take-off side, scaled with the block so any room size fills the view the same way.
            camera.transform.position = block.center + new Vector3(0f, 1.8f, -1.2f) * Mathf.Max(block.extents.x, block.extents.z);
            camera.transform.LookAt(block.center);

            EditorSceneManager.SaveScene(scene, ScenePath);
            Debug.Log($"Saved {ScenePath}");
        }

        /// <summary>
        /// Unity.exe -batchmode -quit -projectPath unity -executeMethod SkyLead.Map.Editor.MapSceneBuilder.CaptureFromCommandLine
        /// -flight &lt;flight.jsonl&gt; -out &lt;prefix&gt; writes &lt;prefix&gt;_top.png and &lt;prefix&gt;_3d.png.
        /// </summary>
        public static void CaptureFromCommandLine()
        {
            string flight = Argument("-flight");
            string prefix = Argument("-out");

            EditorSceneManager.OpenScene(ScenePath);
            var map = UnityEngine.Object.FindAnyObjectByType<VoxelMap>();
            var replay = UnityEngine.Object.FindAnyObjectByType<FlightReplay>();
            replay.Load(flight);
            replay.PlayAll();
            map.RebuildDirtyChunks();

            Camera sceneCamera = Camera.main;
            // In batch mode the first render after the chunks are created drew every submesh with a single
            // material (seen with URP 17 here); later renders are correct, so a first one is thrown away.
            Render(sceneCamera, null);
            Render(sceneCamera, $"{prefix}_3d.png");

            // Straight down with +z (the protocol's forward) at the top, like generate_observations.py's top view.
            Bounds block = BlockBounds(map);
            var top = new GameObject("Top View").AddComponent<Camera>();
            top.CopyFrom(sceneCamera);
            top.orthographic = true;
            top.transform.SetPositionAndRotation(block.center + Vector3.up * 20f, Quaternion.Euler(90f, 0f, 0f));
            float aspect = (float)CaptureWidthPx / CaptureHeightPx;
            top.orthographicSize = Mathf.Max(block.extents.z, block.extents.x / aspect) + TopViewMarginM;
            Render(top, $"{prefix}_top.png");
        }

        static Bounds BlockBounds(VoxelMap map)
        {
            var settings = new SerializedObject(map);
            Vector2 origin = settings.FindProperty("originCm").vector2Value;
            Vector3Int size = settings.FindProperty("sizeVoxels").vector3IntValue;
            Vector3 min = VoxelGrid.ToUnity(origin.x, origin.y, 0f);
            Vector3 max = VoxelGrid.ToUnity(origin.x + size.x * VoxelGrid.VoxelCm, origin.y + size.y * VoxelGrid.VoxelCm, size.z * VoxelGrid.VoxelCm);
            var bounds = new Bounds();
            bounds.SetMinMax(min, max);
            return bounds;
        }

        static Material MaterialAsset(string name, Color color)
        {
            string path = $"{MaterialFolder}/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            // Diffuse lighting only: URP Lit adds environment reflections even when matte, which turned the floor
            // from green seen from above to bluish grey seen at an angle. A map's colours must not depend on the view.
            Shader shader = Shader.Find("Universal Render Pipeline/Simple Lit");
            if (material == null)
            {
                Directory.CreateDirectory(MaterialFolder);
                material = new Material(shader);
                AssetDatabase.CreateAsset(material, path);
            }
            // Set on existing materials too, so rebuilding the scene always gives the same look.
            material.shader = shader;
            material.SetColor("_BaseColor", color);
            EditorUtility.SetDirty(material);
            AssetDatabase.SaveAssets();
            return material;
        }

        /// <summary>Render the camera and save it as a PNG at <paramref name="path"/>, or discard it when the path is null.</summary>
        static void Render(Camera camera, string path)
        {
            var target = new RenderTexture(CaptureWidthPx, CaptureHeightPx, 24);
            camera.targetTexture = target;
            camera.Render();
            if (path == null)
            {
                camera.targetTexture = null;
                target.Release();
                return;
            }
            RenderTexture.active = target;
            var image = new Texture2D(CaptureWidthPx, CaptureHeightPx, TextureFormat.RGB24, false);
            image.ReadPixels(new Rect(0, 0, CaptureWidthPx, CaptureHeightPx), 0, 0);
            RenderTexture.active = null;
            camera.targetTexture = null;
            File.WriteAllBytes(path, image.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(image);
            target.Release();
            Debug.Log($"Saved {path}");
        }

        static string Argument(string name)
        {
            string[] args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, name);
            if (index < 0 || index + 1 >= args.Length)
                throw new ArgumentException($"Missing command line argument {name} <value>");
            return args[index + 1];
        }
    }
}
