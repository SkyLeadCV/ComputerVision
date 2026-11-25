using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using static SkyLead.Map.HudStyle;

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

        // The 3D view's blocks in the plan's light palette: the unexplored block a mid grey, dark enough to read as a solid
        // block and not as a hole in the white floor, the faces of obstacles dark like the plan's walls, and the floor white.
        static readonly Color SolidColor = new(0.64f, 0.67f, 0.71f);
        static readonly Color SurfaceColor = new(0.36f, 0.39f, 0.43f);
        // A bright, neutral ambient for a light scene; the sun only adds soft shading and shadows.
        static readonly Color AmbientColor = new(0.72f, 0.72f, 0.72f);
        const float SunIntensity = 0.6f;
        const float ShadowStrength = 0.35f;

        const string LitShader = "Universal Render Pipeline/Simple Lit";
        // The plan, trail and markers carry their colours in the vertices, which only URP's particle shaders read.
        const string OverlayShader = "Universal Render Pipeline/Particles/Unlit";

        const int CaptureWidthPx = 1280;
        const int CaptureHeightPx = 1024;

        [MenuItem("SkyLead/Build Map Scene")]
        public static void BuildScene()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            RenderSettings.ambientMode = AmbientMode.Flat;
            RenderSettings.ambientLight = AmbientColor;

            var sun = new GameObject("Sun").AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.transform.rotation = Quaternion.Euler(55f, -30f, 0f);
            sun.intensity = SunIntensity;
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = ShadowStrength;

            var mapObject = new GameObject("Voxel Map");
            // MapReceiver brings the map, plan, trail, victims, status panel and replay along as required components.
            mapObject.AddComponent<MapReceiver>();
            var map = mapObject.GetComponent<VoxelMap>();
            SetMaterials(map, ("solidMaterial", MaterialAsset("Solid", SolidColor)),
                ("surfaceMaterial", MaterialAsset("Surface", SurfaceColor)), ("floorMaterial", MaterialAsset("Floor", FreeFloor)));
            Material overlay = OverlayMaterial("Overlay");
            SetMaterials(mapObject.GetComponent<FloorPlan>(), ("overlayMaterial", overlay));
            SetMaterials(mapObject.GetComponent<FlightTrail>(), ("overlayMaterial", overlay));
            SetMaterials(mapObject.GetComponent<VictimMarkers>(), ("overlayMaterial", overlay));
            SetMaterials(mapObject.AddComponent<FloorGrid>(), ("overlayMaterial", overlay));

            var camera = new GameObject("Camera") { tag = "MainCamera" }.AddComponent<Camera>();
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = Background;
            EnableAntialiasing(camera);
            MapView.Frame(MapView.AreaToShow(mapObject.GetComponent<FloorPlan>(), mapObject.GetComponent<FlightTrail>()), out var top, out _);
            camera.transform.SetPositionAndRotation(top.position, top.rotation);
            camera.fieldOfView = MapView.TopFovDeg;
            var view = new SerializedObject(camera.gameObject.AddComponent<MapView>());
            view.FindProperty("map").objectReferenceValue = map;
            view.FindProperty("plan").objectReferenceValue = mapObject.GetComponent<FloorPlan>();
            view.ApplyModifiedPropertiesWithoutUndo();

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
            var plan = UnityEngine.Object.FindAnyObjectByType<FloorPlan>();
            var replay = UnityEngine.Object.FindAnyObjectByType<FlightReplay>();
            replay.Load(flight);
            replay.PlayAll();
            map.RebuildDirtyChunks();

            Camera camera = Camera.main;
            MapView.Frame(MapView.AreaToShow(plan, map.GetComponent<FlightTrail>()), out var top, out var threeD);
            // In batch mode the first render after the chunks are created drew every submesh with a single
            // material (seen with URP 17 here); later renders are correct, so a first one is thrown away.
            Render(camera, null);

            // Each view with its own layers, as MapView shows them: the plan from above, the blocks in 3D.
            map.ShowBlocks(false);
            plan.ShowWalls(true);
            camera.transform.SetPositionAndRotation(top.position, top.rotation);
            camera.fieldOfView = MapView.TopFovDeg;
            Render(camera, $"{prefix}_top.png");

            map.ShowBlocks(true);
            plan.ShowWalls(false);
            camera.transform.SetPositionAndRotation(threeD.position, threeD.rotation);
            camera.fieldOfView = MapView.ThreeDFovDeg;
            Render(camera, null);
            Render(camera, $"{prefix}_3d.png");
        }

        static void SetMaterials(UnityEngine.Object component, params (string field, Material material)[] materials)
        {
            var settings = new SerializedObject(component);
            foreach (var (field, material) in materials)
                settings.FindProperty(field).objectReferenceValue = material;
            settings.ApplyModifiedPropertiesWithoutUndo();
        }

        // Diffuse lighting only by default: URP Lit adds environment reflections even when matte, which turned the
        // floor's colour with the viewing angle. A map's colours must not depend on the view.
        static Material MaterialAsset(string name, Color color, string shaderName = LitShader)
        {
            string path = $"{MaterialFolder}/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            Shader shader = Shader.Find(shaderName);
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

        /// <summary>An alpha-blended, unlit material that takes its colour from the vertices.</summary>
        static Material OverlayMaterial(string name)
        {
            Material material = MaterialAsset(name, Color.white, OverlayShader);
            // The particle shader's transparent mode, set by hand because its editor GUI is not run from code.
            material.SetFloat("_Surface", 1f);
            material.SetFloat("_Blend", 0f);
            material.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
            material.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
            material.SetFloat("_SrcBlendAlpha", (float)BlendMode.One);
            material.SetFloat("_DstBlendAlpha", (float)BlendMode.OneMinusSrcAlpha);
            material.SetFloat("_ZWrite", 0f);
            material.SetOverrideTag("RenderType", "Transparent");
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            material.renderQueue = (int)RenderQueue.Transparent;
            EditorUtility.SetDirty(material);
            AssetDatabase.SaveAssets();
            return material;
        }

        static void EnableAntialiasing(Camera camera)
        {
            UniversalAdditionalCameraData data = camera.GetUniversalAdditionalCameraData();
            // No post-processing: with it on, URP's global effects greyed the light background and darkened its corners
            // (measured on captures), and the map needs none of them. Edges are smoothed by MSAA (PC_RPAsset) instead.
            data.renderPostProcessing = false;
            data.antialiasing = AntialiasingMode.None;
            camera.allowMSAA = true;
        }

        /// <summary>Render the camera and save it as a PNG at <paramref name="path"/>, or discard it when the path is null.</summary>
        static void Render(Camera camera, string path)
        {
            var target = new RenderTexture(CaptureWidthPx, CaptureHeightPx, 24) { antiAliasing = 4 };
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
