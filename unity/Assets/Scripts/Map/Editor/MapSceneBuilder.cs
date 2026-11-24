using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

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

        const string VolumeProfilePath = "Assets/Settings/MapVolumeProfile.asset";

        // A dark operations-table look: unseen solid is near-black gunmetal, so what the drone has proved free (slate
        // floor) and the faces it has seen (cold steel) stand out, and the overlays' amber and red carry the eye.
        static readonly Color SolidColor = new(0.10f, 0.12f, 0.15f);
        static readonly Color SurfaceColor = new(0.45f, 0.62f, 0.68f);
        static readonly Color FloorColor = new(0.14f, 0.24f, 0.26f);
        static readonly Color BackgroundColor = new(0.012f, 0.02f, 0.03f);
        // A cool, dim ambient: the sun gives the shape, and the scene stays dark enough for the glow to read.
        static readonly Color AmbientColor = new(0.30f, 0.34f, 0.38f);
        static readonly Color SunColor = new(0.92f, 0.96f, 1.00f);
        // Above 1, so the bloom picks up the overlays and nothing else.
        const float GlowIntensity = 1.5f;

        const string LitShader = "Universal Render Pipeline/Simple Lit";
        // Overlays carry their colours in the vertices, which only URP's particle shaders read.
        const string OverlayShader = "Universal Render Pipeline/Particles/Unlit";

        // Narrower than the default 60 degrees and farther back, so the room is seen with less distortion.
        const float CameraFovDeg = 38f;
        static readonly Vector3 CameraOffset = new(0f, 2.6f, -2.1f);

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
            sun.color = SunColor;
            // Soft shadows give the walls and furniture depth on the dark floor.
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 0.75f;

            var mapObject = new GameObject("Voxel Map");
            // MapReceiver brings the map, trail, victims, status panel and replay along as required components.
            mapObject.AddComponent<MapReceiver>();
            var map = mapObject.GetComponent<VoxelMap>();
            SetMaterials(map, ("solidMaterial", MaterialAsset("Solid", SolidColor)),
                ("surfaceMaterial", MaterialAsset("Surface", SurfaceColor)), ("floorMaterial", MaterialAsset("Floor", FloorColor)));
            Material glow = OverlayMaterial("Glow", GlowIntensity);
            Material holo = OverlayMaterial("Holo", 1f);
            SetMaterials(mapObject.GetComponent<FlightTrail>(), ("glowMaterial", glow), ("holoMaterial", holo));
            SetMaterials(mapObject.GetComponent<VictimMarkers>(), ("glowMaterial", glow), ("holoMaterial", holo));
            SetMaterials(mapObject.AddComponent<FloorGrid>(), ("holoMaterial", holo));

            var volume = new GameObject("Post Processing").AddComponent<Volume>();
            volume.isGlobal = true;
            volume.sharedProfile = PostProcessingProfile();

            var camera = new GameObject("Camera") { tag = "MainCamera" }.AddComponent<Camera>();
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = BackgroundColor;
            camera.fieldOfView = CameraFovDeg;
            EnablePostProcessing(camera);
            Bounds block = BlockBounds(map);
            // Above and behind the take-off side, scaled with the block so any room size fills the view the same way.
            camera.transform.position = block.center + CameraOffset * Mathf.Max(block.extents.x, block.extents.z);
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
            EnablePostProcessing(top);
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

        static void SetMaterials(UnityEngine.Object component, params (string field, Material material)[] materials)
        {
            var settings = new SerializedObject(component);
            foreach (var (field, material) in materials)
                settings.FindProperty(field).objectReferenceValue = material;
            settings.ApplyModifiedPropertiesWithoutUndo();
        }

        // Diffuse lighting only by default: URP Lit adds environment reflections even when matte, which turned the
        // floor from green seen from above to bluish grey seen at an angle. A map's colours must not depend on the view.
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

        /// <summary>An alpha-blended, unlit material that takes its colour from the vertices, scaled by intensity.</summary>
        static Material OverlayMaterial(string name, float intensity)
        {
            Material material = MaterialAsset(name, Color.white * intensity, OverlayShader);
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

        static VolumeProfile PostProcessingProfile()
        {
            var profile = AssetDatabase.LoadAssetAtPath<VolumeProfile>(VolumeProfilePath);
            if (profile == null)
            {
                profile = ScriptableObject.CreateInstance<VolumeProfile>();
                AssetDatabase.CreateAsset(profile, VolumeProfilePath);
            }
            // Only the overlays are brighter than 1, so the glow stays on the trail, drone and victims.
            Bloom bloom = Override<Bloom>(profile);
            bloom.threshold.Override(0.95f);
            bloom.intensity.Override(2.2f);
            bloom.scatter.Override(0.7f);
            Vignette vignette = Override<Vignette>(profile);
            vignette.intensity.Override(0.38f);
            vignette.smoothness.Override(0.5f);
            Tonemapping tonemapping = Override<Tonemapping>(profile);
            tonemapping.mode.Override(TonemappingMode.Neutral);
            ColorAdjustments adjustments = Override<ColorAdjustments>(profile);
            adjustments.contrast.Override(12f);
            EditorUtility.SetDirty(profile);
            AssetDatabase.SaveAssets();
            return profile;
        }

        /// <summary>The profile's effect of type T, added (and saved inside the profile asset) the first time.</summary>
        static T Override<T>(VolumeProfile profile) where T : VolumeComponent
        {
            if (profile.TryGet(out T component))
                return component;
            component = profile.Add<T>(true);
            component.hideFlags = HideFlags.HideInInspector | HideFlags.HideInHierarchy;
            AssetDatabase.AddObjectToAsset(component, profile);
            return component;
        }

        static void EnablePostProcessing(Camera camera)
        {
            UniversalAdditionalCameraData data = camera.GetUniversalAdditionalCameraData();
            data.renderPostProcessing = true;
            // Thin glowing lines shimmer without anti-aliasing.
            data.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            data.antialiasingQuality = AntialiasingQuality.High;
            camera.allowHDR = true;
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
