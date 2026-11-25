using System;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SkyLead.Simulator.Editor
{
    /// <summary>
    /// Builds the simulator scene from code, so it can be recreated identically instead of hand-editing scene YAML,
    /// and renders Tello photos from given poses on the command line, to check them with the Python vision pipeline.
    /// </summary>
    public static class SimulatorSceneBuilder
    {
        const string ScenePath = "Assets/Scenes/Simulator.unity";
        const string MaterialFolder = "Assets/Materials/Simulator";
        const string FloorTexturePath = "Assets/Textures/Simulator/FloorTile.png";

        // Median colours of the real tiles and joints in the test video's floor (H 41, S 32, V 137 in OpenCV's HSV),
        // so the rendered floor falls inside segment_floor.py's threshold as the real one does.
        static readonly Color32 TileColor = new(130, 137, 118, 255);
        static readonly Color32 JointColor = new(76, 79, 64, 255);
        // One texel per millimetre of a 60 cm tile.
        const int TileTexturePx = 600;
        // Joints a few millimetres wide, as in the real floor; half of each joint lies on either tile.
        const int HalfJointPx = 4;

        // Neutral greys: any tint gives them a hue, and a slightly yellow grey already fell inside the floor threshold.
        static readonly Color WallColor = new(0.92f, 0.92f, 0.92f);
        static readonly Color TablesColor = new(0.85f, 0.85f, 0.85f);
        // Saturated colours far from the floor's hue, so people never pass for floor.
        static readonly Color VictimShirtColor = new(0.15f, 0.25f, 0.55f);
        static readonly Color RescuerShirtColor = new(0.70f, 0.12f, 0.12f);
        // A dark neutral grey: no hue and too dark for the floor threshold.
        static readonly Color TrousersColor = new(0.22f, 0.22f, 0.22f);
        static readonly Color SkinColor = new(0.85f, 0.65f, 0.52f);
        static readonly Color AmbientColor = new(0.6f, 0.6f, 0.6f);
        const float SunIntensity = 0.5f;
        const int JpegQuality = 90;

        [MenuItem("SkyLead/Build Simulator Scene")]
        public static void BuildScene()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = AmbientColor;

            // Shades the walls' faces differently so corners stay visible. Without shadows: a shadow on the floor
            // would darken it out of the floor threshold, which the evenly lit real classroom does not do.
            int layer = SimulatorLayer();
            int mask = 1 << layer;

            var sun = new GameObject("Sun").AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.intensity = SunIntensity;
            sun.shadows = LightShadows.None;
            sun.transform.rotation = Quaternion.Euler(60f, 30f, 0f);
            // The map opened on top has its own sun; each lights only its own scene.
            sun.cullingMask = mask;

            GameObject classroom = ClassroomBuilder.Build(new ClassroomLook
            {
                Floor = FloorMaterial(),
                Wall = LitMaterial("Wall", WallColor),
                Tables = LitMaterial("Tables", TablesColor),
                VictimShirt = LitMaterial("VictimShirt", VictimShirtColor),
                RescuerShirt = LitMaterial("RescuerShirt", RescuerShirtColor),
                Trousers = LitMaterial("Trousers", TrousersColor),
                Skin = LitMaterial("Skin", SkinColor),
            });

            var drone = new GameObject("Simulated Drone").transform;
            Vector2 pad = ClassroomLayout.TakeOffPadCm;
            drone.SetPositionAndRotation(
                ClassroomLayout.ToUnity(pad.x, pad.y, ClassroomLayout.TakeOffHeightCm),
                ClassroomLayout.HeadingToUnity(ClassroomLayout.TakeOffHeadingDeg));
            // Not the main camera: in the mission view the map's camera fills the screen, and this one only renders
            // photos and the onboard feed.
            var cameraObject = new GameObject("Tello Camera");
            cameraObject.transform.SetParent(drone, false);
            var view = cameraObject.AddComponent<Camera>();
            view.clearFlags = CameraClearFlags.SolidColor;
            view.cullingMask = mask;
            var telloCamera = cameraObject.AddComponent<TelloCamera>();
            telloCamera.Configure();
            SetField(telloCamera, "ambientLight", AmbientColor);

            var simulated = drone.gameObject.AddComponent<SimulatedDrone>();
            var obstacles = new SerializedObject(simulated);
            obstacles.FindProperty("obstacles").intValue = mask;
            obstacles.ApplyModifiedPropertiesWithoutUndo();
            SetField(drone.gameObject.AddComponent<SimulatorLink>(), "telloCamera", telloCamera);
            SetField(drone.gameObject.AddComponent<OnboardFeed>(), "telloCamera", telloCamera);

            foreach (Transform part in classroom.GetComponentsInChildren<Transform>(true))
                part.gameObject.layer = layer;
            foreach (Transform part in drone.GetComponentsInChildren<Transform>(true))
                part.gameObject.layer = layer;
            sun.gameObject.layer = layer;

            new GameObject("Mission View").AddComponent<MissionView>();

            EditorSceneManager.SaveScene(scene, ScenePath);
            Debug.Log($"Saved {ScenePath}");
        }

        /// <summary>
        /// Unity.exe -batchmode -quit -projectPath unity -executeMethod SkyLead.Simulator.Editor.SimulatorSceneBuilder.CaptureFromCommandLine
        /// -poses "x,y,heading;..." -out &lt;prefix&gt; writes &lt;prefix&gt;_&lt;n&gt;.jpg for each pose, in room cm and degrees,
        /// at the take-off height.
        /// </summary>
        public static void CaptureFromCommandLine()
        {
            string[] poses = Argument("-poses").Split(';', StringSplitOptions.RemoveEmptyEntries);
            string prefix = Argument("-out");

            EditorSceneManager.OpenScene(ScenePath);
            var camera = UnityEngine.Object.FindAnyObjectByType<TelloCamera>();
            Transform drone = camera.transform.parent;
            for (int i = 0; i < poses.Length; i++)
            {
                float[] pose = Array.ConvertAll(poses[i].Split(','), value => float.Parse(value, CultureInfo.InvariantCulture));
                drone.SetPositionAndRotation(
                    ClassroomLayout.ToUnity(pose[0], pose[1], ClassroomLayout.TakeOffHeightCm),
                    ClassroomLayout.HeadingToUnity(pose[2]));
                // In batch mode the first render of a session can draw with the wrong materials (seen in the map
                // scene with URP 17), so one render is thrown away before each saved one.
                camera.CaptureJpeg(JpegQuality);
                string path = $"{prefix}_{i + 1}.jpg";
                File.WriteAllBytes(path, camera.CaptureJpeg(JpegQuality));
                Debug.Log($"Saved {path}");
            }
        }

        /// <summary>
        /// Unlit, so the floor shows the measured tile colour from every angle; a lit floor's colour would depend on the
        /// lights, and the threshold of segment_floor.py was tuned on the real colour.
        /// </summary>
        static Material FloorMaterial()
        {
            Material material = MaterialAsset("Floor", Shader.Find("Universal Render Pipeline/Unlit"));
            material.SetColor("_BaseColor", Color.white);
            material.SetTexture("_BaseMap", FloorTexture());
            material.SetTextureScale("_BaseMap", new Vector2(ClassroomLayout.WidthCm, ClassroomLayout.DepthCm) / ClassroomLayout.TileCm);
            // The texture's joints sit on whole texture repeats, at x = 60 k - 60 offset, so this puts them at the
            // layout's joint offset.
            material.SetTextureOffset("_BaseMap", new Vector2(-ClassroomLayout.TileJointOffsetXCm / ClassroomLayout.TileCm, 0f));
            EditorUtility.SetDirty(material);
            AssetDatabase.SaveAssets();
            return material;
        }

        /// <summary>One 60 cm tile with half a joint along each border, so repeating it draws the tiled floor.</summary>
        static Texture2D FloorTexture()
        {
            var texture = new Texture2D(TileTexturePx, TileTexturePx, TextureFormat.RGB24, false);
            var pixels = new Color32[TileTexturePx * TileTexturePx];
            for (int row = 0; row < TileTexturePx; row++)
            {
                for (int column = 0; column < TileTexturePx; column++)
                {
                    bool joint = row < HalfJointPx || row >= TileTexturePx - HalfJointPx
                        || column < HalfJointPx || column >= TileTexturePx - HalfJointPx;
                    pixels[row * TileTexturePx + column] = joint ? JointColor : TileColor;
                }
            }
            texture.SetPixels32(pixels);
            Directory.CreateDirectory(Path.GetDirectoryName(FloorTexturePath));
            File.WriteAllBytes(FloorTexturePath, texture.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(texture);
            AssetDatabase.ImportAsset(FloorTexturePath);

            var importer = (TextureImporter)AssetImporter.GetAtPath(FloorTexturePath);
            importer.wrapMode = TextureWrapMode.Repeat;
            importer.mipmapEnabled = true;
            importer.filterMode = FilterMode.Trilinear;
            // The floor is mostly seen at a grazing angle, where plain mipmapping blurs the joints across the view.
            importer.anisoLevel = 16;
            // Compression smears the thin joints into the tiles.
            importer.textureCompression = TextureImporterCompression.Uncompressed;
            importer.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(FloorTexturePath);
        }

        /// <summary>Diffuse only: URP Lit adds environment reflections that change a colour with the viewing angle.</summary>
        static Material LitMaterial(string name, Color color)
        {
            Material material = MaterialAsset(name, Shader.Find("Universal Render Pipeline/Simple Lit"));
            material.SetColor("_BaseColor", color);
            EditorUtility.SetDirty(material);
            AssetDatabase.SaveAssets();
            return material;
        }

        static Material MaterialAsset(string name, Shader shader)
        {
            string path = $"{MaterialFolder}/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                Directory.CreateDirectory(MaterialFolder);
                material = new Material(shader);
                AssetDatabase.CreateAsset(material, path);
            }
            // Set on existing materials too, so rebuilding the scene always gives the same look.
            material.shader = shader;
            return material;
        }

        /// <summary>The Simulator layer, added to the project's layers the first time.</summary>
        static int SimulatorLayer()
        {
            int existing = LayerMask.NameToLayer(MissionView.SimulatorLayer);
            if (existing >= 0)
                return existing;
            var tagManager = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
            SerializedProperty layers = tagManager.FindProperty("layers");
            // Layers 0 to 7 are Unity's own.
            for (int i = 8; i < layers.arraySize; i++)
            {
                SerializedProperty slot = layers.GetArrayElementAtIndex(i);
                if (!string.IsNullOrEmpty(slot.stringValue))
                    continue;
                slot.stringValue = MissionView.SimulatorLayer;
                tagManager.ApplyModifiedPropertiesWithoutUndo();
                return i;
            }
            throw new InvalidOperationException("No free layer for the simulator.");
        }

        static void SetField(UnityEngine.Object component, string field, object value)
        {
            var settings = new SerializedObject(component);
            SerializedProperty property = settings.FindProperty(field);
            if (value is Color color)
                property.colorValue = color;
            else
                property.objectReferenceValue = (UnityEngine.Object)value;
            settings.ApplyModifiedPropertiesWithoutUndo();
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
