using UnityEngine;
using UnityEngine.SceneManagement;
#if UNITY_EDITOR
using UnityEditor.SceneManagement;
#endif

namespace SkyLead.Simulator
{
    /// <summary>
    /// Opens the map on top of the simulator, so one window shows the classroom being excavated while the simulated
    /// drone flies. The two scenes share the world, so each light and camera is kept to its own: everything of the
    /// simulator is on the Simulator layer, which the map's lights and camera leave out.
    /// </summary>
    public sealed class MissionView : MonoBehaviour
    {
        public const string MapScenePath = "Assets/Scenes/Map.unity";
        public const string SimulatorLayer = "Simulator";

        void Awake()
        {
            SceneManager.sceneLoaded += OnSceneLoaded;
            if (SceneManager.GetSceneByPath(MapScenePath).isLoaded)
                return;
#if UNITY_EDITOR
            // Loads by path, so the map scene needs no entry in the build settings.
            EditorSceneManager.LoadSceneInPlayMode(MapScenePath, new LoadSceneParameters(LoadSceneMode.Additive));
#else
            SceneManager.LoadScene(System.IO.Path.GetFileNameWithoutExtension(MapScenePath), LoadSceneMode.Additive);
#endif
        }

        void OnDestroy() => SceneManager.sceneLoaded -= OnSceneLoaded;

        void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            if (scene.path != MapScenePath)
                return;
            // The map's dark ambient light shapes its look; the Tello camera sets the classroom's own while it renders.
            SceneManager.SetActiveScene(scene);
            Separate(scene, LayerMask.GetMask(SimulatorLayer));
        }

        /// <summary>Keep the map's lights and cameras off the simulator's layer.</summary>
        public static void Separate(Scene map, int simulatorMask)
        {
            if (simulatorMask == 0)
            {
                Debug.LogError($"No '{SimulatorLayer}' layer: rebuild the simulator scene with SkyLead > Build Simulator Scene.");
                return;
            }
            foreach (GameObject root in map.GetRootGameObjects())
            {
                foreach (Light light in root.GetComponentsInChildren<Light>(true))
                    light.cullingMask &= ~simulatorMask;
                foreach (Camera view in root.GetComponentsInChildren<Camera>(true))
                    view.cullingMask &= ~simulatorMask;
            }
        }
    }
}
