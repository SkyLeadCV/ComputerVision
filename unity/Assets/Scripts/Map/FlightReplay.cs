using System.IO;
using System.Linq;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>Plays a recorded flight (flights/*.jsonl) into the map, keeping the messages' real timing.</summary>
    public sealed class FlightReplay : MonoBehaviour
    {
        [SerializeField] VoxelMap map;

        [Tooltip("A .jsonl flight recording. Empty plays the newest file in the repository's flights/ folder.")]
        [SerializeField] string flightPath = "";

        [Tooltip("Replay speed: 1 is real time.")]
        [SerializeField, Min(0.01f)] float speed = 1f;

        string[] lines;
        int next;
        float clock;

        public void Load(string path)
        {
            lines = File.ReadAllLines(ResolvePath(path)).Where(line => !string.IsNullOrWhiteSpace(line)).ToArray();
            next = 0;
            clock = 0f;
        }

        /// <summary>Deliver every remaining message at once, for checks that do not need the timing.</summary>
        public void PlayAll() => Advance(float.PositiveInfinity);

        void Start() => Load(flightPath);

        void Update()
        {
            clock += Time.deltaTime * speed;
            Advance(clock);
        }

        void Advance(float untilT)
        {
            while (lines != null && next < lines.Length)
            {
                var header = JsonUtility.FromJson<MessageHeader>(lines[next]);
                if (header.t > untilT)
                    return;
                Dispatch(header.type, lines[next]);
                next++;
            }
        }

        void Dispatch(string type, string line)
        {
            // move, victim, state and session_end are drawn by the trail, markers and status of feature 019.
            switch (type)
            {
                case "session_start":
                    map.StartSession(JsonUtility.FromJson<SessionStart>(line));
                    break;
                case "observation":
                    map.Observe(JsonUtility.FromJson<Observation>(line));
                    break;
            }
        }

        static string ResolvePath(string path)
        {
            if (!string.IsNullOrEmpty(path))
                return path;
            // The Unity project lives in unity/, next to the repository's flights/ folder.
            string folder = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "flights"));
            FileInfo newest = Directory.Exists(folder)
                ? new DirectoryInfo(folder).GetFiles("*.jsonl").OrderByDescending(file => file.LastWriteTimeUtc).FirstOrDefault()
                : null;
            if (newest == null)
                throw new FileNotFoundException($"No flight recording (*.jsonl) in {folder}; run generate_observations.py first.");
            return newest.FullName;
        }
    }
}
