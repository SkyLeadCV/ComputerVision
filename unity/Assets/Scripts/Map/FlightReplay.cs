using System.IO;
using System.Linq;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>Plays a recorded flight (flights/*.jsonl) into the map, keeping the messages' real timing.</summary>
    public sealed class FlightReplay : MonoBehaviour
    {
        [Tooltip("A .jsonl flight recording. Empty plays the newest file in the repository's flights/ folder.")]
        [SerializeField] string flightPath = "";

        [Tooltip("Replay speed: 1 is real time.")]
        [SerializeField, Min(0.01f)] float speed = 1f;

        string[] lines;
        int next;
        float clock;

        /// <summary>The recording being replayed, or null when none was loaded.</summary>
        public string FileName { get; private set; }

        public void Load(string path)
        {
            string resolved = ResolvePath(path);
            lines = File.ReadAllLines(resolved).Where(line => !string.IsNullOrWhiteSpace(line)).ToArray();
            next = 0;
            clock = 0f;
            FileName = Path.GetFileName(resolved);
        }

        /// <summary>Load the configured recording; without one the map simply keeps waiting for a live brain.</summary>
        public bool TryLoad()
        {
            try
            {
                Load(flightPath);
                return true;
            }
            catch (IOException error)
            {
                Debug.LogWarning(error.Message);
                return false;
            }
        }

        public void Stop()
        {
            lines = null;
            FileName = null;
        }

        /// <summary>Deliver every remaining message at once, for checks that do not need the timing.</summary>
        public void PlayAll() => Advance(float.PositiveInfinity);

        void Update()
        {
            if (lines == null)
                return;
            clock += Time.deltaTime * speed;
            Advance(clock);
        }

        void Advance(float untilT)
        {
            var receiver = GetComponent<MapReceiver>();
            while (lines != null && next < lines.Length)
            {
                var header = JsonUtility.FromJson<MessageHeader>(lines[next]);
                if (header.t > untilT)
                    return;
                receiver.Receive(lines[next]);
                next++;
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
