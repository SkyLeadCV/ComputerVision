using System.Collections.Generic;
using System.Text;

namespace SkyLead.Map
{
    /// <summary>Turns a TCP byte stream into the protocol's lines, whatever way the packets cut them.</summary>
    public sealed class LineSplitter
    {
        // Bytes, not text: a packet can end in the middle of a multi-byte UTF-8 character, so a line is only
        // decoded once its '\n' has arrived.
        readonly List<byte> pending = new();

        public List<string> Feed(byte[] data, int count)
        {
            var lines = new List<string>();
            for (int i = 0; i < count; i++)
            {
                if (data[i] != (byte)'\n')
                {
                    pending.Add(data[i]);
                    continue;
                }
                int length = pending.Count;
                // Python on Windows may write "\r\n".
                if (length > 0 && pending[length - 1] == (byte)'\r')
                    length--;
                if (length > 0)
                    lines.Add(Encoding.UTF8.GetString(pending.ToArray(), 0, length));
                pending.Clear();
            }
            return lines;
        }
    }
}
