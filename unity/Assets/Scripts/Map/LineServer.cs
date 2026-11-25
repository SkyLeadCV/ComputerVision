using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;

namespace SkyLead.Map
{
    /// <summary>
    /// Listens for the brain over TCP (docs/protocol.md and docs/simulator.md, section 2) and queues the lines it sends,
    /// for the map and the simulator alike. Sockets block, so they run on background threads; Unity only allows the
    /// scene to be touched from the main thread, which empties the queue and, for the simulator, sends the replies.
    /// </summary>
    public sealed class LineServer : IDisposable
    {
        readonly TcpListener listener;
        readonly ConcurrentQueue<(string line, int connection)> lines = new();
        readonly object gate = new();
        TcpClient client;
        int generation;
        int connections;

        public LineServer(IPAddress address, int port)
        {
            // The protocols only allow the Tailscale address or loopback: every interface would expose the port
            // to the local network.
            if (IPAddress.Any.Equals(address) || IPAddress.IPv6Any.Equals(address))
                throw new ArgumentException("Never listen on every interface; use the Tailscale address or 127.0.0.1.");
            listener = new TcpListener(address, port);
        }

        public IPEndPoint Endpoint => (IPEndPoint)listener.LocalEndpoint;

        /// <summary>Connections accepted so far, so the main thread can notice a new one. Also the id of the current one.</summary>
        public int Connections => Volatile.Read(ref connections);

        public bool Connected
        {
            get { lock (gate) return client != null; }
        }

        /// <summary>This PC's Tailscale address (100.64.0.0/10), or loopback when there is none, for an SSH tunnel.</summary>
        public static IPAddress TailscaleOrLoopback()
        {
            foreach (NetworkInterface adapter in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (adapter.OperationalStatus != OperationalStatus.Up)
                    continue;
                foreach (UnicastIPAddressInformation unicast in adapter.GetIPProperties().UnicastAddresses)
                {
                    if (IsTailscale(unicast.Address))
                        return unicast.Address;
                }
            }
            return IPAddress.Loopback;
        }

        public static bool IsTailscale(IPAddress address)
        {
            byte[] bytes = address.GetAddressBytes();
            return bytes.Length == 4 && bytes[0] == 100 && (bytes[1] & 0xC0) == 64;
        }

        public void Start()
        {
            listener.Start();
            new Thread(AcceptLoop) { IsBackground = true, Name = "Line server" }.Start();
        }

        public bool TryDequeue(out string line) => TryDequeue(out line, out _);

        /// <summary>The next line and the connection it came from, numbered like <see cref="Connections"/>.</summary>
        public bool TryDequeue(out string line, out int connection)
        {
            bool found = lines.TryDequeue(out var item);
            (line, connection) = item;
            return found;
        }

        /// <summary>
        /// Send bytes to the current connection, from the main thread. Returns false when it is gone: a reply has no
        /// one to go to then, and the brain will reconnect.
        /// </summary>
        public bool Reply(byte[] data)
        {
            TcpClient current;
            lock (gate)
                current = client;
            if (current == null)
                return false;
            try
            {
                current.GetStream().Write(data, 0, data.Length);
                return true;
            }
            catch (Exception error) when (error is IOException or ObjectDisposedException or InvalidOperationException)
            {
                return false;
            }
        }

        public void Dispose()
        {
            listener.Stop();
            lock (gate)
            {
                client?.Close();
                client = null;
                generation++;
            }
        }

        void AcceptLoop()
        {
            while (true)
            {
                TcpClient accepted;
                try
                {
                    accepted = listener.AcceptTcpClient();
                }
                catch (Exception error) when (error is SocketException or ObjectDisposedException or InvalidOperationException)
                {
                    return;
                }

                int mine;
                // A new connection replaces the old one: after a wifi drop the old socket can look open for minutes,
                // and the brain only reconnects when its own side failed.
                lock (gate)
                {
                    client?.Close();
                    client = accepted;
                    mine = ++generation;
                }
                int id = Interlocked.Increment(ref connections);
                new Thread(() => ReadLoop(accepted, mine, id)) { IsBackground = true, Name = "Line connection" }.Start();
            }
        }

        void ReadLoop(TcpClient connection, int mine, int id)
        {
            var splitter = new LineSplitter();
            var buffer = new byte[8192];
            try
            {
                NetworkStream stream = connection.GetStream();
                int count;
                while ((count = stream.Read(buffer, 0, buffer.Length)) > 0)
                {
                    foreach (string line in splitter.Feed(buffer, count))
                    {
                        if (Volatile.Read(ref generation) != mine)
                            return;
                        lines.Enqueue((line, id));
                    }
                }
            }
            catch (Exception error) when (error is IOException or ObjectDisposedException or InvalidOperationException)
            {
                // Closed by the brain, by a newer connection or by Dispose: all end the same way.
            }
            finally
            {
                lock (gate)
                {
                    if (client == connection)
                        client = null;
                }
                connection.Close();
            }
        }
    }
}
