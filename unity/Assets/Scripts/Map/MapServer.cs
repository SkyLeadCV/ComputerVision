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
    /// Listens for the brain (docs/protocol.md, section 1) and queues the lines it sends. Sockets block, so they run
    /// on background threads; Unity only allows the scene to be touched from the main thread, which empties the queue.
    /// </summary>
    public sealed class MapServer : IDisposable
    {
        readonly TcpListener listener;
        readonly ConcurrentQueue<string> lines = new();
        readonly object gate = new();
        TcpClient client;
        int generation;
        int connections;

        public MapServer(IPAddress address, int port)
        {
            // The protocol only allows the Tailscale address or loopback: every interface would expose the map
            // to the local network.
            if (IPAddress.Any.Equals(address) || IPAddress.IPv6Any.Equals(address))
                throw new ArgumentException("The map never listens on every interface; use the Tailscale address or 127.0.0.1.");
            listener = new TcpListener(address, port);
        }

        public IPEndPoint Endpoint => (IPEndPoint)listener.LocalEndpoint;

        /// <summary>Connections accepted so far, so the main thread can notice a new one.</summary>
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
            new Thread(AcceptLoop) { IsBackground = true, Name = "Map server" }.Start();
        }

        public bool TryDequeue(out string line) => lines.TryDequeue(out line);

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
                Interlocked.Increment(ref connections);
                new Thread(() => ReadLoop(accepted, mine)) { IsBackground = true, Name = "Map connection" }.Start();
            }
        }

        void ReadLoop(TcpClient connection, int mine)
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
                        lines.Enqueue(line);
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
