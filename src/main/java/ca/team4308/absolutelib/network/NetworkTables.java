package ca.team4308.absolutelib.network;

public class NetworkTables {
	private static final java.util.concurrent.ConcurrentHashMap<String, Object> entries = new java.util.concurrent.ConcurrentHashMap<>();
	private static volatile boolean running = false;
	private static java.net.Socket socket;
	private static java.io.PrintWriter writer;
	private static java.io.BufferedReader reader;
	private static Thread listenerThread;
	
	public static String teamNumber = "4308";
	public static String ip = "127.0.0.1";
	public static int port = 1735;


	/**
	 * Start a simple NetworkTables-like client that talks to an NT4 proxy on the Rio using configured IP and port.
	 */
	 
	public static void start() {
		try {
		startClient(ip, port);
		} catch (java.io.IOException e) {
			System.err.println("Failed to start NetworkTables client: " + e.getMessage());
		}
	}
	/**
	 * Start a simple NetworkTables-like client that talks to an NT4 proxy on the Rio.
	 * @param host rio hostname or IP
	 * @param port port the proxy is listening on (default 1735 if you control proxy)
	 * @throws java.io.IOException if socket fails
	 */
	public static void startClient(String host, int port) throws java.io.IOException {
		if (running) {
			return;
		}
		socket = new java.net.Socket(host, port);
		writer = new java.io.PrintWriter(socket.getOutputStream(), true);
		reader = new java.io.BufferedReader(new java.io.InputStreamReader(socket.getInputStream()));
		running = true;

		listenerThread = new Thread(() -> {
			try {
				String line;
				while (running && (line = reader.readLine()) != null) {
					// expected format: key=type:value
					int eq = line.indexOf('=');
					int colon = line.indexOf(':', eq + 1);
					if (eq > 0 && colon > eq) {
						String key = line.substring(0, eq);
						String type = line.substring(eq + 1, colon);
						String value = line.substring(colon + 1);
						switch (type) {
							case "N":
								entries.put(key, Double.parseDouble(value));
								break;
							case "B":
								entries.put(key, Boolean.parseBoolean(value));
								break;
							case "S":
								entries.put(key, value);
								break;
							default:
								break;
						}
					}
				}
			} catch (Exception e) {
				// fall through
			} finally {
				running = false;
			}
		});
		listenerThread.setDaemon(true);
		listenerThread.start();
	}

	public static void stopClient() {
		running = false;
		try {
			if (reader != null) {
				reader.close();
			}
		} catch (java.io.IOException e) {
			// ignore
		}
		if (writer != null) {
			writer.close();
		}
		try {
			if (socket != null) {
				socket.close();
			}
		} catch (java.io.IOException e) {
			// ignore
		}
		if (listenerThread != null) {
			listenerThread.interrupt();
		}
	}

	public static boolean isRunning() {
		return running;
	}

	public static boolean isConnected() {
		return running && socket != null && socket.isConnected() && !socket.isClosed();
	}

	public static void putNumber(String key, double value) {
		entries.put(key, value);
		sendEntry(key, "N", Double.toString(value));
	}

	public static double getNumber(String key, double defaultValue) {
		Object o = entries.get(key);
		if (o instanceof Number) {
			return ((Number) o).doubleValue();
		}
		return defaultValue;
	}

	public static void putBoolean(String key, boolean value) {
		entries.put(key, value);
		sendEntry(key, "B", Boolean.toString(value));
	}

	public static boolean getBoolean(String key, boolean defaultValue) {
		Object o = entries.get(key);
		if (o instanceof Boolean) {
			return (Boolean) o;
		}
		return defaultValue;
	}

	public static void putString(String key, String value) {
		entries.put(key, value);
		sendEntry(key, "S", value == null ? "" : value);
	}

	public static String getString(String key, String defaultValue) {
		Object o = entries.get(key);
		if (o instanceof String) {
			return (String) o;
		}
		return defaultValue;
	}

	private static void sendEntry(String key, String type, String value) {
		if (!isConnected() || writer == null) {
			return;
		}
		try {
			writer.print(key + "=" + type + ":" + value + "\n");
			writer.flush();
		} catch (Exception e) {
			// ignore; connection may be lost
		}
	}

	public static Object getEntry(String key) {
		return entries.get(key);
	}

	public void setTeamNumber(String teamNumber) {
		this.teamNumber = teamNumber;
	}
	public void setIp(String ip) {
		this.ip = ip;
	}
	public void setPort(int port) {
		this.port = port;
	}

}
