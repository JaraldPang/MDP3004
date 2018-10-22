package android.mdp.android_optimized;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.support.v4.content.LocalBroadcastManager;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Method;
import java.util.UUID;

public class BluetoothConnectionService {
	private final UUID THIS_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
	private BluetoothAdapter bt_adapter;
	private Context context;

	private AcceptThread btt_accept;

	private ConnectThread btt_connect;
	private BluetoothDevice bt_device;

	private ConnectedThread btt_connected;

	BluetoothConnectionService(Context context) {
		this.context = context;
		this.bt_adapter = BluetoothAdapter.getDefaultAdapter();
		start();
	}

	private class AcceptThread extends Thread {
		private final BluetoothServerSocket server_socket;

		AcceptThread() {
			BluetoothServerSocket tmp = null;

			try {
				tmp = bt_adapter.listenUsingInsecureRfcommWithServiceRecord(context.getString(R.string.app_name), THIS_UUID);
			} catch (IOException e) {
				e.printStackTrace();
			}
			server_socket = tmp;
		}

		public void run() {
			BluetoothSocket socket = null;
			try {
				socket = server_socket.accept();
			} catch (IOException e) {
				e.printStackTrace();
			}

			if (socket != null) {
				connected(socket);
			}
		}

		public void cancel() {
			try {
				server_socket.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			btt_accept = null;
		}
	}

	private class ConnectThread extends Thread {
		BluetoothSocket socket = null;

		ConnectThread(BluetoothDevice bd) {
			bt_device = bd;
		}

		public void run() {
			bt_adapter.cancelDiscovery();

			BluetoothSocket tmp = null;
			try {
				tmp = bt_device.createRfcommSocketToServiceRecord(THIS_UUID);
			} catch (IOException e) {
				e.printStackTrace();
			}
			socket = tmp;

			try {
				socket.connect();
				connected(socket);
			} catch (IOException e) {
				socket = reconnect(tmp);
			}
		}

		BluetoothSocket reconnect(BluetoothSocket tmp) {
			Class<?> class2 = tmp.getRemoteDevice().getClass();
			Class<?>[] paramTypes = new Class<?>[]{Integer.TYPE};

			Method m = null;
			try {
				m = class2.getMethod("createRfcommSocket", paramTypes);
				Object[] params = new Object[]{Integer.valueOf(1)};

				BluetoothSocket fbs = (BluetoothSocket) m.invoke(tmp.getRemoteDevice(), params);
				fbs.connect();
				connected(fbs);
				return fbs;
			} catch (Exception e) {
				e.printStackTrace();
			}
			return null;
		}

		void cancel() {
			try {
				socket.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			bt_device = null;
			btt_connect = null;
		}
	}

	public class ConnectedThread extends Thread {
		private BluetoothSocket socket;
		private final InputStream stream_in;
		private final OutputStream stream_out;

		ConnectedThread(BluetoothSocket bs) {
			socket = bs;
			InputStream tmp_in = null;
			OutputStream tmp_out = null;

			try {
				tmp_in = socket.getInputStream();
			} catch (IOException e) {
				e.printStackTrace();
			}

			try {
				tmp_out = socket.getOutputStream();
			} catch (IOException e) {
				e.printStackTrace();
			}

			stream_in = tmp_in;
			stream_out = tmp_out;
		}

		public void run() {
			byte[] buffer = new byte[1024];
			int bytes;

			while (true) {
				try {
					bytes = stream_in.read(buffer);
					String message = new String(buffer, 0, bytes);

					Intent messaging_intent = new Intent("messaging");
					messaging_intent.putExtra("read message", message);
					LocalBroadcastManager.getInstance(context).sendBroadcast(messaging_intent);
				} catch (IOException e) {
					e.printStackTrace();
					break;
				}
			}
		}

		void write(byte[] bytes) {
			try {
				stream_out.flush();
			} catch (IOException e) {
				e.printStackTrace();
			}
			try {
				stream_out.write(bytes);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		public void cancel() {
			try {
				socket.close();
				stream_in.close();
				stream_out.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			btt_connected = null;
		}
	}

	public synchronized void start() {
		if (btt_connect != null) {
			btt_connect.cancel();
		}
		if (btt_connected != null) {
			btt_connected.cancel();
		}
		if (btt_accept == null) {
			btt_accept = new AcceptThread();
			btt_accept.start();
		}
	}

	public void start_client(BluetoothDevice device) {
		btt_connect = new ConnectThread(device);
		btt_connect.start();
	}

	private void connected(BluetoothSocket socket) {
		btt_connected = new ConnectedThread(socket);
		btt_connected.start();
	}

	public void write(byte[] message_out) {
		btt_connected.write(message_out);
	}

	public BluetoothDevice getDevice() {
		return bt_device;
	}
}
