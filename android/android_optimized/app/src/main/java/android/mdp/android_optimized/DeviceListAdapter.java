package android.mdp.android_optimized;

import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import java.util.ArrayList;

public class DeviceListAdapter extends ArrayAdapter<BluetoothDevice> {

	private LayoutInflater inflater;
	private ArrayList<BluetoothDevice> device_list;
	private int resource_id;

	protected DeviceListAdapter(Context c, int r_id, ArrayList<BluetoothDevice> d_list) {
		super(c, r_id, d_list);
		device_list = d_list;
		inflater = (LayoutInflater) c.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
		resource_id = r_id;
	}

	public View getView(int position, View convertView, ViewGroup parent) {
		convertView = inflater.inflate(resource_id, null);

		BluetoothDevice device = device_list.get(position);
		if (device != null) {
			TextView name = convertView.findViewById(R.id.device_txt_name);
			if (name != null) name.setText(device.getName());

			TextView address = convertView.findViewById(R.id.device_txt_address);
			if (address != null) address.setText(device.getAddress());
		}
		return convertView;
	}
}
