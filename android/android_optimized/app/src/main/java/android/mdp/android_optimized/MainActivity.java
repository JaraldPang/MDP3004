package android.mdp.android_optimized;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.graphics.PorterDuff;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.support.v4.content.LocalBroadcastManager;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.SwitchCompat;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.GridLayout;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.math.BigInteger;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Set;

public class MainActivity extends AppCompatActivity {

	final int MAZE_C = 15;
	final int MAZE_R = 20;
	final int ROBOT_SIZE = 3;
	final int OBST_ADD = 1000;
	boolean dev_showchat = false;
	AlertDialog dev_dialog;
	int origin_col = 0,
		origin_row = MAZE_R - ROBOT_SIZE,
		origin_temp = -1;

	Menu menu;
	LayoutInflater inflater;

	String mdf_string1, mdf_string2;

	GridLayout grid_maze;
	ImageView robot;
	int point_robot, point_way;
	boolean point_isset;

	AlertDialog bt_dialog;
	BluetoothAdapter bt_adapter = BluetoothAdapter.getDefaultAdapter();
	ArrayList<BluetoothDevice> bt_newlist = new ArrayList<>(), bt_pairedlist = new ArrayList<>();
	ListView bt_lv_device;
	DeviceListAdapter bt_listadapter;
	BluetoothConnectionService bt_connection = null;
	BluetoothDevice bt_device, bt_prev = null;
	boolean bt_display_isfind, bt_new_finding = false, bt_robust;

	AlertDialog msg_dialog;
	ListView msg_lv_chat, msg_lv_preview;
	ArrayList<MessageText> msg_chatlist = new ArrayList<>();
	private final BroadcastReceiver bt_receiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();

			if (BluetoothAdapter.ACTION_DISCOVERY_STARTED.equals(action)) {
				if (bt_new_finding) {
					new_message("Finding new devices...");
				}
			} else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {
				if (bt_new_finding) {
					if (bt_newlist.size() == 0) {
						new_message("No new devices found");
					}
					bt_new_finding = false;
				}
			} else if (BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)) {
				if (bt_connection != null) {
					bt_device = bt_connection.getDevice();
					bt_prev = bt_device;
					bt_canceldiscover();
					bt_checkpaired();
					robot_reset(1, true);
				}
			} else if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)) {
				if (bt_device != null) {
					bt_device = null;
					bt_checkpaired();
					msg_chatlist.clear();

					if (bt_robust) {
						bt_connection.start_client(bt_prev);
					}
				}
			} else if (BluetoothDevice.ACTION_BOND_STATE_CHANGED.equals(action)) {
				BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
				switch (device.getBondState()) {
					case BluetoothDevice.BOND_NONE:
						break;
					case BluetoothDevice.BOND_BONDING:
						break;
					case BluetoothDevice.BOND_BONDED:
						bt_connection.start_client(device);
						bt_device = device;
						bt_prev = bt_device;
						bt_checkpaired();
						robot_reset(1, true);
						break;
				}
			} else if (BluetoothDevice.ACTION_FOUND.equals(action)) {
				BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
				if (!bt_newlist.contains(device) && !bt_pairedlist.contains(device) && (device != bt_device)) {
					bt_newlist.add(device);
					bt_listview(context, bt_newlist);
				}
			} else {
				//new_message(action);
			}
		}
	};
	ArrayAdapter msg_listadapter;
	Handler time_handler = new Handler();
	long time_start;
	TextView time_tv;
	public Runnable time_runnable = new Runnable() {
		public void run() {
			long time_count_ms = SystemClock.uptimeMillis() - time_start;
			int time_s = (int) (time_count_ms / 1000);

			time_set(time_s / 60, time_s % 60, (int) (time_count_ms % 1000));
			time_handler.postDelayed(this, 0);
		}
	};
	private final BroadcastReceiver msg_receiver = new BroadcastReceiver() {
		@Override
		public void onReceive(Context context, Intent intent) {
			String text = intent.getStringExtra("read message").trim(),
				description = r_string(R.string._success);
			if (r_string(R.string._null).equalsIgnoreCase(text)) return; //no text sent

			if (!text.contains(r_string(R.string._bracket_s)) && !text.contains(r_string(R.string._delimiter)) && !text.contains(r_string(R.string._bracket_e)) &&
				!text.contains(r_string(R.string._delimiter2))) {

				try {
					Enum.Instruction instruction = enum_getinstruction(text);
					msg_chatlist.add(new MessageText(true, text, ((instruction == null) ? r_string(R.string._null) : instruction.getDescription()), getResources()));
					msg_listview();

					if (instruction != null) {
						switch (instruction) {
							case STOP:
								if (view_string(findViewById(R.id.time_btn_start)).equalsIgnoreCase(r_string(R.string.time_isstart_true))) {
									time_stopwatch(); //TODO

									if (!dev_showchat) {//TODO:disable coordinates button
										findViewById(R.id.point_swt_isway).setEnabled(true);
										findViewById(R.id.point_btn_set).setEnabled(true);
									}
								}
								map_checkarrow();
							case SENSOR:
							case CALIBRATING:
								((TextView) findViewById(R.id.txt_status)).setText(instruction.getStatus());
								break;

							default:
								if (!dev_showchat) {
									new_message(text);
								}
								break;
						}
					}
				} catch (Exception e) {
					new_message("CRASH A");
				}

			} else if (text.contains(r_string(R.string._delimiter2))) {

				try {
					int deli = text.indexOf(r_string(R.string._delimiter2));

					Enum.Instruction instruction = enum_getinstruction(text.substring(0, deli).trim());
					int count = Integer.valueOf(text.substring(deli + 1, text.length()).trim());

					switch (instruction) {
						case FORWARD:
						case REVERSE:
							count /= 10;
							break;
						case ROTATE_LEFT:
						case ROTATE_RIGHT:
							count /= 90;
							break;
					}

					msg_chatlist.add(new MessageText(true, text, String.format("%d %s", count, instruction.getDescription()), getResources()));
					msg_listview();
				} catch (Exception e) {
					new_message("CRASH B");
				}

			} else {

				int ch_sta = text.indexOf(r_string(R.string._bracket_s)),
					ch_mid1 = text.indexOf(r_string(R.string._delimiter)),
					ch_mid2 = text.indexOf(r_string(R.string._delimiter), ch_mid1 + 1),
					ch_end = text.indexOf(r_string(R.string._bracket_e));

				try {
					Enum.Instruction instruction = enum_getinstruction(text.substring(0, ch_sta));
					String s1 = text.substring(ch_sta + 1, ch_mid1).trim(),
						s2 = text.substring(ch_mid1 + 1, (ch_mid2 == -1) ? ch_end : ch_mid2).trim();

					int cell;
					Enum.Direction direction = (ch_mid2 == -1) ? null : enum_getdirection_chara(text.substring(ch_mid2 + 1, ch_end));

					switch (instruction) {
						case MDF:
							if (ch_mid2 != -1) {
								description = new_message(String.format("%s requires only 2 strings", instruction.getDescription()));
							} else if (r_string(R.string._null).equalsIgnoreCase(s1) || r_string(R.string._null).equalsIgnoreCase(s2)) {
								description = new_message(String.format("%s cannot have empty strings", instruction.getDescription()));
							} else {
								mdf_string1 = s1;
								mdf_string2 = s2;
								update_arena();
								robot_go();
							}
							break;

						case WAY:
							if (ch_mid2 != -1) {
								description = new_message(String.format("%s requires only 2 input: x, y", instruction.getDescription()));
							} else {
								cell = cell_id(Integer.valueOf(s1), cell_fliprow(Integer.valueOf(s2)));
								description = point_set(true, cell);
							}
							break;
						case ORIGIN:
							if (ch_mid2 != -1) {
								description = new_message(String.format("%s requires only 2 input: x, y", instruction.getDescription()));
							} else {
								cell = cell_id(Integer.valueOf(s1), cell_fliprow(Integer.valueOf(s2)));
								description = point_set(false, cell);
								if (r_string(R.string._success).equalsIgnoreCase(description)) {
									point_setorigin();
								}
							}
							break;
						case OBSTACLE:
							if (ch_mid2 != -1) {
								description = new_message(String.format("%s requires only 2 input: x, y", instruction.getDescription()));
							} else {
								cell = cell_id(Integer.valueOf(s1), cell_fliprow(Integer.valueOf(s2)));
								cell_update((TextView) findViewById(cell), Enum.Cell.OBSTACLE, true);
							}
							break;
						case ARROW:
							if (ch_mid2 == -1) {
								description = new_message(String.format("%s requires 3 input: x, y, direction", instruction.getDescription()));
							} else {
								if (direction == null) {
									description = new_message("Invalid direction found");
								} else {
									cell = cell_id(Integer.valueOf(s1), cell_fliprow(Integer.valueOf(s2)));
									obst_arrow(cell, direction.getChara());
								}
							}
							break;
						case CENTER:
							if (ch_mid2 == -1) {
								description = new_message(String.format("%s requires 3 input: x, y, direction", instruction.getDescription()));
							} else {
								if (direction == null) {
									description = new_message("Invalid direction found");
								} else {
									point_robot = cell_robot(true, cell_id(Integer.valueOf(s1), cell_fliprow(Integer.valueOf(s2))));
									robot.setRotation(direction.get() * 90);
									robot_go();
								}
							}
							break;
						default:
							throw new Exception();
					}
				} catch (Exception e) {
					description = new_message("Syntax Error");
				}

				msg_chatlist.add(new MessageText(true, text, description, getResources()));
				msg_listview();
			}
		}
	};
	ClipboardManager copy_board;
	private View.OnClickListener tv_onClickListener = new View.OnClickListener() {
		@Override
		public void onClick(View v) {
			if (point_isset) {
				SwitchCompat s = findViewById(R.id.point_swt_isway);
				point_set(s.isChecked(), v.getId());
				map_startgoal();
				robot_go();
			}
		}
	};
	private View.OnClickListener onClickListener = new View.OnClickListener() {
		public void onClick(View v) {

			switch (v.getId()) {
				//BLUETOOTH
				case R.id.bt_swt_isrobust:
					bt_robust_option();
					break;

				//MAZE
				case R.id.direction_btn_up:
					msg_movements(true, true, Enum.Instruction.FORWARD);
					break;
				case R.id.direction_btn_down:
					msg_movements(true, true, Enum.Instruction.REVERSE);
					break;
				case R.id.direction_btn_left:
					msg_movements(true, true, Enum.Instruction.ROTATE_LEFT);
					break;
				case R.id.direction_btn_right:
					msg_movements(true, true, Enum.Instruction.ROTATE_RIGHT);
					break;
				case R.id.direction_btn_calib:
					msg_writemsg(Enum.Instruction.CALIBRATING.getText(), Enum.Instruction.CALIBRATING.getDescription());
					break;

				//STOPWATCH
				case R.id.time_swt_isfastest:
					time_option();
					break;
				case R.id.time_btn_start:
					time_stopwatch();

					if (!dev_showchat) {//TODO:disable coordinates button
						boolean t_isenable = ((Button) findViewById(R.id.time_btn_start)).getText().toString().equalsIgnoreCase(r_string(R.string.time_isstart_false));
						findViewById(R.id.point_swt_isway).setEnabled(t_isenable);
						findViewById(R.id.point_btn_set).setEnabled(t_isenable);
					}
					break;

				//SET POINTS
				case R.id.point_swt_isway:
					point_option();
					break;
				case R.id.point_btn_set:
					point_toset();

					if (!dev_showchat) {//TODO:disable timing button
						boolean p_isenable = ((Button) findViewById(R.id.point_btn_set)).getText().toString().equalsIgnoreCase(r_string(R.string.point_isset_false));
						findViewById(R.id.time_swt_isfastest).setEnabled(p_isenable);
						findViewById(R.id.time_btn_start).setEnabled(p_isenable);
					}
					break;
			}
		}
	};

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

//		========== OTHERS ==========
		inflater = LayoutInflater.from(this);

		reg_bt_intentfilter();
		LocalBroadcastManager.getInstance(this).registerReceiver(msg_receiver, new IntentFilter("messaging"));
		msg_lv_preview = findViewById(R.id.msg_lv_preview);

//		========== CLICKABLE CONTROLS ==========
		int[] list_onclick = {R.id.bt_swt_isrobust,
			R.id.direction_btn_up, R.id.direction_btn_down, R.id.direction_btn_left, R.id.direction_btn_right, R.id.direction_btn_calib,
			R.id.time_btn_start, R.id.time_swt_isfastest,
			R.id.point_swt_isway, R.id.point_btn_set};
		for (int onclick : list_onclick) {
			findViewById(onclick).setOnClickListener(onClickListener);
		}

		findViewById(R.id.msg_tp_preview).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				msg_dialog = pop_message().show();
			}
		});

		msg_lv_preview.setOnItemClickListener(new AdapterView.OnItemClickListener() {
			@Override
			public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
				msg_dialog = pop_message().show();
			}
		});

//		========== MAZE ==========
		grid_maze = findViewById(R.id.grid_maze);
		grid_maze.setColumnCount(MAZE_C);
		grid_maze.setRowCount(MAZE_R);

		for (int i = 0; i < MAZE_C * MAZE_R; i++) {
			TextView tv = new TextView(this);
			tv.setGravity(Gravity.CENTER);
			cell_update(tv, Enum.Cell.DEFAULT, true);
			tv.setId(i);
			tv.setOnClickListener(tv_onClickListener);

			grid_maze.addView(tv);
		}
	}

	@Override
	public void onPause() {
		bt_canceldiscover();
		super.onPause();
	}

	@Override
	public void onResume() {
		bt_update(-1);
		super.onResume();
	}

	@Override
	public void onDestroy() {
		unregisterReceiver(bt_receiver);
		super.onDestroy();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		MenuInflater mi = getMenuInflater();
		mi.inflate(R.menu.menu, menu);

		this.menu = menu;
		reset_app();
		return super.onCreateOptionsMenu(menu);
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) {
			case R.id.menu_mdf:
				pop_mdf().show();
				return true;
			case R.id.menu_arrow:
				pop_arrow().show();
				return true;
			case R.id.menu_developer:
				dev_dialog = pop_dev().show();

				//BLUETOOTH
			case R.id.menu_bt:
				bt_update(-1);
				if (bt_adapter.isEnabled() && bt_connection == null) {
					bt_connection = new BluetoothConnectionService(this);
				}
				return true;
			case R.id.menu_bt_on:
				bt_update(1);
				return true;
			case R.id.menu_bt_off:
				bt_update(0);
				return true;
			case R.id.menu_bt_find:
				AlertDialog.Builder bt_dialog2 = pop_bluetooth();
				bt_dialog2.setOnDismissListener(new DialogInterface.OnDismissListener() {
					@Override
					public void onDismiss(DialogInterface dialog) {
						bt_canceldiscover();
					}
				});
				this.bt_dialog = bt_dialog2.show();
				return true;
			case R.id.menu_bt_reconnect:
				if (bt_prev == null) {
					new_message("You have not connected to a device previously");
				} else {
					bt_connection.start_client(bt_prev);
				}
				bt_checkpaired();
				return true;

			default:
				return super.onOptionsItemSelected(item);
		}
	}

	protected AlertDialog.Builder pop_mdf() {
		View v = inflater.inflate(R.layout.pop_mdf, null);

		final TextView tv_s1 = v.findViewById(R.id.s1_txt_data);
		final TextView tv_s2 = v.findViewById(R.id.s2_txt_data);
		tv_s1.setText(mdf_string1);
		tv_s2.setText(mdf_string2);


		tv_s1.setOnLongClickListener(new View.OnLongClickListener() {
			@Override
			public boolean onLongClick(View v) {
				copy_data(R.string.mdf_s1);
				return false;
			}
		});
		tv_s2.setOnLongClickListener(new View.OnLongClickListener() {
			@Override
			public boolean onLongClick(View v) {
				copy_data(R.string.mdf_s2);
				return false;
			}
		});
		return new AlertDialog.Builder(this).setView(v);
	}

	protected void copy_data(int string) {
		String s = (string == R.string.mdf_s1) ? mdf_string1 : mdf_string2;

		ClipData copy_clip = ClipData.newPlainText("Copied text", s);
		copy_board.setPrimaryClip(copy_clip);
		new_message("Copied " + r_string(string));
	}

	protected String r_string(int id) {
		return getResources().getString(id);
	}

	protected String view_string(View v) {
		if (v instanceof TextView)
			return ((TextView) v).getText().toString();
		else if (v instanceof Button)
			return ((Button) v).getText().toString();
		return null;
	}

	protected String new_message(String message) {
		Toast.makeText(this, message, Toast.LENGTH_LONG).show();
		return message;
	}

	protected GridLayout.LayoutParams new_layoutparams(int col, int row, int size) {
		GridLayout.LayoutParams lp = new GridLayout.LayoutParams();
		lp.columnSpec = GridLayout.spec(col, size);
		lp.rowSpec = GridLayout.spec(row, size);
		lp.height = 40 * size;
		lp.width = 40 * size;
		return lp;
	}

	protected Drawable new_drawable(int image, int color) {
		Drawable box = getResources().getDrawable(image, null);
		box.setColorFilter(color, (color == getResources().getColor(R.color.light_gray)) ? PorterDuff.Mode.SRC_ATOP : PorterDuff.Mode.OVERLAY);
		return box;
	}

	protected void enable_layout(int id) {
		View v = findViewById(id);
		if (v instanceof GridLayout) {
			GridLayout gl = (GridLayout) v;
			for (int i = 0; i < gl.getChildCount(); i++) {
				View v2 = gl.getChildAt(i);

				if (v2.getTag() != null) continue;
				v2.setEnabled(dev_showchat);
				v2.setBackground(new_drawable(R.drawable.d_arrow, dev_showchat ? Color.TRANSPARENT : getResources().getColor(R.color.light_gray)));
			}
		}
	}

	protected int enum_getcolor(int id) {
		for (Enum.Cell c : Enum.Cell.values()) {
			if (c.get() == id) {
				return c.getColor();
			}
		}
		return -1;
	}

	protected Enum.Direction enum_getdirection(int id) {
		for (Enum.Direction d : Enum.Direction.values()) {
			if (d.get() == id) {
				return d;
			}
		}
		return null;
	}

	protected Enum.Direction enum_getdirection_chara(String chara) {
		for (Enum.Direction d : Enum.Direction.values()) {
			if (d.getChara().equalsIgnoreCase(chara)) {
				return d;
			}
		}
		return null;
	}

	protected Enum.Instruction enum_getinstruction(String text) {
		for (Enum.Instruction i : Enum.Instruction.values()) {
			if (i.getText().equalsIgnoreCase(text)) {
				return i;
			}
		}
		return null;
	}

	protected int cell_id(int col, int row) {
		return (row * MAZE_C) + col;
	}

	protected int cell_fliprow(int row) {
		return (MAZE_R - 1) - row;
	}

	protected int cell_robot(boolean iswrite, int cell) {
		return cell + ((MAZE_C + 1) * (iswrite ? -1 : 1));
	}

	protected void cell_update(TextView tv, Enum.Cell type, boolean setBackground) {
		Drawable box = new_drawable(R.drawable.d_box, type.getColor());
		if (setBackground) {
			tv.setBackground(box);
		}
		tv.setText(String.valueOf(type.get()));
	}

	protected void reset_app() {
		bt_update(-1);
		bt_checkpaired();
		bt_robust_option();

		mdf_string1 = r_string(R.string._null);
		mdf_string2 = r_string(R.string._null);
		copy_board = (ClipboardManager) getSystemService(Context.CLIPBOARD_SERVICE);

		point_update(false);
		robot_reset(1, true);

		point_isset = false;
		point_option();

		((TextView) findViewById(R.id.time_txt_explore)).setText(R.string.time_default);
		((TextView) findViewById(R.id.time_txt_fastest)).setText(R.string.time_default);
		time_option();

		dev_toggle(false);
	}

	protected void map_reset(boolean reset_all) {
		if (reset_all) {
			point_way = -1;
			point_update(true);

			for (int i = 0; i < MAZE_C * MAZE_R; i++) {
				cell_update((TextView) grid_maze.findViewById(i), Enum.Cell.DEFAULT, true);
			}
			ArrayList<View> arrowlist = new ArrayList<>();
			for (int i = 0; i < grid_maze.getChildCount(); i++) {
				View v2 = grid_maze.getChildAt(i);
				if ((v2.getId() / OBST_ADD) == 1) {
					arrowlist.add(v2);
				}
			}
			for (View arrow : arrowlist) {
				grid_maze.removeView(arrow);
			}
		} else {
			boolean isobst;
			for (int i = 0; i < MAZE_C * MAZE_R; i++) {
				TextView tv = (TextView) grid_maze.findViewById(i);
				isobst = tv.getText().toString().equalsIgnoreCase(String.valueOf(Enum.Cell.OBSTACLE.get()));
				cell_update(tv, isobst ? Enum.Cell.OBSTACLE : Enum.Cell.DEFAULT, point_way != i);
			}
		}
	}

	protected void map_startgoal() {
		int[] list_void = {
			cell_id(origin_col, origin_row), //ORIGIN
			cell_id(12, 0) //GOAL
		};

		Drawable box = new_drawable(R.drawable.d_box, Enum.Cell.VOID.getColor());
		for (int v : list_void) {
			int add = -1;
			for (int i = 0; i < ROBOT_SIZE * ROBOT_SIZE; i++) {
				if (i != 0 && (i % ROBOT_SIZE) == 0) {
					add += (MAZE_C - ROBOT_SIZE + 1);
				} else {
					add++;
				}
				if ((v + add) != point_way) {
					grid_maze.findViewById(v + add).setBackground(box);
				}
			}
		}
	}

	protected void map_checkarrow() {
		ArrayList<View> arrowlist = new ArrayList<>();
		for (int i = 0; i < grid_maze.getChildCount(); i++) {
			View v2 = grid_maze.getChildAt(i);
			if ((v2.getId() / OBST_ADD) == 1) {
				TextView tv = (TextView) findViewById(v2.getId() % OBST_ADD);
				if (!tv.getText().toString().equalsIgnoreCase(String.valueOf(Enum.Cell.OBSTACLE.get()))) {
					arrowlist.add(v2);
				}
			}
		}
		for (View arrow : arrowlist) {
			grid_maze.removeView(arrow);
		}
	}

	protected void reg_bt_intentfilter() {
		IntentFilter filter = new IntentFilter();
		filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
		filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
		filter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
		filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECT_REQUESTED);
		filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
		filter.addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
		filter.addAction(BluetoothDevice.ACTION_FOUND);
		filter.addAction(BluetoothDevice.ACTION_PAIRING_REQUEST);
		registerReceiver(bt_receiver, filter);
	}

	protected void obst_arrow(int cell, String face) {
		int col = cell % MAZE_C,
			row = cell / MAZE_C;

		TextView tv = findViewById(OBST_ADD + cell);
		if (tv == null) {
			tv = new TextView(this);
			tv.setBackground(new_drawable(R.drawable.d_arrow, Color.TRANSPARENT));
			tv.setRotation(0);
			tv.setLayoutParams(new_layoutparams(col, row, 1));
			tv.setTextColor(Color.WHITE);
			tv.setGravity(Gravity.CENTER);
			tv.setId(OBST_ADD + cell);

			tv.setText(face);
			grid_maze.addView(tv);
		} else {
			tv.setText(face);
		}
	}

	protected AlertDialog.Builder pop_arrow() {
		View v = inflater.inflate(R.layout.pop_arrow, null);
		TextView tv_arrow = v.findViewById(R.id.arrow_txt_data);

		int count = 1;
		String text = r_string(R.string._null);
		for (int i = 0; i < grid_maze.getChildCount(); i++) {
			View v2 = grid_maze.getChildAt(i);
			if ((v2.getId() / OBST_ADD) == 1) {
				int cell = v2.getId() % OBST_ADD;
				if (count != 1) text += "\n";
				text += String.format("%d. %s (%d,%d)", count, ((TextView) v2).getText(), cell % MAZE_C, cell_fliprow(cell / MAZE_C));
				count++;
			}
		}
		if (text == r_string(R.string._null)) {
			text = "There are no Obstacles with an Up Arrow";
		}
		tv_arrow.setText(text);

		return new AlertDialog.Builder(this).setView(v);
	}

	protected void robot_reset(int reset_all, boolean reset_point) {
		// 1:: reset_all
		// 0:: reset explore
		//-1:: reset position only
		if (reset_point) {
			point_robot = cell_id(origin_col, origin_row);
		}
		if (reset_all > -1) map_reset(reset_all == 1);
		map_startgoal();
		robot_go();
		robot.setRotation(0);
	}

	protected void robot_go() {
		if (robot == null) {
			robot = new ImageView(this);
			robot.setImageDrawable(new_drawable(R.drawable.d_robot, Color.TRANSPARENT));
			grid_maze.addView(robot);
		}

		int col = point_robot % MAZE_C,
			row = point_robot / MAZE_C;
		robot.setLayoutParams(new_layoutparams(col, row, ROBOT_SIZE));

		if (bt_device == null) {//TODO:ROBOT BOTTOM
			TextView tv;
			for (int r = row; r < row + ROBOT_SIZE; r++) {
				for (int c = col; c < col + ROBOT_SIZE; c++) {
					int cell = cell_id(c, r);
					tv = grid_maze.findViewById(cell);
					cell_update(tv, Enum.Cell.PASSED, cell != point_way);
				}
			}
		}
	}

	protected void robot_rotate(int direction) {
		robot.setRotation(robot.getRotation() + (direction * 90));
	}

	protected void robot_move(Enum.Direction direction) {
		int temp_location = point_robot,
			col = point_robot % MAZE_C,
			row = point_robot / MAZE_C;

		switch (direction) {
			case UP:
				if (row > 0)
					temp_location -= MAZE_C;
				break;
			case DOWN:
				if (row < (MAZE_R - ROBOT_SIZE))
					temp_location += MAZE_C;
				break;
			case LEFT:
				if (col > 0)
					temp_location -= 1;
				break;
			case RIGHT:
				if (col < (MAZE_C - ROBOT_SIZE))
					temp_location += 1;
				break;
		}
		point_robot = temp_location;
		map_startgoal();
		robot_go();
	}

	protected AlertDialog.Builder pop_bluetooth() {
		View v = inflater.inflate(R.layout.pop_bluetooth, null);
		bt_lv_device = v.findViewById(R.id.bt_lv_devices);
		bt_lv_device.setOnItemClickListener(new AdapterView.OnItemClickListener() {
			@Override
			public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
				bt_canceldiscover();
				if (bt_display_isfind) {
					bt_newlist.get(position).createBond();
				} else {
					bt_connection.start_client(bt_pairedlist.get(position));
					bt_checkpaired();
				}
			}
		});
		v.findViewById(R.id.bt_btn_find).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				bt_canceldiscover();
				bt_display_isfind = true;

				bt_adapter.startDiscovery();
				bt_newlist.clear();
				bt_listview(v.getContext(), bt_newlist);
				bt_new_finding = true;
			}
		});
		v.findViewById(R.id.bt_btn_paired).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				bt_canceldiscover();
				bt_display_isfind = false;

				bt_getpaired();
				bt_listview(v.getContext(), bt_pairedlist);
			}
		});
		return new AlertDialog.Builder(this).setView(v);
	}

	protected void bt_canceldiscover() {
		if (bt_adapter.isDiscovering()) {
			if (bt_new_finding) bt_new_finding = false;
			bt_adapter.cancelDiscovery();
		}
	}

	protected void bt_listview(Context context, ArrayList<BluetoothDevice> devicelist) {
		bt_listadapter = new DeviceListAdapter(context, R.layout.adapter_device, devicelist);
		bt_lv_device.setAdapter(bt_listadapter);
	}

	protected void bt_getpaired() {
		Set<BluetoothDevice> set = bt_adapter.getBondedDevices();
		bt_pairedlist.clear();
		for (BluetoothDevice bd : set) {
			bt_pairedlist.add(bd);
		}

		if (bt_pairedlist.size() == 0) {
			new_message("There are no paired devices");
		}
	}

	protected void bt_checkpaired() {
		if (bt_device == null) {
			((TextView) findViewById(R.id.bt_lbl_connected)).setText(R.string.bt_isconnect_false);
			((TextView) findViewById(R.id.bt_txt_connected)).setText(r_string(R.string._null));
			findViewById(R.id.bt_txt_connected).setVisibility(View.INVISIBLE);

			findViewById(R.id.msg_lv_preview).setVisibility(View.GONE);
			findViewById(R.id.msg_tp_preview).setVisibility(View.GONE);
		} else {
			if (bt_dialog.isShowing()) bt_dialog.dismiss();

			((TextView) findViewById(R.id.bt_lbl_connected)).setText(R.string.bt_isconnect_true);
			((TextView) findViewById(R.id.bt_txt_connected)).setText(bt_device.getName());
			findViewById(R.id.bt_txt_connected).setVisibility(View.VISIBLE);

			if (dev_showchat) {
				findViewById(R.id.msg_lv_preview).setVisibility(View.VISIBLE);
				if (msg_chatlist.size() == 0) {
					findViewById(R.id.msg_tp_preview).setVisibility(View.VISIBLE);
				}
			} else {
				findViewById(R.id.msg_lv_preview).setVisibility(View.GONE);
				findViewById(R.id.msg_tp_preview).setVisibility(View.GONE);
			}
		}
		if (msg_dialog != null && msg_dialog.isShowing()) msg_dialog.dismiss();
	}

	protected void bt_update(int toggle) {
		// 1:: on bluetooth
		// 0:: off bluetooth
		//-1:: verify bluetooth details
		if (bt_adapter == null || menu == null) return;

		boolean on = bt_adapter.isEnabled();
		if (toggle == 1) {
			Intent intent_ACTION_REQUEST_ENABLE = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
			startActivityForResult(intent_ACTION_REQUEST_ENABLE, 1);
			on = true;
		} else if (toggle == 0) {
			bt_adapter.disable();
			on = false;
			msg_chatlist.clear();
		}

		menu.findItem(R.id.menu_bt).setIcon(new_drawable(on ? R.drawable.d_bt_on : R.drawable.d_bt_off, Color.TRANSPARENT));
		menu.findItem(R.id.menu_bt_on).setVisible(!on);
		menu.findItem(R.id.menu_bt_off).setVisible(on);
		menu.findItem(R.id.menu_bt_find).setVisible(on);
		menu.findItem(R.id.menu_bt_reconnect).setVisible(on);
	}

	protected AlertDialog.Builder pop_message() {
		View v = inflater.inflate(R.layout.pop_message, null);
		final TextView tv = v.findViewById(R.id.msg_txt_data);
		msg_lv_chat = v.findViewById(R.id.msg_lv_chat);
		msg_listview();

		v.findViewById(R.id.msg_btn_send).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (view_string(tv).trim().length() > 0) {
					msg_writemsg(view_string(tv).trim(), r_string(R.string._null));
				}
			}
		});
		return new AlertDialog.Builder(this).setView(v);
	}

	protected void msg_writemsg(String text, String description) {
		if (bt_device == null) {

			new_message("unable to send message");

		} else {

			text = msg_appendto(text);
			byte[] bytes = text.getBytes(Charset.defaultCharset());
			bt_connection.write(bytes);

			msg_chatlist.add(new MessageText(false, text, description, getResources()));
			msg_listview();

		}
	}

	protected String msg_appendto(String text) {
		boolean contains_delimeter = text.contains(r_string(R.string._delimiter)),
			contains_bracket = text.contains(r_string(R.string._bracket_s));

		if (contains_delimeter && contains_bracket) {
			return Enum.To.ALGORITHM.getCode() + text;
		} else if (enum_getinstruction(text) != null) {
			return Enum.To.ARDUINO.getCode() + text;
		} else {
			return text;
		}
	}

	protected void msg_listview() {
		msg_listadapter = new ChatListAdapter(this, R.layout.adapter_message, msg_chatlist);

		if (msg_lv_preview != null) {
			if (msg_chatlist.size() != 0)
				findViewById(R.id.msg_tp_preview).setVisibility(View.GONE);

			msg_lv_preview.setAdapter(msg_listadapter);
			msg_lv_preview.post(new Runnable() {
				@Override
				public void run() {
					msg_lv_preview.setSelection(msg_listadapter.getCount() - 1);
				}
			});
		}

		if (msg_lv_chat != null) {
			msg_lv_chat.setAdapter(msg_listadapter);
			msg_lv_chat.post(new Runnable() {
				@Override
				public void run() {
					msg_lv_chat.setSelection(msg_listadapter.getCount() - 1);
				}
			});
		}
	}

	protected void bt_robust_option() {
		SwitchCompat s = findViewById(R.id.bt_swt_isrobust);
		if (s.isChecked()) {
			s.setText(R.string.bt_isrobust_true);
			bt_robust = true;
		} else {
			s.setText(R.string.bt_isrobust_false);
			bt_robust = false;
		}
	}

	protected void time_option() {
		SwitchCompat s = findViewById(R.id.time_swt_isfastest);
		if (s.isChecked()) {
			s.setText(R.string.time_fastest);
			time_tv = findViewById(R.id.time_txt_fastest);
		} else {
			s.setText(R.string.time_explore);
			time_tv = findViewById(R.id.time_txt_explore);
		}
	}

	protected void time_reset() {
		time_start = 0L;
		time_set(0, 0, 0);
	}

	protected void time_stopwatch() {
		Button b = (Button) findViewById(R.id.time_btn_start);
		if (view_string(b).equalsIgnoreCase(r_string(R.string.time_isstart_false))) {
			time_reset();
			if (((SwitchCompat) findViewById(R.id.time_swt_isfastest)).isChecked()) {
				msg_writemsg("al_startf", "");

				robot_reset(-1, true);
			} else {
				msg_writemsg("al_starte", "");

				int temp_way = point_way;
				robot_reset(1, true);
				if (temp_way != -1) {
					point_set(true, temp_way);
				}
			}
			time_start = SystemClock.uptimeMillis();
			time_handler.postDelayed(time_runnable, 0);
			findViewById(R.id.time_swt_isfastest).setEnabled(false);
			b.setText(R.string.time_isstart_true);
		} else {
			time_handler.removeCallbacks(time_runnable);

			findViewById(R.id.time_swt_isfastest).setEnabled(true);
			b.setText(R.string.time_isstart_false);
		}
	}

	protected void time_set(int min, int sec, int millisec) {
		time_tv.setText(String.format("%d:%s:%s", min, String.format("%02d", sec), String.format("%03d", millisec)));
	}

	protected void point_option() {
		SwitchCompat s = findViewById(R.id.point_swt_isway);
		if (s.isChecked()) {
			s.setText(R.string.point_swt_way);
		} else {
			s.setText(R.string.point_swt_origin);
		}
	}

	protected void point_toset() {
		Enum.Instruction instruction = ((SwitchCompat) findViewById(R.id.point_swt_isway)).isChecked() ? Enum.Instruction.WAY : Enum.Instruction.ORIGIN;
		if (point_isset && instruction == Enum.Instruction.WAY && point_way == -1) {
			point_isset = !point_isset;
			findViewById(R.id.point_swt_isway).setEnabled(!point_isset);
			((TextView) findViewById(R.id.point_btn_set)).setText(r_string(point_isset ? R.string.point_isset_true : R.string.point_isset_false));
		} else {
			if (point_isset) {
				String col, row;
				if (instruction == Enum.Instruction.WAY) {
					col = view_string(findViewById(R.id.point_txt_way_x));
					row = view_string(findViewById(R.id.point_txt_way_y));
				} else {
					col = view_string(findViewById(R.id.point_txt_origin_x));
					row = view_string(findViewById(R.id.point_txt_origin_y));
					point_setorigin();
				}
				msg_writemsg(String.format("%s{%s,%s}", instruction.getText(), col, row), instruction.getDescription());
			} else {
				origin_temp = cell_id(origin_col, origin_row);
			}
			point_isset = !point_isset;
			findViewById(R.id.point_swt_isway).setEnabled(!point_isset);
			((TextView) findViewById(R.id.point_btn_set)).setText(r_string(point_isset ? R.string.point_isset_true : R.string.point_isset_false));
		}
	}

	protected String point_set(boolean isway, int cell) {
		if (isway) {
			Drawable box;
			if (point_way > -1) {
				box = new_drawable(R.drawable.d_box, enum_getcolor(Integer.valueOf(view_string(grid_maze.findViewById(point_way)))));
				grid_maze.findViewById(point_way).setBackground(box);
			}
			box = new_drawable(R.drawable.d_box, Enum.Cell.WAY.getColor());
			grid_maze.findViewById(cell).setBackground(box);

			point_way = cell;
		} else {
			int new_cell = cell_robot(true, cell),
				col = new_cell % MAZE_C,
				row = new_cell / MAZE_C;

			if (row < (MAZE_R / 2)) {
				return new_message("Robot cannot be placed on the second-half");
			} else if ((col > (MAZE_C - ROBOT_SIZE)) || (row > (MAZE_R - ROBOT_SIZE))) {
				return new_message("Robot cannot move to that point");
			} else {
				point_robot = new_cell;
				map_reset(false);
				robot_go();
				origin_temp = cell_id(col, row);
			}
		}
		point_update(isway);
		return r_string(R.string._success);
	}

	protected void point_setorigin() {
		origin_col = origin_temp % MAZE_C;
		origin_row = origin_temp / MAZE_C;
		robot_reset(0, false);
	}

	protected void point_update(boolean isway) {
		if (isway) {
			((TextView) findViewById(R.id.point_txt_way_x)).setText((point_way == -1) ? "-" : String.valueOf(point_way % MAZE_C));
			((TextView) findViewById(R.id.point_txt_way_y)).setText((point_way == -1) ? "-" : String.valueOf(cell_fliprow(point_way / MAZE_C)));
		} else {
			int cell = cell_robot(false, (origin_temp == -1) ? cell_id(origin_col, origin_row) : origin_temp);
			((TextView) findViewById(R.id.point_txt_origin_x)).setText(String.valueOf(cell % MAZE_C));
			((TextView) findViewById(R.id.point_txt_origin_y)).setText(String.valueOf(cell_fliprow(cell / MAZE_C)));
		}
	}

	protected void update_arena() {
		if (r_string(R.string._null).equalsIgnoreCase(mdf_string1) || r_string(R.string._null).equalsIgnoreCase(mdf_string2)) {
			return;
		}

		String s1 = new BigInteger(mdf_string1, 16).toString(2),//mdf_string1 :: explored
			s2 = new BigInteger("f" + mdf_string2, 16).toString(2);//mdf_string2 :: obstacles based on explored

		int s2_i = 4;
		Enum.Cell type;
		for (int i = 0; i < s1.length() - 4; i++) {
			int cell = cell_id(i % MAZE_C, cell_fliprow(i / MAZE_C));

			if (s1.charAt(i + 2) == '1') {
				type = (s2.charAt(s2_i) == '1') ? Enum.Cell.OBSTACLE : Enum.Cell.PASSED;
				s2_i++;
			} else {
				type = Enum.Cell.DEFAULT;
			}
			cell_update((TextView) findViewById(cell), type, point_way != cell);
		}
		map_startgoal();
	}

	protected void msg_movements(boolean towrite, boolean tolist, Enum.Instruction instruction) {
		if (instruction != null) {
			switch (instruction) {
				case FORWARD:
					robot_move(enum_getdirection((((int) robot.getRotation()) % 360) / 90));
					break;
				case REVERSE:
					robot_move(enum_getdirection((((int) (robot.getRotation() + 180)) % 360) / 90));
					break;
				case ROTATE_LEFT:
					robot_rotate(Enum.Direction.LEFT.get());
					break;
				case ROTATE_RIGHT:
					robot_rotate(Enum.Direction.RIGHT.get());
					break;
			}
			((TextView) findViewById(R.id.txt_status)).setText(instruction.getStatus());
			if (bt_device != null && towrite) {
				msg_writemsg(instruction.getText(), instruction.getDescription());
			}
		}
	}

	//DEVELOPER
	protected void dev_toggle(boolean toggle) {
		if (toggle) {
			dev_showchat = !dev_showchat;
		}

		enable_layout(R.id.layout_direction);
		bt_checkpaired();
	}

	protected AlertDialog.Builder pop_dev() {
		View v = inflater.inflate(R.layout.pop_developer, null);
		v.findViewById(R.id.dev_restart).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				reset_app();
				dev_dialog.dismiss();
			}
		});
		v.findViewById(R.id.dev_sensor).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				msg_writemsg("ar_g", "");
				dev_dialog.dismiss();
			}
		});
		v.findViewById(R.id.dev_crash).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				msg_writemsg("al_crash", "");
				dev_dialog.dismiss();
			}
		});

		Button btn = (Button) v.findViewById(R.id.dev_chat);
		btn.setText("chat :: " + String.valueOf(dev_showchat));
		btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				dev_toggle(true);
				dev_dialog.dismiss();
			}
		});

		return new AlertDialog.Builder(this).setView(v);
	}
}