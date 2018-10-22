package android.mdp.android_optimized;

import android.content.res.Resources;

public class MessageText {

	private int color;
	private String text;
	private String description;

	MessageText(boolean isread, String t, String d, Resources r) {
		color = isread ? r.getColor(R.color.light_sky_blue) : r.getColor(R.color.pale_canary);
		text = t;
		description = d;
	}

	public int getColor() {
		return color;
	}

	public String getText() {
		return text;
	}

	public String getDescription() {
		return description;
	}
}
