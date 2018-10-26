package android.mdp.android_optimized;

import android.graphics.Color;

public class Enum {

	public enum Cell {
		DEFAULT(0, Color.GREEN),
		PASSED(1, Color.YELLOW),
		OBSTACLE(2, Color.BLACK),
		WAY(3, Color.RED),
		VOID(4, Color.BLUE);

		Cell(int id, int color) {
			this.id = id;
			this.color = color;
		}

		private int id;
		private int color;

		public int get() {
			return id;
		}

		public int getColor() {
			return color;
		}
	}

	public enum Direction {
		UP(0, "U"),
		RIGHT(1, "R"),
		DOWN(2, "D"),
		LEFT(3, "L");

		Direction(int id, String chara) {
			this.id = id;
			this.chara = chara;
		}

		private int id;
		private String chara;

		public int get() {
			return id;
		}

		public String getChara() {
			return chara;
		}
	}

	public enum Instruction {
		FORWARD("Forward", "Moving Forward", "w"),
		REVERSE("Reverse", "Moving Backward", "s"),
		ROTATE_LEFT("Rotate Left", "Rotating Left", "a"),
		ROTATE_RIGHT("Rotate Right", "Rotating Right", "d"),

		STOP("Robot Stop Moving", "Robot Stop", "stop"),
		CALIBRATING("Robot Calibrate", "Robot Calibrating", "c"),
		SENSOR("Robot Sensor", "Robot Sensing", "g"),

		SEND_ARENA_INFO("Send Arena Info", "", "sendArena"),
		MDF("MDF string", "", "mdf"),//mdf{s1,s2}

		ORIGIN("Origin point", "", "origin"),//origin{x,y}
		WAY("Way point", "", "way"),//way{x,y}
		OBSTACLE("Obstacle", "", "obstacle"),//obstacle{x,y}
		ARROW("Obstacle with Up Arrow", "", "arrow"),//arrow{x,y,direction}
		CENTER("Robot location", "", "center");//center(x,y,direction}

		Instruction(String description, String status, String text) {
			this.description = description;
			this.status = status;
			this.text = text;
		}

		private String description;
		private String status;
		private String text;

		public String getDescription() {
			return description;
		}

		public String getStatus() {
			return status;
		}

		public String getText() {
			return text;
		}
	}

	public enum To {
		ARDUINO("ar_"),
		ALGORITHM("al_"),
		RASPBERRYPI("rp_");

		To(String code) {
			this.code = code;
		}

		private String code;

		public String getCode() {
			return code;
		}
	}
}
