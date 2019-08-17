package frc.bumblelib.util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BumbleDrawing {
	/* Class that includes functions to draw on camera stream (Mat) */
	
	public static void drawCross(Mat img, int XPoint, int Ypoint, Scalar Color, int Length, int thickness) {
		Point WidthRigth = new Point(XPoint - Length / 2, Ypoint);
		Point WidthLeft = new Point(XPoint + Length / 2, Ypoint);
		Point HeightTop = new Point(XPoint, Ypoint + Length / 2);
		Point HeightDown = new Point(XPoint, Ypoint - Length / 2);
		Imgproc.line(img, WidthLeft, WidthRigth, Color, thickness);
		Imgproc.line(img, HeightTop, HeightDown, Color, thickness);
	}

	public static void drawRectangle(Mat img, int XPoint, int Ypoint, int Height, int Length, Scalar color,
			int thickness) {
		Point TopLeft = new Point(XPoint - Length / 2, Ypoint + Height / 2);
		Point DownRigth = new Point(XPoint + Length / 2, Ypoint - Height / 2);
		Imgproc.rectangle(img, TopLeft, DownRigth, color, thickness);
	}
}
